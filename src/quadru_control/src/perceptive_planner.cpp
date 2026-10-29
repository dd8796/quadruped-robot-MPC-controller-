#include "quadru_control/perceptive_planner.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>

namespace quadru_control
{

PerceptivePlanner::PerceptivePlanner(const std::array<LegKinematics, 4> & legs, const std::string & layer)
: legs_(legs), layer_(layer), rtMap_(std::shared_ptr<const grid_map::GridMap>()),
  footholdResults_(FootholdResults{})
{
}

PerceptivePlanner::~PerceptivePlanner()
{
  stop();
}

void PerceptivePlanner::start(Logger logger)
{
  if (running_.exchange(true)) {return;}
  logger_ = std::move(logger);
  thread_ = std::thread([this]() {loop();});
}

void PerceptivePlanner::stop()
{
  if (!running_.exchange(false)) {return;}
  requestCv_.notify_all();
  if (thread_.joinable()) {thread_.join();}
}

void PerceptivePlanner::reset()
{
  std::lock_guard<std::mutex> lock(requestMutex_);
  footholdPending_.fill(false);
  footholdMaster_ = FootholdResults{};
  footholdResults_.writeFromNonRT(footholdMaster_);
}

void PerceptivePlanner::setTerrainMap(std::shared_ptr<const grid_map::GridMap> map)
{
  {
    std::lock_guard<std::mutex> lock(mapMutex_);
    workerMap_ = map;
  }
  rtMap_.writeFromNonRT(std::move(map));
}

const grid_map::GridMap * PerceptivePlanner::readTerrainMap()
{
  const auto * p = rtMap_.readFromRT();
  return (p != nullptr && *p) ? p->get() : nullptr;
}

std::shared_ptr<const grid_map::GridMap> PerceptivePlanner::terrainForWorker()
{
  std::lock_guard<std::mutex> lock(mapMutex_);
  return workerMap_;
}

bool PerceptivePlanner::postFootholdRequest(int legMpc, const FootholdRequest & request)
{
  std::unique_lock<std::mutex> lock(requestMutex_, std::try_to_lock);
  if (!lock.owns_lock()) {return false;}
  footholdRequests_[legMpc] = request;
  footholdPending_[legMpc] = true;
  lock.unlock();
  requestCv_.notify_one();
  return true;
}

void PerceptivePlanner::loop()
{
  while (running_.load()) {
    {
      std::unique_lock<std::mutex> lock(requestMutex_);
      requestCv_.wait_for(lock, std::chrono::milliseconds(50), [this]() {
        return !running_.load() || footholdPending_[0] || footholdPending_[1] || footholdPending_[2] ||
               footholdPending_[3];
      });
    }
    if (!running_.load()) {break;}
    processPending();
  }
}

void PerceptivePlanner::processPending()
{
  std::array<FootholdRequest, 4> footholds;
  std::array<bool, 4> footholdTodo{};
  {
    std::lock_guard<std::mutex> lock(requestMutex_);
    footholds = footholdRequests_;
    footholdTodo = footholdPending_;
    footholdPending_.fill(false);
  }
  bool footholdsChanged = false;
  for (int m = 0; m < 4; ++m) {
    if (footholdTodo[m]) {
      evaluateFoothold(m, footholds[m]);
      footholdsChanged = true;
    }
  }
  if (footholdsChanged) {footholdResults_.writeFromNonRT(footholdMaster_);}
}

void PerceptivePlanner::evaluateFoothold(int legMpc, const FootholdRequest & req)
{
  const auto t0 = std::chrono::steady_clock::now();
  FootholdResult & res = footholdMaster_[legMpc];
  res = FootholdResult{};
  res.swingId = req.swingId;
  const auto map = terrainForWorker();
  if (!map || !map->exists(layer_)) {return;}

  // II-A: local heightmap around the nominal, per-pixel criteria.
  res.heightmap = extractFootholdHeightmap(*map, layer_, req.nominalW, req.windowHalf);
  LegContext ctx;
  ctx.leg = &legs_[req.legUrdf];
  ctx.qCurrent = req.qLeg;
  ctx.R = req.R;
  ctx.trunkNow = req.trunkNow;
  ctx.trunkTouchdown = req.trunkTouchdown;
  ctx.swingStart = req.swingStart;
  res.safe = evaluateFootholdSafety(
    res.heightmap, *map, layer_, ctx, req.params, {}, &res.stats);

  if (!req.convex) {
    // [8]: the realtime loop picks the nearest safe point every tick.
    res.hasTarget = res.safe.sum() > 0;
  } else if (const auto region = selectNearestRegion(
      decomposeConvexRegions(res.heightmap, res.safe), req.nominalW))
  {
    // II-C: nearest convex region.
    res.hasTarget = true;
    res.region = *region;
    res.targetW = req.nominalW.cwiseMax(region->min).cwiseMin(region->max);
    res.targetZ = region->height + req.params.footRadius;
  }
  res.evalMs = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count();

  if (logger_) {
    char buf[320];
    if (res.hasTarget) {
      std::snprintf(
        buf, sizeof(buf), "appui patte MPC=%d (vol %llu) : evaluation de surete %.2f ms, %d/%d pixels surs",
        legMpc, static_cast<unsigned long long>(req.swingId), res.evalMs, res.safe.sum(),
        static_cast<int>(res.safe.size()));
    } else {
      std::snprintf(
        buf, sizeof(buf), "aucun appui sur pour la patte MPC=%d (rejets : inconnu %d, rugosite %d, "
        "atteignabilite %d, collision frontale %d, collision segments %d) -- appui nominal conserve",
        legMpc, res.stats.unknown, res.stats.roughness, res.stats.reachability,
        res.stats.frontalCollision, res.stats.legCollision);
    }
    logger_(buf);
  }
}

}  // namespace quadru_control
