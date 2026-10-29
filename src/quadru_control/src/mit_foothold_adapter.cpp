#include "quadru_control/mit_foothold_adapter.hpp"

#include "quadru_control/controller_config.hpp"
#include "quadru_control/terrain_awareness.hpp"
#include "terrain_mapping/terrain_processing.hpp"

namespace quadru_control
{

namespace
{
constexpr float kTouchdownZ = -0.003f;   // MIT touchdown target, slightly below ground
}  // namespace

MitFootholdAdapter::MitFootholdAdapter(PerceptivePlanner & planner, double footRadius, double liftHeight)
: planner_(planner), footRadius_(footRadius), liftHeight_(liftHeight)
{
}

void MitFootholdAdapter::reset()
{
  swing_.fill(Swing{});
  swingId_.fill(0);
  footGround_.setZero();
  commits.fill(Commit{});
}

void MitFootholdAdapter::adapt(
  int leg, float swingPhase, float swingTime, const Vec3<float> & pFoot, Vec3<float> & Pf)
{
  Swing & sw = swing_[leg];
  if (swingPhase <= 0.f) {
    // Stance: the foot sits at the height targeted by its last swing.
    if (sw.active) {
      footGround_[leg] = static_cast<float>(sw.target.z());
      sw = Swing{};
    }
    return;
  }
  const Eigen::Vector2d nominal(Pf[0], Pf[1]);
  if (!sw.active) {
    sw.active = true;
    ++swingId_[leg];
  }
  // Ground under the nominal foothold, else under the leg's last foothold.
  double ground = footGround_[leg];
  if (ctx_.map != nullptr) {
    if (const auto h = lookupTerrainHeight(*ctx_.map, nominal.x(), nominal.y(), cfg::kFootHeightSearchRadiusCells)) {
      ground = *h;
    }
  }
  if (!sw.committed) {sw.target = Eigen::Vector3d(nominal.x(), nominal.y(), ground);}

  const bool adapt = ctx_.map != nullptr && ctx_.results != nullptr &&
    ctx_.map->exists(terrain_mapping::kLayerFiltered) && ctx_.mode != FootholdMode::kNominal;
  if (adapt && !sw.requested) {
    PerceptivePlanner::FootholdRequest req;
    req.swingId = swingId_[leg];
    req.convex = ctx_.mode == FootholdMode::kConvexMpc;
    req.legUrdf = leg;   // MIT order = URDF order
    req.nominalW = nominal;
    req.windowHalf = cfg::kFootholdWindowHalf;
    req.qLeg = ctx_.q.segment<3>(3 * leg);
    req.R = ctx_.trunkR;
    req.trunkNow = ctx_.trunkOrigin;
    req.trunkTouchdown = ctx_.trunkOrigin + ctx_.velocity * (1.0 - swingPhase) * swingTime;
    req.swingStart = Eigen::Vector3d(pFoot[0], pFoot[1], pFoot[2] + footRadius_);
    req.params.footRadius = footRadius_;
    req.params.swingDuration = swingTime;
    req.params.liftHeight = liftHeight_;
    if (planner_.postFootholdRequest(leg, req)) {
      sw.requested = true;
      sw.convex = req.convex;
    }
  }
  if (sw.requested && ctx_.results != nullptr && !sw.committed) {
    const auto & fr = (*ctx_.results)[leg];
    if (fr.swingId == swingId_[leg]) {
      if (!sw.evaluated) {
        sw.evaluated = true;
        sw.hasTarget = fr.hasTarget;
        if (fr.hasTarget) {sw.region = fr.region;}
      }
      if (sw.hasTarget && sw.convex) {
        // II-C: nearest point of the region to the current nominal.
        const Eigen::Vector2d xy = nominal.cwiseMax(sw.region.min).cwiseMin(sw.region.max);
        sw.target = Eigen::Vector3d(xy.x(), xy.y(), sw.region.height);
      } else if (sw.hasTarget) {
        // [8]: nearest safe point to the current nominal.
        if (const auto f = nearestSafeFoothold(fr.heightmap, fr.safe, nominal)) {sw.target = *f;}
      }
    }
  }
  if (!sw.committed && swingPhase > cfg::kFootholdCommitPhase) {
    sw.committed = true;
    Commit & c = commits[leg];
    c.valid = true;
    c.pointWorld = Eigen::Vector3d(sw.target.x(), sw.target.y(), sw.target.z() + footRadius_);
    c.hasRegion = sw.evaluated && sw.hasTarget && sw.convex;
    c.region = sw.region;
  }
  // MIT target: z = ground (foot center plane) + MIT contact depth.
  Pf[0] = static_cast<float>(sw.target.x());
  Pf[1] = static_cast<float>(sw.target.y());
  Pf[2] = static_cast<float>(sw.target.z()) + kTouchdownZ;
}

}  // namespace quadru_control
