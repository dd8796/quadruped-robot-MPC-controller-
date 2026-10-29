#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <Eigen/Dense>

#include "grid_map_core/grid_map_core.hpp"
#include "realtime_tools/realtime_buffer.hpp"

#include "quadru_control/leg_kinematics.hpp"
#include "quadru_control/foothold_safety.hpp"

namespace quadru_control
{

// Non-realtime planning thread: foothold safety and convex decomposition (arXiv 2307.14775,
// II-A/II-C), once per swing and per leg. The realtime loop only posts requests (try_lock, fixed
// size) and reads the latest results (RealtimeBuffer).
class PerceptivePlanner
{
public:
  static constexpr int kNoLeg = -1;

  struct FootholdRequest
  {
    std::uint64_t swingId = 0;    // swing identifier of the leg
    bool convex = true;           // true: convex region (II-C), false: nearest safe pixel ([8])
    int legUrdf = 0;
    Eigen::Vector2d nominalW = Eigen::Vector2d::Zero();
    double windowHalf = 0.12;     // m, half-size of the foothold heightmap
    Eigen::Vector3d qLeg = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d trunkNow = Eigen::Vector3d::Zero();
    Eigen::Vector3d trunkTouchdown = Eigen::Vector3d::Zero();
    Eigen::Vector3d swingStart = Eigen::Vector3d::Zero();
    SafetyParams params;
  };

  struct FootholdResult
  {
    std::uint64_t swingId = 0;    // 0 = never evaluated
    bool hasTarget = false;
    ConvexRegion region;          // convex mode: selected region (world)
    Eigen::Vector2d targetW = Eigen::Vector2d::Zero();  // nominal clamped into the region
    double targetZ = 0.0;         // foot center at touchdown (ground + radius)
    FootholdHeightmap heightmap;  // heuristic mode: safety map, re-read every tick
    Eigen::MatrixXi safe;
    SafetyStats stats;
    double evalMs = 0.0;
  };
  using FootholdResults = std::array<FootholdResult, 4>;

  using Logger = std::function<void (const std::string &)>;

  PerceptivePlanner(const std::array<LegKinematics, 4> & legs, const std::string & layer);
  ~PerceptivePlanner();
  PerceptivePlanner(const PerceptivePlanner &) = delete;
  PerceptivePlanner & operator=(const PerceptivePlanner &) = delete;

  void start(Logger logger = nullptr);
  void stop();
  // Clears requests and results. Thread must be stopped.
  void reset();

  // Non-realtime (ROS callback).
  void setTerrainMap(std::shared_ptr<const grid_map::GridMap> map);

  // Realtime side, never blocks. Returns false if the lock is taken (retry next tick).
  bool postFootholdRequest(int legMpc, const FootholdRequest & request);
  // Valid until the next call; call once per tick.
  const FootholdResults * readFootholdResults() {return footholdResults_.readFromRT();}
  // nullptr if no map yet.
  const grid_map::GridMap * readTerrainMap();

  // Processes pending requests in the calling thread (tests).
  void processPending();

private:
  void loop();
  void evaluateFoothold(int legMpc, const FootholdRequest & request);
  std::shared_ptr<const grid_map::GridMap> terrainForWorker();

  const std::array<LegKinematics, 4> & legs_;
  const std::string layer_;

  // Requests: written by the realtime loop under try_lock, consumed by the thread.
  std::mutex requestMutex_;
  std::condition_variable requestCv_;
  std::array<FootholdRequest, 4> footholdRequests_{};
  std::array<bool, 4> footholdPending_{};

  // Map: replaced by the ROS callback, read by the thread and the realtime loop. Old maps are
  // released on the writer side.
  std::mutex mapMutex_;
  std::shared_ptr<const grid_map::GridMap> workerMap_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<const grid_map::GridMap>> rtMap_;

  // Results: written by the thread, read by the realtime loop.
  FootholdResults footholdMaster_{};   // all 4 legs, thread-owned
  realtime_tools::RealtimeBuffer<FootholdResults> footholdResults_;

  std::thread thread_;
  std::atomic<bool> running_{false};
  Logger logger_;
};

}  // namespace quadru_control
