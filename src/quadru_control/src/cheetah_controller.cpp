#include "quadru_control/cheetah_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include "grid_map_ros/GridMapRosConverter.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "terrain_mapping/terrain_processing.hpp"
#include "quadru_control/controller_config.hpp"

namespace quadru_control
{

using namespace cfg;

namespace
{
// MIT body frame (x forward) <-> URDF trunk frame (forward = -x): rotation of pi about z.
// URDF legs 1..4 are already in MIT order (FR, FL, HR, HL).
const Eigen::Matrix3d kRf = Eigen::Vector3d(-1.0, -1.0, 1.0).asDiagonal();
constexpr double kWarmupDuration = 0.1;     // s, IMU wait before capturing the start pose
constexpr double kMaxDt = 0.05;             // s, larger periods hold the stand
// Gait in MIT OffsetDurationGait form: 18 segments (MIT horizon <= 19), stance =
// round(duty * 18) segments, diagonal pairs half a cycle apart.
constexpr int kGaitSegments = 18;
constexpr double kVelIntegralMax = 0.2;    // m/s
constexpr double kMaxRollPitch = 0.5;
// ~/diag_control: legs in URDF order; foot vectors hip-relative, MPC body frame (x forward);
// f_mpc, x_ref, x_est in the MPC world frame, state order (roll, pitch, yaw, x, y, z, wx, wy, wz,
// vx, vy, vz); v_cmd = (vx, vy, yaw rate), body frame; mpc = (solved, solve ms, MPC period ms).
constexpr int kControlDiagDecimation = 2;   // 500 Hz
constexpr const char * kControlDiagLayout =
  "t,tau[12],q[12],qd[12],p_des[12],p[12],v_des[12],v[12],swing[4],f_mpc[12],x_ref[12],x_est[12],"
  "v_cmd[3],mpc[3]";
constexpr int kControlDiagSize = 1 + 12 * 7 + 4 + 12 * 3 + 3 + 3;       // rad, MIT SafetyChecker::checkSafeOrientation
const std::array<bool, 4> kAllFeet = {true, true, true, true};

int parseFootholdMode(const std::string & mode)
{
  if (mode == "nominal") {return static_cast<int>(FootholdMode::kNominal);}
  if (mode == "heuristic") {return static_cast<int>(FootholdMode::kHeuristic);}
  return static_cast<int>(FootholdMode::kConvexMpc);
}

const std::array<std::string, 12> kJointNames = {
  "leg1_abduction", "leg1_flex", "leg1_knee", "leg2_abduction", "leg2_flex", "leg2_knee",
  "leg3_abduction", "leg3_flex", "leg3_knee", "leg4_abduction", "leg4_flex", "leg4_knee"};
}  // namespace

CheetahController::CheetahController()
: legs_(buildAllLegs()), model_(RobotModel::trunkOnly())
{
}

controller_interface::InterfaceConfiguration CheetahController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : kJointNames) {config.names.push_back(name + "/effort");}
  return config;
}

controller_interface::InterfaceConfiguration CheetahController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : kJointNames) {config.names.push_back(name + "/position");}
  for (const auto & name : kJointNames) {config.names.push_back(name + "/velocity");}
  return config;
}

controller_interface::CallbackReturn CheetahController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CheetahController::on_configure(const rclcpp_lifecycle::State &)
{
  stand_ = std::make_unique<StandController>(legs_, model_);
  auto node = get_node();
  contacts_.configure(node);
  publishers_.configure(node);
  feetPub_ = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
    node->create_publisher<std_msgs::msg::Float64MultiArray>("~/diag_feet", rclcpp::SystemDefaultsQoS()));
  controlPub_ = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
    node->create_publisher<std_msgs::msg::Float64MultiArray>("~/diag_control", rclcpp::SystemDefaultsQoS()));
  controlPub_->msg_.layout.dim.resize(1);
  controlPub_->msg_.layout.dim[0].label = kControlDiagLayout;
  controlPub_->msg_.layout.dim[0].size = kControlDiagSize;
  controlPub_->msg_.layout.dim[0].stride = kControlDiagSize;
  controlPub_->msg_.data.resize(kControlDiagSize);
  walkSpeed_ = auto_declare<double>("walk_speed", 0.3);
  cmdAccelLimit_ = auto_declare<double>("cmd_accel_limit", 0.0);
  velIntegralGain_ = auto_declare<double>("vel_integral_gain", 0.0);
  gaitCycle_ = auto_declare<double>("gait_cycle_s", kTCycle);
  gaitDuty_ = auto_declare<double>("gait_duty", kDuty);
  swingHeight_ = auto_declare<double>("swing_height", kStepH);

  planner_ = std::make_unique<PerceptivePlanner>(legs_, terrain_mapping::kLayerFiltered);
  footholds_ = std::make_unique<MitFootholdAdapter>(*planner_, kFootRadius, swingHeight_);
  terrainAware_ = auto_declare<bool>("terrain_aware", false);
  footholdMode_ = parseFootholdMode(auto_declare<std::string>("foothold_adaptation", "convex_mpc"));
  paramCallbackHandle_ = node->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : params) {
        if (p.get_name() == "terrain_aware") {terrainAware_.store(p.as_bool());}
        if (p.get_name() == "foothold_adaptation") {footholdMode_.store(parseFootholdMode(p.as_string()));}
        if (p.get_name() == "use_localization") {useLocalization_.store(p.as_bool());}
        if (p.get_name() == "swing_control") {swingOsc_.store(p.as_string() == "osc");}
        if (p.get_name() == "swing_omega") {swingOmega_.store(p.as_double());}
        if (p.get_name() == "swing_zeta") {swingZeta_.store(p.as_double());}
        if (p.get_name() == "walk_speed") {walkSpeed_.store(p.as_double());}
        if (p.get_name() == "cmd_accel_limit") {cmdAccelLimit_.store(p.as_double());}
        if (p.get_name() == "vel_integral_gain") {velIntegralGain_.store(p.as_double());}
        if (p.get_name() == "joint_damping_comp") {jointDampingComp_.store(p.as_double());}
        if (p.get_name() == "joint_friction_comp") {jointFrictionComp_.store(p.as_double());}
        if (p.get_name() == "foot_rolling_comp") {footRollingComp_.store(p.as_bool());}
      }
      return result;
    });
  // External localization (trunk pose, world). Empty: leg odometry only.
  const std::string localizationTopic = auto_declare<std::string>("localization_topic", "");
  useLocalization_ = auto_declare<bool>("use_localization", true);
  swingOsc_ = auto_declare<std::string>("swing_control", "osc") == "osc";
  swingOmega_ = auto_declare<double>("swing_omega", 50.0);
  swingZeta_ = auto_declare<double>("swing_zeta", 1.0);
  jointDampingComp_ = auto_declare<double>("joint_damping_comp", 0.0);
  jointFrictionComp_ = auto_declare<double>("joint_friction_comp", 0.0);
  footRollingComp_ = auto_declare<bool>("foot_rolling_comp", false);
  dynamics_ = std::make_unique<FloatingBaseModel>(
    model_.mass(), kTrunkCom, kRf * model_.inertia() * kRf, legs_);
  localizationBuffer_.writeFromNonRT(LocalizationSample{});
  if (!localizationTopic.empty()) {
    localizationSub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      localizationTopic, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        LocalizationSample s;
        const auto & p = msg->pose.pose;
        s.trunkPosition = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
        s.trunkOrientation = Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
        s.stampNs = std::chrono::steady_clock::now().time_since_epoch().count();
        localizationBuffer_.writeFromNonRT(s);
        localizationReceived_.store(true, std::memory_order_release);
      });
    RCLCPP_INFO(node->get_logger(), "localisation externe : %s", localizationTopic.c_str());
  }
  terrainMapSub_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
    "/terrain_mapping_node/elevation_map", rclcpp::SensorDataQoS(),
    [this](const grid_map_msgs::msg::GridMap::SharedPtr msg) {
      auto map = std::make_shared<grid_map::GridMap>();
      if (grid_map::GridMapRosConverter::fromMessage(*msg, *map)) {planner_->setTerrainMap(std::move(map));}
    });

  imuSub_ = node->create_subscription<sensor_msgs::msg::Imu>(
    "/chien/imu", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      imuBuffer_.writeFromNonRT(*msg);
      imuReceived_.store(true, std::memory_order_release);
    });
  cmdVelBuffer_.writeFromNonRT(CmdVelSample{});
  cmdVelSub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      CmdVelSample s;
      s.vx = msg->linear.x;
      s.vy = msg->linear.y;
      s.wz = msg->angular.z;
      s.stampNs = std::chrono::steady_clock::now().time_since_epoch().count();
      cmdVelBuffer_.writeFromNonRT(s);
      cmdVelEverReceived_.store(true, std::memory_order_release);
    });
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CheetahController::on_activate(const rclcpp_lifecycle::State &)
{
  state_ = ControlState::kStabilize;
  firstUpdate_ = true;
  initialized_ = false;
  faultLogged_ = false;
  warmupElapsed_ = 0.0;
  t_ = 0.0;
  faultR_.setIdentity();
  contactPhase_.setConstant(0.5f);
  imuReceived_.store(false);
  cmdVelEverReceived_.store(false);
  lastCmd_ = cheetah::LocomotionCommand{};
  velIntegral_.setZero();
  contacts_.reset();
  footholds_->reset();
  haveLocOffset_ = false;
  locOffset_.setZero();
  lastLocStampNs_ = 0;
  planner_->stop();
  planner_->reset();
  planner_->start(
    [logger = get_node()->get_logger(),
    last = std::chrono::steady_clock::time_point{}](const std::string & text) mutable {
      const auto now = std::chrono::steady_clock::now();
      if (now - last < std::chrono::seconds(2)) {return;}
      last = now;
      RCLCPP_INFO(logger, "%s", text.c_str());
    });
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CheetahController::on_deactivate(const rclcpp_lifecycle::State &)
{
  planner_->stop();
  return controller_interface::CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------------- robot adaptation

cheetah::RobotParams CheetahController::buildRobotParams(const Vector12 & q0, const Eigen::Matrix3d & R0)
{
  // Single rigid body = whole robot (as in MIT): total mass, composite inertia about the global
  // CoM at the start pose. trunkOnly().inertia() is in the MPC frame: rotate it back to the trunk.
  Eigen::Vector3d comMit;
  Eigen::Matrix3d inertiaMit;
  RobotModel::compositeInertia(
    model_.mass(), kTrunkCom, kRf * model_.inertia() * kRf, legs_, q0, comMit, inertiaMit);
  comTrunk_ = kRf * comMit;

  cheetah::RobotParams r;
  r.mass = static_cast<float>(model_.totalMass());
  r.inertia = inertiaMit.cast<float>();
  double minFootZ = std::numeric_limits<double>::infinity();
  double fMax = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 4; ++i) {
    const Eigen::Vector3d q3 = q0.segment<3>(3 * i);
    abdOriginTrunk_[i] = legs_[i].chainPoints(q3)[0];
    Eigen::Matrix3d J;
    const Eigen::Vector3d foot = legs_[i].forwardKinematics(q3, &J);
    r.hipLocation[i] = (kRf * (abdOriginTrunk_[i] - comTrunk_)).cast<float>();
    r.nominalFoot[i] = (kRf * (foot - comTrunk_)).cast<float>();
    minFootZ = std::min(minFootZ, (R0 * (foot - comTrunk_)).z());
    // Max normal force per foot: first motor to saturate at the stance pose.
    const Eigen::Vector3d tauPerNewton = J.transpose() * (R0.transpose() * Eigen::Vector3d::UnitZ());
    fMax = std::min(fMax, kTauMax / tauPerNewton.cwiseAbs().maxCoeff());
  }
  // CoM height above the plane of the foot centers (MIT feet are at z = 0 in contact).
  r.bodyHeight = static_cast<float>(-minFootZ);
  r.fMax = static_cast<float>(fMax);
  r.swingHeight = static_cast<float>(swingHeight_);
  r.gaitSegments = kGaitSegments;
  const int stance = static_cast<int>(std::lround(gaitDuty_ * kGaitSegments));
  r.gaitOffsets = Vec4<int>(0, kGaitSegments / 2, kGaitSegments / 2, 0);
  r.gaitDurations = Vec4<int>::Constant(stance);
  RCLCPP_INFO(
    get_node()->get_logger(),
    "Robot (MIT) : masse %.2f kg, inertie diag (%.3f %.3f %.3f), hauteur CdM %.3f m, Fmax %.0f N/pied ; "
    "allure : cycle %.2f s, appui %d/%d pas, vol %.0f ms, hauteur de vol %.2f m, vitesse %.2f m/s",
    r.mass, r.inertia(0, 0), r.inertia(1, 1), r.inertia(2, 2), r.bodyHeight, r.fMax, gaitCycle_, stance,
    kGaitSegments, 1000.0 * gaitCycle_ * (kGaitSegments - stance) / kGaitSegments, swingHeight_,
    walkSpeed_.load());
  return r;
}

void CheetahController::readLegs(const Vector12 & q, const Vector12 & qd, cheetah::LegData legs[4]) const
{
  for (int i = 0; i < 4; ++i) {
    Eigen::Matrix3d J;
    const Eigen::Vector3d foot = legs_[i].forwardKinematics(q.segment<3>(3 * i), &J);
    const Eigen::Matrix3d Jmit = kRf * J;
    legs[i].p = (kRf * (foot - abdOriginTrunk_[i])).cast<float>();
    legs[i].J = Jmit.cast<float>();
    legs[i].v = (Jmit * qd.segment<3>(3 * i)).cast<float>();
  }
}

void CheetahController::readImu(
  const sensor_msgs::msg::Imu & imu, Quat<float> & q, Vec3<float> & gyro, Vec3<float> & acc) const
{
  const Eigen::Quaterniond qTrunk(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z);
  const Eigen::Quaterniond qMit(qTrunk.normalized().toRotationMatrix() * kRf);   // MIT body -> world
  q << static_cast<float>(qMit.w()), static_cast<float>(qMit.x()), static_cast<float>(qMit.y()),
    static_cast<float>(qMit.z());
  gyro = (kRf * Eigen::Vector3d(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z)).cast<float>();
  acc = (kRf * Eigen::Vector3d(
      imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z)).cast<float>();
}

CheetahController::Vector12 CheetahController::legTorques(
  const cheetah::LegData legs[4], const cheetah::LegCommand cmds[4], const Vector12 & q,
  const Vector12 & qd) const
{
  const bool osc = swingOsc_.load();
  const double w = swingOmega_.load();
  const double zeta = swingZeta_.load();
  Eigen::Matrix<double, FloatingBaseModel::kNv, FloatingBaseModel::kNv> M;
  if (osc) {M = dynamics_->massMatrix(q);}
  const Eigen::Vector3d gTrunk = faultR_.transpose() * Eigen::Vector3d(0.0, 0.0, -kG);
  Vector12 tau;
  for (int i = 0; i < 4; ++i) {
    const auto & c = cmds[i];
    Vec3<float> force = c.forceFeedForward;
    force += c.kpCartesian * (c.pDes - legs[i].p);
    force += c.kdCartesian * (c.vDes - legs[i].v);
    const Eigen::Vector3d qdLeg = qd.segment<3>(3 * i);
    if (osc && contactPhase_[i] <= 0.f) {
      // Swing leg, operational space (trunk frame): Lambda = (J M^-1 J^T)^-1.
      const Eigen::Matrix3d J = kRf * legs[i].J.cast<double>();
      const Eigen::Matrix3d Mleg = M.block<3, 3>(6 + 3 * i, 6 + 3 * i);
      const Eigen::Matrix3d Lambda = (J * Mleg.inverse() * J.transpose()).inverse();
      const Eigen::Vector3d e = kRf * (c.pDes - legs[i].p).cast<double>();
      const Eigen::Vector3d de = kRf * (c.vDes - legs[i].v).cast<double>();
      const Eigen::Vector3d a = kRf * c.aDes.cast<double>();
      tau.segment<3>(3 * i) = J.transpose() * (Lambda * (a + w * w * e + 2.0 * zeta * w * de)) +
        legs_[i].gravityCompensation(q.segment<3>(3 * i), gTrunk);
      continue;
    }
    tau.segment<3>(3 * i) = (legs[i].J.transpose() * force).cast<double>() - c.kdJoint.cast<double>() * qdLeg;
  }
  const double b = jointDampingComp_.load();
  const double fc = jointFrictionComp_.load();
  if (b > 0.0 || fc > 0.0) {
    tau += b * qd + fc * (qd.array() / 0.05).tanh().matrix();
  }
  return tau.cwiseMax(-kTauLimit).cwiseMin(kTauLimit);
}

void CheetahController::contactPointVelocities(
  const Vector12 & q, const Vector12 & qd, const Quat<float> & qMit, const Vec3<float> & gyroMit,
  cheetah::LegData legs[4]) const
{
  // Contact point = center + d, d = foot radius straight down (world), in the trunk frame.
  const Eigen::Quaterniond qm(qMit[0], qMit[1], qMit[2], qMit[3]);
  const Eigen::Matrix3d Rwt = qm.normalized().toRotationMatrix() * kRf;
  const Eigen::Vector3d d = Rwt.transpose() * Eigen::Vector3d(0.0, 0.0, -kFootRadius);
  const Eigen::Vector3d wBody = kRf * gyroMit.cast<double>();
  for (int i = 0; i < 4; ++i) {
    const Eigen::Vector3d qdLeg = qd.segment<3>(3 * i);
    const Eigen::Vector3d wFoot = legs_[i].angularJacobian(q.segment<3>(3 * i)) * qdLeg;
    const Eigen::Vector3d vRel = kRf * legs[i].v.cast<double>() + (wFoot + wBody).cross(d);
    legs[i].v = (kRf * vRel).cast<float>();
  }
}

BodyState CheetahController::trunkState() const
{
  const auto & se = estimator_->result();
  BodyState b;
  const Eigen::Matrix3d Rwb = se.rBody.transpose().cast<double>();
  b.R = Rwb * kRf;                                         // trunk -> world
  // Global CoM (MIT, above foot centers) -> trunk link CoM, world frame.
  const Eigen::Vector3d comWorld = se.position.cast<double>() + Eigen::Vector3d(0, 0, kFootRadius);
  b.p = comWorld + b.R * (kTrunkCom - comTrunk_);
  b.v = se.vWorld.cast<double>();
  b.w = se.omegaWorld.cast<double>();
  return b;
}

cheetah::LocomotionCommand CheetahController::command()
{
  const cheetah::LocomotionCommand target = commandTarget();
  const double aMax = cmdAccelLimit_.load();
  if (aMax <= 0.0) {
    lastCmd_ = target;
  } else {
    const float step = static_cast<float>(aMax * dt_);
    const auto slew = [step](float from, float to) {return from + std::clamp(to - from, -step, step);};
    lastCmd_.vx = slew(lastCmd_.vx, target.vx);
    lastCmd_.vy = slew(lastCmd_.vy, target.vy);
    lastCmd_.yawRate = target.yawRate;
  }
  const double ki = velIntegralGain_.load();
  const Eigen::Vector2d vCmd(lastCmd_.vx, lastCmd_.vy);
  if (ki <= 0.0 || vCmd.norm() < 1e-3) {
    velIntegral_.setZero();
    return lastCmd_;
  }
  // Integrate only once the command is steady (not during the acceleration ramp).
  if (vCmd.isApprox(Eigen::Vector2d(target.vx, target.vy), 1e-4)) {
    const auto & vBody = estimator_->result().vBody;
    velIntegral_ += ki * dt_ * (vCmd - Eigen::Vector2d(vBody[0], vBody[1]));
    velIntegral_ = velIntegral_.cwiseMax(-kVelIntegralMax).cwiseMin(kVelIntegralMax);
  }
  cheetah::LocomotionCommand out = lastCmd_;
  out.vx += static_cast<float>(velIntegral_.x());
  out.vy += static_cast<float>(velIntegral_.y());
  return out;
}

cheetah::LocomotionCommand CheetahController::commandTarget()
{
  cheetah::LocomotionCommand cmd;
  if (!cmdVelEverReceived_.load(std::memory_order_acquire)) {
    cmd.vx = static_cast<float>(walkSpeed_.load());   // no command received: walk straight
    return cmd;
  }
  const CmdVelSample * s = cmdVelBuffer_.readFromRT();
  const std::int64_t ageNs = std::chrono::steady_clock::now().time_since_epoch().count() - s->stampNs;
  if (ageNs < 0 || ageNs > kSensorTimeoutNs) {return cmd;}   // stale command: stop
  cmd.vx = static_cast<float>(s->vx);
  cmd.vy = static_cast<float>(s->vy);
  cmd.yawRate = static_cast<float>(s->wz);
  return cmd;
}

// ------------------------------------------------------------------------- control loop

controller_interface::return_type CheetahController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  Vector12 q, qd;
  for (int i = 0; i < kNumJoints; ++i) {
    q[i] = state_interfaces_[i].get_value();
    qd[i] = state_interfaces_[kNumJoints + i].get_value();
  }
  contacts_.read(time.nanoseconds());
  if (firstUpdate_) {
    stand_->setTarget(q);
    firstUpdate_ = false;
  }

  const auto holdStand = [&](const Eigen::Matrix3d & R) {
      const Vector12 tau = stand_->torques(q, qd, kAllFeet, R, true);
      writeTorques(tau);
      return tau;
    };

  const double dt = period.seconds();
  const bool imuOk = imuReceived_.load(std::memory_order_acquire);
  if (state_ == ControlState::kFault || dt <= 0.0 || dt > kMaxDt || !imuOk) {
    if (imuOk) {
      const auto & imu = *imuBuffer_.readFromRT();
      const Eigen::Quaterniond qi(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z);
      if (qi.coeffs().allFinite() && qi.norm() > 0.5) {faultR_ = qi.normalized().toRotationMatrix();}
    }
    const Vector12 tau = holdStand(faultR_);
    if (state_ == ControlState::kFault) {publish(time, tau);}
    return controller_interface::return_type::OK;
  }
  const sensor_msgs::msg::Imu & imu = *imuBuffer_.readFromRT();

  if (!initialized_) {
    warmupElapsed_ += dt;
    const Eigen::Quaterniond qi(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z);
    if (warmupElapsed_ < kWarmupDuration || !(qi.coeffs().allFinite() && qi.norm() > 0.5)) {
      holdStand(faultR_);
      return controller_interface::return_type::OK;
    }
    dt_ = dt;
    robot_ = buildRobotParams(q, qi.normalized().toRotationMatrix());
    const int iterations = std::max(1, static_cast<int>(std::lround(gaitCycle_ / kGaitSegments / dt_)));
    locomotion_ = std::make_unique<cheetah::ConvexMPCLocomotion>(static_cast<float>(dt_), iterations, robot_);
    cheetah::StateEstimator::Params ep;
    ep.dt = static_cast<float>(dt_);
    estimator_ = std::make_unique<cheetah::StateEstimator>(robot_, ep);
    cheetah::LegData legs[4];
    readLegs(q, qd, legs);
    Quat<float> qm;
    Vec3<float> gyro, acc;
    readImu(imu, qm, gyro, acc);
    estimator_->reset(qm, legs);
    initialized_ = true;
  }
  t_ += dt;

  // Same order as MIT RobotRunner: leg data, state estimation, controller.
  cheetah::LegData legs[4];
  readLegs(q, qd, legs);
  Quat<float> qm;
  Vec3<float> gyro, acc;
  readImu(imu, qm, gyro, acc);
  if (footRollingComp_.load()) {
    cheetah::LegData legsEst[4] = {legs[0], legs[1], legs[2], legs[3]};
    contactPointVelocities(q, qd, qm, gyro, legsEst);
    estimator_->run(qm, gyro, acc, legsEst, contactPhase_);
  } else {
    estimator_->run(qm, gyro, acc, legs, contactPhase_);
  }
  applyLocalization();
  faultR_ = trunkState().R;

  const auto & se = estimator_->result();
  if (std::abs(se.rpy[0]) >= kMaxRollPitch || std::abs(se.rpy[1]) >= kMaxRollPitch) {
    state_ = ControlState::kFault;
    if (!faultLogged_) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "watchdog declenche (orientation) : roulis %.3f rad, tangage %.3f rad -- maintien actif",
        se.rpy[0], se.rpy[1]);
      faultLogged_ = true;
    }
    publish(time, holdStand(faultR_));
    return controller_interface::return_type::OK;
  }

  Vector12 tau;
  if (state_ == ControlState::kStabilize) {
    // Hold the stand while the estimator converges.
    contactPhase_.setConstant(0.5f);
    tau = holdStand(faultR_);
    if (t_ >= kStabilizeDuration) {
      locomotion_->initialize();
      state_ = ControlState::kLocomotion;
    }
  } else {
    updatePerception(q);
    sentCmd_ = command();
    locomotion_->run(se, legs, sentCmd_, legCmds_, contactPhase_);
    for (int i = 0; i < 4; ++i) {
      auto & c = footholds_->commits[i];
      if (!c.valid) {continue;}
      publishers_.publishFoothold(time, c.pointWorld);
      if (c.hasRegion) {publishers_.publishFootholdRegion(time, c.region);}
      c.valid = false;
    }
    mpcStats_ = {locomotion_->lastSolveOk ? 1.0 : 0.0, locomotion_->lastSolveMs, 0.0, 0.0};
    tau = legTorques(legs, legCmds_, q, qd);
    writeTorques(tau);
    publishControl(time, tau, q, qd, legs);
  }
  publish(time, tau);
  return controller_interface::return_type::OK;
}

void CheetahController::applyLocalization()
{
  if (!useLocalization_.load() || !localizationReceived_.load(std::memory_order_acquire)) {return;}
  const LocalizationSample & s = *localizationBuffer_.readFromRT();
  const Vec3<float> kfPosition = estimator_->result().position;
  Quat<float> orientation = estimator_->result().orientation;   // IMU by default
  if (s.stampNs != lastLocStampNs_) {
    // New sample: trunk pose (world) -> MIT pose (global CoM, above foot centers), then offset.
    lastLocStampNs_ = s.stampNs;
    const Eigen::Matrix3d Rwt = s.trunkOrientation.normalized().toRotationMatrix();
    const Eigen::Vector3d com = s.trunkPosition + Rwt * comTrunk_ - Eigen::Vector3d(0, 0, kFootRadius);
    locOffset_ = com.cast<float>() - kfPosition;
    haveLocOffset_ = true;
    const Eigen::Quaterniond qMit(Rwt * kRf);
    orientation << static_cast<float>(qMit.w()), static_cast<float>(qMit.x()), static_cast<float>(qMit.y()),
      static_cast<float>(qMit.z());
  }
  if (haveLocOffset_) {estimator_->overridePose(kfPosition + locOffset_, orientation);}
}

void CheetahController::updatePerception(const Vector12 & q)
{
  const grid_map::GridMap * map = terrainAware_ ? planner_->readTerrainMap() : nullptr;
  if (map == nullptr) {
    locomotion_->setFootholdAdapter(nullptr);   // no perception: MIT nominal footholds, flat ground
    locomotion_->setTerrainPitch(0.f);
    return;
  }
  const BodyState body = trunkState();
  MitFootholdAdapter::Context ctx;
  ctx.map = map;
  ctx.results = planner_->readFootholdResults();
  ctx.mode = static_cast<FootholdMode>(footholdMode_.load());
  ctx.trunkR = body.R;
  ctx.trunkOrigin = body.p - body.R * kTrunkCom;
  ctx.velocity = body.v;
  ctx.q = q;
  footholds_->setContext(ctx);
  locomotion_->setFootholdAdapter(footholds_.get());
  // Ground height under the feet drives the estimator and the body height reference (the
  // ground under the body itself is hidden from the camera).
  const Vec4<float> & ground = footholds_->footGround();
  estimator_->setFootGroundHeights(ground);
  locomotion_->setGroundHeight(ground.mean());
  // Pitch reference = slope between front (legs 0, 1) and rear (legs 2, 3) footholds.
  const float front = 0.5f * (robot_.nominalFoot[0][0] + robot_.nominalFoot[1][0]);
  const float rear = 0.5f * (robot_.nominalFoot[2][0] + robot_.nominalFoot[3][0]);
  const float rise = 0.5f * (ground[0] + ground[1]) - 0.5f * (ground[2] + ground[3]);
  locomotion_->setTerrainPitch(-std::atan2(rise, front - rear));
}

void CheetahController::writeTorques(const Vector12 & tau)
{
  for (int i = 0; i < kNumJoints; ++i) {command_interfaces_[i].set_value(tau[i]);}
}

void CheetahController::publishControl(
  const rclcpp::Time & time, const Vector12 & tau, const Vector12 & q, const Vector12 & qd,
  const cheetah::LegData legs[4])
{
  if (++controlDiagTick_ < kControlDiagDecimation || !controlPub_ || !controlPub_->trylock()) {return;}
  controlDiagTick_ = 0;
  auto & d = controlPub_->msg_.data;
  std::size_t k = 0;
  const auto put = [&d, &k](double v) {d[k++] = v;};
  const auto put3 = [&put](const auto & v) {put(v[0]); put(v[1]); put(v[2]);};
  put(time.seconds());
  for (int j = 0; j < kNumJoints; ++j) {put(tau[j]);}
  for (int j = 0; j < kNumJoints; ++j) {put(q[j]);}
  for (int j = 0; j < kNumJoints; ++j) {put(qd[j]);}
  for (int i = 0; i < 4; ++i) {put3(legCmds_[i].pDes);}
  for (int i = 0; i < 4; ++i) {put3(legs[i].p);}
  for (int i = 0; i < 4; ++i) {put3(legCmds_[i].vDes);}
  for (int i = 0; i < 4; ++i) {put3(legs[i].v);}
  for (int i = 0; i < 4; ++i) {put(contactPhase_[i] > 0.f ? 0.0 : 1.0);}
  for (int i = 0; i < 4; ++i) {put3(locomotion_->Fr_des[i]);}
  const float * ref = locomotion_->reference();
  for (int j = 0; j < 12; ++j) {put(ref[j]);}
  const auto & se = estimator_->result();
  put3(se.rpy);
  put3(se.position);
  put3(se.omegaWorld);
  put3(se.vWorld);
  put(sentCmd_.vx);
  put(sentCmd_.vy);
  put(sentCmd_.yawRate);
  put(mpcStats_[0]);
  put(mpcStats_[1]);
  put(1000.0 * locomotion_->mpcPeriod());
  controlPub_->unlockAndPublish();
}

void CheetahController::publish(const rclcpp::Time & time, const Vector12 & tau)
{
  if (!initialized_) {return;}
  if (state_ == ControlState::kLocomotion && feetPub_ && feetPub_->trylock()) {
    // Foot setpoint (hip-relative, MIT body frame) -> foot center, world frame.
    const auto & se = estimator_->result();
    auto & data = feetPub_->msg_.data;
    data.resize(16);
    for (int i = 0; i < 4; ++i) {
      const Vec3<float> ref = se.position + se.rBody.transpose() * (robot_.hipLocation[i] + legCmds_[i].pDes);
      data[3 * i] = ref[0];
      data[3 * i + 1] = ref[1];
      data[3 * i + 2] = ref[2] + kFootRadius;
      data[12 + i] = legCmds_[i].kpCartesian(0, 0) > 0.f ? 1.0 : 0.0;
    }
    feetPub_->unlockAndPublish();
  }
  const BodyState body = trunkState();
  publishers_.publishOdom(time, body);
  ControllerDiagnostics d;
  for (int i = 0; i < 4; ++i) {
    d.plannedContact[i] = contactPhase_[i] > 0.f;
    d.estimatedContact[i] = d.plannedContact[i];
    d.commandedForceZ[i] = locomotion_ ? locomotion_->Fr_des[i][2] : 0.0;
  }
  d.rawContact = contacts_.rawContacts();
  d.tau = tau;
  d.tiltRad = std::acos(std::clamp(body.R(2, 2), -1.0, 1.0));
  d.heightErr = std::abs(estimator_->result().position.z() - robot_.bodyHeight);
  d.state = static_cast<int>(state_);
  d.mpcStats = mpcStats_;
  publishers_.publishDiagnostics(d);
}

}  // namespace quadru_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(quadru_control::CheetahController, controller_interface::ControllerInterface)
