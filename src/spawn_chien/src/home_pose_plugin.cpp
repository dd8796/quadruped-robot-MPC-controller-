#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace spawn_chien
{

class HomePosePlugin final : public gazebo::ModelPlugin
{
  struct JointTarget
  {
    gazebo::physics::JointPtr joint;
    double position;
    double holdForce = 0.0;
  };

public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = std::move(model);
    world_ = model_->GetWorld();
    initialPose_ = model_->WorldPose();
    startTime_ = world_->SimTime();
    for (auto jointElement = sdf->GetElement("joint"); jointElement;
         jointElement = jointElement->GetNextElement("joint"))
    {
      const std::string name = jointElement->Get<std::string>("name");
      const double position = jointElement->Get<double>("position");
      const auto joint = model_->GetJoint(name);
      if (!joint) {
        gzerr << "HomePosePlugin: joint '" << name << "' is missing\n";
        continue;
      }
      if (!joint->SetPosition(0, position)) {
        gzerr << "HomePosePlugin: failed to set '" << name << "' to " << position << " rad\n";
        continue;
      }
      joint->SetVelocity(0, 0.0);
      targets_.push_back(JointTarget{joint, position, 0.0});
      gzdbg << "HomePosePlugin: set " << name << " = " << position << " rad\n";
    }
    // Handover signal: the locomotion controller publishes diagnostics once active. Torque
    // differences alone fail when the controller also holds a stand.
    rosNode_ = gazebo_ros::Node::Get(sdf);
    controllerSub_ = rosNode_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/mevius_mpc_controller/diag_watchdog", rclcpp::QoS(10),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr) {controllerActive_.store(true);});
    gzmsg << "HomePosePlugin: holding " << targets_.size()
          << " joints with a temporary PD stand\n";
    updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&HomePosePlugin::OnUpdate, this));
  }

private:
  void OnUpdate()
  {
    if (released_) {
      return;
    }

    const double elapsed = (world_->SimTime() - startTime_).Double();
    if (elapsed - lastDiagnosticTime_ >= 0.5 && elapsed <= 5.0) {
      lastDiagnosticTime_ = elapsed;
      const auto pose = model_->WorldPose();
      const auto rpy = pose.Rot().Euler();
      const auto & first = targets_.front();
      gzmsg << "HomePosePlugin: t=" << elapsed
            << " root=(" << pose.Pos().X() << "," << pose.Pos().Y() << "," << pose.Pos().Z()
            << ") rpy=(" << rpy.X() << "," << rpy.Y() << "," << rpy.Z()
            << ") q0=" << first.joint->Position(0)
            << " q0_target=" << first.position << " tau0=" << first.holdForce << "\n";
    }
    int commandedJoints = 0;
    for (const auto & target : targets_) {
      if (std::abs(target.joint->GetForce(0) - target.holdForce) > 2.0) {
        ++commandedJoints;
      }
    }

    // Release the robot only once ros2_control writes effort commands: at least 8/12 joints differ
    // from this plugin's stand for 100 consecutive physics updates.
    constexpr int kEffortJointsRequired = 8;
    constexpr int kHandoffUpdatesRequired = 100;
    constexpr double kMinimumHoldTime = 1.0;
    if (elapsed >= kMinimumHoldTime && commandedJoints >= kEffortJointsRequired) {
      ++activeEffortUpdates_;
    } else {
      activeEffortUpdates_ = 0;
    }
    const bool controllerActive = elapsed >= kMinimumHoldTime && controllerActive_.load();
    // No timeout release: a robot released without an active controller falls.
    if (controllerActive || activeEffortUpdates_ >= kHandoffUpdatesRequired) {
      released_ = true;
      if (controllerActive) {
        gzmsg << "HomePosePlugin: handing off after " << elapsed
              << " s (controleur de locomotion actif)\n";
      } else if (activeEffortUpdates_ >= kHandoffUpdatesRequired) {
        gzmsg << "HomePosePlugin: handing off after " << elapsed
              << " s (active effort joints=" << commandedJoints << ")\n";
      }
      return;
    }

    for (auto & target : targets_) {
      const double error = target.position - target.joint->Position(0);
      const double velocity = target.joint->GetVelocity(0);
      target.holdForce = std::clamp(80.0 * error - 3.0 * velocity, -20.0, 20.0);
      target.joint->SetForce(0, target.holdForce);
    }
    model_->SetWorldPose(initialPose_);
    model_->SetLinearVel(ignition::math::Vector3d::Zero);
    model_->SetAngularVel(ignition::math::Vector3d::Zero);
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  ignition::math::Pose3d initialPose_;
  gazebo::common::Time startTime_;
  gazebo::event::ConnectionPtr updateConnection_;
  std::vector<JointTarget> targets_;
  int activeEffortUpdates_ = 0;
  bool released_ = false;
  gazebo_ros::Node::SharedPtr rosNode_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr controllerSub_;
  std::atomic<bool> controllerActive_{false};
  double lastDiagnosticTime_ = -1.0;
};

GZ_REGISTER_MODEL_PLUGIN(HomePosePlugin)

}  // namespace spawn_chien
