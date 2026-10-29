// Robot-centric elevation map from the RGBD point cloud, fused per cell with a scalar Kalman
// filter (elevation + variance). Follows ETH elevation_mapping (Fankhauser et al. 2018) without its
// dependencies. The map moves with the robot (GridMap::move).
#include <cmath>
#include <limits>
#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include "terrain_mapping/terrain_processing.hpp"

class TerrainMappingNode : public rclcpp::Node
{
public:
  TerrainMappingNode()
  : Node("terrain_mapping_node"),
    tfBuffer_(this->get_clock()),
    tfListener_(tfBuffer_)
  {
    mapFrame_ = declare_parameter<std::string>("map_frame", "odom");
    baseFrame_ = declare_parameter<std::string>("base_frame", "trunk");
    resolution_ = declare_parameter<double>("resolution", 0.02);
    lengthX_ = declare_parameter<double>("length_x", 4.0);
    lengthY_ = declare_parameter<double>("length_y", 2.0);
    // Variance growth per second of observed cells. Hind feet land on cells seen seconds ago.
    processNoisePerS_ = declare_parameter<double>("process_noise_per_s", 1e-5);
    // ~(2 cm)^2: map resolution and simulated depth noise.
    measurementNoise_ = declare_parameter<double>("measurement_noise", 4e-4);
    // Camera far clip is 10 m; points near it are no-return pixels, not ground.
    maxValidRange_ = declare_parameter<double>("max_valid_range_m", 3.0);
    // Flying pixels can land metres below the ground and initialize a fresh cell with a low
    // variance that later measurements cannot correct: reject points far from the trunk height.
    maxPlausibleDeltaFromRobot_ = declare_parameter<double>("max_plausible_delta_from_robot_m", 0.5);
    // The trunk link origin sits near the hips, so standing ground is above it: the band is
    // asymmetric.
    plausibleAboveRobot_ = declare_parameter<double>("max_plausible_above_robot_m", 0.6);
    // Remove the legs and the trunk from the cloud, otherwise the front legs become obstacles.
    selfFilter_ = declare_parameter<bool>("self_filter", true);
    selfFilterRadius_ = declare_parameter<double>("self_filter_radius_m", 0.05);
    const auto footOffset = declare_parameter<std::vector<double>>(
      "foot_offset", {0.19206, -0.15791, -0.04000});   // foot center, legN_foot frame
    footOffset_ = Eigen::Vector3f(footOffset[0], footOffset[1], footOffset[2]);

    map_.setGeometry(grid_map::Length(lengthX_, lengthY_), resolution_, grid_map::Position(0, 0));
    map_.add("elevation", std::numeric_limits<float>::quiet_NaN());
    map_.add("variance", std::numeric_limits<float>::quiet_NaN());
    map_.setFrameId(mapFrame_);
    lastUpdateTime_ = this->get_clock()->now();

    cloudSub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/chien/camera/points", rclcpp::SensorDataQoS(),
      std::bind(&TerrainMappingNode::cloudCallback, this, std::placeholders::_1));
    mapPub_ = create_publisher<grid_map_msgs::msg::GridMap>("~/elevation_map", 1);
    // One point per valid cell (x, y, elevation_filtered), for RViz without grid_map_rviz_plugin.
    cloudPub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/elevation_cloud", 1);

    RCLCPP_INFO(
      get_logger(),
      "terrain_mapping_node demarre : map_frame=%s base_frame=%s res=%.3fm %gx%gm (centree robot)",
      mapFrame_.c_str(), baseFrame_.c_str(), resolution_, lengthX_, lengthY_);
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    // Recenter on the robot before writing; keep the last position if the TF is not available yet.
    const auto robotPosOpt = lookupXY(baseFrame_);
    if (robotPosOpt) {
      map_.move(*robotPosOpt);
    }
    const auto robotZOpt = lookupZ(baseFrame_);

    const rclcpp::Time now = this->get_clock()->now();
    const double dt = std::max(0.0, (now - lastUpdateTime_).seconds());
    lastUpdateTime_ = now;
    predictVariance(dt);
    // toMessage() copies the map timestamp, which grid_map leaves at 0 otherwise.
    map_.setTimestamp(now.nanoseconds());

    // No-return depth pixels are saturated at the far clip: reject them on depth in the sensor
    // frame, before the transform hides them.
    pcl::PointCloud<pcl::PointXYZ> rawCloud;
    pcl::fromROSMsg(*msg, rawCloud);
    pcl::PointCloud<pcl::PointXYZ> rangeOk;
    rangeOk.reserve(rawCloud.size());
    for (const auto & p : rawCloud.points) {
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
        p.z < maxValidRange_)
      {
        rangeOk.push_back(p);
      }
    }
    sensor_msgs::msg::PointCloud2 rangeOkMsg;
    pcl::toROSMsg(rangeOk, rangeOkMsg);
    rangeOkMsg.header = msg->header;

    sensor_msgs::msg::PointCloud2 transformed;
    try {
      const geometry_msgs::msg::TransformStamped tf = tfBuffer_.lookupTransform(
        mapFrame_, msg->header.frame_id, msg->header.stamp, tf2::durationFromSec(0.2));
      tf2::doTransform(rangeOkMsg, transformed, tf);
    } catch (const tf2::TransformException & ex) {
      // Common at startup: fall back to the latest transform.
      try {
        const geometry_msgs::msg::TransformStamped tf =
          tfBuffer_.lookupTransform(mapFrame_, msg->header.frame_id, tf2::TimePointZero);
        tf2::doTransform(rangeOkMsg, transformed, tf);
      } catch (const tf2::TransformException & ex2) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "transform %s->%s indisponible : %s",
          mapFrame_.c_str(), msg->header.frame_id.c_str(), ex2.what());
        publishMap();
        return;
      }
    }

    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    pcl::fromROSMsg(transformed, pclCloud);

    // VoxelGrid sizes its bounding box from all points: drop non-finite and off-map points first.
    pcl::PointCloud<pcl::PointXYZ> finiteCloud;
    finiteCloud.reserve(pclCloud.size());
    for (const auto & p : pclCloud.points) {
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
        map_.isInside(grid_map::Position(p.x, p.y)))
      {
        finiteCloud.push_back(p);
      }
    }

    pcl::PointCloud<pcl::PointXYZ> filtered;
    if (!finiteCloud.empty()) {
      pcl::VoxelGrid<pcl::PointXYZ> voxel;
      voxel.setInputCloud(finiteCloud.makeShared());
      voxel.setLeafSize(
        static_cast<float>(resolution_) / 2.f, static_cast<float>(resolution_) / 2.f,
        static_cast<float>(resolution_) / 2.f);
      voxel.filter(filtered);
    }

    const auto robotShapes = selfFilter_ ? computeRobotShapes() : std::nullopt;
    std::size_t selfRejected = 0;
    for (const auto & p : filtered.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}
      if (robotShapes && terrain_mapping::isRobotPoint(Eigen::Vector3f(p.x, p.y, p.z), *robotShapes)) {
        ++selfRejected;
        continue;
      }
      // Plausibility band around the trunk height.
      if (robotZOpt) {
        if (p.z < *robotZOpt - maxPlausibleDeltaFromRobot_ ||
          p.z > *robotZOpt + plausibleAboveRobot_)
        {
          continue;
        }
      }
      const grid_map::Position pos(p.x, p.y);
      if (!map_.isInside(pos)) {continue;}
      grid_map::Index idx;
      map_.getIndex(pos, idx);
      fuseMeasurement(idx, p.z);
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "filtre du robot : %zu points retires sur %zu (%s)",
      selfRejected, filtered.size(), robotShapes ? "TF ok" : "TF indisponible, inactif");
    publishMap();
  }

  // Kalman prediction on every observed cell.
  void predictVariance(double dt)
  {
    if (dt <= 0.0) {return;}
    const float growth = static_cast<float>(processNoisePerS_ * dt);
    for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
      float & v = map_.at("variance", *it);
      if (!std::isnan(v)) {v += growth;}
    }
  }

  // Kalman update; the first measurement of a cell initializes it.
  void fuseMeasurement(const grid_map::Index & idx, float zMeas)
  {
    float & e = map_.at("elevation", idx);
    float & v = map_.at("variance", idx);
    if (std::isnan(v)) {
      e = zMeas;
      v = static_cast<float>(measurementNoise_);
      return;
    }
    const float gain = v / (v + static_cast<float>(measurementNoise_));
    e = e + gain * (zMeas - e);
    v = (1.f - gain) * v;
  }

  // Robot shapes in the map frame; nullopt if a TF is missing.
  std::optional<terrain_mapping::RobotShapes> computeRobotShapes()
  {
    auto toIso = [](const geometry_msgs::msg::TransformStamped & t) {
        Eigen::Isometry3f iso = Eigen::Isometry3f::Identity();
        iso.translation() = Eigen::Vector3f(
          t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
        iso.linear() = Eigen::Quaternionf(
          t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y,
          t.transform.rotation.z).normalized().toRotationMatrix();
        return iso;
      };
    terrain_mapping::RobotShapes shapes;
    try {
      shapes.mapToTrunk = toIso(tfBuffer_.lookupTransform(mapFrame_, baseFrame_, tf2::TimePointZero)).inverse();
      for (int leg = 1; leg <= 4; ++leg) {
        const std::string n = "leg" + std::to_string(leg);
        const Eigen::Isometry3f thigh = toIso(tfBuffer_.lookupTransform(mapFrame_, n + "_lower", tf2::TimePointZero));
        const Eigen::Isometry3f shank = toIso(tfBuffer_.lookupTransform(mapFrame_, n + "_foot", tf2::TimePointZero));
        const float r = static_cast<float>(selfFilterRadius_);
        shapes.limbs.push_back({thigh.translation(), shank.translation(), r});   // hip -> knee
        shapes.limbs.push_back({shank.translation(), shank * footOffset_, r});   // knee -> foot
      }
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
    // Trunk collision box (URDF) + 3 cm margin.
    shapes.trunkBoxCenter = Eigen::Vector3f(-0.0546f, 0.0013f, 0.5183f);
    shapes.trunkBoxHalf = Eigen::Vector3f(0.292f, 0.158f, 0.087f) + Eigen::Vector3f::Constant(0.03f);
    return shapes;
  }

  std::optional<grid_map::Position> lookupXY(const std::string & frame)
  {
    try {
      const geometry_msgs::msg::TransformStamped tf =
        tfBuffer_.lookupTransform(mapFrame_, frame, tf2::TimePointZero);
      return grid_map::Position(tf.transform.translation.x, tf.transform.translation.y);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "transform %s->%s indisponible : %s",
        mapFrame_.c_str(), frame.c_str(), ex.what());
      return std::nullopt;
    }
  }

  std::optional<double> lookupZ(const std::string & frame)
  {
    try {
      const geometry_msgs::msg::TransformStamped tf =
        tfBuffer_.lookupTransform(mapFrame_, frame, tf2::TimePointZero);
      return tf.transform.translation.z;
    } catch (const tf2::TransformException &) {
      return std::nullopt;  // already logged by lookupXY()
    }
  }

  void publishMap()
  {
    // Publish a copy with the elevation_filtered layer; map_ itself stays unprocessed.
    grid_map::GridMap processed = map_;
    processed.convertToDefaultStartIndex();
    terrain_mapping::processTerrain(processed, processingParams_);

    pcl::PointCloud<pcl::PointXYZ> cells;
    for (grid_map::GridMapIterator it(processed); !it.isPastEnd(); ++it) {
      const float e = processed.at(terrain_mapping::kLayerFiltered, *it);
      if (std::isnan(e)) {continue;}
      grid_map::Position pos;
      processed.getPosition(*it, pos);
      const pcl::PointXYZ pt(static_cast<float>(pos.x()), static_cast<float>(pos.y()), e);
      cells.push_back(pt);
    }
    const rclcpp::Time stamp(static_cast<int64_t>(map_.getTimestamp()));
    sensor_msgs::msg::PointCloud2 cellsMsg;
    pcl::toROSMsg(cells, cellsMsg);
    cellsMsg.header.frame_id = mapFrame_;
    cellsMsg.header.stamp = stamp;
    cloudPub_->publish(cellsMsg);

    const auto mapMsg = grid_map::GridMapRosConverter::toMessage(processed);
    mapPub_->publish(*mapMsg);
  }


  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  grid_map::GridMap map_;
  rclcpp::Time lastUpdateTime_;

  std::string mapFrame_, baseFrame_;
  double resolution_, lengthX_, lengthY_;
  bool selfFilter_ = true;
  double selfFilterRadius_ = 0.05;
  Eigen::Vector3f footOffset_ = Eigen::Vector3f::Zero();
  double processNoisePerS_, measurementNoise_, maxValidRange_, maxPlausibleDeltaFromRobot_;
  double plausibleAboveRobot_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloudSub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr mapPub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub_;
  terrain_mapping::TerrainProcessingParams processingParams_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TerrainMappingNode>());
  rclcpp::shutdown();
  return 0;
}
