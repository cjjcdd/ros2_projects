// ============================================================================
// odometry_reader.cpp
//
// NODE: odometry_reader
// TOPIC IN:  /odom  (nav_msgs/msg/Odometry)
// TOPICS OUT:
//   /odom_path        (nav_msgs/msg/Path)          – robot trajectory
//   /odom_pose_marker (visualization_msgs/Marker)  – sphere at robot position
//   /odom_info        (std_msgs/msg/String)         – human-readable printout
//
// PURPOSE:
//   Demonstrates how to read EVERY field of the nav_msgs/Odometry message.
//   This directly covers the "YOL ÖLÇÜMÜ (ODOMETRİ)" slides:
//     - /odom topic structure
//     - position (x, y, z)
//     - orientation (quaternion → yaw)
//     - linear and angular velocity
//     - timestamp
// ============================================================================

#include <chrono>
#include <cmath>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/string.hpp"

// Our shared helpers (quaternion conversion, etc.)
#include "localization_demo/localization_helpers.hpp"

using namespace std::chrono_literals;
using namespace localization_helpers;

// ============================================================================
class OdometryReader : public rclcpp::Node
{
public:
  // ── Member variables ───────────────────────────────────────────────────────

  // Latest received odometry message (stored for timer callback use)
  nav_msgs::msg::Odometry current_odom_;

  // Accumulated trajectory (every pose the robot has visited)
  nav_msgs::msg::Path trajectory_;

  // Total distance traveled (meters)
  double total_distance_ = 0.0;

  // Previous position for distance calculation
  double prev_x_ = 0.0;
  double prev_y_ = 0.0;
  bool   first_odom_ = true;

  // ── Constructor ────────────────────────────────────────────────────────────
  OdometryReader() : Node("odometry_reader")
  {
    // Initialize the path's coordinate frame.
    // "odom" = the fixed frame that odometry is measured relative to.
    trajectory_.header.frame_id = "odom";

    // ─────────────────────────────────────────────────────────────────────
    // ODOMETRY CALLBACK
    //
    // This lambda runs every time a new message arrives on /odom.
    // It demonstrates reading EVERY field of nav_msgs/msg/Odometry.
    // ─────────────────────────────────────────────────────────────────────
    auto odomCb = [this](nav_msgs::msg::Odometry::UniquePtr msg) -> void
    {
      // ── RULE: use -> because msg is a UniquePtr (pointer) ──
      // ── RULE: use .  for nested value types ──

      // ── 1. POSITION ──────────────────────────────────────────────────
      // msg->pose is PoseWithCovariance
      // msg->pose.pose is the actual Pose inside it  (note DOUBLE .pose.pose)
      // msg->pose.pose.position is geometry_msgs/Point
      double x = msg->pose.pose.position.x;   // meters, world X
      double y = msg->pose.pose.position.y;   // meters, world Y
      double z = msg->pose.pose.position.z;   // meters, always ~0 for ground robots

      // ── 2. ORIENTATION (Quaternion → Yaw) ────────────────────────────
      // msg->pose.pose.orientation is geometry_msgs/Quaternion {x, y, z, w}
      // We convert to a single yaw angle using our helper.
      double yaw = quaternionToYaw(msg->pose.pose.orientation);
      double yaw_deg = yaw * 180.0 / M_PI;    // convert to degrees for display

      // ── 3. VELOCITIES ─────────────────────────────────────────────────
      // msg->twist is TwistWithCovariance
      // msg->twist.twist is the actual Twist
      double linear_x  = msg->twist.twist.linear.x;   // forward speed (m/s)
      double linear_y  = msg->twist.twist.linear.y;   // lateral speed (m/s)
      double angular_z = msg->twist.twist.angular.z;  // yaw rate (rad/s)

      // ── 4. TIMESTAMP ──────────────────────────────────────────────────
      // Split into seconds and nanoseconds parts.
      // Combine: total_seconds = sec + nanosec * 10^-9
      double time_sec = static_cast<double>(msg->header.stamp.sec)
                      + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

      // ── 5. FRAME ID ───────────────────────────────────────────────────
      std::string frame = msg->header.frame_id;  // usually "odom"

      // ── 6. DISTANCE TRAVELED ──────────────────────────────────────────
      if (first_odom_) {
        prev_x_ = x;
        prev_y_ = y;
        first_odom_ = false;
      } else {
        double step = distance2D(prev_x_, prev_y_, x, y);
        // Only accumulate if the robot actually moved (avoids counting noise)
        if (step > 0.001) {
          total_distance_ += step;
          prev_x_ = x;
          prev_y_ = y;
        }
      }

      // ── 7. STORE latest message (used by timer callback) ──────────────
      current_odom_ = *msg;   // dereference UniquePtr to copy the VALUE

      // ── 8. APPEND TO TRAJECTORY PATH ─────────────────────────────────
      // Build a PoseStamped (pose + frame + time) from the odometry message
      geometry_msgs::msg::PoseStamped ps = poseStampedFromOdom(current_odom_);
      trajectory_.poses.push_back(ps);         // add to vector of poses
      trajectory_.header.stamp = msg->header.stamp;  // update path timestamp
      path_pub_->publish(trajectory_);         // publish updated path

      // ── 9. TERMINAL LOG ───────────────────────────────────────────────
      RCLCPP_INFO(this->get_logger(),
        "[ODOM] frame=%s | pos=(%.3f, %.3f, %.3f) | yaw=%.2f° | "
        "lin_x=%.3f m/s | ang_z=%.3f rad/s | dist=%.2f m | t=%.2f s",
        frame.c_str(), x, y, z, yaw_deg,
        linear_x, angular_z, total_distance_, time_sec);

      // ── 10. PUBLISH HUMAN-READABLE STRING ─────────────────────────────
      std_msgs::msg::String info;
      std::ostringstream ss;
      ss << "Position: (" << std::fixed << x << ", " << y << ") m | "
         << "Yaw: " << yaw_deg << " deg | "
         << "Vel: " << linear_x << " m/s | "
         << "Distance: " << total_distance_ << " m";
      info.data = ss.str();
      info_pub_->publish(info);
    };

    // ─────────────────────────────────────────────────────────────────────
    // TIMER CALLBACK (500 ms)
    // Publishes a sphere marker at the robot's current position.
    // ─────────────────────────────────────────────────────────────────────
    auto timerCb = [this]() -> void
    {
      visualization_msgs::msg::Marker marker;

      // ── Header ────────────────────────────────────────────────────────
      marker.header.frame_id = "odom";
      marker.header.stamp    = this->get_clock()->now();

      // ── Identity ──────────────────────────────────────────────────────
      marker.ns     = "robot_position";   // namespace (group name)
      marker.id     = 0;                  // unique ID within the namespace
      marker.type   = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;  // ADD or MODIFY

      // ── Pose: where to place the marker ───────────────────────────────
      marker.pose.position.x  = current_odom_.pose.pose.position.x;
      marker.pose.position.y  = current_odom_.pose.pose.position.y;
      marker.pose.position.z  = 0.15;  // slightly above ground so it's visible
      marker.pose.orientation = current_odom_.pose.pose.orientation;

      // ── Scale: sphere diameter in meters ──────────────────────────────
      marker.scale.x = 0.3;
      marker.scale.y = 0.3;
      marker.scale.z = 0.3;

      // ── Color: RGBA, all values in range [0.0, 1.0] ───────────────────
      // IMPORTANT: alpha (a) must be > 0 or the marker is invisible
      marker.color.r = 0.2f;
      marker.color.g = 0.6f;
      marker.color.b = 1.0f;
      marker.color.a = 1.0f;

      marker_pub_->publish(marker);
    };

    // ── Create publishers ─────────────────────────────────────────────────
    path_pub_   = this->create_publisher<nav_msgs::msg::Path>(
                    "odom_path", 10);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
                    "odom_pose_marker", 10);

    info_pub_   = this->create_publisher<std_msgs::msg::String>(
                    "odom_info", 10);

    // ── Create subscriber ─────────────────────────────────────────────────
    // Topic name: "/odom"  (change to "/turtlebot4/odom" for real robot)
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                  "/odom", 10, odomCb);

    // ── Create timer ──────────────────────────────────────────────────────
    timer_ = this->create_wall_timer(500ms, timerCb);

    RCLCPP_INFO(this->get_logger(), "odometry_reader started. Listening on /odom");
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr            path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr          info_pub_;
  rclcpp::TimerBase::SharedPtr                                 timer_;
};

// ============================================================================
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryReader>());
  rclcpp::shutdown();
  return 0;
}
