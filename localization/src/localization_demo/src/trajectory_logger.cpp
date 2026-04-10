// ============================================================================
// trajectory_logger.cpp
//
// NODE: trajectory_logger
// TOPICS IN:
//   /odom  (nav_msgs/msg/Odometry)
// TOPICS OUT:
//   /logged_path        (nav_msgs/msg/Path)           – full recorded trajectory
//   /start_marker       (visualization_msgs/Marker)   – green sphere at start
//   /current_marker     (visualization_msgs/Marker)   – blue sphere at current pos
//   /heading_arrow      (visualization_msgs/Marker)   – arrow showing heading
//   /distance_label     (visualization_msgs/Marker)   – text: distance traveled
//   /return_angle_label (visualization_msgs/Marker)   – text: bearing back to start
//
// PURPOSE:
//   Records the complete robot trajectory with analytics:
//   - Total distance traveled
//   - Distance from starting point (straight line)
//   - Bearing angle back to the starting point
//   - Heading direction arrow
//
//   Covers slides:
//   - Odometry output (position, velocity)
//   - How nav_msgs/Path is built incrementally
//   - Practical use of atan2() for bearing calculation
//   - Practical use of hypot() for Euclidean distance
// ============================================================================

#include <chrono>
#include <cmath>
#include <sstream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "localization_demo/localization_helpers.hpp"

using namespace std::chrono_literals;
using namespace localization_helpers;

// ============================================================================
class TrajectoryLogger : public rclcpp::Node
{
public:

  // ── State ──────────────────────────────────────────────────────────────────
  nav_msgs::msg::Path logged_path_;

  // Start position (set on first odom message)
  double start_x_     = 0.0;
  double start_y_     = 0.0;
  bool   started_     = false;

  // Current position
  double current_x_     = 0.0;
  double current_y_     = 0.0;
  double current_theta_ = 0.0;
  double current_vel_   = 0.0;

  // Accumulated odometry distance
  double total_dist_    = 0.0;
  double prev_x_        = 0.0;
  double prev_y_        = 0.0;

  // Minimum movement threshold before recording a new waypoint (meters)
  // Without this threshold the path vector would grow extremely large
  // due to the high odom publish rate (50 Hz).
  static constexpr double RECORD_THRESHOLD = 0.05;

  // ── Constructor ────────────────────────────────────────────────────────────
  TrajectoryLogger() : Node("trajectory_logger")
  {
    logged_path_.header.frame_id = "odom";

    // ─────────────────────────────────────────────────────────────────────
    // ODOMETRY CALLBACK
    // ─────────────────────────────────────────────────────────────────────
    auto odomCb = [this](nav_msgs::msg::Odometry::UniquePtr msg) -> void
    {
      // ── Read all fields we need ───────────────────────────────────────
      double x     = msg->pose.pose.position.x;
      double y     = msg->pose.pose.position.y;
      double theta = quaternionToYaw(msg->pose.pose.orientation);
      double vel   = msg->twist.twist.linear.x;   // forward speed

      // ── Set start position on first callback ──────────────────────────
      if (!started_) {
        start_x_ = x;
        start_y_ = y;
        prev_x_  = x;
        prev_y_  = y;
        started_ = true;
        RCLCPP_INFO(this->get_logger(),
          "Start position recorded: (%.3f, %.3f)", start_x_, start_y_);
      }

      // ── Update current state ──────────────────────────────────────────
      current_x_     = x;
      current_y_     = y;
      current_theta_ = theta;
      current_vel_   = vel;

      // ── Accumulate distance traveled ──────────────────────────────────
      // hypot(dx, dy) = sqrt(dx² + dy²) — the Euclidean distance formula
      double step = std::hypot(x - prev_x_, y - prev_y_);
      if (step > 0.001) {   // ignore noise-level movements
        total_dist_ += step;
        prev_x_ = x;
        prev_y_ = y;
      }

      // ── Record waypoint if robot moved enough ─────────────────────────
      // Check distance from last recorded point in the path
      double dist_from_last = 0.0;
      if (!logged_path_.poses.empty()) {
        const auto & last = logged_path_.poses.back().pose.position;
        dist_from_last = std::hypot(x - last.x, y - last.y);
      }

      if (dist_from_last >= RECORD_THRESHOLD || logged_path_.poses.empty()) {
        // Build a PoseStamped and append to path
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp    = msg->header.stamp;
        ps.header.frame_id = "odom";

        // Copy position fields individually (clearest for learning)
        ps.pose.position.x = x;
        ps.pose.position.y = y;
        ps.pose.position.z = 0.0;

        // Copy full quaternion orientation
        ps.pose.orientation = msg->pose.pose.orientation;

        // Push to the path vector
        logged_path_.poses.push_back(ps);

        // Update path header timestamp
        logged_path_.header.stamp = msg->header.stamp;

        // Publish the updated path
        path_pub_->publish(logged_path_);
      }

      // ── Compute analytics ─────────────────────────────────────────────

      // Straight-line distance from start to current position
      double dist_from_start = distance2D(start_x_, start_y_, x, y);

      // Bearing angle from current position back to start
      // atan2(dy, dx) gives the angle from (x,y) to target in world frame
      double dx_to_start = start_x_ - x;
      double dy_to_start = start_y_ - y;
      double bearing_to_start_rad = std::atan2(dy_to_start, dx_to_start);
      double bearing_to_start_deg = bearing_to_start_rad * 180.0 / M_PI;

      // ── Log to terminal ───────────────────────────────────────────────
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "[TRAJ] pos=(%.2f, %.2f) θ=%.1f° | vel=%.2f m/s | "
        "path_dist=%.2f m | dist_from_start=%.2f m | "
        "bearing_to_start=%.1f° | waypoints=%zu",
        x, y, theta * 180.0 / M_PI, vel,
        total_dist_, dist_from_start,
        bearing_to_start_deg,
        logged_path_.poses.size());
    };

    // ─────────────────────────────────────────────────────────────────────
    // TIMER CALLBACK (250 ms)
    // Publishes RViz markers for start, current, heading, and labels.
    // ─────────────────────────────────────────────────────────────────────
    auto timerCb = [this]() -> void
    {
      if (!started_) return;
      auto now = this->get_clock()->now();

      // ── Start position marker (GREEN sphere) ──────────────────────────
      visualization_msgs::msg::Marker start_m;
      start_m.header.frame_id = "odom";
      start_m.header.stamp    = now;
      start_m.ns = "start_pos"; start_m.id = 0;
      start_m.type   = visualization_msgs::msg::Marker::SPHERE;
      start_m.action = visualization_msgs::msg::Marker::ADD;
      start_m.pose.position.x = start_x_;
      start_m.pose.position.y = start_y_;
      start_m.pose.position.z = 0.1;
      start_m.pose.orientation.w = 1.0;
      start_m.scale.x = start_m.scale.y = start_m.scale.z = 0.35;
      start_m.color.r = 0.0f; start_m.color.g = 1.0f;
      start_m.color.b = 0.0f; start_m.color.a = 1.0f;
      start_pub_->publish(start_m);

      // ── Current position marker (BLUE sphere) ─────────────────────────
      visualization_msgs::msg::Marker cur_m = start_m;
      cur_m.ns = "current_pos"; cur_m.id = 1;
      cur_m.pose.position.x = current_x_;
      cur_m.pose.position.y = current_y_;
      cur_m.pose.position.z = 0.2;
      cur_m.pose.orientation = yawToQuaternion(current_theta_);
      cur_m.color.r = 0.0f; cur_m.color.g = 0.4f;
      cur_m.color.b = 1.0f; cur_m.color.a = 1.0f;
      current_pub_->publish(cur_m);

      // ── Heading arrow (WHITE) ─────────────────────────────────────────
      visualization_msgs::msg::Marker heading;
      heading.header.frame_id = "odom";
      heading.header.stamp    = now;
      heading.ns = "heading"; heading.id = 2;
      heading.type   = visualization_msgs::msg::Marker::ARROW;
      heading.action = visualization_msgs::msg::Marker::ADD;
      heading.pose.position.x = current_x_;
      heading.pose.position.y = current_y_;
      heading.pose.position.z = 0.2;
      heading.pose.orientation = yawToQuaternion(current_theta_);
      heading.scale.x = 0.6;   // shaft length
      heading.scale.y = 0.08;
      heading.scale.z = 0.12;
      heading.color.r = 1.0f; heading.color.g = 1.0f;
      heading.color.b = 1.0f; heading.color.a = 1.0f;
      heading_pub_->publish(heading);

      // ── Distance label (TEXT) ─────────────────────────────────────────
      double dist_from_start = distance2D(start_x_, start_y_,
                                          current_x_, current_y_);
      visualization_msgs::msg::Marker dist_label;
      dist_label.header.frame_id = "odom";
      dist_label.header.stamp    = now;
      dist_label.ns = "dist_label"; dist_label.id = 3;
      dist_label.type   = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      dist_label.action = visualization_msgs::msg::Marker::ADD;
      dist_label.pose.position.x = current_x_;
      dist_label.pose.position.y = current_y_;
      dist_label.pose.position.z = 0.7;
      dist_label.scale.z = 0.18;
      dist_label.color.r = 1.0f; dist_label.color.g = 1.0f;
      dist_label.color.b = 0.0f; dist_label.color.a = 1.0f;

      std::ostringstream ss;
      ss << std::fixed << std::setprecision(2);
      ss << "Path: " << total_dist_ << " m\n"
         << "From start: " << dist_from_start << " m\n"
         << "Speed: " << current_vel_ << " m/s";
      dist_label.text = ss.str();
      dist_label_pub_->publish(dist_label);

      // ── Line from current pos back to start ───────────────────────────
      visualization_msgs::msg::Marker return_line;
      return_line.header.frame_id = "odom";
      return_line.header.stamp    = now;
      return_line.ns = "return_line"; return_line.id = 4;
      return_line.type   = visualization_msgs::msg::Marker::LINE_STRIP;
      return_line.action = visualization_msgs::msg::Marker::ADD;
      return_line.scale.x = 0.03;
      return_line.color.r = 0.8f; return_line.color.g = 0.8f;
      return_line.color.b = 0.8f; return_line.color.a = 0.5f;

      geometry_msgs::msg::Point p1, p2;
      p1.x = current_x_; p1.y = current_y_; p1.z = 0.1;
      p2.x = start_x_;   p2.y = start_y_;   p2.z = 0.1;
      return_line.points.push_back(p1);
      return_line.points.push_back(p2);
      return_angle_pub_->publish(return_line);
    };

    // ── Publishers ────────────────────────────────────────────────────────
    path_pub_         = this->create_publisher<nav_msgs::msg::Path>("logged_path", 10);
    start_pub_        = this->create_publisher<visualization_msgs::msg::Marker>("start_marker", 10);
    current_pub_      = this->create_publisher<visualization_msgs::msg::Marker>("current_marker", 10);
    heading_pub_      = this->create_publisher<visualization_msgs::msg::Marker>("heading_arrow", 10);
    dist_label_pub_   = this->create_publisher<visualization_msgs::msg::Marker>("distance_label", 10);
    return_angle_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("return_line", 10);

    // ── Subscriber ────────────────────────────────────────────────────────
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, odomCb);

    // ── Timer ─────────────────────────────────────────────────────────────
    timer_ = this->create_wall_timer(250ms, timerCb);

    RCLCPP_INFO(this->get_logger(), "trajectory_logger started.");
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr             path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr start_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr current_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr heading_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr dist_label_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr return_angle_pub_;
  rclcpp::TimerBase::SharedPtr                                  timer_;
};

// ============================================================================
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryLogger>());
  rclcpp::shutdown();
  return 0;
}
