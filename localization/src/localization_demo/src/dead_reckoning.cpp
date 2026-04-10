// ============================================================================
// dead_reckoning.cpp
//
// NODE: dead_reckoning
// TOPICS IN:
//   /odom  (nav_msgs/msg/Odometry)  – true robot position (ground truth)
// TOPICS OUT:
//   /dr_path          (nav_msgs/msg/Path)   – dead-reckoning estimated trajectory
//   /true_path        (nav_msgs/msg/Path)   – ground truth trajectory
//   /dr_marker        (visualization_msgs/Marker) – DR sphere (orange)
//   /true_marker      (visualization_msgs/Marker) – True sphere (green)
//   /dr_error_marker  (visualization_msgs/Marker) – Error line (red)
//   /dr_error_text    (visualization_msgs/Marker) – Error text label
//
// PURPOSE:
//   Implements Dead Reckoning (Ölü Konum Tahmini) as taught in the slides:
//
//   ALGORITHM:
//     θ_new = θ_old + ω × Δt
//     x_new = x_old + v × cos(θ_new) × Δt
//     y_new = y_old + v × sin(θ_new) × Δt
//
//   Compares DR estimate vs true odometry to visualize accumulated drift.
//   The growing gap between orange (DR) and green (true) = drift error.
//
//   This covers slides:
//     - Dead Reckoning definition
//     - Why dead reckoning is insufficient (drift)
//     - Wheel slip sources of noise
// ============================================================================

#include <chrono>
#include <cmath>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "localization_demo/localization_helpers.hpp"

using namespace std::chrono_literals;
using namespace localization_helpers;

// ============================================================================
class DeadReckoning : public rclcpp::Node
{
public:

  // ── Dead Reckoning state ─────────────────────────────────────────────────
  // These variables accumulate the DR estimate over time.
  double dr_x_     = 0.0;   // estimated X position (meters)
  double dr_y_     = 0.0;   // estimated Y position (meters)
  double dr_theta_ = 0.0;   // estimated heading (radians)

  // Timestamp of the PREVIOUS callback (needed to compute Δt)
  double last_time_sec_ = -1.0;   // -1 = not yet initialized

  // ── Ground truth state ────────────────────────────────────────────────────
  double true_x_     = 0.0;
  double true_y_     = 0.0;
  double true_theta_ = 0.0;

  // ── Accumulated paths ─────────────────────────────────────────────────────
  nav_msgs::msg::Path dr_path_;
  nav_msgs::msg::Path true_path_;

  // ── Constructor ────────────────────────────────────────────────────────────
  DeadReckoning() : Node("dead_reckoning")
  {
    dr_path_.header.frame_id   = "odom";
    true_path_.header.frame_id = "odom";

    // ─────────────────────────────────────────────────────────────────────
    // ODOMETRY CALLBACK
    //
    // Called every time /odom publishes a new message.
    // We use the VELOCITIES (twist) to update the DR estimate,
    // and the POSITION (pose) as ground truth for comparison.
    // ─────────────────────────────────────────────────────────────────────
    auto odomCb = [this](nav_msgs::msg::Odometry::UniquePtr msg) -> void
    {
      // ── Step 1: Read current timestamp (in seconds) ───────────────────
      double current_time = odomToTimeSec(*msg);

      // ── Step 2: On the very first callback, seed the DR estimate ──────
      if (last_time_sec_ < 0.0) {
        // Start DR from where the robot currently is.
        dr_x_     = msg->pose.pose.position.x;
        dr_y_     = msg->pose.pose.position.y;
        dr_theta_ = quaternionToYaw(msg->pose.pose.orientation);
        last_time_sec_ = current_time;

        RCLCPP_INFO(this->get_logger(),
          "DR initialized at (%.3f, %.3f, %.3f rad)",
          dr_x_, dr_y_, dr_theta_);
        return;  // nothing more to do on first call
      }

      // ── Step 3: Compute time delta Δt ─────────────────────────────────
      // Δt = how many seconds have passed since the last callback.
      // For a 50 Hz odometry topic this is typically ~0.02 seconds.
      double dt = current_time - last_time_sec_;
      last_time_sec_ = current_time;  // update for next iteration

      // Safety: ignore zero or negative Δt (can happen with duplicate stamps)
      if (dt <= 0.0) return;

      // ── Step 4: Read velocities from twist ────────────────────────────
      // v  = linear forward velocity  (m/s)
      // w  = angular (yaw) velocity   (rad/s)
      // These come from wheel encoders inside the robot driver.
      double v = msg->twist.twist.linear.x;
      double w = msg->twist.twist.angular.z;

      // ── Step 5: DEAD RECKONING UPDATE EQUATIONS ───────────────────────
      //
      //   θ_new = θ_old + ω × Δt
      //   x_new = x_old + v × cos(θ_new) × Δt
      //   y_new = y_old + v × sin(θ_new) × Δt
      //
      // Why update theta FIRST?
      //   Because the robot turns AND moves simultaneously.
      //   Using the new heading gives a better approximation of the arc.
      dr_theta_ += w * dt;
      dr_theta_  = normalizeAngle(dr_theta_);  // keep in [-π, π]
      dr_x_     += v * std::cos(dr_theta_) * dt;
      dr_y_     += v * std::sin(dr_theta_) * dt;

      // ── Step 6: Read GROUND TRUTH from pose ───────────────────────────
      // This is what the odometry (encoder-based) says.
      // In a real scenario this would be from a GPS or motion capture.
      true_x_     = msg->pose.pose.position.x;
      true_y_     = msg->pose.pose.position.y;
      true_theta_ = quaternionToYaw(msg->pose.pose.orientation);

      // ── Step 7: Compute drift error ───────────────────────────────────
      double error = distance2D(dr_x_, dr_y_, true_x_, true_y_);

      // ── Step 8: Append to paths ───────────────────────────────────────
      // DR estimated path
      auto dr_ps = makePoseStamped(dr_x_, dr_y_, dr_theta_, "odom",
                                   this->get_clock());
      dr_path_.poses.push_back(dr_ps);
      dr_path_.header.stamp = msg->header.stamp;

      // True odometry path
      auto true_ps = poseStampedFromOdom(*msg);
      true_path_.poses.push_back(true_ps);
      true_path_.header.stamp = msg->header.stamp;

      // Publish both paths
      dr_path_pub_->publish(dr_path_);
      true_path_pub_->publish(true_path_);

      // ── Step 9: Log ───────────────────────────────────────────────────
      RCLCPP_INFO(this->get_logger(),
        "[DR] est=(%.3f, %.3f, %.2f°) | true=(%.3f, %.3f) | "
        "error=%.4f m | dt=%.4f s",
        dr_x_, dr_y_, dr_theta_ * 180.0 / M_PI,
        true_x_, true_y_, error, dt);
    };

    // ─────────────────────────────────────────────────────────────────────
    // TIMER CALLBACK (200 ms)
    // Publishes RViz markers showing DR vs truth and the error between them.
    // ─────────────────────────────────────────────────────────────────────
    auto timerCb = [this]() -> void
    {
      auto now = this->get_clock()->now();

      // ── DR sphere marker (ORANGE) ──────────────────────────────────────
      visualization_msgs::msg::Marker dr_marker;
      dr_marker.header.frame_id = "odom";
      dr_marker.header.stamp    = now;
      dr_marker.ns              = "dead_reckoning";
      dr_marker.id              = 0;
      dr_marker.type            = visualization_msgs::msg::Marker::SPHERE;
      dr_marker.action          = visualization_msgs::msg::Marker::ADD;
      dr_marker.pose.position.x = dr_x_;
      dr_marker.pose.position.y = dr_y_;
      dr_marker.pose.position.z = 0.2;
      dr_marker.pose.orientation = yawToQuaternion(dr_theta_);
      dr_marker.scale.x = dr_marker.scale.y = dr_marker.scale.z = 0.3;
      dr_marker.color.r = 1.0f;   // orange
      dr_marker.color.g = 0.5f;
      dr_marker.color.b = 0.0f;
      dr_marker.color.a = 1.0f;
      dr_marker_pub_->publish(dr_marker);

      // ── True position sphere (GREEN) ──────────────────────────────────
      visualization_msgs::msg::Marker true_marker = dr_marker;  // copy then modify
      true_marker.ns = "true_position";
      true_marker.id = 1;
      true_marker.pose.position.x = true_x_;
      true_marker.pose.position.y = true_y_;
      true_marker.pose.orientation = yawToQuaternion(true_theta_);
      true_marker.color.r = 0.0f;  // green
      true_marker.color.g = 1.0f;
      true_marker.color.b = 0.3f;
      true_marker_pub_->publish(true_marker);

      // ── Error line (RED) — connects DR estimate to true position ───────
      // This visually shows the drift.
      visualization_msgs::msg::Marker error_line;
      error_line.header.frame_id = "odom";
      error_line.header.stamp    = now;
      error_line.ns              = "drift_error";
      error_line.id              = 2;
      error_line.type            = visualization_msgs::msg::Marker::LINE_STRIP;
      error_line.action          = visualization_msgs::msg::Marker::ADD;
      error_line.scale.x         = 0.05;  // line width
      error_line.color.r = 1.0f;  // red
      error_line.color.g = 0.0f;
      error_line.color.b = 0.0f;
      error_line.color.a = 1.0f;

      // For LINE_STRIP: push two points to draw the connecting line
      geometry_msgs::msg::Point p1, p2;
      p1.x = dr_x_;    p1.y = dr_y_;    p1.z = 0.2;
      p2.x = true_x_;  p2.y = true_y_;  p2.z = 0.2;
      error_line.points.push_back(p1);
      error_line.points.push_back(p2);
      error_line_pub_->publish(error_line);

      // ── Error distance label (TEXT) ────────────────────────────────────
      double error = distance2D(dr_x_, dr_y_, true_x_, true_y_);
      visualization_msgs::msg::Marker text_marker;
      text_marker.header.frame_id = "odom";
      text_marker.header.stamp    = now;
      text_marker.ns              = "error_text";
      text_marker.id              = 3;
      text_marker.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action          = visualization_msgs::msg::Marker::ADD;
      text_marker.pose.position.x = (dr_x_ + true_x_) / 2.0;  // midpoint
      text_marker.pose.position.y = (dr_y_ + true_y_) / 2.0;
      text_marker.pose.position.z = 0.5;
      text_marker.scale.z         = 0.2;  // text height in meters
      text_marker.color.r = 1.0f;
      text_marker.color.g = 1.0f;
      text_marker.color.b = 0.0f;
      text_marker.color.a = 1.0f;

      // Format error into string: "Err: 0.123 m"
      std::ostringstream ss;
      ss << std::fixed;
      ss.precision(3);
      ss << "Drift: " << error << " m";
      text_marker.text = ss.str();
      error_text_pub_->publish(text_marker);
    };

    // ── Publishers ────────────────────────────────────────────────────────
    dr_path_pub_     = this->create_publisher<nav_msgs::msg::Path>("dr_path", 10);
    true_path_pub_   = this->create_publisher<nav_msgs::msg::Path>("true_path", 10);
    dr_marker_pub_   = this->create_publisher<visualization_msgs::msg::Marker>("dr_marker", 10);
    true_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("true_marker", 10);
    error_line_pub_  = this->create_publisher<visualization_msgs::msg::Marker>("dr_error_marker", 10);
    error_text_pub_  = this->create_publisher<visualization_msgs::msg::Marker>("dr_error_text", 10);

    // ── Subscriber ────────────────────────────────────────────────────────
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, odomCb);

    // ── Timer ─────────────────────────────────────────────────────────────
    timer_ = this->create_wall_timer(200ms, timerCb);

    RCLCPP_INFO(this->get_logger(),
      "dead_reckoning started.\n"
      "  ORANGE marker = DR estimate\n"
      "  GREEN  marker = True position\n"
      "  RED    line   = Drift error");
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr             dr_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr             true_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr dr_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr true_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr error_line_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr error_text_pub_;
  rclcpp::TimerBase::SharedPtr                                  timer_;
};

// ============================================================================
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeadReckoning>());
  rclcpp::shutdown();
  return 0;
}
