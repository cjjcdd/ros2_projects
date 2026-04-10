// ============================================================================
// localization_belief.cpp
//
// NODE: localization_belief
// TOPICS IN:
//   /odom  (nav_msgs/msg/Odometry)
// TOPICS OUT:
//   /belief_cloud     (visualization_msgs/Marker) – POINTS cloud of particles
//   /belief_mean      (visualization_msgs/Marker) – mean pose arrow
//   /belief_spread    (visualization_msgs/Marker) – LINE_LIST showing spread
//
// PURPOSE:
//   Demonstrates the Particle Filter / AMCL localization belief concept
//   from the slides (Particle Filters, AMCL sections).
//
//   CONCEPT:
//     Instead of saying "the robot IS at (x, y)", a particle filter says
//     "the robot MIGHT BE at any of these N points, with varying probability".
//     Each particle = one hypothesis about the robot's position.
//
//   SIMULATION APPROACH:
//     We don't have a full AMCL running, so we SIMULATE the belief cloud:
//     - Particles start scattered (high uncertainty = "kidnapped robot")
//     - As the robot moves and the odom updates, particles converge
//     - Each particle is perturbed by Gaussian noise around the true position
//     - Uncertainty (spread) decreases as more odom data arrives
//
//   This covers slides:
//     - Localization as estimation problem
//     - Particle Filters
//     - AMCL (Adaptive Monte Carlo Localization)
//     - Kidnapped Robot Problem
// ============================================================================

#include <chrono>
#include <cmath>
#include <random>
#include <vector>
#include <algorithm>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"

#include "localization_demo/localization_helpers.hpp"

using namespace std::chrono_literals;
using namespace localization_helpers;

// ── A single particle: one hypothesis about where the robot is ──────────────
struct Particle
{
  double x     = 0.0;
  double y     = 0.0;
  double theta = 0.0;
  double weight = 1.0;   // higher weight = more likely this is the true pose
};

// ============================================================================
class LocalizationBelief : public rclcpp::Node
{
public:

  // ── Particle filter state ─────────────────────────────────────────────────
  static constexpr int    NUM_PARTICLES  = 200;
  static constexpr double INIT_SPREAD    = 2.0;   // meters, initial scatter radius
  static constexpr double MIN_SPREAD     = 0.05;  // meters, converged uncertainty

  std::vector<Particle> particles_;

  // How much uncertainty we have right now (shrinks as robot moves)
  double current_spread_ = INIT_SPREAD;

  // Latest true position from odom
  double true_x_     = 0.0;
  double true_y_     = 0.0;
  double true_theta_ = 0.0;

  // Random number generator (for Gaussian noise)
  std::mt19937 rng_;

  // Odom callback counter (used to simulate convergence over time)
  int odom_count_ = 0;

  // Kidnapped robot flag
  bool kidnapped_ = false;
  int  kidnap_timer_ = 0;

  // ── Constructor ────────────────────────────────────────────────────────────
  LocalizationBelief() : Node("localization_belief"),
                         rng_(std::random_device{}())
  {
    // Initialize N particles scattered around the origin
    // (simulates "global localization" — we have NO idea where robot is)
    initializeParticles(0.0, 0.0, INIT_SPREAD);

    // ─────────────────────────────────────────────────────────────────────
    // ODOMETRY CALLBACK
    // Updates particle positions and simulates belief convergence.
    // ─────────────────────────────────────────────────────────────────────
    auto odomCb = [this](nav_msgs::msg::Odometry::UniquePtr msg) -> void
    {
      odom_count_++;

      // ── Get true position ─────────────────────────────────────────────
      true_x_     = msg->pose.pose.position.x;
      true_y_     = msg->pose.pose.position.y;
      true_theta_ = quaternionToYaw(msg->pose.pose.orientation);

      // ── Simulate kidnapped robot every 300 callbacks ───────────────────
      // This demonstrates the "Kaçırılan Robot Sorunu" from the slides.
      // The robot is teleported to a new position without knowing it.
      if (odom_count_ % 300 == 0 && odom_count_ > 0) {
        kidnapped_ = true;
        kidnap_timer_ = 50;  // reset over next 50 callbacks
        current_spread_ = INIT_SPREAD;  // uncertainty explodes again
        // Scatter particles across the whole map (simulates re-localization)
        initializeParticles(true_x_, true_y_, INIT_SPREAD);
        RCLCPP_WARN(this->get_logger(),
          "KIDNAPPED ROBOT EVENT! Particles re-scattered. Belief reset.");
      }

      // ── Simulate gradual convergence ──────────────────────────────────
      // As the robot moves and collects sensor data, the particle cloud
      // shrinks toward the true position. Here we simulate this linearly.
      if (kidnap_timer_ > 0) {
        kidnap_timer_--;
      }

      // Spread decreases exponentially toward MIN_SPREAD
      current_spread_ = MIN_SPREAD + (current_spread_ - MIN_SPREAD) * 0.97;

      // ── Update each particle ──────────────────────────────────────────
      // In a real particle filter, particles are:
      //   1. Propagated using the motion model
      //   2. Weighted by how well they match the sensor data
      //   3. Resampled proportional to their weight
      //
      // Here we simulate this by scattering particles around the true pose
      // with decreasing noise (simulates step 2 + 3 converging).
      std::normal_distribution<double> pos_noise(0.0, current_spread_);
      std::normal_distribution<double> ang_noise(0.0, current_spread_ * 0.5);

      for (auto & p : particles_) {
        // Scatter each particle around the true position with Gaussian noise
        p.x     = true_x_     + pos_noise(rng_);
        p.y     = true_y_     + pos_noise(rng_);
        p.theta = true_theta_ + ang_noise(rng_);
        p.theta = normalizeAngle(p.theta);

        // Weight: inversely proportional to distance from true pose
        double dist = distance2D(p.x, p.y, true_x_, true_y_);
        p.weight = std::exp(-dist * dist / (2.0 * current_spread_ * current_spread_));
      }
    };

    // ─────────────────────────────────────────────────────────────────────
    // TIMER CALLBACK (100 ms)
    // Publishes the particle cloud and belief statistics as RViz markers.
    // ─────────────────────────────────────────────────────────────────────
    auto timerCb = [this]() -> void
    {
      auto now = this->get_clock()->now();

      // ── POINTS marker: each particle is one point ──────────────────────
      // visualization_msgs::Marker::POINTS draws one dot per entry in .points
      visualization_msgs::msg::Marker cloud;
      cloud.header.frame_id = "odom";
      cloud.header.stamp    = now;
      cloud.ns              = "particles";
      cloud.id              = 0;
      cloud.type            = visualization_msgs::msg::Marker::POINTS;
      cloud.action          = visualization_msgs::msg::Marker::ADD;
      cloud.scale.x         = 0.07;  // point width
      cloud.scale.y         = 0.07;  // point height

      // We'll also need per-point colors (uses marker.colors vector)
      cloud.points.reserve(particles_.size());
      cloud.colors.reserve(particles_.size());

      for (const auto & p : particles_) {
        geometry_msgs::msg::Point pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = 0.05;
        cloud.points.push_back(pt);

        // Color each particle by its weight:
        //   low weight  → red (unlikely pose)
        //   high weight → green (likely pose)
        std_msgs::msg::ColorRGBA c;
        c.r = static_cast<float>(1.0 - p.weight);
        c.g = static_cast<float>(p.weight);
        c.b = 0.0f;
        c.a = 0.8f;
        cloud.colors.push_back(c);
      }
      cloud_pub_->publish(cloud);

      // ── ARROW marker: belief mean (best estimate) ──────────────────────
      // Compute weighted mean position
      double sum_w = 0.0, mean_x = 0.0, mean_y = 0.0;
      for (const auto & p : particles_) {
        mean_x += p.weight * p.x;
        mean_y += p.weight * p.y;
        sum_w  += p.weight;
      }
      if (sum_w > 0.0) { mean_x /= sum_w; mean_y /= sum_w; }

      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = "odom";
      arrow.header.stamp    = now;
      arrow.ns              = "belief_mean";
      arrow.id              = 1;
      arrow.type            = visualization_msgs::msg::Marker::ARROW;
      arrow.action          = visualization_msgs::msg::Marker::ADD;
      arrow.pose.position.x = mean_x;
      arrow.pose.position.y = mean_y;
      arrow.pose.position.z = 0.2;
      arrow.pose.orientation = yawToQuaternion(true_theta_);
      arrow.scale.x = 0.5;   // arrow shaft length
      arrow.scale.y = 0.08;  // arrow shaft diameter
      arrow.scale.z = 0.12;  // arrow head diameter
      arrow.color.r = 0.0f;
      arrow.color.g = 0.8f;
      arrow.color.b = 1.0f;
      arrow.color.a = 1.0f;
      mean_pub_->publish(arrow);

      // ── Spread circle: shows uncertainty radius ────────────────────────
      // We use LINE_STRIP to draw a circle showing current_spread_
      visualization_msgs::msg::Marker circle;
      circle.header.frame_id = "odom";
      circle.header.stamp    = now;
      circle.ns              = "uncertainty_circle";
      circle.id              = 2;
      circle.type            = visualization_msgs::msg::Marker::LINE_STRIP;
      circle.action          = visualization_msgs::msg::Marker::ADD;
      circle.scale.x         = 0.03;  // line width
      circle.color.r = 1.0f;
      circle.color.g = 1.0f;
      circle.color.b = 0.0f;
      circle.color.a = 0.7f;

      // Generate 36 points around a circle of radius = current_spread_
      const int CIRCLE_SEGS = 36;
      for (int i = 0; i <= CIRCLE_SEGS; ++i) {
        double angle = 2.0 * M_PI * i / CIRCLE_SEGS;
        geometry_msgs::msg::Point pt;
        pt.x = true_x_ + current_spread_ * std::cos(angle);
        pt.y = true_y_ + current_spread_ * std::sin(angle);
        pt.z = 0.05;
        circle.points.push_back(pt);
      }
      spread_pub_->publish(circle);

      // ── Terminal log ──────────────────────────────────────────────────
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[BELIEF] spread=%.3f m | mean=(%.3f, %.3f) | %s",
        current_spread_, mean_x, mean_y,
        kidnap_timer_ > 0 ? "KIDNAPPED!" : "converging...");
    };

    // ── Publishers ────────────────────────────────────────────────────────
    cloud_pub_  = this->create_publisher<visualization_msgs::msg::Marker>("belief_cloud", 10);
    mean_pub_   = this->create_publisher<visualization_msgs::msg::Marker>("belief_mean", 10);
    spread_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("belief_spread", 10);

    // ── Subscriber ────────────────────────────────────────────────────────
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, odomCb);

    // ── Timer ─────────────────────────────────────────────────────────────
    timer_ = this->create_wall_timer(100ms, timerCb);

    RCLCPP_INFO(this->get_logger(),
      "localization_belief started.\n"
      "  RED/GREEN dots  = particle cloud (belief)\n"
      "  CYAN arrow      = belief mean (best estimate)\n"
      "  YELLOW circle   = uncertainty radius\n"
      "  Every 300 odom msgs → kidnapped robot event!");
  }

private:

  // Initialize all particles scattered around (cx, cy) with given spread
  void initializeParticles(double cx, double cy, double spread)
  {
    std::uniform_real_distribution<double> dist_r(0.0, spread);
    std::uniform_real_distribution<double> dist_a(-M_PI, M_PI);

    particles_.resize(NUM_PARTICLES);
    for (auto & p : particles_) {
      double r     = dist_r(rng_);
      double angle = dist_a(rng_);
      p.x      = cx + r * std::cos(angle);
      p.y      = cy + r * std::sin(angle);
      p.theta  = dist_a(rng_);
      p.weight = 1.0 / NUM_PARTICLES;
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      odom_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mean_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr spread_pub_;
  rclcpp::TimerBase::SharedPtr                                  timer_;
};

// ============================================================================
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationBelief>());
  rclcpp::shutdown();
  return 0;
}
