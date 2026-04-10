#pragma once
// ============================================================================
// localization_demo/localization_helpers.hpp
//
// Shared utility functions used by every node in this package.
// Include this header in any .cpp file.
// ============================================================================

#include <cmath>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/rclcpp.hpp"

namespace localization_helpers
{

// ────────────────────────────────────────────────────────────────────────────
// quaternionToYaw()
//
// Convert a geometry_msgs Quaternion to a yaw angle in radians.
//
// A quaternion (x, y, z, w) encodes 3D rotation. For a ground robot we
// only care about rotation around the vertical (Z) axis, which is "yaw".
//
//   yaw = 0       → robot faces +X (East)
//   yaw = +π/2    → robot faces +Y (North / Left)
//   yaw = ±π      → robot faces -X (West)
//   yaw = -π/2    → robot faces -Y (South / Right)
// ────────────────────────────────────────────────────────────────────────────
inline double quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
{
  // Step 1: Convert geometry_msgs::Quaternion → tf2::Quaternion
  //         (different type, same math — needed to call getRPY)
  tf2::Quaternion tf2_q;
  tf2::fromMsg(q, tf2_q);

  // Step 2: Extract Roll / Pitch / Yaw from the rotation matrix
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, yaw);

  // For a flat-ground robot roll ≈ 0 and pitch ≈ 0.
  // We only return yaw.
  return yaw;
}

// ────────────────────────────────────────────────────────────────────────────
// yawToQuaternion()
//
// Build a geometry_msgs Quaternion from a yaw angle (radians).
// Useful when you want to set marker orientation from a known angle.
// ────────────────────────────────────────────────────────────────────────────
inline geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  tf2::Quaternion tf2_q;
  tf2_q.setRPY(0.0, 0.0, yaw);  // roll=0, pitch=0, yaw=angle
  tf2_q.normalize();
  return tf2::toMsg(tf2_q);
}

// ────────────────────────────────────────────────────────────────────────────
// odomToTimeSec()
//
// Convert an Odometry header timestamp to a single double (seconds).
// Useful for computing dt = current_time - last_time.
// ────────────────────────────────────────────────────────────────────────────
inline double odomToTimeSec(const nav_msgs::msg::Odometry & odom)
{
  return static_cast<double>(odom.header.stamp.sec)
       + static_cast<double>(odom.header.stamp.nanosec) * 1e-9;
}

// ────────────────────────────────────────────────────────────────────────────
// normalizeAngle()
//
// Keep an angle inside the range [-π, π].
// Without this, dead reckoning angles grow unboundedly: 7.2 rad, 100 rad...
// ────────────────────────────────────────────────────────────────────────────
inline double normalizeAngle(double angle)
{
  while (angle >  M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

// ────────────────────────────────────────────────────────────────────────────
// distance2D()
//
// Euclidean distance between two (x,y) points.
// ────────────────────────────────────────────────────────────────────────────
inline double distance2D(double x1, double y1, double x2, double y2)
{
  return std::hypot(x2 - x1, y2 - y1);
}

// ────────────────────────────────────────────────────────────────────────────
// makePoseStamped()
//
// Build a geometry_msgs::PoseStamped from raw (x, y, yaw) values.
// This is the type used in nav_msgs/Path and RViz pose displays.
// ────────────────────────────────────────────────────────────────────────────
inline geometry_msgs::msg::PoseStamped makePoseStamped(
  double x, double y, double yaw,
  const std::string & frame_id,
  rclcpp::Clock::SharedPtr clock)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header.stamp    = clock->now();
  ps.header.frame_id = frame_id;
  ps.pose.position.x = x;
  ps.pose.position.y = y;
  ps.pose.position.z = 0.0;
  ps.pose.orientation = yawToQuaternion(yaw);
  return ps;
}

// ────────────────────────────────────────────────────────────────────────────
// poseStampedFromOdom()
//
// Extract a PoseStamped directly from an Odometry message.
// Copies the full quaternion orientation (not just yaw).
// ────────────────────────────────────────────────────────────────────────────
inline geometry_msgs::msg::PoseStamped poseStampedFromOdom(
  const nav_msgs::msg::Odometry & odom)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header           = odom.header;        // copies stamp AND frame_id
  ps.pose.position    = odom.pose.pose.position;
  ps.pose.orientation = odom.pose.pose.orientation;
  return ps;
}

}  // namespace localization_helpers
