# Odometry & Localization — Complete C++ / ROS 2 Roadmap

> **Topic:** Odometry, Dead Reckoning, Localization (AMCL / EKF / SLAM)  
> **Language:** C++ with ROS 2 Jazzy  
> **Focus:** Reading, writing, and operating on every message type you will encounter in a localization lab — syntax first, theory second.

---

## 📋 Table of Contents

1. [The Big Picture — What Localization Code Does](#1-the-big-picture)
2. [The nav_msgs/Odometry Message — Fully Decoded](#2-the-nav_msgsodometry-message--fully-decoded)
3. [Reading Odometry in a Callback — Every Field Explained](#3-reading-odometry-in-a-callback--every-field-explained)
4. [Quaternion → Yaw Conversion (The Most Common Operation)](#4-quaternion--yaw-conversion)
5. [geometry_msgs Types You Must Know](#5-geometry_msgs-types-you-must-know)
6. [Dead Reckoning in C++ — Full Implementation](#6-dead-reckoning-in-c--full-implementation)
7. [Publishing a Pose / Marker from Odometry Data](#7-publishing-a-pose--marker-from-odometry-data)
8. [ROS 2 Terminal Commands for Odometry Debugging](#8-ros-2-terminal-commands-for-odometry-debugging)
9. [The nav_msgs/Path Message — Drawing a Trajectory](#9-the-nav_msgspath-message--drawing-a-trajectory)
10. [Complete Node Template — Localization Style](#10-complete-node-template--localization-style)
11. [C++ Data Type Cheat Sheet for ROS 2](#11-c-data-type-cheat-sheet-for-ros-2)
12. [Common Mistakes and Fixes](#12-common-mistakes-and-fixes)
13. [Quick Reference Links](#13-quick-reference-links)

---

## 1. The Big Picture

Every localization lab task boils down to this pipeline:

```
/odom topic (nav_msgs/Odometry)
        │
        │  subscribe → odomCb() callback
        ▼
  Read position (x, y, z)        ← geometry_msgs::msg::Point
  Read orientation (quaternion)  ← geometry_msgs::msg::Quaternion
  Convert quaternion → yaw       ← tf2 library
        │
        │  compute something (trajectory, distance, angle, dead-reckoning pose...)
        ▼
  Publish result:
    - nav_msgs/Path        → trajectory line in RViz
    - visualization_msgs/Marker → arrow, sphere, line
    - geometry_msgs/PoseStamped → single pose
    - std_msgs/Float64      → a single number (distance, angle...)
```

The `/odom` topic is your main data source. Everything else is derived from it.

---

## 2. The nav_msgs/Odometry Message — Fully Decoded

This is the most important message in this lab. Memorize its structure.

```
nav_msgs/msg/Odometry
├── std_msgs/Header header
│   ├── builtin_interfaces/Time stamp     ← timestamp
│   └── string frame_id                  ← usually "odom"
│
├── string child_frame_id                ← usually "base_link"
│
├── geometry_msgs/PoseWithCovariance pose
│   ├── geometry_msgs/Pose pose
│   │   ├── geometry_msgs/Point position
│   │   │   ├── float64 x    ← robot X in meters
│   │   │   ├── float64 y    ← robot Y in meters
│   │   │   └── float64 z    ← robot Z (0 for ground robots)
│   │   └── geometry_msgs/Quaternion orientation
│   │       ├── float64 x    ← quaternion X component
│   │       ├── float64 y    ← quaternion Y component
│   │       ├── float64 z    ← quaternion Z component
│   │       └── float64 w    ← quaternion W component
│   └── float64[36] covariance           ← 6x6 noise matrix (ignore for now)
│
└── geometry_msgs/TwistWithCovariance twist
    ├── geometry_msgs/Twist twist
    │   ├── geometry_msgs/Vector3 linear
    │   │   ├── float64 x    ← forward velocity (m/s)
    │   │   ├── float64 y    ← lateral velocity (m/s)
    │   │   └── float64 z    ← vertical velocity
    │   └── geometry_msgs/Vector3 angular
    │       ├── float64 x    ← roll rate
    │       ├── float64 y    ← pitch rate
    │       └── float64 z    ← yaw rate (turning speed, rad/s)
    └── float64[36] covariance
```

**In C++, accessing these fields always follows the same pattern: arrow (`->`) for pointers, dot (`.`) for values.**

```cpp
// msg is a UniquePtr: nav_msgs::msg::Odometry::UniquePtr msg

double x   = msg->pose.pose.position.x;         // robot X
double y   = msg->pose.pose.position.y;         // robot Y
double qx  = msg->pose.pose.orientation.x;      // quaternion x
double qw  = msg->pose.pose.orientation.w;      // quaternion w
double vx  = msg->twist.twist.linear.x;         // forward speed
double wz  = msg->twist.twist.angular.z;        // turning speed
```

Notice the **double `.pose.pose`** — the first `.pose` accesses `PoseWithCovariance`, the second `.pose` accesses the actual `Pose` inside it. This trips up almost everyone.

---

## 3. Reading Odometry in a Callback — Every Field Explained

```cpp
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

// In your class:
nav_msgs::msg::Odometry stored_odom;  // ← store the last received odom here

auto odomCb = [this](nav_msgs::msg::Odometry::UniquePtr msg) -> void
{
    // ── 1. Store the full message for later use ──────────────────────────
    stored_odom = *msg;   // dereference pointer to copy the message value

    // ── 2. Read position ─────────────────────────────────────────────────
    double x = msg->pose.pose.position.x;   // meters, world frame
    double y = msg->pose.pose.position.y;   // meters, world frame
    double z = msg->pose.pose.position.z;   // usually ~0 for ground robots

    // ── 3. Read orientation quaternion ───────────────────────────────────
    // A quaternion (x, y, z, w) encodes 3D rotation.
    // For a ground robot only the Z axis rotation matters → "yaw".
    geometry_msgs::msg::Quaternion geom_q = msg->pose.pose.orientation;

    // Convert geometry_msgs quaternion to tf2 quaternion (needed for getRPY)
    tf2::Quaternion tf2_q;
    tf2::fromMsg(geom_q, tf2_q);

    // Extract roll, pitch, yaw from the quaternion
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, yaw);
    // yaw = rotation around vertical axis, in RADIANS
    // 0 rad = facing +X, π/2 rad = facing +Y, -π/2 rad = facing -Y

    // ── 4. Read velocities ───────────────────────────────────────────────
    double linear_vel  = msg->twist.twist.linear.x;   // forward speed m/s
    double angular_vel = msg->twist.twist.angular.z;  // turning speed rad/s

    // ── 5. Read timestamp ────────────────────────────────────────────────
    // Useful for computing Δt between two messages
    double time_sec = msg->header.stamp.sec
                    + msg->header.stamp.nanosec * 1e-9;  // convert to seconds

    RCLCPP_INFO(this->get_logger(),
        "Pos: (%.3f, %.3f), Yaw: %.3f rad, Vel: %.3f m/s",
        x, y, yaw, linear_vel);
};

odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/turtlebot4/odom", 10, odomCb);
```

---

## 4. Quaternion → Yaw Conversion

This is **the single most used operation** in every localization lab. Here it is completely isolated so you can copy-paste it anywhere.

```cpp
// ── REQUIRED INCLUDES ──
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// ── FUNCTION: convert a geometry_msgs Quaternion to a yaw angle (radians) ──
double quaternionToYaw(const geometry_msgs::msg::Quaternion& q)
{
    tf2::Quaternion tf2_q;
    tf2::fromMsg(q, tf2_q);          // convert message type → tf2 type
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, yaw);
    return yaw;
}

// ── USAGE ──
double robot_yaw = quaternionToYaw(msg->pose.pose.orientation);
```

**What yaw means:**

```
     +Y (left)
      ↑
      │    robot facing +X → yaw = 0.0 rad
      │    robot facing +Y → yaw = +1.5708 rad  (π/2, turned left 90°)
      │    robot facing -X → yaw = ±π rad       (180°, turned around)
      │    robot facing -Y → yaw = -1.5708 rad  (-π/2, turned right 90°)
      └────────────────────────────────→ +X (forward)
```

**Degrees ↔ Radians reminder:**

```cpp
double deg_to_rad = angle_degrees * M_PI / 180.0;
double rad_to_deg = angle_radians * 180.0 / M_PI;

// Common values:
// 0°   = 0.0 rad
// 90°  = 1.5708 rad   (M_PI / 2)
// 180° = 3.1416 rad   (M_PI)
// 270° = 4.7124 rad   (3 * M_PI / 2), or equivalently -1.5708 rad
```

---

## 5. geometry_msgs Types You Must Know

### 5.1 Point vs Vector3 vs Pose vs PoseStamped

```cpp
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// ── Point: a location in 3D space ──
geometry_msgs::msg::Point p;
p.x = 1.0;
p.y = 2.5;
p.z = 0.0;

// ── Vector3: a direction or velocity (no position meaning) ──
geometry_msgs::msg::Vector3 v;
v.x = 0.5;   // forward velocity
v.z = 0.3;   // angular velocity

// ── Pose: position + orientation together ──
geometry_msgs::msg::Pose pose;
pose.position.x = 1.0;
pose.position.y = 2.0;
pose.position.z = 0.0;
pose.orientation.x = 0.0;
pose.orientation.y = 0.0;
pose.orientation.z = 0.0;
pose.orientation.w = 1.0;  // w=1 means no rotation (facing +X)

// ── PoseStamped: Pose + Header (frame_id + timestamp) ──
// Used in Path messages and RViz visualization
geometry_msgs::msg::PoseStamped ps;
ps.header.frame_id = "odom";
ps.header.stamp = this->get_clock()->now();
ps.pose.position.x = 1.0;
ps.pose.position.y = 2.0;
ps.pose.orientation.w = 1.0;
```

### 5.2 Building a PoseStamped from Odometry (copy-paste pattern)

This is something you'll do constantly — converting raw odometry into a stamped pose for path logging or publishing:

```cpp
geometry_msgs::msg::PoseStamped makePoseStamped(
    const nav_msgs::msg::Odometry& odom,
    rclcpp::Clock::SharedPtr clock)
{
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp    = clock->now();
    ps.header.frame_id = "odom";

    // Copy position
    ps.pose.position.x = odom.pose.pose.position.x;
    ps.pose.position.y = odom.pose.pose.position.y;
    ps.pose.position.z = odom.pose.pose.position.z;

    // Copy orientation (full quaternion)
    ps.pose.orientation = odom.pose.pose.orientation;

    return ps;
}
```

### 5.3 Building a Quaternion from a Yaw Angle

Sometimes you know the yaw and need to produce a quaternion (e.g., for setting marker orientation):

```cpp
// Convert a yaw angle (radians) → geometry_msgs Quaternion
geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
    tf2::Quaternion tf2_q;
    tf2_q.setRPY(0.0, 0.0, yaw);   // roll=0, pitch=0, yaw=angle
    tf2_q.normalize();
    return tf2::toMsg(tf2_q);
}

// Usage:
geometry_msgs::msg::Quaternion q = yawToQuaternion(1.5708); // 90°
```

---

## 6. Dead Reckoning in C++ — Full Implementation

Dead reckoning = estimating current pose from: **previous pose + velocity × time elapsed**.

This is the core algorithm your lab will likely ask you to implement or extend.

```cpp
// ── DEAD RECKONING STATE ──────────────────────────────────────────────────
// Store this as class member variables:
double dr_x     = 0.0;   // estimated X position
double dr_y     = 0.0;   // estimated Y position
double dr_theta = 0.0;   // estimated heading (yaw) in radians
double last_time_sec = -1.0;  // -1 = not yet initialized

// ── DEAD RECKONING UPDATE (call this inside the odom callback) ────────────
auto odomCb = [this](nav_msgs::msg::Odometry::UniquePtr msg) -> void
{
    // ── Step 1: Get current time in seconds ──
    double current_time_sec = msg->header.stamp.sec
                            + msg->header.stamp.nanosec * 1e-9;

    // ── Step 2: Initialize on first call ──
    if (last_time_sec < 0.0)
    {
        // Seed dead reckoning from the first odometry reading
        dr_x     = msg->pose.pose.position.x;
        dr_y     = msg->pose.pose.position.y;
        dr_theta = quaternionToYaw(msg->pose.pose.orientation);
        last_time_sec = current_time_sec;
        return;
    }

    // ── Step 3: Compute time delta ──
    double dt = current_time_sec - last_time_sec;
    last_time_sec = current_time_sec;

    if (dt <= 0.0) return;   // safety check — ignore duplicate timestamps

    // ── Step 4: Read velocities from the message ──
    double v  = msg->twist.twist.linear.x;   // forward speed (m/s)
    double w  = msg->twist.twist.angular.z;  // angular speed (rad/s)

    // ── Step 5: Update pose using forward kinematics ──
    // These are the standard differential-drive equations:
    //   new_theta = old_theta + angular_velocity × Δt
    //   new_x     = old_x + linear_velocity × cos(new_theta) × Δt
    //   new_y     = old_y + linear_velocity × sin(new_theta) × Δt
    dr_theta += w * dt;
    dr_x     += v * cos(dr_theta) * dt;
    dr_y     += v * sin(dr_theta) * dt;

    // ── Step 6: Normalize angle to [-π, π] ──
    // Without this, angle can grow unboundedly (e.g. 7.2 rad instead of 0.9 rad)
    while (dr_theta >  M_PI) dr_theta -= 2.0 * M_PI;
    while (dr_theta < -M_PI) dr_theta += 2.0 * M_PI;

    RCLCPP_INFO(this->get_logger(),
        "Dead Reckoning → x: %.3f  y: %.3f  θ: %.3f rad",
        dr_x, dr_y, dr_theta);
};
```

**Why dead reckoning drifts:** Every `dt` step accumulates a tiny error. Wheel slip, uneven ground, and sensor noise all make `v` and `w` slightly wrong. Over time these tiny errors add up (this is the "unbounded" error your lecture showed).

---

## 7. Publishing a Pose / Marker from Odometry Data

### 7.1 Publishing a Single Pose Arrow in RViz

```cpp
#include "geometry_msgs/msg/pose_stamped.hpp"

// In constructor:
pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "robot_pose", 10);

// In callback or timer:
geometry_msgs::msg::PoseStamped ps;
ps.header.stamp    = this->get_clock()->now();
ps.header.frame_id = "odom";

// Fill from odometry:
ps.pose.position  = stored_odom.pose.pose.position;
ps.pose.orientation = stored_odom.pose.pose.orientation;

pose_pub_->publish(ps);
```

In RViz: Add → PoseStamped, set topic to `robot_pose`.

### 7.2 Publishing a Sphere Marker at Robot Position

```cpp
#include "visualization_msgs/msg/marker.hpp"

visualization_msgs::msg::Marker marker;
marker.header.frame_id = "odom";
marker.header.stamp    = this->get_clock()->now();
marker.ns              = "robot_pos";
marker.id              = 0;
marker.type            = visualization_msgs::msg::Marker::SPHERE;
marker.action          = visualization_msgs::msg::Marker::ADD;

// Place at robot's current position
marker.pose.position.x = stored_odom.pose.pose.position.x;
marker.pose.position.y = stored_odom.pose.pose.position.y;
marker.pose.position.z = 0.0;
marker.pose.orientation.w = 1.0;

// Size
marker.scale.x = 0.2;  // diameter in meters
marker.scale.y = 0.2;
marker.scale.z = 0.2;

// Color (RGBA, 0.0–1.0)
marker.color.r = 0.0;
marker.color.g = 1.0;
marker.color.b = 0.0;
marker.color.a = 1.0;  // alpha must not be 0 or it is invisible!

marker_pub_->publish(marker);
```

### 7.3 Marker Types Quick Reference

| Constant | What it draws |
|---|---|
| `Marker::SPHERE` | A 3D ball at a point |
| `Marker::ARROW` | An arrow (uses pose orientation for direction) |
| `Marker::LINE_STRIP` | Polyline through all `.points` |
| `Marker::LINE_LIST` | Pairs of points → separate line segments |
| `Marker::CYLINDER` | Useful for showing a robot footprint |
| `Marker::TEXT_VIEW_FACING` | Floating text label |

For `LINE_STRIP`, you push points like this:

```cpp
marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
marker.scale.x = 0.05;  // line width in meters (scale.y and .z are ignored)

geometry_msgs::msg::Point p;
p.x = 1.0; p.y = 0.0; p.z = 0.0;
marker.points.push_back(p);  // each push_back adds one vertex
p.x = 2.0; p.y = 1.0;
marker.points.push_back(p);
```

---

## 8. ROS 2 Terminal Commands for Odometry Debugging

These are the commands from your lecture slides, explained so you can use them confidently during the lab.

```bash
# List all active topics
ros2 topic list

# See what is being published on /odom (press Ctrl+C to stop)
ros2 topic echo /odom

# See just ONE message then stop automatically
ros2 topic echo /odom --once

# Check the message type of a topic
ros2 topic type /odom
# Output: nav_msgs/msg/Odometry

# See publisher and subscriber counts
ros2 topic info /odom

# See the full field structure of a message type
ros2 interface show nav_msgs/msg/Odometry

# See fields of any other message type you encounter
ros2 interface show geometry_msgs/msg/PoseStamped
ros2 interface show visualization_msgs/msg/Marker
ros2 interface show sensor_msgs/msg/LaserScan

# Check how fast a topic is publishing (Hz)
ros2 topic hz /odom

# List all available message/service/action types
ros2 interface list

# See all running nodes
ros2 node list

# See what topics a specific node subscribes/publishes
ros2 node info /your_node_name
```

---

## 9. The nav_msgs/Path Message — Drawing a Trajectory

If the lab asks you to draw the robot's trajectory (the path it has traveled), you use `nav_msgs/Path`.

```cpp
#include "nav_msgs/msg/path.hpp"

// In your class:
nav_msgs::msg::Path trajectory;   // stores all visited poses
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

// In constructor:
trajectory.header.frame_id = "odom";
path_pub_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);

// In odomCb — append current pose to path each callback:
auto odomCb = [this](nav_msgs::msg::Odometry::UniquePtr msg) -> void
{
    // Build a PoseStamped from current odom
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp    = msg->header.stamp;
    ps.header.frame_id = "odom";
    ps.pose            = msg->pose.pose;   // copy the entire Pose in one line

    // Add to trajectory
    trajectory.poses.push_back(ps);

    // Update header timestamp
    trajectory.header.stamp = msg->header.stamp;

    // Publish the path
    path_pub_->publish(trajectory);
};
```

In RViz: Add → Path, set topic to `trajectory`. You will see the robot's trail drawn as a line.

**Resetting the path (useful if the task says "reset on button press or on start"):**

```cpp
trajectory.poses.clear();
```

---

## 10. Complete Node Template — Localization Style

This is a full, compilable starting point that combines everything above. Fill in the `// YOUR CODE HERE` sections for any specific task.

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class LocalizationNode : public rclcpp::Node {
public:

    // ── Stored state ──────────────────────────────────────────────────────
    nav_msgs::msg::Odometry current_odom;   // latest full odom message
    nav_msgs::msg::Path     trajectory;     // accumulated path

    // Dead reckoning state
    double dr_x     = 0.0;
    double dr_y     = 0.0;
    double dr_theta = 0.0;
    double last_time_sec = -1.0;

    // ── Constructor ───────────────────────────────────────────────────────
    LocalizationNode() : Node("LocalizationNode")
    {
        trajectory.header.frame_id = "odom";

        // ── Odometry callback ──
        auto odomCb = [this](nav_msgs::msg::Odometry::UniquePtr msg) -> void
        {
            // Always save the latest message
            current_odom = *msg;

            // ── Read position ──
            double x = msg->pose.pose.position.x;
            double y = msg->pose.pose.position.y;

            // ── Convert quaternion → yaw ──
            tf2::Quaternion tf2_q;
            tf2::fromMsg(msg->pose.pose.orientation, tf2_q);
            double roll, pitch, yaw;
            tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, yaw);

            // ── Read velocities ──
            double v = msg->twist.twist.linear.x;
            double w = msg->twist.twist.angular.z;

            // ── Append to path ──
            geometry_msgs::msg::PoseStamped ps;
            ps.header.stamp    = msg->header.stamp;
            ps.header.frame_id = "odom";
            ps.pose            = msg->pose.pose;
            trajectory.poses.push_back(ps);
            trajectory.header.stamp = msg->header.stamp;
            path_pub_->publish(trajectory);

            // ── Dead reckoning update ──
            double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
            if (last_time_sec < 0.0) {
                dr_x = x; dr_y = y; dr_theta = yaw;
                last_time_sec = t;
                return;
            }
            double dt = t - last_time_sec;
            last_time_sec = t;
            if (dt > 0.0) {
                dr_theta += w * dt;
                dr_x     += v * cos(dr_theta) * dt;
                dr_y     += v * sin(dr_theta) * dt;
                while (dr_theta >  M_PI) dr_theta -= 2.0 * M_PI;
                while (dr_theta < -M_PI) dr_theta += 2.0 * M_PI;
            }

            // ── YOUR CODE HERE ──
            // e.g. compute distance traveled, publish marker, etc.
        };

        // ── Timer callback (publish at fixed rate) ──
        auto timerCb = [this]() -> void
        {
            // Publish a marker at current dead-reckoning position
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "odom";
            m.header.stamp    = this->get_clock()->now();
            m.ns   = "dr_pos";
            m.id   = 0;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = dr_x;
            m.pose.position.y = dr_y;
            m.pose.position.z = 0.0;
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = m.scale.z = 0.25;
            m.color.r = 1.0; m.color.g = 0.4; m.color.b = 0.0; m.color.a = 1.0;
            marker_pub_->publish(m);

            RCLCPP_INFO(this->get_logger(),
                "Odom: (%.2f, %.2f) | DR: (%.2f, %.2f, %.2f rad)",
                current_odom.pose.pose.position.x,
                current_odom.pose.pose.position.y,
                dr_x, dr_y, dr_theta);
        };

        // ── Create publishers ──
        path_pub_   = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("dr_marker", 10);

        // ── Create subscribers ──
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/turtlebot4/odom", 10, odomCb);

        // ── Create timer (500ms publish rate) ──
        timer_ = this->create_wall_timer(500ms, timerCb);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr        path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
```

---

## 11. C++ Data Type Cheat Sheet for ROS 2

This is the section for your specific pain points: syntax, types, reading, writing, setting.

### 11.1 Variable Declarations

```cpp
// Primitives
double  x = 0.0;          // 64-bit decimal — always use for distances, angles
float   f = 0.0f;         // 32-bit decimal — ROS messages use float internally
int     i = 0;            // 32-bit integer
long    l = 0L;           // 64-bit integer
bool    b = false;        // true or false
size_t  s = 0;            // unsigned integer for array sizes (.size() returns this)

// IMPORTANT: ROS 2 message fields that look like numbers are float64 in the
// interface definition, which maps to C++ double. Always declare as double.
double msg_x = msg->pose.pose.position.x;   // ✅ correct
float  msg_x = msg->pose.pose.position.x;   // ⚠️ loses precision
```

### 11.2 The Arrow (`->`) vs Dot (`.`) Rule

```cpp
// Rule: Use -> when the left side is a POINTER (*) or UniquePtr/SharedPtr
//       Use .  when the left side is a plain VALUE (struct/class instance)

nav_msgs::msg::Odometry::UniquePtr msg;  // msg is a pointer
msg->pose              // ✅ use ->
msg->pose.pose         // ✅ use -> for msg, then . for the nested value
msg->pose.pose.position.x  // ✅ chain dots after the first ->

nav_msgs::msg::Odometry stored;  // stored is a value (not a pointer)
stored.pose            // ✅ use .
stored.pose.pose.position.x  // ✅ all dots

// Comparing stored (value) vs msg (pointer):
stored = *msg;         // dereference pointer with * to copy its value
stored.pose.pose.position.x == msg->pose.pose.position.x  // both read same field
```

### 11.3 std::vector Operations (for `.points`, `.poses`, `ranges`)

```cpp
std::vector<double> v = {1.0, 2.0, 3.0};

v.size()         // → 3 (number of elements)
v[0]             // → 1.0 (access by index, starts at 0)
v.at(0)          // → 1.0 (same but throws exception if out of bounds — safer)
v.push_back(4.0) // add 4.0 to end → {1.0, 2.0, 3.0, 4.0}
v.clear()        // remove all elements → {}
v.empty()        // → false if size > 0

// Iterating with index (what you did in Lab 1):
for (size_t i = 0; i < v.size(); i++) {
    double val = v[i];
}

// Iterating with range-for (cleaner syntax, same result):
for (double val : v) {
    // val is each element
}

// The path message's poses vector:
trajectory.poses.push_back(my_pose_stamped);  // add a pose
trajectory.poses.size()                        // how many poses stored
trajectory.poses.clear()                       // reset the path
trajectory.poses[0].pose.position.x           // access first pose's X
```

### 11.4 Math Operations

```cpp
#include <cmath>   // already included in most ROS2 nodes

// Trigonometry (input is RADIANS, not degrees)
cos(angle)     // cosine
sin(angle)     // sine
atan2(y, x)    // angle from origin to point (x,y), result in [-π, π]

// Useful atan2 usage: compute angle from robot to a target point
double angle_to_target = atan2(target_y - robot_y, target_x - robot_x);

// Distance between two points
double dx = x2 - x1;
double dy = y2 - y1;
double dist = sqrt(dx*dx + dy*dy);
// or equivalently: double dist = hypot(dx, dy);

// Absolute value
double abs_val = fabs(-3.5);   // → 3.5  (use fabs for doubles, not abs)

// Clamp a value between min and max
double clamped = std::max(-1.0, std::min(1.0, value));

// Constants
M_PI       // π = 3.14159...
M_PI_2     // π/2 = 1.5708...
```

### 11.5 Conditional and Comparison Operators

```cpp
// Basic comparisons
a == b   // equal
a != b   // not equal
a >  b   // greater than
a >= b   // greater than or equal
a <  b   // less than
a <= b   // less than or equal

// Logical operators
(a > 0) && (b > 0)   // AND: both must be true
(a > 0) || (b > 0)   // OR: at least one must be true
!(a > 0)             // NOT: inverts the condition

// Compound condition (like in Lab 1)
if ((laser_msg->ranges[i] > max_distance) && (std::isfinite(laser_msg->ranges[i])))

// Check if robot has moved more than 0.1 m since last recorded pose:
double dx = current_x - last_x;
double dy = current_y - last_y;
if (hypot(dx, dy) > 0.1) {
    // record new pose
    last_x = current_x;
    last_y = current_y;
}
```

### 11.6 Common ROS 2 Message Assignment Patterns

```cpp
// ── Setting header ──
msg.header.stamp    = this->get_clock()->now();  // current time
msg.header.frame_id = "odom";                    // coordinate frame name

// ── Copying a position from odom ──
my_point = current_odom.pose.pose.position;      // copy whole Point struct

// ── Copying a pose from odom ──
my_pose.position    = current_odom.pose.pose.position;
my_pose.orientation = current_odom.pose.pose.orientation;

// ── Setting a quaternion to "no rotation" (facing +X) ──
q.x = 0.0; q.y = 0.0; q.z = 0.0; q.w = 1.0;

// ── Resetting a vector of points ──
marker.points.clear();

// ── Setting marker color with transparency ──
marker.color.r = 1.0;
marker.color.g = 0.0;
marker.color.b = 0.0;
marker.color.a = 0.8;  // 0.0 = invisible, 1.0 = opaque
```

---

## 12. Common Mistakes and Fixes

| Mistake | Symptom | Fix |
|---|---|---|
| `msg.pose` instead of `msg->pose` | Compile error: `request for member 'pose' in 'msg', which is of pointer type` | Change `.` to `->` for pointer variables |
| `msg->pose.pose.x` | Compile error: `has no member named 'x'` | Missing one level: `msg->pose.pose.position.x` |
| Forgot `double` `pose.pose` duplication | Wrong value (reads orientation instead of position) | Remember: `PoseWithCovariance.pose.position.x` not `.pose.position.x` |
| Using `abs()` on double | Wrong result (truncates to int) | Use `fabs()` for doubles |
| Angle keeps growing past 2π | Robot rotates >360° and angle becomes 10, 20 rad | Add angle normalization loop |
| Path marker invisible in RViz | Nothing appears | `color.a` is 0 or forgot to set it — must be > 0 |
| `colcon build` error after adding `#include` | Package not found | Add the package to `CMakeLists.txt` `find_package()` and `target_link_libraries()` |
| Topic name wrong | No data in callback | Run `ros2 topic list` and `ros2 topic echo /topic` to confirm the exact name |
| `size_t` vs `int` comparison warning | Compiler warning about signed/unsigned | Use `size_t i` in for loops iterating over `.size()` |

---

## 13. Quick Reference Links

| Resource | URL | When to use |
|---|---|---|
| **nav_msgs/Odometry** | https://docs.ros.org/en/jazzy/p/nav_msgs/interfaces/msg/Odometry.html | Check every field in the odom message |
| **nav_msgs/Path** | https://docs.ros.org/en/jazzy/p/nav_msgs/interfaces/msg/Path.html | Drawing robot trajectory |
| **geometry_msgs/Pose** | https://docs.ros.org/en/jazzy/p/geometry_msgs/interfaces/msg/Pose.html | Understanding position + orientation types |
| **geometry_msgs/PoseStamped** | https://docs.ros.org/en/jazzy/p/geometry_msgs/interfaces/msg/PoseStamped.html | Single pose with frame + time |
| **visualization_msgs/Marker** | https://docs.ros.org/en/jazzy/p/visualization_msgs/interfaces/msg/Marker.html | All marker types and fields |
| **tf2 Quaternion** | https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html | Coordinate frame transforms |
| **C++ cmath** | https://en.cppreference.com/w/cpp/header/cmath | cos, sin, atan2, hypot, fabs |
| **C++ std::vector** | https://en.cppreference.com/w/cpp/container/vector | push_back, clear, size, [] |
| **ROS 2 rclcpp** | https://docs.ros.org/en/jazzy/p/rclcpp/ | Node, Publisher, Subscription, Timer |

---

## Summary — One-Page Mental Model for Any Localization Task

```
STEP 1: Subscribe to /turtlebot4/odom
        └─ save full message: current_odom = *msg;

STEP 2: Extract what you need
        position → msg->pose.pose.position.x / .y
        yaw      → tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, yaw)
        velocity → msg->twist.twist.linear.x / angular.z
        time     → msg->header.stamp.sec + nanosec * 1e-9

STEP 3: Compute (dead reckoning, distance, angle, trajectory...)
        dr_theta += w * dt
        dr_x     += v * cos(dr_theta) * dt
        dr_y     += v * sin(dr_theta) * dt

STEP 4: Publish result
        PoseStamped → ps.pose = msg->pose.pose; path_pub_->publish(path)
        Marker      → m.pose.position.x = dr_x; marker_pub_->publish(m)

STEP 5: Build and run
        cd ~/ros2_ws && colcon build && source install/setup.bash
        ros2 launch lab1 lab1.launch
```

Good luck! 🚀
