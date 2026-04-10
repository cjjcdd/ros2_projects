LIDAR Data Processing & Visualization — Complete Roadmap


---

## 📋 Table of Contents

1. [What You Must Deliver](#1-what-you-must-deliver)
2. [The Two Comment Markers — Your Edit Zone](#2-the-two-comment-markers--your-edit-zone)
3. [C++ Crash Course — Only What You Need](#3-c-crash-course--only-what-you-need)
4. [ROS 2 Concepts You Must Know](#4-ros-2-concepts-you-must-know)
5. [The LaserScan Message — Decoded](#5-the-laserscan-message--decoded)
6. [The Marker Message — Decoded](#6-the-marker-message--decoded)
7. [The Full Algorithm — Step by Step](#7-the-full-algorithm--step-by-step)
8. [The Complete Solution Code (With Line-by-Line Comments)](#8-the-complete-solution-code-with-line-by-line-comments)
9. [The Full lidar_ray.cpp File in Context](#9-the-full-lidar_raycpp-file-in-context)
10. [How to Build and Run](#10-how-to-build-and-run)
11. [Verification Checklist](#11-verification-checklist)
12. [Quick Reference Links](#12-quick-reference-links)

---

## 1. What You Must Deliver

The Turtlebot4 robot spins its LIDAR 360°, taking **1440 distance measurements** per scan (one every 0.25°).

Your job — expressed in plain English:

> "At every scan, find the **single longest valid distance** in the 1440 readings, calculate **where that point is in the world (X, Y)**, and draw a **straight line** from the robot to that point in RViz."

The line must update **continuously** as the robot moves.

---


## 3. C++ Crash Course — Only What You Need

You only need five C++ concepts for this lab.

### 3.1 Variables and Types

```cpp
double max_distance = 0.0;   // A decimal number (64-bit float). Always use double for distances.
int    max_index    = 0;     // A whole number (integer). Used for array positions.
long unsigned int i = 0;     // A large positive integer. Used when iterating over big arrays.
```

**Key rule:** In C++, you must declare the type before every variable name, e.g. `double x = 5.3;`

### 3.2 The `->` Operator (Pointer Member Access)

In ROS 2 callbacks, messages arrive as **smart pointers** (think of them as a reference to the object, not the object itself). To read a field inside the message you use `->` instead of `.`:

```cpp
laser_msg->ranges        // accesses the 'ranges' array inside the laser_msg pointer
laser_msg->angle_min     // accesses the start angle
laser_msg->ranges.size() // calls the size() method on the ranges array
```

If you used `.` on a pointer you would get a compile error. Always use `->` for ROS 2 message arguments.

### 3.3 Vectors and Array Access

`ranges` is a `std::vector<float>` — basically a dynamic array.

```cpp
laser_msg->ranges[0]            // first element (index 0)
laser_msg->ranges[1439]         // last element (index 1439)
laser_msg->ranges.size()        // returns 1440 (the total count)
laser_msg->ranges[i]            // element at position i
```

Array indices start at **0** in C++. Element 0 is the first ray, element 1439 is the last.

### 3.4 The For Loop

```cpp
for (long unsigned int i = 0; i < laser_msg->ranges.size(); i++)
{
    // This block runs once for each value i = 0, 1, 2, ..., 1439
    // laser_msg->ranges[i] is the distance for ray number i
}
```

Breaking it down:
- `long unsigned int i = 0` — create counter `i`, start at 0
- `i < laser_msg->ranges.size()` — keep looping while `i` is less than 1440
- `i++` — add 1 to `i` after each iteration

### 3.5 Math Functions from `<cmath>`

These are already `#include`d in the file.

| Function | What it does | Example |
|---|---|---|
| `std::isfinite(x)` | Returns `true` if `x` is a normal number (not infinity or NaN) | `std::isfinite(3.14)` → true |
| `cos(angle_radians)` | Cosine of an angle in **radians** | `cos(0.0)` → 1.0 |
| `sin(angle_radians)` | Sine of an angle in **radians** | `sin(0.0)` → 0.0 |

> ⚠️ **Important:** When a LIDAR ray hits nothing (open space, glass, sky) it returns `inf` (infinity). You **must** filter these out or your coordinates will be `inf` too, breaking the visualization. Use `std::isfinite()` to check.

---

## 4. ROS 2 Concepts You Must Know

### 4.1 The Overall Flow

```
LIDAR Hardware/Simulation
         │
         │  publishes sensor_msgs/LaserScan on topic "/scan"
         ▼
  laserCb() callback in your node
         │
         │  you calculate the furthest point
         ▼
  push a geometry_msgs::Point to lidar_rays.points
         │
         │  a timer publishes the Marker every 500ms
         ▼
     RViz displays the line
```

### 4.2 What Is Already Done For You

The starter code (outside your edit zone) already:
- Subscribes to `/scan` and calls `laserCb()` on every scan
- Subscribes to `/turtlebot4/odom` and tracks the robot's position
- Converts the robot's quaternion orientation to a **yaw angle** (rotation around the Z axis)
- Adds a 90° offset (`offset_90_deg`) to correct for the LIDAR's physical mounting
- Stores the robot's world position in `turtlebot4_pose` (a `geometry_msgs::msg::Point` with `.x`, `.y`, `.z`)
- Stores the robot's heading in `turtlebot4_yaw` (a `double`, in radians)
- Creates a Marker, clears its points each scan, and pushes `turtlebot4_pose` as the **start point** of the line
- Declares a `geometry_msgs::msg::Point p` initialized to `turtlebot4_pose`; you set `p.x` and `p.y` then the code pushes it as the **end point**

**You only need to:** find the max distance, compute the angle, and fill in `p.x` and `p.y`.

### 4.3 Coordinate Systems

ROS 2 uses a **right-handed coordinate system**:
- X points **forward** (in front of the robot)
- Y points **left**
- The LIDAR scans from `angle_min` (negative) to `angle_max` (positive)

When you multiply by `cos` and `sin` of the global angle, you convert from polar (distance, angle) to Cartesian (x, y) in the world frame.

---

## 5. The LaserScan Message — Decoded

The full type is `sensor_msgs::msg::LaserScan`. Inside your callback it arrives as `laser_msg`.

| Field | Type | Meaning for this lab |
|---|---|---|
| `laser_msg->ranges` | `std::vector<float>` | The 1440 distance values in **meters** |
| `laser_msg->ranges.size()` | `size_t` (≈ `long unsigned int`) | Always 1440 for this robot |
| `laser_msg->angle_min` | `float` | Starting angle of the scan in **radians** (usually negative, e.g. -π) |
| `laser_msg->angle_max` | `float` | Ending angle (usually positive, e.g. +π) |
| `laser_msg->angle_increment` | `float` | Angle between consecutive rays = 0.25° in radians ≈ 0.004363 rad |
| `laser_msg->range_min` | `float` | Minimum measurable distance; values below this are noise |
| `laser_msg->range_max` | `float` | Maximum measurable distance; values above this are effectively infinity |

**Official docs:** https://docs.ros.org/en/jazzy/p/sensor_msgs/interfaces/msg/LaserScan.html

### Why Values Can Be `inf` or `NaN`

- If a ray hits nothing within `range_max`, the sensor returns `std::numeric_limits<float>::infinity()` (positive infinity).
- Some sensors return `NaN` (Not a Number) for invalid readings.
- `std::isfinite(x)` returns `false` for both `inf` and `NaN`, catching both cases.

---

## 6. The Marker Message — Decoded

The full type is `visualization_msgs::msg::Marker`. The Marker is already configured in the constructor. The part you interact with is `lidar_rays.points` — a `std::vector` of `geometry_msgs::msg::Point`.

A `LINE_STRIP` marker draws a polyline through all the points in order:
- `points[0]` = start of the line (set to `turtlebot4_pose` — the robot's position)
- `points[1]` = end of the line (this is `p` — **you set this**)

After your code block, the starter code does:
```cpp
this->lidar_rays.points.push_back(p);
```
So you just need to correctly assign `p.x` and `p.y` before that line.

**Official docs:** https://docs.ros.org/en/jazzy/p/visualization_msgs/interfaces/msg/Marker.html

---

## 7. The Full Algorithm — Step by Step

Here is the complete mental model before seeing any code.

### Step 1 — Find the Maximum Distance

Loop through all 1440 values. Track the biggest valid (finite) one and remember its index.

```
max_distance = 0
max_index = 0

for i from 0 to 1439:
    if ranges[i] > max_distance AND ranges[i] is a normal number:
        max_distance = ranges[i]
        max_index = i
```

### Step 2 — Calculate the Local Angle of That Ray

Each index `i` corresponds to a specific angle relative to the LIDAR:

```
angle = angle_min + (i × angle_increment)
```

For example, index 720 (the middle ray) would be approximately:
```
angle = -π + 720 × 0.004363 ≈ 0.0 radians (straight ahead of the sensor)
```

### Step 3 — Convert to Global (World) Angle

The LIDAR angle is relative to the sensor. The robot may be rotated in the world. Add the robot's yaw (already corrected with the 90° offset):

```
global_angle = turtlebot4_yaw + angle
```

### Step 4 — Convert Polar to Cartesian (X, Y)

This is standard 2D trigonometry. Given a point at distance `r` along direction `θ` from position `(x₀, y₀)`:

```
X = x₀ + r × cos(θ)
Y = y₀ + r × sin(θ)
```

In code:
```
p.x = turtlebot4_pose.x + max_distance × cos(global_angle)
p.y = turtlebot4_pose.y + max_distance × sin(global_angle)
```

The diagram below illustrates this:

```
         World Y ↑
                 │
                 │          ✦ furthest point (p.x, p.y)
                 │         /
                 │        /  ← max_distance
                 │       /
                 │  θ   /  ← global_angle
   ──────────────●──────────────► World X
             robot pos
           (turtlebot4_pose.x, .y)
```

---

## 8. The Complete Solution Code (With Line-by-Line Comments)

This is **exactly** what goes between the two comment markers. Every single line is explained.

```cpp
//KODUNUZU BU SATIRDAN ITIBAREN YAZINIZ !!!

// ── Step 1: Initialize tracking variables ──────────────────────────────────
// max_distance holds the longest valid distance found so far. Start at 0.
double max_distance = 0.0;

// max_index holds which ray (0–1439) produced that max distance.
double max_index = 0;


// ── Step 2: Scan all 1440 rays to find the maximum ─────────────────────────
// laser_msg->ranges.size() returns 1440. We loop from i=0 to i=1439.
// 'long unsigned int' is the correct type for the .size() method's return type.
for (long unsigned int i = 0; i < laser_msg->ranges.size(); i++)
{
    // Condition 1: Is this ray longer than the current maximum?
    // Condition 2: Is it a real number? (not infinity, not NaN)
    //   → std::isfinite() returns false for infinity and NaN, true for normal numbers.
    //   → Without this check, an 'inf' ray would become max_distance and break p.x/p.y.
    if ((laser_msg->ranges[i] > max_distance) && (std::isfinite(laser_msg->ranges[i])))
    {
        max_distance = laser_msg->ranges[i]; // save the new maximum distance
        max_index = i;                        // save which ray number it was
    }
}
// After this loop: max_distance = the furthest real distance (meters)
//                  max_index    = the index of that ray in the ranges array


// ── Step 3: Calculate the angle of the furthest ray ────────────────────────
// The LIDAR starts scanning at angle_min (a negative radian value) and steps
// by angle_increment (≈ 0.004363 rad = 0.25°) for each subsequent ray.
// So ray at index i points at: angle_min + i * angle_increment (radians)
double angle = laser_msg->angle_min + max_index * laser_msg->angle_increment;

// The above angle is relative to the LIDAR/robot's own forward direction.
// We must rotate it into the world frame by adding the robot's current yaw.
// turtlebot4_yaw is already computed above (with the 90° mounting offset applied).
double global_angle = turtlebot4_yaw + angle;

// Optional debug log — prints to terminal. Remove if too noisy.
RCLCPP_INFO(this->get_logger(), "Max Distance: %.2f meters at angle %.2f rad", max_distance, global_angle);


// ── Step 4: Convert polar → Cartesian coordinates ──────────────────────────
// We know the distance (r = max_distance) and direction (θ = global_angle).
// Standard 2D polar-to-Cartesian conversion, offset by robot's world position:
//   X = robot_X + r * cos(θ)
//   Y = robot_Y + r * sin(θ)
p.x = turtlebot4_pose.x + max_distance * cos(global_angle);
p.y = turtlebot4_pose.y + max_distance * sin(global_angle);
// p.z is inherited from turtlebot4_pose.z (initialized earlier) — leave it.

//BU SATIRDAN SONRA HERHANGI BIR KOD DEGISTIRMEYINIZ !!!
```

---

## 9. The Full `lidar_ray.cpp` File in Context

Here is the complete file so you can see exactly where your code slots in (your additions are inside the marked zone):

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class LIDAR_rays : public rclcpp::Node {
public:
    visualization_msgs::msg::Marker lidar_rays;
    nav_msgs::msg::Odometry current_odom;
    geometry_msgs::msg::Point turtlebot4_pose;

    LIDAR_rays() : Node("Trajectory") {
        // ── Marker setup (LINE_STRIP from robot to furthest point) ──
        lidar_rays.header.frame_id = "turtlebot4/lidar_rays";
        lidar_rays.ns = "lidar_rays";
        lidar_rays.action = visualization_msgs::msg::Marker::ADD;
        lidar_rays.pose.orientation.w = 1.0;
        lidar_rays.id = 0;
        lidar_rays.type = visualization_msgs::msg::Marker::LINE_STRIP;
        lidar_rays.scale.x = 0.1;          // line width in meters
        lidar_rays.color.r = 1.0;          // red component (0.0–1.0)
        lidar_rays.color.g = 1.0;          // green component
        lidar_rays.color.b = 1.0;          // blue component → white line
        lidar_rays.color.a = 1.0;          // alpha (opacity); 0 = invisible, 1 = solid

        current_odom.pose.pose.position.x = 0.0;
        current_odom.pose.pose.position.y = 0.0;

        // ── Odometry callback: updates robot pose every time odom is published ──
        auto odomCb = [this](nav_msgs::msg::Odometry::UniquePtr odom_msg) -> void {
            current_odom.pose = odom_msg->pose;
        };

        // ── Laser callback: runs every time the LIDAR completes a full scan ──
        auto laserCb = [this](sensor_msgs::msg::LaserScan::UniquePtr laser_msg) -> void {

            // Convert quaternion orientation to roll/pitch/yaw (Euler angles)
            geometry_msgs::msg::Quaternion geom_q = current_odom.pose.pose.orientation;
            tf2::Quaternion tf2_q;
            tf2::fromMsg(geom_q, tf2_q);
            double offset_90_deg = 1.5708;      // 90° in radians, corrects LIDAR mount angle
            double roll, pitch, turtlebot4_yaw;
            tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, turtlebot4_yaw);
            turtlebot4_yaw += offset_90_deg;    // ← corrected yaw, ready to use
            turtlebot4_pose = current_odom.pose.pose.position; // ← robot world position (x, y, z)

            // Prepare the Marker: clear old points, add the robot's position as point[0]
            this->lidar_rays.points.clear();
            this->lidar_rays.points.push_back(turtlebot4_pose);

            // p will become point[1] (the end of the line). Pre-initialized to robot pos.
            geometry_msgs::msg::Point p = turtlebot4_pose;

            // ════════════════════════════════════════════════════════════════════
            //KODUNUZU BU SATIRDAN ITIBAREN YAZINIZ !!!

            double max_distance = 0.0;
            double max_index = 0;

            for (long unsigned int i = 0; i < laser_msg->ranges.size(); i++)
            {
                if ((laser_msg->ranges[i] > max_distance) && (std::isfinite(laser_msg->ranges[i])))
                {
                    max_distance = laser_msg->ranges[i];
                    max_index = i;
                }
            }

            double angle = laser_msg->angle_min + max_index * laser_msg->angle_increment;
            double global_angle = turtlebot4_yaw + angle;
            RCLCPP_INFO(this->get_logger(), "Max Distance: %.2f meters at angle %.2f rad", max_distance, global_angle);
            p.x = turtlebot4_pose.x + max_distance * cos(global_angle);
            p.y = turtlebot4_pose.y + max_distance * sin(global_angle);

            //BU SATIRDAN SONRA HERHANGI BIR KOD DEGISTIRMEYINIZ !!!
            // ════════════════════════════════════════════════════════════════════

            // Add the computed end-point to the Marker (point[1])
            this->lidar_rays.points.push_back(p);
            this->lidar_rays.header.stamp = this->get_clock()->now();
        };

        // ── Timer: publishes the Marker to RViz every 500 ms ──
        rays_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("lidar_rays", 10);
        auto raysCb = [this]() -> void {
            this->rays_pub_->publish(this->lidar_rays);
            RCLCPP_INFO(this->get_logger(), "LIDAR rays are publising...");
        };
        timer_ = this->create_wall_timer(500ms, raysCb);

        // ── Subscriptions ──
        odom_sub_  = this->create_subscription<nav_msgs::msg::Odometry>("/turtlebot4/odom", 10, odomCb);
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, laserCb);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rays_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LIDAR_rays>());
    rclcpp::shutdown();
    return 0;
}
```

---

## 10. How to Build and Run

Follow these terminal commands in order. Every time you change `lidar_ray.cpp`, repeat from step 1.

```bash
# Step 1 — Navigate to your workspace root
cd ~/ros2_ws

# Step 2 — Build the project (recompiles only changed files)
colcon build

# Step 3 — Source the workspace so ROS 2 can find your new binary
# (run this in every new terminal you open)
source install/setup.bash

# Step 4 — Launch the full lab (Gazebo + RViz + your node + teleop)
ros2 launch lab1 lab1.launch
```

After the launch:
- A **new terminal window** opens for keyboard teleop — use it to drive the robot.
- **RViz** shows the robot and the white line to the furthest LIDAR point.
- Your terminal prints `Max Distance: X.XX meters at angle Y.YY rad` every scan.

### If You See a Compile Error

| Error message | What it means | Fix |
|---|---|---|
| `error: 'max_distance' was not declared` | Variable used before declared | Add `double max_distance = 0.0;` before the loop |
| `error: expected ';' before '}'` | Missing semicolon | Check the line above the `}` |
| `invalid operands to binary expression` | Type mismatch | Make sure `max_index` is `double` or cast it |
| `'cos' was not declared` | `<cmath>` not included | It is already included — check you didn't accidentally delete a `#include` |

---

## 11. Verification Checklist

Before calling the instructor, confirm all of these:

- [ ] `colcon build` finishes with **no errors** (warnings are okay)
- [ ] In RViz, a **white line** is visible starting at the robot
- [ ] When you **drive the robot**, the line changes direction/length dynamically
- [ ] The terminal prints `Max Distance:` values that look reasonable (e.g. 2–10 meters indoors)
- [ ] The line points **toward the most open direction** (longest gap in the environment)

### Common Problems and Fixes

**Problem:** Line stays static / doesn't move with robot.  
**Fix:** Make sure you are using `turtlebot4_pose` (updated every callback) and not a hardcoded position.

**Problem:** Line points to a weird direction.  
**Fix:** This is the known TF offset issue mentioned in the lab sheet. The 90° correction (`offset_90_deg`) handles most of it. Small remaining offset is acceptable.

**Problem:** `Max Distance:` prints `inf`.  
**Fix:** Your `std::isfinite()` check is missing or incorrect. Re-check the `if` condition inside the loop.

**Problem:** Line doesn't appear in RViz at all.  
**Fix:** Check that in RViz the `lidar_rays` topic is subscribed. Also verify `colcon build` completed without errors and you sourced `install/setup.bash`.

---

## 12. Quick Reference Links

| Resource | URL | When to use it |
|---|---|---|
| **LaserScan message fields** | https://docs.ros.org/en/jazzy/p/sensor_msgs/interfaces/msg/LaserScan.html | To check what fields exist inside `laser_msg` |
| **Marker message fields** | https://docs.ros.org/en/jazzy/p/visualization_msgs/interfaces/msg/Marker.html | To understand how `lidar_rays` is configured |
| **geometry_msgs/Point** | https://docs.ros.org/en/jazzy/p/geometry_msgs/interfaces/msg/Point.html | To see `.x`, `.y`, `.z` fields on `p` and `turtlebot4_pose` |
| **C++ std::vector** | https://en.cppreference.com/w/cpp/container/vector | Reference for `.size()`, `[]` indexing, `.push_back()` |
| **C++ `<cmath>` functions** | https://en.cppreference.com/w/cpp/header/cmath | Reference for `cos()`, `sin()`, `std::isfinite()` |
| **ROS 2 Jazzy docs** | https://docs.ros.org/en/jazzy/ | General ROS 2 documentation |

---

## Summary — The Minimal Mental Model

If you remember only one thing from this document:

```
1. Loop through all 1440 laser_msg->ranges values.
2. Track the biggest one that passes std::isfinite().
3. Compute:  angle        = angle_min + index * angle_increment
             global_angle = turtlebot4_yaw + angle
4. Compute:  p.x = turtlebot4_pose.x + max_distance * cos(global_angle)
             p.y = turtlebot4_pose.y + max_distance * sin(global_angle)
5. Build → Run → Show instructor.
```

Good luck! 🚀
