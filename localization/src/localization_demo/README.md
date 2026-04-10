# 🤖 BLM4830 Week 6 — Localization Demo
## Complete ROS 2 Jazzy C++ Project

> **4 nodes. Every localization concept from the slides. Full colcon build.**

---

## Project Overview

| Node | What it demonstrates | Key slides covered |
|---|---|---|
| `odometry_reader` | Reads every field of `nav_msgs/Odometry`, publishes path + marker | Odometri, /odom topic, ros2 topic commands |
| `dead_reckoning` | DR algorithm with drift vs ground truth in RViz | Ölü Konum Tahmini, drift, hata birikimi |
| `localization_belief` | Particle cloud, convergence, kidnapped robot event | Particle Filter, AMCL, Kaçırılan Robot |
| `trajectory_logger` | Path recording, total distance, bearing-to-start | Odometri çıktısı, nav_msgs/Path |

---

## Directory Structure

```
blm4830_localization/
└── src/
    └── localization_demo/
        ├── CMakeLists.txt
        ├── package.xml
        ├── include/
        │   └── localization_demo/
        │       └── localization_helpers.hpp   ← shared math utilities
        ├── src/
        │   ├── odometry_reader.cpp
        │   ├── dead_reckoning.cpp
        │   ├── localization_belief.cpp
        │   └── trajectory_logger.cpp
        ├── launch/
        │   └── localization_demo.launch.py
        └── rviz/
            └── localization_demo.rviz
```

---

## Step 1 — Prerequisites

You need ROS 2 Jazzy installed. Verify with:

```bash
ros2 --version
# Expected: ros2cli 0.x.x   (Jazzy)
```

Install required packages if missing:

```bash
sudo apt update

# Core ROS 2 Jazzy packages
sudo apt install -y \
  ros-jazzy-rclcpp \
  ros-jazzy-nav-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-sensor-msgs \
  ros-jazzy-visualization-msgs \
  ros-jazzy-std-msgs \
  ros-jazzy-tf2 \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-rviz2 \
  ros-jazzy-teleop-twist-keyboard

# Build tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep
```

---

## Step 2 — Create the Workspace

If you already have `~/ros2_ws`, skip the mkdir line and just copy the package in.

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Copy this entire project folder here.
# After copying, your structure should be:
# ~/ros2_ws/src/localization_demo/CMakeLists.txt
# ~/ros2_ws/src/localization_demo/package.xml
# ~/ros2_ws/src/localization_demo/src/*.cpp
# etc.
```

---

## Step 3 — Build

```bash
cd ~/ros2_ws

# Build only this package (faster than building everything)
colcon build --packages-select localization_demo

# If you want verbose output to see errors more clearly:
colcon build --packages-select localization_demo --event-handlers console_direct+
```

Expected output at the end:
```
Summary: 1 package finished [~Xs]
  1 package had stderr output: localization_demo   ← warnings are OK
```

If build **fails**, common fixes are in the Troubleshooting section below.

---

## Step 4 — Source the Workspace

**You must do this in EVERY new terminal you open.**

```bash
source ~/ros2_ws/install/setup.bash

# Add to ~/.bashrc so it runs automatically in every terminal:
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Step 5 — Run

### Option A: Launch all 4 nodes at once (recommended)

```bash
# Terminal 1 — launch the demo
ros2 launch localization_demo localization_demo.launch.py

# For real Turtlebot4:
ros2 launch localization_demo localization_demo.launch.py odom_topic:=/turtlebot4/odom
```

### Option B: Run individual nodes

```bash
# Terminal 1
ros2 run localization_demo odometry_reader

# Terminal 2
ros2 run localization_demo dead_reckoning

# Terminal 3
ros2 run localization_demo localization_belief

# Terminal 4
ros2 run localization_demo trajectory_logger
```

### Start RViz (separate terminal)

```bash
# Open RViz with the pre-configured layout
rviz2 -d ~/ros2_ws/src/localization_demo/rviz/localization_demo.rviz

# Or just open blank RViz and add topics manually:
rviz2
```

### Drive the robot (separate terminal)

```bash
# Keyboard teleop — press arrow keys to drive
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# If using Turtlebot4 with a different cmd_vel topic:
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/turtlebot4/cmd_vel
```

### Publish fake odometry for testing without a robot

If you have no robot available, publish fake odometry to test all nodes:

```bash
# Terminal — publish a fake /odom message at 10 Hz
ros2 topic pub /odom nav_msgs/msg/Odometry \
  "{header: {frame_id: 'odom'}, \
    pose: {pose: {position: {x: 1.0, y: 0.5, z: 0.0}, \
                  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, \
    twist: {twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, \
                    angular: {x: 0.0, y: 0.0, z: 0.1}}}}" \
  --rate 10
```

---

## What to See in RViz

Add these displays to RViz (or load the .rviz file):

| Display Type | Topic | Color | What it shows |
|---|---|---|---|
| Path | `/odom_path` | Blue | True odometry trajectory |
| Path | `/dr_path` | Orange | Dead reckoning estimate |
| Path | `/true_path` | Green | Ground truth (same as odom_path) |
| Path | `/logged_path` | Yellow | Trajectory logger path |
| Marker | `/odom_pose_marker` | Blue sphere | Robot position (odometry_reader) |
| Marker | `/dr_marker` | Orange sphere | DR estimated position |
| Marker | `/true_marker` | Green sphere | True position |
| Marker | `/dr_error_marker` | Red line | Drift gap between DR and truth |
| Marker | `/dr_error_text` | Yellow text | Drift in meters |
| Marker | `/belief_cloud` | Red/green dots | Particle filter belief cloud |
| Marker | `/belief_mean` | Cyan arrow | Best pose estimate |
| Marker | `/belief_spread` | Yellow circle | Uncertainty radius |
| Marker | `/start_marker` | Green sphere | Where robot started |
| Marker | `/current_marker` | Blue sphere | Current position |
| Marker | `/heading_arrow` | White arrow | Heading direction |
| Marker | `/distance_label` | Yellow text | Distance stats |

**Fixed Frame** must be set to `odom` in RViz Global Options.

---

## Debugging with ROS 2 Commands

```bash
# See all active topics
ros2 topic list

# Watch live odometry values
ros2 topic echo /odom

# Watch one message then stop
ros2 topic echo /odom --once

# Check topic message type
ros2 topic type /odom

# Check who is publishing/subscribing
ros2 topic info /odom

# Check publish rate (Hz)
ros2 topic hz /odom

# See full message field structure
ros2 interface show nav_msgs/msg/Odometry
ros2 interface show visualization_msgs/msg/Marker
ros2 interface show geometry_msgs/msg/PoseStamped

# See running nodes
ros2 node list

# See what a node publishes/subscribes
ros2 node info /odometry_reader
ros2 node info /dead_reckoning
```

---

## Code Architecture — How the Nodes Connect

```
/odom (nav_msgs/Odometry)
    │
    ├──► odometry_reader ──► /odom_path        (nav_msgs/Path)
    │                   ──► /odom_pose_marker  (Marker: sphere)
    │                   ──► /odom_info         (std_msgs/String)
    │
    ├──► dead_reckoning  ──► /dr_path          (nav_msgs/Path)   orange
    │                   ──► /true_path         (nav_msgs/Path)   green
    │                   ──► /dr_marker         (Marker: sphere)  orange
    │                   ──► /true_marker       (Marker: sphere)  green
    │                   ──► /dr_error_marker   (Marker: line)    red
    │                   ──► /dr_error_text     (Marker: text)
    │
    ├──► localization_belief ──► /belief_cloud  (Marker: points)
    │                       ──► /belief_mean   (Marker: arrow)
    │                       ──► /belief_spread (Marker: circle)
    │
    └──► trajectory_logger ──► /logged_path    (nav_msgs/Path)   yellow
                           ──► /start_marker   (Marker: sphere)  green
                           ──► /current_marker (Marker: sphere)  blue
                           ──► /heading_arrow  (Marker: arrow)   white
                           ──► /distance_label (Marker: text)
                           ──► /return_line    (Marker: line)
```

---

## Key C++ Patterns Used in This Project

### Reading odometry (the double .pose.pose pattern)

```cpp
// msg is nav_msgs::msg::Odometry::UniquePtr
// UniquePtr = pointer → use ->
// nested fields = value types → use .

double x   = msg->pose.pose.position.x;   // PoseWithCovariance.Pose.Point.x
double y   = msg->pose.pose.position.y;
double vx  = msg->twist.twist.linear.x;   // TwistWithCovariance.Twist.Vector3.x
double wz  = msg->twist.twist.angular.z;
```

### Quaternion → Yaw

```cpp
tf2::Quaternion tf2_q;
tf2::fromMsg(msg->pose.pose.orientation, tf2_q);
double roll, pitch, yaw;
tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, yaw);
// yaw is in radians. Multiply by 180/M_PI for degrees.
```

### Dead reckoning update

```cpp
double dt = current_time - last_time;
dr_theta += angular_vel * dt;
dr_theta  = normalizeAngle(dr_theta);   // keep in [-π, π]
dr_x     += linear_vel * cos(dr_theta) * dt;
dr_y     += linear_vel * sin(dr_theta) * dt;
```

### Building nav_msgs/Path

```cpp
nav_msgs::msg::Path path;
path.header.frame_id = "odom";

geometry_msgs::msg::PoseStamped ps;
ps.header.stamp    = msg->header.stamp;
ps.header.frame_id = "odom";
ps.pose            = msg->pose.pose;      // copy entire Pose in one line
path.poses.push_back(ps);                // append to vector
path_publisher->publish(path);
```

### Publisher and subscriber creation

```cpp
// Publisher
auto pub = this->create_publisher<nav_msgs::msg::Path>("topic_name", 10);
pub->publish(my_path);

// Subscriber (lambda callback)
auto sub = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    [this](nav_msgs::msg::Odometry::UniquePtr msg) -> void {
        // msg-> to access fields
    });

// Timer
auto timer = this->create_wall_timer(
    std::chrono::milliseconds(500),
    [this]() -> void {
        // runs every 500ms
    });
```

---

## Troubleshooting

### "Package not found" build error

```bash
# Make sure ROS 2 is sourced BEFORE building
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select localization_demo
```

### "tf2_geometry_msgs not found"

```bash
sudo apt install ros-jazzy-tf2-geometry-msgs
```

### Node starts but no output appears

```bash
# Check if /odom is being published
ros2 topic hz /odom
# If output is "average rate: 0.000" — no robot/simulator is running.
# Publish fake odom (see Step 5 above).
```

### RViz shows nothing

1. Set **Fixed Frame** to `odom` (Global Options → Fixed Frame)
2. Add each display manually: Add → By topic → select the topic
3. Make sure `color.a` (alpha) is not 0 in your marker code

### "colcon build" succeeds but "ros2 run" says executable not found

```bash
# You forgot to source after building
source ~/ros2_ws/install/setup.bash
```

### Particles not visible in RViz

The `belief_cloud` marker uses per-point colors (`marker.colors` vector). Make sure your RViz version supports this. If not, replace with a single color:

```cpp
cloud.color.r = 0.0f;
cloud.color.g = 1.0f;
cloud.color.b = 0.0f;
cloud.color.a = 0.8f;
// and remove the cloud.colors.push_back() calls
```

---

## Extending for the Actual Lab Task

When the lab task is announced, you'll modify **one of the existing nodes** or add a **new node** to the same package.

### To add a new node:

1. Create `src/my_new_node.cpp`
2. In `CMakeLists.txt`, add:
   ```cmake
   add_executable(my_new_node src/my_new_node.cpp)
   ament_target_dependencies(my_new_node ${DEPS})
   ```
   And inside the `install(TARGETS ...)` block:
   ```cmake
   my_new_node
   ```
3. Rebuild:
   ```bash
   cd ~/ros2_ws && colcon build --packages-select localization_demo
   source install/setup.bash
   ros2 run localization_demo my_new_node
   ```

### Template for a new node

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "localization_demo/localization_helpers.hpp"
#include <chrono>
using namespace std::chrono_literals;
using namespace localization_helpers;

class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node") {
        auto odomCb = [this](nav_msgs::msg::Odometry::UniquePtr msg) -> void {
            double x   = msg->pose.pose.position.x;
            double y   = msg->pose.pose.position.y;
            double yaw = quaternionToYaw(msg->pose.pose.orientation);
            // YOUR CODE HERE
        };
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, odomCb);
    }
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}
```
