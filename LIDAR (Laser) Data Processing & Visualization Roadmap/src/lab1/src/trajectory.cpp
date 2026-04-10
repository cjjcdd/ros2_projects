#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <chrono>
#include <math.h>

using namespace std::chrono_literals;

class Trajectory : public rclcpp::Node{
public:
	visualization_msgs::msg::Marker trajectory;
	nav_msgs::msg::Odometry prev_odom;
    Trajectory() : Node("Trajectory"){
		trajectory.header.frame_id = "turtlebot4/trajectory";
		trajectory.ns = "traj";
		trajectory.action = visualization_msgs::msg::Marker::ADD;
		trajectory.pose.orientation.w = 1.0;
		trajectory.id = 0;

		trajectory.type = visualization_msgs::msg::Marker::LINE_STRIP;
		trajectory.scale.x = 0.1;
		trajectory.color.r = 0.0;
		trajectory.color.g = 1.0;
		trajectory.color.b = 1.0;
		trajectory.color.a = 1.0;

		prev_odom.pose.pose.position.x = 0.0;
		prev_odom.pose.pose.position.y = 0.0;
		prev_odom.pose.pose.position.z = 0.0;
		prev_odom.pose.pose.orientation.x = 0.0;
		prev_odom.pose.pose.orientation.y = 0.0;
		prev_odom.pose.pose.orientation.z = 0.0;
        auto odomCb = [this](nav_msgs::msg::Odometry::UniquePtr msg1) -> void {
			double diff = sqrt(pow((msg1->pose.pose.position.x - prev_odom.pose.pose.position.x), 2) + pow((msg1->pose.pose.position.y - prev_odom.pose.pose.position.y), 2));
			if(diff > 0.2){
				prev_odom.pose = msg1->pose;
				this->trajectory.header.stamp = this->get_clock()->now();
				this->trajectory.points.push_back(msg1->pose.pose.position);
			}
        };
        traj_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/turtlebot4/traj", 10);
        auto trajCb = [this]() -> void {
			this->traj_pub_->publish(this->trajectory);
            RCLCPP_INFO(this->get_logger(), "Trajectory is publising...");
        };
        timer_ = this->create_wall_timer(500ms, trajCb);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/turtlebot4/odom", 10, odomCb);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub_;
};

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
  	rclcpp::spin(std::make_shared<Trajectory>());
  	rclcpp::shutdown();
  	return 0;
}