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

class LIDAR_rays : public rclcpp::Node{
public:
	visualization_msgs::msg::Marker lidar_rays;
	nav_msgs::msg::Odometry current_odom;
	geometry_msgs::msg::Point turtlebot4_pose;
    LIDAR_rays() : Node("Trajectory"){
		lidar_rays.header.frame_id = "turtlebot4/lidar_rays";
		lidar_rays.ns = "lidar_rays";
		lidar_rays.action = visualization_msgs::msg::Marker::ADD;
		lidar_rays.pose.orientation.w = 1.0;
		lidar_rays.id = 0;

		lidar_rays.type = visualization_msgs::msg::Marker::LINE_STRIP;
		lidar_rays.scale.x = 0.1;
		lidar_rays.color.r = 1.0;
		lidar_rays.color.g = 1.0;
		lidar_rays.color.b = 1.0;
		lidar_rays.color.a = 1.0;

		current_odom.pose.pose.position.x = 0.0;
		current_odom.pose.pose.position.y = 0.0;

        auto odomCb = [this](nav_msgs::msg::Odometry::UniquePtr odom_msg) -> void {
			current_odom.pose = odom_msg->pose;
        };

		auto laserCb = [this](sensor_msgs::msg::LaserScan::UniquePtr laser_msg) -> void {

			geometry_msgs::msg::Quaternion geom_q = current_odom.pose.pose.orientation;
			tf2::Quaternion tf2_q;
			tf2::fromMsg(geom_q, tf2_q);
			double offset_90_deg = 1.5708;
			double roll, pitch, turtlebot4_yaw;
			tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, turtlebot4_yaw);
			turtlebot4_yaw += offset_90_deg;
			//RCLCPP_INFO(this->get_logger(), "Yaw: %.3f rad", turtlebot4_yaw);
			turtlebot4_pose = current_odom.pose.pose.position;

			this->lidar_rays.points.clear();
			this->lidar_rays.points.push_back(turtlebot4_pose);

			geometry_msgs::msg::Point p = turtlebot4_pose;
			// Ust satirlarda yapilan islem sonucunda robotun turtlebot4_pose pozunda turtlebot4_yaw acisinda oldugu bulunmustur.
			// Bu bilgileri kullanmalisiniz. Sizler tekrar hesaplamaya ugrasmayiniz !!!

			//KODUNUZU BU SATIRDAN ITIBAREN YAZINIZ !!!
			double max_distance = 0.0; 
			double max_index = 0;

			for (long unsigned int i = 0; i < laser_msg->ranges.size(); i++)
			{
				// Check if the ray is longer than the current max AND is a real number
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

			this->lidar_rays.points.push_back(p);
			this->lidar_rays.header.stamp = this->get_clock()->now();
        };

        rays_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("lidar_rays", 10);
        auto raysCb = [this]() -> void {
			this->rays_pub_->publish(this->lidar_rays);
            RCLCPP_INFO(this->get_logger(), "LIDAR rays are publising...");
        };
        timer_ = this->create_wall_timer(500ms, raysCb);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/turtlebot4/odom", 10, odomCb);
		laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, laserCb);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rays_pub_;
};

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
  	rclcpp::spin(std::make_shared<LIDAR_rays>());
  	rclcpp::shutdown();
  	return 0;
}