#ifndef __ODOM_TF_CONVERTER_LIB__
#define __ODOM_TF_CONVERTER_LIB__

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>

namespace OmniRobotController{
	class OdomToTf : public rclcpp::Node{
		public:
			explicit OdomToTf(const rclcpp::NodeOptions& options);
			virtual ~OdomToTf(void);
		private:
			rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
			std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

			void odomCallback(const nav_msgs::msg::Odometry& msg_);
	};
}

#endif