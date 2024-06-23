#include "omni_sim/odom_tf_converter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace OmniRobotController{
	OdomToTf::OdomToTf(const rclcpp::NodeOptions& options) : rclcpp::Node("odom_tf_converter", options){
		tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
		odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&OdomToTf::odomCallback, this, std::placeholders::_1));
	}

	OdomToTf::~OdomToTf(void){
		;
	}

	void OdomToTf::odomCallback(const nav_msgs::msg::Odometry& msg_){
		geometry_msgs::msg::TransformStamped tf_msg;

		tf_msg.header = msg_.header;
		tf_msg.child_frame_id = msg_.child_frame_id;

		tf_msg.transform.translation.x = msg_.pose.pose.position.x;
		tf_msg.transform.translation.y = msg_.pose.pose.position.y;
		tf_msg.transform.translation.z = msg_.pose.pose.position.z;

		tf_msg.transform.rotation = msg_.pose.pose.orientation;

		tf_broadcaster->sendTransform(tf_msg);
	}
}

RCLCPP_COMPONENTS_REGISTER_NODE(OmniRobotController::OdomToTf)