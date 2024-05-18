#include "omni_sim/omni_robot_controller.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <cmath>

namespace OmniRobotController{
	OmniControl::OmniControl(const rclcpp::NodeOptions& options) : rclcpp::Node("omni_control", options){
		using namespace std::placeholders;

		double wheel_d = 0.0f;

		this->declare_parameter("wheel_r", 0.05);
		this->declare_parameter("wheel_d", 0.2*std::sqrt(2));

		wheel_pos_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
		wheel_vel_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands", 10);
		vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&OmniControl::velCallback, this, _1));

		wheel_d = this->get_parameter("wheel_d").as_double();

		for(uint8_t i=0;i<4;i++){
			wheel_vec[i][X] = wheel_d * std::cos(M_PI * (i+0.5f) / 2.0f);
			wheel_vec[i][Y] = wheel_d * std::sin(M_PI * (1+0.5f) / 2.0f);
		}
	}

	OmniControl::~OmniControl(void){
		;
	}

	void OmniControl::velCallback(const geometry_msgs::msg::Twist& msg_){
		double wheel_vel[4][2] = {0.0f};
		double wheel_r = 0.0f;
		std_msgs::msg::Float64MultiArray wheel_pos;
		std_msgs::msg::Float64MultiArray wheel_ang;

		wheel_r = this->get_parameter("wheel_r").as_double();
		for(uint8_t i=0;i<4;i++){
			wheel_vel[i][X] = -msg_.angular.z * wheel_vec[i][X] + msg_.linear.x;
			wheel_vel[i][Y] =  msg_.angular.z * wheel_vec[i][Y] + msg_.linear.y;
			wheel_pos.data.push_back(std::atan2(wheel_vel[i][Y], wheel_vel[i][X]));
			wheel_ang.data.push_back(std::hypot(wheel_vel[i][X], wheel_vel[i][Y])/wheel_r);
		}

		wheel_pos_pub->publish(wheel_pos);
	}
}

RCLCPP_COMPONENTS_REGISTER_NODE(OmniRobotController::OmniControl)