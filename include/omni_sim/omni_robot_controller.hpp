#ifndef __OMNI_ROBOT_CONTROLLER_LIB__
#define __OMNI_ROBOT_CONTROLLER_LIB__

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace OmniRobotController{
	class OmniControl : public rclcpp::Node{
		public:
			explicit OmniControl(const rclcpp::NodeOptions& options);
			virtual ~OmniControl(void);
		private:
			rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_pos_pub;
			rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_vel_pub;
			rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub;

			void velCallback(const geometry_msgs::msg::Twist& msg_);

			double wheel_vec[4][2];

			enum Wheel{
				FR, FL, BL, BR
			};

			enum Coordinate{
				X, Y
			};
	};
}

#endif