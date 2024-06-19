#ifndef __ODOM_SOLVER_LIB__
#define __ODOM_SOLVER_LIB__

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odoemtry.hpp>

namespace OmniColtrollers{
	class OdomSolver{
		public:
			OdomSolver(double wheel_d_);
			void updateOdom(std::shared_ptr<double>& wheel_feedback_, std::shared_ptr<double>& rotate_feedback_, rclcpp::Time time_);
			void returnOdom(std::shared_ptr<geometry_msgs::msg::TwistStamped>& odom_);
			void returnTF(std::shared_ptr<nav_msgs::msg::Odometry>& odom_);
		private:
			rclcpp::Time pre_time;
			geometry_msgs::msg::Twist odometry;
			tf2::Quaternion orientation;
			double wheel_vec[4][2];
			double wheel_d;
			double yaw;
	};
}

#endif