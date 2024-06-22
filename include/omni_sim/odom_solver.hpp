#ifndef __ODOM_SOLVER_LIB__
#define __ODOM_SOLVER_LIB__

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace omni_controllers{
	class OdomSolver{
		public:
			OdomSolver(void);
			void initOdom(double wheel_d_);
			void updateOdom(std::shared_ptr<double[]>& wheel_feedback_, std::shared_ptr<double[]>& rotate_feedback_, rclcpp::Time time_);
			void returnOdom(nav_msgs::msg::Odometry& odom_);
			void returnTF(tf2_msgs::msg::TFMessage& odom_);
		private:
			rclcpp::Time pre_time;
			nav_msgs::msg::Odometry odometry;
			tf2::Quaternion orientation;
			double wheel_vec[4][2];
			double wheel_d;
			double yaw;

			enum Coordinate{
				X, Y
			};
	};
}

#endif