#include "omni_sim/odom_solver.hpp"

#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace omni_controllers{
	OdomSolver::OdomSolver(void){
		;
	}
	
	void OdomSolver::initOdom(double wheel_d_){
		wheel_d = wheel_d_;

		odometry.pose.pose.position.x = 0;
		odometry.pose.pose.position.y = 0;
		odometry.pose.pose.position.z = 0;
		odometry.pose.pose.orientation.x = 0;
		odometry.pose.pose.orientation.y = 0;
		odometry.pose.pose.orientation.z = 0;
		odometry.pose.pose.orientation.w = 0;
		odometry.twist.twist.linear.x = 0;
		odometry.twist.twist.linear.y = 0;
		odometry.twist.twist.linear.z = 0;
		odometry.twist.twist.angular.x = 0;
		odometry.twist.twist.angular.y = 0;
		odometry.twist.twist.angular.z = 0;
		yaw = 0.0f;

		for(uint8_t i=0;i<4;i++){
			wheel_vec[i][X] = wheel_d * std::cos(M_PI * (i+0.5f) / 2.0f);
			wheel_vec[i][Y] = wheel_d * std::sin(M_PI * (i+0.5f) / 2.0f);
		}
	}

	void OdomSolver::updateOdom(std::shared_ptr<double[]>& wheel_feedback_, std::shared_ptr<double[]>& rotate_feedback_, rclcpp::Time time_){
		double vx[4] = {0};
		double vy[4] = {0};
		double time_d;

		time_d = time_.seconds() - pre_time.seconds();

		for(uint8_t i=0; i<4;i++){
			vx[i] = wheel_feedback_.get()[i] * std::cos(rotate_feedback_.get()[i]);
			vy[i] = wheel_feedback_.get()[i] * std::sin(rotate_feedback_.get()[i]);
		}

		odometry.twist.twist.linear.x = 0;
		odometry.twist.twist.linear.y = 0;
		odometry.twist.twist.angular.z = 0;

		for(uint8_t i=0; i<4; i++){
			odometry.twist.twist.linear.x += vx[i];
			odometry.twist.twist.linear.y += vy[i];
			odometry.twist.twist.angular.z += -wheel_vec[i][1]*vx[i] + wheel_vec[i][0]*vy[i];
		}
		odometry.twist.twist.linear.x /= 4;
		odometry.twist.twist.linear.y /= 4;
		odometry.twist.twist.angular.z /= 4*wheel_d*wheel_d;

		odometry.pose.pose.position.x += std::cos(yaw) * odometry.twist.twist.linear.x * time_d - std::sin(yaw) * odometry.twist.twist.linear.y * time_d;
		odometry.pose.pose.position.y += std::sin(yaw) * odometry.twist.twist.linear.x * time_d + std::cos(yaw) * odometry.twist.twist.linear.y * time_d;
		yaw += odometry.twist.twist.angular.z * time_d;

		orient.setRPY(0,0, yaw);

		odometry.pose.pose.orientation.x = orient.x();
		odometry.pose.pose.orientation.y = orient.y();
		odometry.pose.pose.orientation.z = orient.z();
		odometry.pose.pose.orientation.w = orient.w();

		pre_time = time_;
	}

	void OdomSolver::returnOdom(nav_msgs::msg::Odometry& odom_){
		odom_.pose = odometry.pose;
		odom_.twist = odometry.twist;
	}

	void OdomSolver::returnTF(tf2_msgs::msg::TFMessage& odom_){
		odom_.transforms.front().transform.translation.x = odometry.pose.pose.position.x;
		odom_.transforms.front().transform.translation.y = odometry.pose.pose.position.y;
		odom_.transforms.front().transform.rotation.x = odometry.pose.pose.orientation.x;
		odom_.transforms.front().transform.rotation.y = odometry.pose.pose.orientation.y;
		odom_.transforms.front().transform.rotation.z = odometry.pose.pose.orientation.z;
		odom_.transforms.front().transform.rotation.w = odometry.pose.pose.orientation.w;
	}
}