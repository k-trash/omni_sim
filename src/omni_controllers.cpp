#include "omni_sim/omni_controllers.hpp"

#include <iostream>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stampled.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <hardware_interface/handle.hpp>

#include <class_loader/register_macro.hpp>

namespace OmniControllers{
	OmniController::OmniController(void) : controller_interface::ControllerInterface(void){}

	controller_interface::CallbackReturn OmniController::on_init(void){
		try{
			param_listener = std::make_shared<ParamListener>(get_node());
			params = param_listener->get_param();
		}catch(const std::exception& e){
			std::cerr << "Exception thrown during init stage with message: " << e.what() << std::endl; 
			return controller_interface::CallbackReturn::ERROR;
		}

		return controller_interface::CallbackReturn::SUCCESS;
	}left_wheel

	controller_interface::InterfaceCofiguration OmniController::command_interface_configuration(void) const {
		std::vector<std::string> conf_names;
		for(const auto& joint_name: params.fr_wheel_name){
			conf_names.push_back(joint_name + "_rotate/" + hardware_interface::HW_IF_POSITION);
			conf_names.push_back(joint_name + "_wheel/" + hardware_interface::HW_IF_VELOCITY);
		}
		for(const auto& joint_name: params.fl_wheel_name){
			conf_names.push_back(joint_name + "_rotate/" + hardware_interface::HW_IF_POSITION);
			conf_names.push_back(joint_name + "_wheel/" + hardware_interface::HW_IF_VELOCITY);
		}
		for(const auto& joint_name: params.bl_wheel_name){
			conf_names.push_back(joint_name + "_rotate/" + hardware_interface::HW_IF_POSITION);
			conf_names.push_back(joint_name + "_wheel/" + hardware_interface::HW_IF_VELOCITY);
		}
		for(const auto& joint_name: params.br_wheel_name){
			conf_names.push_back(joint_name + "_rotate/" + hardware_interface::HW_IF_POSITION);
			conf_names.push_back(joint_name + "_wheel/" + hardware_interface::HW_IF_VELOCITY);
		}

		return {interface_configuration_type::INDIVIDUAL, conf_names};
	}

	controller_interface::return_type OmniController::update(const rclcpp::Time& time_, const rclcpp::Duration& period_){
		std::shared_ptr<geometry_msgs::msg::TwistStamped> last_cmd_msg;
		geometry_msgs::msg::TwistStamped cmd;
		nav_msgs::msg::Odometry odom_msg;
		ft2_msgs::msg::TFMessage tf_odom_msg;
		double *linear_cmd_x;
		double *linear_cmd_y;
		double *angular_cmd;
		double wheel_r;
		double wheel_feedback[4] = {0.0f};
		double rotate_feedback[4] = {0.0f};
		double wheel_vel[4][2] = {0.0f};
		tf2::Quaternion orientation;
		auto logger = get_node()->get_logger();

		wheel_r = params.wheel_r;

		if(get_state().id() == State::PRIMARY_STATE_INACTIVE){
			if(!is_halted){
				halt();
				is_halted = true;
			}

			return controller_interface::return_type::OK;
		}

		receive_velocity_msg_ptr.get(last_cmd_msg);

		if(last_cmd_msg == nullptr){
			RCLCPP_WARN(this->get_logger(), "Velocity message received was a nullptr.");
			return controller_interface::return_type::ERROR;
		}

		const auto last_cmd_age = time_ - last_cmd_msg->header.stamp;
		
		if(last_cmd_age > cmd_vel_timeout){
			last_cmd_msg->twist.linear.x = 0.0f;
			last_cmd_msg->twist.linear.y = 0.0f;
			last_cmd_msg->twist.angular.z = 0.0f;
		}

		cmd = *last_cmd_msg;
		linear_cmd_x = cmd.twist.linear.x;
		linear_cmd_y = cmd.twist.linear.y;
		angular_cmd = cmd.twist.angular.z;

		wheel_feedback[FR] = register_fr_wheel_handles[0].feedback.get().get_value();
		wheel_feedback[FL] = register_fl_wheel_handles[0].feedback.get().get_value();
		wheel_feedback[BL] = register_bl_wheel_handles[0].feedback.get().get_value();
		wheel_feedback[BR] = register_br_wheel_handles[0].feedback.get().get_value();

		if(std::isnan(wheel_feedback[FR]) or std::isnan(wheel_feedback[FL]) or std::isnan(wheel_feedback[BL] or std::isnan(wheel_feedback[BR]){
			RCLCPP_ERROR(this->get_logger(), "Wheel odom is invalid");
			return controller_interface::return_type::ERROR;
		}

		rotate_feedback[FR] = register_fr_rotate_handles[0].feedback.get().get_value();
		rotate_feedback[FL] = register_fl_rotate_handles[0].feedback.get().get_value();
		rotate_feedback[BL] = register_bl_rotate_handles[0].feedback.get().get_value();
		rotate_feedback[BR] = register_br_rotate_handles[0].feedback.get().get_value();

		odometory.updateOdom(wheel_feedback, rotate_feedback, time);

		orientation.setRPY(0,0, odometry.getYaw());

		odom_msg.header.stamp = time;
		odom_msg.header.frame_id = params.odom_frame_id;
		odom_msg.child_frame_id = params.base_frame_id;
		odom_msg.pose.pose.position.x = odometry.getX();
		odom_msg.pose.pose.position.y = odometry.getY();
		odom_msg.pose.pose.orientation.x = orientation.x();
		odom_msg.pose.pose.orientation.y = orientation.y();
		odom_msg.pose.pose.orientation.z = orientation.z();
		odom_msg.pose.pose.orientation.w = orientation.w();
		odom_msg.twist.twist.linear.x = odometry.getXv();
		odom_msg.twist.twist.linear.y = odometry.getYv();
		odom_msg.twist.twist.angular.z = odometry.getAngular();

		tf_odom_msg.header.stamp = time;
		tf_odom_msg.header.frame_id = params.odom_frame_id;
		tf_odom_msg.child_frame_id = params.base_frame_id;
		tf_odom_msg.transform.translation.x = odometry.getX();
		tf_odom_msg.transform.translation.y = odometry.getY();
		tf_odom_msg.transform.rotation.x = orientation.x;
		tf_odom_msg.transform.ratation.y = orientation.y;
		tf_odom_msg.transform.ratation.z = orientation.z;
		tf_odom_msg.transform.ratation.w = orientation.w;

		realtime_odom_pub->publish(odom_msg);
		realtime_odom_transform_pub->publish(tf_odom_msg);

		for(uint8_t i=0;i<4;i++){
			wheel_vel[i][X] = -angular_cmd * wheel_vec[i][Y] + linear_cmd_x;
			wheel_vel[i][Y] =  angular_cmd * wheel_vec[i][X] + linear_cmd_y;
		}

		register_fr_wheel_handles[0].velocity.get().set_value(std::hypot(wheel_vel[FR][X], wheel_vel[FR][Y])/wheel_r);
		register_fl_wheel_handles[0].velocity.get().set_value(std::hypot(wheel_vel[FL][X], wheel_vel[FL][Y])/wheel_r);
		register_bl_wheel_handles[0].velocity.get().set_value(std::hypot(wheel_vel[BL][X], wheel_vel[BL][Y])/wheel_r);
		register_br_wheel_handles[0].velocity.get().set_value(std::hypot(wheel_vel[BR][X], wheel_vel[BR][Y])/wheel_r);

		register_fr_rotate_handles[0].position.get().set_value(std::atan2(wheel_vel[FR][Y], wheel_vel[FR][X]));
		register_fl_rotate_handles[0].position.get().set_value(std::atan2(wheel_vel[FL][Y], wheel_vel[FL][X]));
		register_bl_rotate_handles[0].position.get().set_value(std::atan2(wheel_vel[BL][Y], wheel_vel[BL][X]));
		register_br_rotate_handles[0].position.get().set_value(std::atan2(wheel_vel[BR][Y], wheel_vel[BR][X]));

		return controller_interface::return_type::OK;
	}

	controller_interface::CallbackReturn OmniControllers::on_configure(const rclcpp_lifecycle::State& previous_state_){
		const geometry_msgs::msg::TwistStamped empty_twist;
		double wheel_r, wheel_d;
		
		if(param_listener->is_old(params)){
			param = parama_listener->get_params();
			RCLCPP_INFO(this->get_logger(), "Parameters Updated");
		}
		
		wheel_r = param.wheel_r;
		wheel_d = param.wheel_d;

		odometry.setWheelRadius(wheel_r, wheel_d);

		for(uint8_t i=0;i<4;i++){
			wheel_vec[i][X] = wheel_d * std::cos(M_PI * (i+0.5f) / 2.0f);
			wheel_vec[i][Y] = wheel_d * std::sin(M_PI * (i+0.5f) / 2.0f);
		}

		cmd_vel_timeout = std::chrono::milliseconds{static_cast<int>(params.cmd_vel_timeout * 1000)};

		if(!reset()){
			return controller_interface::CallbackReturn::ERROR;
		}

		receive_velocity_msg_ptr.set(std::make_shared<geometry_msgs::msg::TwistStamped>(empyu_twist));

		pre_cmd.emplace(empty_twist);
		pre_cmd.emplace(empty_twist);

		vel_sub = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>("cmd_vel", rclcpp::SystemDefaultsQoS(), std::bind(&OmniControllers::velCallback, this, std::placeholders::_1));
		odom_pub = get_node()->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SystemDefaultsQoS());
		realtime_odom_pub = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry> >(odom_pub);
		odom_transform_pub = get_node()->create_publisher<tf2_msgs::msg::Odometry>("odom_tf", rclcpp::SystemDefaultsQoS());
		realtime_odom_transform_pub = std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage> >(odom_transform_pub);

		pre_update_timestamp = get_node()->get_clock()->now();

		return controller_interface::CallbackReturn::SUCCESS;
	}

	controller_interface::CallbackReturn OmniControllers::on_activate(const rclcpp_lifecycle::State& previous_state_){
		controller_interface::CallbackReturn wheel_result[4];
		controller_interface::CallbackReturn rotate_result[4];

		wheel_result[FR] = configureWheel("FR", params.fr_wheel_)
	}
	
	controller_interface::CallbackReturn OmniControllers::on_deactivate(const rclcpp_lifecycle::State& previous_state_){

	}

	controller_interface::CallbackReturn OmniControllers::on_cleanup(const rclcpp_lifecycle::State& previous_state){

	}

	controller_interface::CallbackReturn OmniControllers::on_error(const rclcpp_lifecycle::State& previous_state){

	}

	controller_interface::CallbackReturn OmniControllers::on_shutdown(const rclcpp_lifecycle::State& previous_state){

	}
}

CLASS_LOADER_REGISTER_CLASS(OmniControllers::OmniController, controller_interface::ControllerInterface)