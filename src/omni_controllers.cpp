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
		double *linear_cmd_x;
		double *linear_cmd_y;
		double *angular_cmd;
		auto logger = get_node()->get_logger();

		if(get_state().id() == State::PRIMARY_STATE_INACTIVE){
			if(!is_halted){
				halt();
				is_halted = true;
			}

			return controller_interface::return_type::OK;
		}

		receive_velocity_msg_ptr.get(last_cmd_msg);

		if(last_cmd_msg == nullptr){
			RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
			return controller_interface::return_type::ERROR;
		}

		const auto last_cmd_age = time_ - last_cmd_msg->header.stamp;
		
		if(last_cmd_age > cmd_vel_timeout){
			last_cmd_msg->twist.linear.x = 0.0f;
			last_cmd_msg->twist.linear.y = 0.0f;
			last_cmd_msg->twist.angular.z = 0.0f;
		}

		cmd = *lastcmdmsg;
		linear_cmd_x = cmd.twist.linear.x;
		linear_cmd_y = cmd.twist.linear.x;
		angular_cmd = cmd.twist.angular.z;
	}
}

CLASS_LOADER_REGISTER_CLASS(OmniControllers::OmniController, controller_interface::ControllerInterface)