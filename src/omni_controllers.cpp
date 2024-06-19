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
			//setup parameter listener
			param_listener = std::make_shared<ParamListener>(get_node());
			params = param_listener->get_param();
		}catch(const std::exception& e){
			std::cerr << "Exception thrown during init stage with message: " << e.what() << std::endl; 
			return controller_interface::CallbackReturn::ERROR;
		}

		return controller_interface::CallbackReturn::SUCCESS;
	}

	controller_interface::InterfaceCofiguration OmniController::command_interface_configuration(void) const {
		std::vector<std::string> conf_names;
			
		wheel_joint_name.clear();
		rotate_joint_name.clear();
		
		for(const auto& joint_name: params.wheel_name){
			wheel_joint_name.push_back(joint_name);
			conf_names.push_back(joint_name + "/" +  hardware_interface::HW_IF_POSITION);
		}
		for(const auto& joint_name: params.rotate_name){
			rotate_joint_name.push_back(joint_name);
			conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
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
		std::shared_ptr<double> wheel_feedback(new double[4], std::default_delete<double[]>());
		std::shared_ptr<double> rotate_feedback(new double[4], std::default_delete<double[]>());
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
			RCLCPP_WARN(get_node()->get_logger(), "Velocity message received was a nullptr.");
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

		wheel_feedback[FR] = register_wheel_handles[FR].feedback.get().get_value() * wheel_r;
		wheel_feedback[FL] = register_wheel_handles[FL].feedback.get().get_value() * wheel_r;
		wheel_feedback[BL] = register_wheel_handles[BL].feedback.get().get_value() * wheel_r;
		wheel_feedback[BR] = register_wheel_handles[BR].feedback.get().get_value() * wheel_r;

		if(std::isnan(wheel_feedback[FR]) or std::isnan(wheel_feedback[FL]) or std::isnan(wheel_feedback[BL] or std::isnan(wheel_feedback[BR]){
			RCLCPP_ERROR(get_node()->get_logger(), "Wheel odom is invalid");
			return controller_interface::return_type::ERROR;
		}

		rotate_feedback[FR] = register_rotate_handles[FR].feedback.get().get_value();
		rotate_feedback[FL] = register_rotate_handles[FL].feedback.get().get_value();
		rotate_feedback[BL] = register_rotate_handles[BL].feedback.get().get_value();
		rotate_feedback[BR] = register_rotate_handles[BR].feedback.get().get_value();

		if(std::isnan(rotate_feedback[FR]) or std::isnan(rotate_feedback[FL]) or std::isnan(rotate_feedback[BL] or std::isnan(rotate_feedback[BR]){
			RCLCPP_ERROR(get_node()->get_logger(), "Rotation odom is invalid");
			return controller_interface::return_type::ERROR;
		}

		odometry.updateOdom(wheel_feedback, rotate_feedback, time_);

		odometry.returnOdom(odom_msg);

		odom_msg.header.stamp = time_;
		odom_msg.header.frame_id = params.odom_frame_id;
		odom_msg.child_frame_id = params.base_frame_id;

		odometry.returnTF(tf_odom_msg);
	
		tf_odom_msg.header.stamp = time_;
		tf_odom_msg.header.frame_id = params.odom_frame_id;
		tf_odom_msg.child_frame_id = params.base_frame_id;

		realtime_odom_pub->publish(odom_msg);
		realtime_odom_transform_pub->publish(tf_odom_msg);

		for(uint8_t i=0;i<4;i++){
			wheel_vel[i][X] = -angular_cmd * wheel_vec[i][Y] + linear_cmd_x;
			wheel_vel[i][Y] =  angular_cmd * wheel_vec[i][X] + linear_cmd_y;
		}

		for(uint8_t i=0; i<4; i++){
			register_wheel_handles[i].velocity.get().set_value(std::hypot(wheel_vel[i][X], wheel_vel[i][Y])/wheel_r);
			register_rotate_handles[i].position.get().set_value(std::atan2(wheel_vel[i][Y], wheel_vel[i][X]));
		}

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

		receive_vel_msg_ptr.set(std::make_shared<geometry_msgs::msg::TwistStamped>(empyu_twist));

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
		controller_interface::CallbackReturn wheel_result;
		controller_interface::CallbackReturn rotate_result;

		wheel_result = configureWheel(wheel_joint_name, registered_wheel_handles);
		rotate_result = configureWheel(rotate_joint_name, registered_rotate_handles);

		if(wheel_result == controller_interface::CallbackReturn::ERROR or rotate_result == controller_interface::CallbackReturn::ERROR){
			return controller_interface::CallbackReturn::ERROR;
		}

		if(registered_wheel_handles.empty() or registered_rotate_handles.empty()){
			RCLCPP_ERROR(get_node()->get_logger(), "Wheel interfaces are non exsistent");
			return controller_interfaces::CallbackReturn::ERROR;
		}

		is_halted = false;
		subscriber_is_active = true;

		RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active");

		return controller_interface::CallbackReturn::SUCCESS;
	}
	
	controller_interface::CallbackReturn OmniControllers::on_deactivate(const rclcpp_lifecycle::State& previous_state_){
		subscriber_is_active = false;

		if(!is_halted){
			halt();
			is_halted = true;
		}

		registered_wheel_handles.clear();
		registered_rotate_handles.clear();

		return controller_interface::CallbackReturn::SUCCESS;
	}

	controller_interface::CallbackReturn OmniControllers::on_cleanup(const rclcpp_lifecycle::State& previous_state){
		if(!reset()){
			return controller_interface::CallbackReturn::ERROR;
		}

		received_vel_msg_ptr.set(std::make_shared<geometry_msgs::msg::TwistStamped>());

		return controller_interface::CallbackReturn::SUCCESS;
	}

	controller_interface::CallbackReturn OmniControllers::on_error(const rclcpp_lifecycle::State& previous_state){
		if(!reset()){
			return controller_interface::CallbackReturn::ERROR;
		}

		return controller_interface::CallbackReturn::SUCCESS;
	}

	controller_interface::CallbackReturn OmniControllers::on_shutdown(const rclcpp_lifecycle::State& previous_state){
		return controller_interface::CallbackReturn::SUCCESS;
	}

	void velCallback(const std::shared_ptr<geometry_msgs::msg::TwistStamped> cmd_vel_){
		if(!subscriber_is_active){
			RCLCPP_WARN(get_node()->get_logger(), "Can't accept new command due to subscriber no activated");
			return;
		}
		if((cmd_vel_->header.stamp.sec == 0) and (cmd_vel_->header.stamp.nanosec == 0)){
			RCLCPP_WARN_ONCE(get_node()->get_logger(), "Received TwistStamped with zero timestamp, setting it to current " "time, this message will only be shown once");
			cmd_vel_->header.stamp = get_node()->get_clock()->now();
		}

		received_vel_msg_ptr.set(std::move(cmd_vel_));
	}

	controller_interface::CallbackReturn configureWheel(const std::vector<std::string>& wheel_names_, std::vector<WheelHandle>& registered_handles_){
		if(wheel_names_.empty()){
			RCLCPP_ERROR(get_node()->get_logger(), "No wheel name specified");
			return controller_interface::CallbackReturn::ERROR;
		}

		registered_handles_.reserve(wheel_names.size());
		for(const auto& wheel_name : wheel_names){
			const auto state_handle = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(),
			[&wheel_name, &interface_name](const auto & interface)
			{
				return interface.get_prefix_name() == wheel_name &&
				interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
			});

			if(state_handle == state_interfaces_.cend){		//state_interfaces_ https://control.ros.org/galactic/doc/api/classhardware__interface_1_1LoanedStateInterface.html
				RCLCPP_ERROR(get_node()->get_logger(), "Unable to obrain joint state handle for %s", wheel_name.c_str());
				return controller_interface::CallbackReturn::ERROR;
			}

			const auto command_handle = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&wheel_name](const auto & interface)
			{
				return interface.get_prefix_name() == wheel_name &&
				interface.get_interface_name() == HW_IF_VELOCITY;
			});

			if(command_handle == command_interface_.end()){
				RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain joint command handle for %s", wheel_name.c_str());
				return controller_interface::CallbackReturn::ERROR;
			}

			registered_handles_.emplace_back(WheelHandle{std::ref(*state_handle), std::ref(*command_handle)});
		}

		return controller_interface::CallbackReturn::SUCCESS;
	}

	controller_interface::CallbackReturn configureRotate(const std::vector<std::string>& rotate_names_, std::vector<RotateHandle>& registered_handles_){
		if(rotate_names_.empty()){
			RCLCPP_ERROR(get_node()->get_logger(), "No wheel name specified");
			return controller_interface::CallbackReturn::ERROR;
		}

		registered_handles_.reserve(rotate_names.size());
		for(const auto& rotate_name : rotate_names_){
			const auto state_handle = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(),
			[&rotate_name, &interface_name](const auto & interface)
			{
				return interface.get_prefix_name() == rotate_name &&
				interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
			});

			if(state_handle == state_interfaces_.cend){		//state_interfaces_ https://control.ros.org/galactic/doc/api/classhardware__interface_1_1LoanedStateInterface.html
				RCLCPP_ERROR(get_node()->get_logger(), "Unable to obrain joint state handle for %s", rotate_name.c_str());
				return controller_interface::CallbackReturn::ERROR;
			}

			const auto command_handle = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&wheel_name](const auto & interface)
			{
				return interface.get_prefix_name() == rotate_name &&
				interface.get_interface_name() == HW_IF_VELOCITY;
			});

			if(command_handle == command_interface_.end()){
				RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain joint command handle for %s", rotate_name.c_str());
				return controller_interface::CallbackReturn::ERROR;
			}
		}

		return controller_interface::CallbackReturn::SUCCESS;
	}
	bool reset(void){
		std::queue<Twist> empty;
		std::swap(pre_cmd, empty);

		for(uint8_t i=0; i<4; i++){
			registered_wheel_handles[i].clear();
			registered_rotate_handles[i].clear();
		}

		subscriber_is_active = false;
		vel_sub.reset();

		received_vel_msg_ptr.set(nullptr);
		is_halted = false;

		return true;
	}

	void halt(void){
		const auto halt_wheels = [](auto & wheel_handles){
			for (const auto & wheel_handle : wheel_handles){
				wheel_handle.velocity.get().set_value(0.0);
			}
		};

		halt_wheels(registered_wheel_handles);
		halt_wheels(registered_rotate_handles);
	}
}

CLASS_LOADER_REGISTER_CLASS(OmniControllers::OmniController, controller_interface::ControllerInterface)