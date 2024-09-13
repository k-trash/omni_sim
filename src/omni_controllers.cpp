#include "omni_sim/omni_controllers.hpp"

#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <class_loader/register_macro.hpp>


namespace omni_controllers{
	OmniController::OmniController(void) : controller_interface::ControllerInterface(){}

	OmniController::~OmniController(void){
		;
	}

	controller_interface::CallbackReturn OmniController::on_init(void){
		try{
			//setup parameter listener
			param_listener = std::make_shared<ParamListener>(get_node());
			params = param_listener->get_params();
		}catch(const std::exception& e){
			std::cerr << "Exception thrown during init stage with message: " << e.what() << std::endl; 
			return controller_interface::CallbackReturn::ERROR;
		}

		return controller_interface::CallbackReturn::SUCCESS;
	}

	controller_interface::InterfaceConfiguration OmniController::command_interface_configuration(void) const {
		std::vector<std::string> conf_names;
			
		for(const auto& joint_name: params.wheel_name){
			conf_names.push_back(joint_name + "/" +  hardware_interface::HW_IF_EFFORT);
		}
		for(const auto& joint_name: params.rotate_name){
			conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
		}

		return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
	}
	
	controller_interface::InterfaceConfiguration OmniController::state_interface_configuration(void) const {
		std::vector<std::string> conf_names;
			
		for(const auto& joint_name: params.wheel_name){
			conf_names.push_back(joint_name + "/" +  hardware_interface::HW_IF_VELOCITY);
		}
		for(const auto& joint_name: params.rotate_name){
			conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
		}

		return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
	}

	controller_interface::return_type OmniController::update(const rclcpp::Time& time_, const rclcpp::Duration& period_){
		std::shared_ptr<geometry_msgs::msg::TwistStamped> last_cmd_msg;
		geometry_msgs::msg::TwistStamped cmd;
		auto& odom_msg = realtime_odom_pub->msg_;
		auto& tf_odom_msg = realtime_odom_transform_pub->msg_;
		double linear_cmd_x;
		double linear_cmd_y;
		double angular_cmd;
		double wheel_r;
		std::shared_ptr<double[]> wheel_feedback(new double[4], std::default_delete<double[]>());
		std::shared_ptr<double[]> rotate_feedback(new double[4], std::default_delete<double[]>());
		double wheel_vel[4][2] = {0.0f};
		tf2::Quaternion orientation;
		auto logger = get_node()->get_logger();

		wheel_r = params.wheel_r;
		tf_odom_msg.transforms.resize(1);

		if(get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE){
			if(!is_halted){
				halt();
				is_halted = true;
			}

			RCLCPP_DEBUG(logger, "state is inactive.");

			return controller_interface::return_type::OK;
		}

		receive_vel_msg_ptr.get(last_cmd_msg);

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

		wheel_feedback[FR] = registered_wheel_handles[FR].feedback.get().get_value() * wheel_r;
		wheel_feedback[FL] = registered_wheel_handles[FL].feedback.get().get_value() * wheel_r;
		wheel_feedback[BL] = registered_wheel_handles[BL].feedback.get().get_value() * wheel_r;
		wheel_feedback[BR] = registered_wheel_handles[BR].feedback.get().get_value() * wheel_r;

		if(std::isnan(wheel_feedback[FR]) or std::isnan(wheel_feedback[FL]) or std::isnan(wheel_feedback[BL]) or std::isnan(wheel_feedback[BR])){
			RCLCPP_ERROR(get_node()->get_logger(), "Wheel odom is invalid");
			return controller_interface::return_type::ERROR;
		}

		rotate_feedback[FR] = registered_rotate_handles[FR].feedback.get().get_value();
		rotate_feedback[FL] = registered_rotate_handles[FL].feedback.get().get_value();
		rotate_feedback[BL] = registered_rotate_handles[BL].feedback.get().get_value();
		rotate_feedback[BR] = registered_rotate_handles[BR].feedback.get().get_value();

		if(std::isnan(rotate_feedback[FR]) or std::isnan(rotate_feedback[FL]) or std::isnan(rotate_feedback[BL]) or std::isnan(rotate_feedback[BR])){
			RCLCPP_ERROR(get_node()->get_logger(), "Rotation odom is invalid");
			return controller_interface::return_type::ERROR;
		}

		odometry.updateOdom(wheel_feedback, rotate_feedback, time_);

		odometry.returnOdom(odom_msg);

		odom_msg.header.stamp = time_;
		odom_msg.header.frame_id = params.odom_frame_id;
		odom_msg.child_frame_id = params.base_frame_id;

		odometry.returnTF(tf_odom_msg);
	
		tf_odom_msg.transforms.front().header.stamp = time_;
		tf_odom_msg.transforms.front().header.frame_id = params.odom_frame_id;
		tf_odom_msg.transforms.front().child_frame_id = params.base_frame_id;

		realtime_odom_pub->unlockAndPublish();
		realtime_odom_transform_pub->unlockAndPublish();

		for(uint8_t i=0;i<4;i++){
			wheel_vel[i][X] = -angular_cmd * wheel_vec[i][Y] + linear_cmd_x;
			wheel_vel[i][Y] =  angular_cmd * wheel_vec[i][X] + linear_cmd_y;
		}

		for(uint8_t i=0; i<4; i++){
			registered_wheel_handles[i].effort.get().set_value(pid[i].calcEffort(std::hypot(wheel_vel[i][X], wheel_vel[i][Y]), wheel_feedback[i]));
			registered_rotate_handles[i].position.get().set_value(std::atan2(wheel_vel[i][Y], wheel_vel[i][X]));
		}

		return controller_interface::return_type::OK;
	}

	controller_interface::CallbackReturn OmniController::on_configure(const rclcpp_lifecycle::State& previous_state_){
		const geometry_msgs::msg::TwistStamped empty_twist;
		double wheel_r, wheel_d;
		
		if(param_listener->is_old(params)){
			params = param_listener->get_params();
			RCLCPP_INFO(get_node()->get_logger(), "Parameters Updated");
		}
		
		wheel_r = params.wheel_r;
		wheel_d = params.wheel_d;

		odometry.initOdom(wheel_d);

		for(uint8_t i=0;i<4;i++){
			wheel_vec[i][Coordinate::X] = wheel_d * std::cos(M_PI * (i+0.5f) / 2.0f);		//set wheel vector x
			wheel_vec[i][Coordinate::Y] = wheel_d * std::sin(M_PI * (i+0.5f) / 2.0f);		//set wheel vector y

			pid[i].setPid(0.1, 0.00001, 0.00001, 0.001, 0.0);		//set PID parameters
		}

		cmd_vel_timeout = std::chrono::milliseconds{static_cast<int>(params.cmd_vel_timeout * 1000)};

		if(!reset()){
			return controller_interface::CallbackReturn::ERROR;
		}

		receive_vel_msg_ptr.set(std::make_shared<geometry_msgs::msg::TwistStamped>(empty_twist));

		pre_cmd.emplace(empty_twist);
		pre_cmd.emplace(empty_twist);

		vel_sub = get_node()->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SystemDefaultsQoS(), std::bind(&OmniController::velCallback, this, std::placeholders::_1));
		odom_pub = get_node()->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SystemDefaultsQoS());
		realtime_odom_pub = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry> >(odom_pub);
		odom_transform_pub = get_node()->create_publisher<tf2_msgs::msg::TFMessage>("odom_tf", rclcpp::SystemDefaultsQoS());
		realtime_odom_transform_pub = std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage> >(odom_transform_pub);

		pre_update_timestamp = get_node()->get_clock()->now();

		RCLCPP_INFO(get_node()->get_logger(), "configuration succeeded");

		return controller_interface::CallbackReturn::SUCCESS;
	}

	controller_interface::CallbackReturn OmniController::on_activate(const rclcpp_lifecycle::State& previous_state_){
		controller_interface::CallbackReturn wheel_result;
		controller_interface::CallbackReturn rotate_result;

		wheel_joint_name.clear();
		rotate_joint_name.clear();
		
		for(const auto& joint_name: params.wheel_name){
			wheel_joint_name.push_back(joint_name);
		}
		for(const auto& joint_name: params.rotate_name){
			rotate_joint_name.push_back(joint_name);
		}

		wheel_result = configureWheel(wheel_joint_name, registered_wheel_handles);
		rotate_result = configureRotate(rotate_joint_name, registered_rotate_handles);

		if(wheel_result == controller_interface::CallbackReturn::ERROR or rotate_result == controller_interface::CallbackReturn::ERROR){
			return controller_interface::CallbackReturn::ERROR;
		}

		if(registered_wheel_handles.empty() or registered_rotate_handles.empty()){
			RCLCPP_ERROR(get_node()->get_logger(), "Wheel interfaces are non exsistent");
			return controller_interface::CallbackReturn::ERROR;
		}

		is_halted = false;
		subscriber_is_active = true;

		RCLCPP_INFO(get_node()->get_logger(), "Subscriber and publisher are now active");

		return controller_interface::CallbackReturn::SUCCESS;
	}
	
	controller_interface::CallbackReturn OmniController::on_deactivate(const rclcpp_lifecycle::State& previous_state_){
		subscriber_is_active = false;

		if(!is_halted){
			halt();
			is_halted = true;
		}

		registered_wheel_handles.clear();
		registered_rotate_handles.clear();

		RCLCPP_INFO(get_node()->get_logger(), "Subscriber and publisher are now deactivated");

		return controller_interface::CallbackReturn::SUCCESS;
	}

	controller_interface::CallbackReturn OmniController::on_cleanup(const rclcpp_lifecycle::State& previous_state){
		if(!reset()){
			return controller_interface::CallbackReturn::ERROR;
		}

		receive_vel_msg_ptr.set(std::make_shared<geometry_msgs::msg::TwistStamped>());

		RCLCPP_INFO(get_node()->get_logger(), "node clean up");

		return controller_interface::CallbackReturn::SUCCESS;
	}

	controller_interface::CallbackReturn OmniController::on_error(const rclcpp_lifecycle::State& previous_state){
		if(!reset()){
			return controller_interface::CallbackReturn::ERROR;
		}

		RCLCPP_INFO(get_node()->get_logger(), "node on error");

		return controller_interface::CallbackReturn::SUCCESS;
	}

	controller_interface::CallbackReturn OmniController::on_shutdown(const rclcpp_lifecycle::State& previous_state){
		RCLCPP_INFO(get_node()->get_logger(), "Node now shutdown");
		return controller_interface::CallbackReturn::SUCCESS;
	}

	void OmniController::velCallback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_){
		geometry_msgs::msg::TwistStamped cmd_stamped;

		if(!subscriber_is_active){
			RCLCPP_WARN(get_node()->get_logger(), "Can't accept new command due to subscriber no activated");
			return;
		}

		cmd_stamped.header.stamp = get_node()->get_clock()->now();
		cmd_stamped.twist.linear = cmd_vel_->linear;
		cmd_stamped.twist.angular = cmd_vel_->angular;

		receive_vel_msg_ptr.set(std::move(std::make_shared<geometry_msgs::msg::TwistStamped>(cmd_stamped)));
	}

	//attach controller for each wheels
	controller_interface::CallbackReturn OmniController::configureWheel(const std::vector<std::string>& wheel_names_, std::vector<WheelHandle>& registered_handles_){
		if(wheel_names_.empty()){
			RCLCPP_ERROR(get_node()->get_logger(), "No wheel name specified");
			return controller_interface::CallbackReturn::ERROR;
		}

		registered_handles_.reserve(wheel_names_.size());
		for(const auto& wheel_name : wheel_names_){
			const auto interface_name = hardware_interface::HW_IF_EFFORT;
			const auto state_handle = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(),
			[&wheel_name, &interface_name](const auto & interface)
			{
				return interface.get_prefix_name() == wheel_name &&
				interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
			});

			if(state_handle == state_interfaces_.cend()){		//state_interfaces_ https://control.ros.org/galactic/doc/api/classhardware__interface_1_1LoanedStateInterface.html
				RCLCPP_ERROR(get_node()->get_logger(), "Unable to obrain joint state handle for %s", wheel_name.c_str());
				return controller_interface::CallbackReturn::ERROR;
			}

			const auto command_handle = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&wheel_name](const auto & interface)
			{
				return interface.get_prefix_name() == wheel_name &&
				interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
			});

			if(command_handle == command_interfaces_.end()){
				RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain joint command handle for %s", wheel_name.c_str());
				return controller_interface::CallbackReturn::ERROR;
			}

			registered_handles_.emplace_back(WheelHandle{std::ref(*state_handle), std::ref(*command_handle)});
		}

		return controller_interface::CallbackReturn::SUCCESS;
	}

	controller_interface::CallbackReturn OmniController::configureRotate(const std::vector<std::string>& rotate_names_, std::vector<RotateHandle>& registered_handles_){
		if(rotate_names_.empty()){
			RCLCPP_ERROR(get_node()->get_logger(), "No wheel name specified");
			return controller_interface::CallbackReturn::ERROR;
		}

		registered_handles_.reserve(rotate_names_.size());
		for(const auto& rotate_name : rotate_names_){
			const auto interface_name = hardware_interface::HW_IF_POSITION;
			const auto state_handle = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(),
			[&rotate_name, &interface_name](const auto & interface)
			{
				return interface.get_prefix_name() == rotate_name &&
				interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
			});

			if(state_handle == state_interfaces_.cend()){		//state_interfaces_ https://control.ros.org/galactic/doc/api/classhardware__interface_1_1LoanedStateInterface.html
				RCLCPP_ERROR(get_node()->get_logger(), "Unable to obrain joint state handle for %s", rotate_name.c_str());
				return controller_interface::CallbackReturn::ERROR;
			}

			const auto command_handle = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&rotate_name](const auto & interface)
			{
				return interface.get_prefix_name() == rotate_name &&
				interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
			});

			if(command_handle == command_interfaces_.end()){
				RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain joint command handle for %s", rotate_name.c_str());
				return controller_interface::CallbackReturn::ERROR;
			}

			registered_handles_.emplace_back(RotateHandle{std::ref(*state_handle), std::ref(*command_handle)});
		}

		return controller_interface::CallbackReturn::SUCCESS;
	}

	bool OmniController::reset(void){
		std::queue<geometry_msgs::msg::TwistStamped> empty;
		std::swap(pre_cmd, empty);

		registered_wheel_handles.clear();
		registered_rotate_handles.clear();

		subscriber_is_active = false;
		vel_sub.reset();

		receive_vel_msg_ptr.set(nullptr);
		is_halted = false;

		return true;
	}

	void OmniController::halt(void){
		const auto halt_wheels = [](auto & wheel_handles){
			for (const auto & wheel_handle : wheel_handles){
				wheel_handle.effort.get().set_value(0.0);
			}
		};

		const auto halt_rotates = [](auto & rotate_handles){
			for (const auto & rotate_handle : rotate_handles){
				rotate_handle.position.get().set_value(0.0);
			}
		};

		halt_wheels(registered_wheel_handles);
		halt_rotates(registered_rotate_handles);
	}
}

#include <pluginlib/class_list_macros.hpp>

CLASS_LOADER_REGISTER_CLASS(omni_controllers::OmniController, controller_interface::ControllerInterface)
