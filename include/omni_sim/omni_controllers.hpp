#ifndef __OMNI_CONTROLLERS_LIB__
#define __OMNI_CONTROLLERS_LIB__

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "omni_controllers_parameters.hpp"

namespace OmniControllers{
	class OmniController : public controller_interface::ControllerInterface{
		public:
			explicit OmniController(void);
			virtual ~OmniController(void);

			controller_interface::InterfaceConfiguration command_interface_configuration(void) const override;
			controller_interface::InterfaceConfiguration state_interface_configuration(void) const override;
			controller_interface::return_type update(const rclcpp::Time& time_, const rclcpp::Duration& period_) override;
			controller_interface::CallbackReturn on_init(void) override;
			controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state_) override;
			controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state_) override;
			controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state_) override;
			controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
			controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
			controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
		private:
			struct WheelHandle{
				std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback;
				std::reference_wrapper<hardware_interface::LoanedCommandInterface> position;
				std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity;
			};

			std::vector<WheelHandle> registered_fr_wheel_handles;
			std::vector<WheelHandle> registered_fl_wheel_handles;
			std::vector<WheelHandle> registered_bl_wheel_handles;
			std::vector<WheelHandle> registered_br_wheel_handles;

			rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub = nullptr;
			rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub = nullptr;

			realtime_tools::RealtimeBox<std::shared_ptr<geometry_msgs::msg::TwistStamped> > receive_velocity_msg_ptr{nullptr};

			std::shared_ptr<ParamListener> param_listener;
			Params params;

			std::chrono::illiseconds cmd_vel_timeout{500};
			rclcpp::Time previous_update_timestamp{0};

			bool is_halted = false;

			controller_interface::CallbackReturn configure_side(const std::string& side_, const std::vector<std::string>& wheel_names_, std::vector<WheelHandle>& registered_handles_);
			bool reset(void);
			void halt(void);
	};
}

#endif