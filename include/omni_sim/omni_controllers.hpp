#ifndef __OMNI_CONTROLLERS_LIB__
#define __OMNI_CONTROLLERS_LIB__

#include <vector>
#include <queue>
#include <string>

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
#include <pluginlib/class_list_macros.hpp>

#include <omni_sim/odom_solver.hpp>
#include <omni_sim/pid_controller.hpp>

#include "omni_controllers_param.hpp"

namespace omni_controllers{
extern "C"{
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
				std::reference_wrapper<hardware_interface::LoanedCommandInterface> effort;
			};

			struct RotateHandle{
				std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback;
				std::reference_wrapper<hardware_interface::LoanedCommandInterface> position;
			};

			std::vector<WheelHandle> registered_wheel_handles;
			std::vector<RotateHandle> registered_rotate_handles;
			std::vector<std::string> wheel_joint_name;
			std::vector<std::string> rotate_joint_name;

			OdomSolver odometry;

			rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub = nullptr;
			std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry> > realtime_odom_pub = nullptr;
			rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr odom_transform_pub = nullptr;
			std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage> > realtime_odom_transform_pub = nullptr;
			rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub = nullptr;

			realtime_tools::RealtimeBox<std::shared_ptr<geometry_msgs::msg::TwistStamped> > receive_vel_msg_ptr{nullptr};
			std::queue<geometry_msgs::msg::TwistStamped> pre_cmd;

			enum WHEEL_NUM{
				FR, FL, BL, BR
			};

			enum Coordinate{
				X=0, Y
			};

			PidController pid[4];

			std::shared_ptr<ParamListener> param_listener;
			Params params;

			std::chrono::milliseconds cmd_vel_timeout{500};
			rclcpp::Time pre_update_timestamp{0};

			bool is_halted = false;
			bool subscriber_is_active;

			double wheel_vec[4][2];

			void velCallback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_);
			controller_interface::CallbackReturn configureWheel(const std::vector<std::string>& wheel_names_, std::vector<WheelHandle>& registered_handles_);
			controller_interface::CallbackReturn configureRotate(const std::vector<std::string>& rotate_names_, std::vector<RotateHandle>& registered_handles_);
			bool reset(void);
			void halt(void);
	};
}

}

#endif