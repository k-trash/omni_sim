#ifndef __OMNI_CONTROLLERS_LIB__
#define __OMNI_CONTROLLERS_LIB__

#include <rclcpp/rclcpp.hpp>
#include <rclcpp

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
		private:

	};
}

#endif