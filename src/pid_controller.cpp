#include "omni_sim/pid_controller.hpp"

namespace omni_controllers{
	PidController::PidController(void){
		pre_effort = 0.0f;
		pre_target = 0.0f;
		error[1] = error[2] = 0.0f;
	}

	PidController::~PidController(void){
		;
	}

	void PidController::setPid(double p_, double i_, double d_, double fi_, double fj_){
		p = p_;
		i = i_;
		d = d_;
		fi = fi_;
		fj = fj_;
	}

	double PidController::calcEffort(double target_speed_, double now_speed_){
		double effort;

		error[0] = target_speed_ - now_speed_;
		effort = pre_effort +  error[0]*(p+i+d) - error[1]*(p+2*d) + error[2]*d + target_speed_*(fi+fj) - pre_target*fi;

		pre_effort = effort;
		error[2] = error[1];
		error[1] = error[0];
		pre_target = target_speed_;

		return effort;
	}
}