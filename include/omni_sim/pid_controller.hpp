#ifndef __PID_CONTROLLER_LIB__
#define __PID_CONTROLLER_LIB__

namespace omni_controllers{
	class PidController{
		public:
			explicit PidController(void);
			virtual ~PidController(void);
			void setPid(double p_, double i_, double d_, double fi_, double fj_);
			double calcEffort(double target_speed_, double now_speed_);
		private:
			double p, i, d, fi, fj;
			double pre_target;
			double pre_effort;
			double error[3];
	};
}

#endif