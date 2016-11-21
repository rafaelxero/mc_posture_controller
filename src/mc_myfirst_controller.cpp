#include <mc_rtc/logging.h>
#include "mc_myfirst_controller.h"

namespace mc_control {

    MCMyFirstController::MCMyFirstController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
	: MCController(robot_module, dt) {

	solver().addConstraintSet(contactConstraint);
	solver().addConstraintSet(kinematicsConstraint);
	solver().addTask(postureTask.get());
	solver().setContacts({});

	head_joint_index = robot().jointIndexByName("HEAD_JOINT0");
	
	LOG_SUCCESS("MCMyFirstController init done " << this);
    }

    bool MCMyFirstController::run() {

	bool ret = MCController::run();

	if (std::abs(postureTask->posture()[head_joint_index][0] - robot().mbc().q[head_joint_index][0]) < 0.05) {
	    switch_target();
	}

	return ret;
    }

    void MCMyFirstController::reset(const ControllerResetData & reset_data) {
    
	MCController::reset(reset_data);
	target_left = true;
	switch_target();
    }

    void MCMyFirstController::switch_target() {
    
	double target;

	if (target_left) {
	    target = robot().qu()[head_joint_index][0];
	}
	else {
	    target = robot().ql()[head_joint_index][0];
	}
	
	std::vector<std::vector<double>> cur_obj = postureTask->posture();
	cur_obj[head_joint_index][0] = target;
	postureTask->posture(cur_obj);
	target_left = !target_left;
    }
}
