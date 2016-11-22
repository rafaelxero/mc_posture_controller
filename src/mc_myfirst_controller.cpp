#include <mc_rtc/logging.h>
#include "mc_myfirst_controller.h"

namespace mc_control {

    MCMyFirstController::MCMyFirstController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
	: MCController(robot_module, dt) {

	solver().addConstraintSet(contactConstraint);
	solver().addConstraintSet(dynamicsConstraint);
	solver().addTask(postureTask.get());
	solver().setContacts({{robots(), 0, 1, "LFullSole", "AllGround"}, 
		    {robots(), 0, 1, "RFullSole", "AllGround"}});

	head_joint_index = robot().jointIndexByName("HEAD_JOINT0");
	
	comTask = std::make_shared<mc_tasks::CoMTask>(robots(), robots().robotIndex(), 2.0, 100.0);
	solver().addTask(comTask);
	postureTask->stiffness(1.0);
	
	LOG_SUCCESS("MCMyFirstController init done " << this);
    }

    bool MCMyFirstController::run() {

	bool ret = MCController::run();

	if (comTask->eval().norm() < 0.02) {
	    switch_target();
	}

	return ret;
    }

    void MCMyFirstController::reset(const ControllerResetData & reset_data) {
    
	MCController::reset(reset_data);
	comTask->reset();
	comZero = rbd::computeCoM(robot().mb(), robot().mbc());
	target_left = true;
	switch_target();
    }

    void MCMyFirstController::switch_target() {
    
	Eigen::Vector3d move(0.0, 0.06, 0.0), target;

	if (target_left) {
	    target = comZero + move;
	}
	else {
	    target = comZero - move;
	}
	
	comTask->com(target);
	target_left = !target_left;
    }
}
