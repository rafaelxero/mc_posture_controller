#include <mc_rtc/logging.h>
#include "mc_myfirst_controller.h"

namespace mc_control {

    MCMyFirstController::MCMyFirstController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
	: MCController(robot_module, dt) {

	solver().addConstraintSet(contactConstraint);
	solver().addConstraintSet(dynamicsConstraint);
	solver().addConstraintSet(selfCollisionConstraint);

	solver().addTask(postureTask.get());

	solver().setContacts({{robots(), 0, 1, "LFullSole", "AllGround"}, 
		    {robots(), 0, 1, "RFullSole", "AllGround"}});

	head_joint_index = robot().jointIndexByName("HEAD_JOINT0");
	
	comTask = std::make_shared<mc_tasks::CoMTask>(robots(), robots().robotIndex(), 2.0, 100.0);
	solver().addTask(comTask);
	postureTask->stiffness(1.0);
	
	efTask = std::make_shared<mc_tasks::EndEffectorTask>("RARM_LINK7", robots(), robots().robotIndex(), 5.0, 1000.0);
	solver().addTask(efTask);

	selfCollisionConstraint.addCollisions(solver(), {
		{"RARM_LINK5", "RLEG_LINK2", 0.05, 0.01, 0.0},
		{"RARM_LINK6", "RLEG_LINK2", 0.05, 0.01, 0.0},
		{"RARM_LINK7", "RLEG_LINK2", 0.05, 0.01, 0.0}});

	LOG_SUCCESS("MCMyFirstController init done " << this);
    }

    bool MCMyFirstController::run() {

	bool ret = MCController::run();

	if (efTask->eval().norm() < 0.02) {
	    switch_target();
	}

	return ret;
    }

    void MCMyFirstController::reset(const ControllerResetData & reset_data) {
    
	MCController::reset(reset_data);
	comTask->reset();
	efTask->reset();
	transformZero = efTask->get_ef_pose();
	target_left = true;
	switch_target();
    }

    void MCMyFirstController::switch_target() {
    
	sva::PTransformd move(Eigen::Vector3d(0.0, 0.06, 0.0));
	efTask->set_ef_pose(transformZero);

	if (target_left) {
	    efTask->add_ef_pose(move);
	}
	else {
	    efTask->add_ef_pose(move.inv());
	}
	
	target_left = !target_left;
    }
}
