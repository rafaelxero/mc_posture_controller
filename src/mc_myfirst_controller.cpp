#include <mc_rtc/logging.h>
#include <mc_rbdyn/RobotLoader.h>
#include "mc_myfirst_controller.h"

namespace mc_control {

    MCMyFirstController::MCMyFirstController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
	: MCController({robot_module,
		    mc_rbdyn::RobotLoader::get_robot_module("env", std::string("/home/rcisneros/src/jrl/catkin_ws/src/mc_ros/mc_int_obj_description"), std::string("door")),
		    mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH), std::string("ground"))}, dt),
	  doorKinematicsConstraint(robots(), 1, dt)
    {
	postureTask->stiffness(1.0);
	
	efTask = std::make_shared<mc_tasks::EndEffectorTask>("RARM_LINK7", robots(), 0, 5.0, 100.0);

	auto& door = robots().robots()[1];

	door.mbc().zero(door.mb());
	door.mb().transform(0, sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(0.4, 0.7, 0.02)));
	rbd::forwardKinematics(door.mb(), door.mbc());

	doorPostureTask = std::make_shared<tasks::qp::PostureTask>(robots().mbs(), 1, door.mbc().q, 100.0, 1.0);

	sva::PTransformd offset(sva::RotY(-M_PI/2)*sva::RotZ(M_PI/2)*sva::RotY(M_PI/2), Eigen::Vector3d(0, -0.08, 0));

	efTask->set_ef_pose(offset*door.surface("Handle").X_0_s(door));
	
	qpsolver->addConstraintSet(contactConstraint);
	qpsolver->addConstraintSet(kinematicsConstraint);
	qpsolver->addConstraintSet(doorKinematicsConstraint);
	qpsolver->addConstraintSet(selfCollisionConstraint);
	
	qpsolver->addTask(postureTask.get());
	qpsolver->addTask(doorPostureTask.get());
	solver().addTask(efTask);
	
	solver().setContacts({
		{robots(), 0, 2, "LFullSole", "AllGround"},
		{robots(), 0, 2, "RFullSole", "AllGround"},
	    });

	LOG_SUCCESS("MCMyFirstController init done " << this);
    }

    bool MCMyFirstController::run() {

	bool ret = MCController::run();
	switch_phase();
	return ret;
    }

    void MCMyFirstController::reset(const ControllerResetData & reset_data) {
    
	MCController::reset(reset_data);
    }

    void MCMyFirstController::switch_phase() {
    
	if (phase == APPROACH) {
	    
	    if (efTask->eval().norm() < 1e-2) {

		std::cout << "APPROACH DONE" << std::endl;
		
		solver().removeTask(efTask);
		solver().setContacts({
			{robots(), 0, 2, "LFullSole", "AllGround"},
			{robots(), 0, 2, "RFullSole", "AllGround"},
			{robots(), 0, 1, "RightFingers", "Handle"}
		    });
		
		auto qDoor = doorPostureTask->posture();
		
		qDoor[2][0] -= 0.001;
		doorPostureTask->posture(qDoor);
		phase = DoorPhase::HANDLE;
	    }
	}
	else if (phase == HANDLE) {
	    
	    auto& door = robots().robots()[1];
            auto qDoor = doorPostureTask->posture();
	    qDoor[2][0] -= 0.001;
	    doorPostureTask->posture(qDoor);

	    if (fabs(door.mbc().q[2][0] < -0.4)) {
		
		auto qDoor = doorPostureTask->posture();
		qDoor[1][0] += 1.0;
		doorPostureTask->posture(qDoor);
		phase = DoorPhase::OPEN;
		std::cout << "HANDLE DONE" << std::endl;
	    }
	}
    }
}
