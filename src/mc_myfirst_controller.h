#pragma once
#include <mc_control/mc_controller.h>
#include <mc_control/api.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/EndEffectorTask.h>
#include <RBDyn/FK.h>

namespace mc_control {

    struct MC_CONTROL_DLLAPI MCMyFirstController : public MCController {

    public:

	MCMyFirstController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt);

	virtual bool run() override;
	virtual void reset(const ControllerResetData & reset_data) override;
	void switch_phase();
    
	std::shared_ptr<mc_tasks::EndEffectorTask> efTask;
	
	std::shared_ptr<tasks::qp::PostureTask> doorPostureTask;
	mc_solver::KinematicsConstraint doorKinematicsConstraint;
	
	enum DoorPhase {
	    APPROACH = 0,
	    HANDLE,
	    OPEN
	};

	DoorPhase phase = APPROACH;
    };

    SIMPLE_CONTROLLER_CONSTRUCTOR("MyFirst", mc_control::MCMyFirstController)
}
