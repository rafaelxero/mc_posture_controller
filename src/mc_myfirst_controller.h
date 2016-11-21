#pragma once
#include <mc_control/mc_controller.h>
#include <mc_control/api.h>

namespace mc_control {

    struct MC_CONTROL_DLLAPI MCMyFirstController : public MCController {

    public:

	MCMyFirstController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt);

	virtual bool run() override;
	virtual void reset(const ControllerResetData & reset_data) override;
	void switch_target();
    
	int head_joint_index;
	bool target_left;
    };

    SIMPLE_CONTROLLER_CONSTRUCTOR("MyFirst", mc_control::MCMyFirstController)
}
