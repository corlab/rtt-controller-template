#ifndef RTT_LWR_OSF_CONTROLLER_HPP
#define RTT_LWR_OSF_CONTROLLER_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/base/RunnableInterface.hpp>
#include <rtt/Activity.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>

#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/robot/JointState.hpp>

#include <rtt-core-extensions/rtt-jointaware-taskcontext.hpp>

#include "JointPositionCtrlMapping-LeftArm.hpp"

#define DEFAULT_NR_JOINTS_LWR 7

class RttControllerTemplate: public cogimon::RTTJointAwareTaskContext {
public:
	RttControllerTemplate(std::string const& name);
	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

	void retrieveJointMappingsHook(std::string const& port_name,
			std::map<std::string, int> const& mapping);

	void processJointMappingsHook();

protected:
	/**
	 * OutputPorts publish data.
	 */
	RTT::OutputPort<rstrt::kinematics::JointAngles> cmdJntPos_Port;

	/**
	 * InputPorts read data.
	 */
	RTT::InputPort<rstrt::robot::JointState> currJntState_Port;
	RTT::FlowStatus currJntState_Flow;
	/**
	 * Hold the read value
	 */
	rstrt::robot::JointState currJntState;

	/**
	 * Hold the write value
	 */
	rstrt::kinematics::JointAngles outJntPos;

	/** ####### STORAGE FIELDS FOR CONTROLLERS ####### */
	Eigen::VectorXd jnt_trq_cmd_;

	// Control gain
	Eigen::VectorXd kg_;

	Eigen::VectorXd h;

	Eigen::VectorXd yD;

	/**
	 * Mapping for cmdJntPos_Port
	 */
	JointPositionCtrlMapping_FullArm jp_FullArm;

	float _angle_step_change;
};
#endif
