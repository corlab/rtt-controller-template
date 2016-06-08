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

#include "../baseclasses/RTTArmControllerBase.hpp"
#include "../parsertools/KDLParser.hpp"

#include "JointPositionCtrlMapping-FullArm.hpp"

#define DEFAULT_ROOT_LINK "lwr_arm_base_link"
#define DEFAULT_TIP_LINK "lwr_arm_7_link"
#define DEFAULT_NR_JOINTS_LWR 7

class RttControllerTemplate: public RTTArmControllerBase {
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
	RTT::OutputPort<rstrt::dynamics::JointTorques> cmdJntTrq_Port;

	/**
	 * InputPorts read data.
	 */
	RTT::InputPort<rstrt::kinematics::JointAngles> currJntPos_Port;
	RTT::FlowStatus currJntPos_Flow;
	RTT::InputPort<rstrt::kinematics::JointVelocities> currJntVel_Port;
	RTT::FlowStatus currJntVel_Flow;
	RTT::InputPort<rstrt::dynamics::JointTorques> currJntTrq_Port;
	RTT::FlowStatus currJntTrq_Flow;
	/**
	 * Hold the read value
	 */
	rstrt::kinematics::JointAngles currJntPos;
	rstrt::dynamics::JointTorques currJntTrq;
	rstrt::kinematics::JointVelocities currJntVel;
	Eigen::VectorXd lastJntVel;
	Eigen::VectorXd currJntAcc;

	/**
	 * Hold the write value
	 */
	rstrt::kinematics::JointAngles outJntPos;
	rstrt::dynamics::JointTorques outJntTrq;

	/** ####### STORAGE FIELDS FOR CONTROLLERS ####### */
	Eigen::VectorXd jnt_trq_cmd_;

	// Control gain
	Eigen::VectorXd kg_;

	Eigen::VectorXd h;

	Eigen::VectorXd yD;

	double getSimulationTime();

	/**
	 * Mapping for cmdJntPos_Port
	 */
	JointPositionCtrlMapping_FullArm jp_FullArm;
};
#endif
