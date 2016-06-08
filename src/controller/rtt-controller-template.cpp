#include "rtt-controller-template.hpp"

#include <rtt/Component.hpp>
#include <iostream>
#include <rtt/Activity.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

using namespace std;
using namespace RTT;
using namespace Orocos;

#define l(lvl) log(lvl) << "[" << this->getName() << "] "
#define JOINT_NAMES_MAPPING_LOOKUP( memberDict, remoteDict, jointName ) memberDict.jointName = remoteDict.at(#jointName)

RttControllerTemplate::RttControllerTemplate(std::string const& name) :
		RTTArmControllerBase(name, "lwr_arm_base_link", "lwr_arm_7_link", 7),
		// Name, initial value
		cmdJntPos_Port("cmdJntPos", 0.0), cmdJntTrq_Port("cmdJntTrq", 0.0), currJntPos_Port(
				"currJntPos"), currJntVel_Port("currJntVel"), currJntTrq_Port(
				"currJntTrq"), currJntPos_Flow(RTT::NoData), currJntVel_Flow(
				RTT::NoData), currJntTrq_Flow(RTT::NoData) {

	this->ports()->addPort(cmdJntPos_Port).doc(
			"Sending joint position commands.");
	this->ports()->addPort(cmdJntTrq_Port).doc(
			"Sending joint torque commands.");

	this->ports()->addPort(currJntPos_Port).doc(
			"Receiving current joint position.");
	this->ports()->addPort(currJntVel_Port).doc(
			"Receiving current joint velocity.");
	this->ports()->addPort(currJntTrq_Port).doc(
			"Receiving current joint torque.");

	outJntPos = rstrt::kinematics::JointAngles(DEFAULT_NR_JOINTS_LWR);
	outJntPos.angles.setZero();
	cmdJntPos_Port.setDataSample(outJntPos);

	outJntTrq = rstrt::dynamics::JointTorques(DEFAULT_NR_JOINTS_LWR);
	outJntTrq.torques.setZero();
	cmdJntTrq_Port.setDataSample(outJntTrq);

	currJntPos = rstrt::kinematics::JointAngles(DEFAULT_NR_JOINTS_LWR);
	currJntPos.angles.setZero();
	currJntVel = rstrt::kinematics::JointVelocities(DEFAULT_NR_JOINTS_LWR);
	currJntVel.velocities.setZero();
	currJntTrq = rstrt::dynamics::JointTorques(DEFAULT_NR_JOINTS_LWR);
	currJntTrq.torques.setZero();

	this->addOperation("parseURDFforKDL",
			&RttControllerTemplate::parseURDFforKDL, this, OwnThread).doc(
			"Parses a URDF string to create a KDL::Tree.").arg("urdfString",
			"URDF string to parse.");
}

void RttControllerTemplate::retrieveJointMappingsHook(
		std::string const& port_name,
		std::map<std::string, int> const& mapping) {
	if (port_name == "cmdJntPos") {
		JOINT_NAMES_MAPPING_LOOKUP(jp_FullArm, mapping, lwr_arm_0_joint);
		JOINT_NAMES_MAPPING_LOOKUP(jp_FullArm, mapping, lwr_arm_1_joint);
		JOINT_NAMES_MAPPING_LOOKUP(jp_FullArm, mapping, lwr_arm_2_joint);
		JOINT_NAMES_MAPPING_LOOKUP(jp_FullArm, mapping, lwr_arm_3_joint);
		JOINT_NAMES_MAPPING_LOOKUP(jp_FullArm, mapping, lwr_arm_4_joint);
		JOINT_NAMES_MAPPING_LOOKUP(jp_FullArm, mapping, lwr_arm_5_joint);
		JOINT_NAMES_MAPPING_LOOKUP(jp_FullArm, mapping, lwr_arm_6_joint);
	}
}

void RttControllerTemplate::processJointMappingsHook() {

}

bool RttControllerTemplate::configureHook() {
	initKDLTools();
	return true;
}

bool RttControllerTemplate::startHook() {
	if (!this->isConfigured()) {
		l(Error) << "Not configured yet: Needs to be configured first!"
				<< endlog();
		return false;
	}
	l(Info) << "started !" << endlog();
	return true;

}

double RttControllerTemplate::getSimulationTime() {
	return 1E-9
			* RTT::os::TimeService::ticks2nsecs(
					RTT::os::TimeService::Instance()->getTicks());
}

void RttControllerTemplate::updateHook() {
	/** Read feedback from robot */

	// check if port is connected
	if (currJntPos_Port.connected()) {
		// read into "currJntPos" and save state of data into "currJntPos_Flow", which can be "NewData", "OldData" or "NoData".
		currJntPos_Flow = currJntPos_Port.read(currJntPos);
	}
	if (currJntVel_Port.connected()) {
		currJntVel_Flow = currJntVel_Port.read(currJntVel);
	}
	if (currJntTrq_Port.connected()) {
		currJntTrq_Flow = currJntTrq_Port.read(currJntTrq);
	}

	// check for NoData
	if ((currJntPos_Flow == RTT::NoData) || (currJntVel_Flow == RTT::NoData)
			|| (currJntTrq_Flow == RTT::NoData)) {
		// skip this step, because we don't receive all the necessary data.
		return;
	}

	//	double delta_t = getSimulationTime() - last_SimulationTime;
	//    currJntAcc = (jnt_vel_ - lastJntVel) / 0.001;//delta_t ;

	// calculate mass(M_), coriolis(C_), gravity(G_), jacobian(jac_) (based on velocities)
	updateDynamicsAndKinematics(currJntPos, currJntVel, currJntTrq);

	// start simple joint position controller
	kg_.setConstant(1);
	jnt_trq_cmd_ = G_.data; // + kg_.asDiagonal() * (q_des_FirstPoint.data - jnt_pos_);
	outJntTrq.torques = jnt_trq_cmd_.cast<float>();

	// write torques to robot
	if (cmdJntTrq_Port.connected()) {
		cmdJntTrq_Port.write(outJntTrq);
	}
}

void RttControllerTemplate::stopHook() {
	l(Info) << "executes stopping !" << endlog();
}

void RttControllerTemplate::cleanupHook() {
	initKDLTools();
	l(Info) << "cleaning up !" << endlog();
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(RTTController)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
//ORO_LIST_COMPONENT_TYPE(RttControllerTemplate)
ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(RttControllerTemplate)
