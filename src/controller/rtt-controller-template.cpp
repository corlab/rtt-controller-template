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
#define JOINT_NAMES_MAPPING_LOOKUP( it, memberDict, remoteDict, jointName ) {it = remoteDict.find(#jointName); if (it != remoteDict.end()) { memberDict.jointName = it->second; } it = remoteDict.end(); }

RttControllerTemplate::RttControllerTemplate(std::string const& name) :
		cogimon::RTTJointAwareTaskContext(name), cmdJntPos_Port("cmdJntPos"), currJntState_Port(
				"currJntPos"), currJntState_Flow(RTT::NoData), _angle_step_change(0.5) {

	this->ports()->addPort(cmdJntPos_Port).doc(
			"Sending joint position commands.");

	this->ports()->addPort(currJntState_Port).doc(
			"Receiving current joint state.");

	outJntPos = rstrt::kinematics::JointAngles(DEFAULT_NR_JOINTS_LWR);
	outJntPos.angles.fill(0);
	cmdJntPos_Port.setDataSample(outJntPos);

	currJntState = rstrt::robot::JointState(DEFAULT_NR_JOINTS_LWR);
}

void RttControllerTemplate::retrieveJointMappingsHook(
		std::string const& port_name,
		std::map<std::string, int> const& mapping) {
	if (port_name == "cmdJntPos") {
		std::map<std::string, int>::const_iterator it;
		JOINT_NAMES_MAPPING_LOOKUP(it, jp_FullArm, mapping, LShSag);
		JOINT_NAMES_MAPPING_LOOKUP(it, jp_FullArm, mapping, LShLat);
		JOINT_NAMES_MAPPING_LOOKUP(it, jp_FullArm, mapping, LShYaw);
		JOINT_NAMES_MAPPING_LOOKUP(it, jp_FullArm, mapping, LElbj);
		JOINT_NAMES_MAPPING_LOOKUP(it, jp_FullArm, mapping, LForearmPlate);
		JOINT_NAMES_MAPPING_LOOKUP(it, jp_FullArm, mapping, LWrj1);
		JOINT_NAMES_MAPPING_LOOKUP(it, jp_FullArm, mapping, LWrj2);
	}
}

void RttControllerTemplate::processJointMappingsHook() {

}

bool RttControllerTemplate::configureHook() {
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

void RttControllerTemplate::updateHook() {

	if (currJntState_Port.connected()) {
		currJntState_Flow = currJntState_Port.read(currJntState);
	}

	outJntPos.angles(1) = _angle_step_change;
	_angle_step_change *= -1;

//	outJntPos.angles(jp_FullArm.)

// write torques to robot
	if (cmdJntPos_Port.connected()) {
		cmdJntPos_Port.write(outJntPos);
	}
}

void RttControllerTemplate::stopHook() {
	l(Info) << "executes stopping !" << endlog();
}

void RttControllerTemplate::cleanupHook() {
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
