/**
ROS/IOP Bridge
Copyright (c) 2017 Fraunhofer

This program is dual licensed; you can redistribute it and/or
modify it under the terms of the GNU General Public License
version 2 as published by the Free Software Foundation, or
enter into a proprietary license agreement with the copyright
holder.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; or you can read the full license at
<http://www.gnu.de/documents/gpl-2.0.html>
*/

/** \author Alexander Tiderko */

#include "urn_jaus_jss_ugv_StabilizerDriverClient/StabilizerDriverClient_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>
#include <JausUtils.h>
#include <algorithm> // for std::find
#include <iterator>
#include <stdlib.h>
#include <string>     // std::string, std::stoi


using namespace JTS;
using namespace iop::ocu;


namespace urn_jaus_jss_ugv_StabilizerDriverClient
{



StabilizerDriverClient_ReceiveFSM::StabilizerDriverClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: SlaveHandlerInterface(cmp, "StabilizerDriverClient", 1.0),
  logger(cmp->get_logger().get_child("StabilizerDriverClient"))
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new StabilizerDriverClient_ReceiveFSMContext(*this);

	this->pManagementClient_ReceiveFSM = pManagementClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
	p_valid_capabilities = false;
	p_hz = 1.0;

}



StabilizerDriverClient_ReceiveFSM::~StabilizerDriverClient_ReceiveFSM()
{
	delete context;
}

void StabilizerDriverClient_ReceiveFSM::setupNotifications()
{
	pManagementClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_StabilizerDriverClient_ReceiveFSM_Receiving_Ready", "ManagementClient_ReceiveFSM");
	pManagementClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_StabilizerDriverClient_ReceiveFSM_Receiving_Ready", "ManagementClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pManagementClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ManagementClient_ReceiveFSM_Receiving_Ready", "StabilizerDriverClient_ReceiveFSM");
	registerNotification("Receiving", pManagementClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ManagementClient_ReceiveFSM_Receiving", "StabilizerDriverClient_ReceiveFSM");

}


void StabilizerDriverClient_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "StabilizerDriverClient");
	// get the ROS parameter
	p_names.clear();
	p_stabilizer.clear();
	p_efforts.clear();
	p_positions.clear();
	cfg.declare_param<std::vector<std::string> >("joint_names", p_names, false,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY,
		"Specifies a list with joint names. This is important to get the position of flipper. If no names are specified they will be generated from reported capabilities from the robot.",
		"Default: []");
	cfg.declare_param<double>("hz", p_hz, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"Sets how often the reports are requested. If use_queries is True hz must be greather then 0. In this case each time a Query message is sent to get a report. If use_queries is False an event is created to get Reports. In this case 0 disables the rate and an event of type on_change will be created.",
		"Default: 1.0");
	cfg.param_vector<std::vector<std::string> >("joint_names", p_names, p_names);
	cfg.param("hz", p_hz, p_hz, false);
	// subscribe to ROS joint state commands
	p_sub_jointstates = cfg.create_subscription<sensor_msgs::msg::JointState>("cmd_joint_states", 1, std::bind(&StabilizerDriverClient_ReceiveFSM::pRosCmdJointState, this, std::placeholders::_1));
	p_sub_cmd_vel = cfg.create_subscription<std_msgs::msg::Float64MultiArray>("flipper_velocity_controller/command", 1, std::bind(&StabilizerDriverClient_ReceiveFSM::pRosCmdVelocity, this, std::placeholders::_1));
	p_pub_jointstates = cfg.create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);

	// initialize the control layer, which handles the access control staff
	this->set_rate(p_hz);
	this->set_supported_service(*this, "urn:jaus:jss:ugv:StabilizerDriver", 1, 0);
	this->set_event_name("stabilizer capabilities");
	this->set_query_before_event(true, 1.0);
}

void StabilizerDriverClient_ReceiveFSM::register_events(JausAddress remote_addr, double hz)
{
	pEventsClient_ReceiveFSM->create_event(*this, remote_addr, p_query_position, p_hz);
}

void StabilizerDriverClient_ReceiveFSM::unregister_events(JausAddress remote_addr)
{
	pEventsClient_ReceiveFSM->cancel_event(*this, remote_addr, p_query_position);
	stop_query(remote_addr);
}

void StabilizerDriverClient_ReceiveFSM::send_query(JausAddress remote_addr)
{
	if (!p_valid_capabilities) {
		QueryStabilizerCapabilities query_cap_msg;
		sendJausMessage(query_cap_msg, remote_addr);
	} else {
		sendJausMessage(p_query_position, remote_addr);
	}
}

void StabilizerDriverClient_ReceiveFSM::stop_query(JausAddress remote_addr)
{
	p_valid_capabilities = false;
	p_stabilizer.clear();
	p_efforts.clear();
	p_positions.clear();
	this->set_event_name("stabilizer capabilities");
	this->set_query_before_event(true, 1.0);
}

void StabilizerDriverClient_ReceiveFSM::reportStabilizerCapabilitiesAction(ReportStabilizerCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	RCLCPP_DEBUG(logger, "reportStabilizerCapabilitiesAction from %d.%d.%d",
					subsystem_id, node_id, component_id);
	ReportStabilizerCapabilities::Body::StabilizerCapabilities *caplist = msg.getBody()->getStabilizerCapabilities();
	//  ROS_INFO("Manipulator Specification has %d joints", jointlist->getNumberOfElements()+1);
	p_stabilizer.clear();
	p_efforts.clear();
	p_positions.clear();
	for (unsigned int index = 0; index < caplist->getNumberOfElements(); index++) {
		ReportStabilizerCapabilities::Body::StabilizerCapabilities::StabilizerCapabilitiesSeq *stabseq = caplist->getElement(index);
		unsigned char stabilizer_id = stabseq->getStabilizerCapabilitiesRec()->getStabilizerID();
		std::string name = "";
		if (stabilizer_id < p_names.size()) {
			name = p_names[stabilizer_id];
		} else  {
			std::stringstream sstm;
			sstm << "actuator_" << index << "_joint";
			name = sstm.str();
		}
		p_stabilizer[stabilizer_id] = name;
		p_efforts[stabilizer_id] = 0.;
		p_positions[stabilizer_id] = 0.;
		//    ROS_INFO("  add %s joint", sstm.str().c_str());
	}
	// publish the current state of the manipulator
	auto rosmsg = sensor_msgs::msg::JointState();
	rosmsg.header.stamp = cmp->now();
	std::map<unsigned char, std::string>::iterator st_it;
	for (st_it = p_stabilizer.begin(); st_it != p_stabilizer.end(); ++st_it) {
		rosmsg.name.push_back(st_it->second);
		rosmsg.velocity.push_back(0.);
		rosmsg.position.push_back(0.);
	}
	p_pub_jointstates->publish(rosmsg);
	p_valid_capabilities = true;
	// create a new query message with all id
	p_query_position = QueryStabilizerPosition();
	for (std::map<unsigned char, std::string>::iterator it = p_stabilizer.begin(); it != p_stabilizer.end(); it++) {
		QueryStabilizerPosition::Body::StabilizerID::QueryStabilizerRec st_rec;
		st_rec.setStabilizerID(it->first);
		p_query_position.getBody()->getStabilizerID()->addElement(st_rec);
	}
	// force event for request sensor data
	this->set_event_name("stabilizer position");
	this->set_query_before_event(false);
}

void StabilizerDriverClient_ReceiveFSM::reportStabilizerEffortAction(ReportStabilizerEffort msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	RCLCPP_DEBUG(logger, "reportStabilizerEffortAction from %d.%d.%d",
					subsystem_id, node_id, component_id);
	auto rosmsg = sensor_msgs::msg::JointState();
	rosmsg.header.stamp = cmp->now();
	ReportStabilizerEffort::Body::StabilizerEffort *efforts = msg.getBody()->getStabilizerEffort();
	for (unsigned int index = 0; index < efforts->getNumberOfElements(); index++) {
		unsigned char stabilizer_id = efforts->getElement(index)->getStabilizerID();
		rosmsg.name.push_back(p_stabilizer[stabilizer_id]);
		double effort = efforts->getElement(index)->getEffort();
		rosmsg.velocity.push_back((int)(effort * 100) / 100.);
	}
	p_pub_jointstates->publish(rosmsg);

}

void StabilizerDriverClient_ReceiveFSM::reportStabilizerPositionAction(ReportStabilizerPosition msg, Receive::Body::ReceiveRec transportData)
{
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	RCLCPP_DEBUG(logger, "reportStabilizerPositionAction from %d.%d.%d",
					subsystem_id, node_id, component_id);
	// publish to ROS
	auto rosmsg = sensor_msgs::msg::JointState();
	rosmsg.header.stamp = cmp->now();
	ReportStabilizerPosition::Body::StabilizerPosition *efforts = msg.getBody()->getStabilizerPosition();
	for (unsigned int index = 0; index < efforts->getNumberOfElements(); index++) {
		unsigned char stabilizer_id = efforts->getElement(index)->getStabilizerID();
		rosmsg.name.push_back(p_stabilizer[stabilizer_id]);
		double position = efforts->getElement(index)->getPosition();
		rosmsg.position.push_back((int)(position * 100) / 100.);
	}
	p_pub_jointstates->publish(rosmsg);
}


void StabilizerDriverClient_ReceiveFSM::pRosCmdJointState(const sensor_msgs::msg::JointState::SharedPtr joint_state)
{
	if (has_remote_addr()) {
		bool has_position = false;
		SetStabilizerEffort msg_effort;
		SetStabilizerPosition msg_position;
		std::map<unsigned char, std::string>::iterator st_it;
		for (st_it = p_stabilizer.begin(); st_it != p_stabilizer.end(); ++st_it) {
			int jname_idx = getNameIndexFromJointState(joint_state, st_it->second);
			if (jname_idx > -1) {
				if (jname_idx < joint_state->position.size()) {
					has_position = true;
					SetStabilizerPosition::Body::StabilizerPosition::StabilizerPositionRec pos;
					pos.setStabilizerID(st_it->first);
					pos.setPosition(joint_state->position[jname_idx]);
					msg_position.getBody()->getStabilizerPosition()->addElement(pos);
				}
				if (jname_idx < joint_state->velocity.size()) {
					has_position = true;
					SetStabilizerEffort::Body::StabilizerEffort::StabilizerEffortRec effort;
					effort.setStabilizerID(st_it->first);
					effort.setEffort(joint_state->velocity[jname_idx]);
					msg_effort.getBody()->getStabilizerEffort()->addElement(effort);
				}
			}
		}
		if (has_position) {
			sendJausMessage(msg_position, p_remote_addr);
			// the position is reported by an event
		} else {
			// request QueryStabilizerCapabilities
			QueryStabilizerCapabilities query_cap_msg;
			sendJausMessage(query_cap_msg, p_remote_addr);
		}
	}
}

void StabilizerDriverClient_ReceiveFSM::pRosCmdVelocity(const std_msgs::msg::Float64MultiArray::SharedPtr cmd_vel)
{
	if (has_remote_addr()) {
		SetStabilizerEffort msg;
		for (unsigned int index = 0; index < cmd_vel->data.size(); index++) {
			std::map<unsigned char, std::string>::iterator it;
			it = p_stabilizer.find(index);
			if (it != p_stabilizer.end()) {
				SetStabilizerEffort::Body::StabilizerEffort::StabilizerEffortRec value;
				value.setStabilizerID(index);
				if (index < cmd_vel->data.size()) {
					// joint name in message found, set the velocity value
					double vel = cmd_vel->data[index];
					// TODO: make vel relative; we need the max and min values of stabilizer capabilities
					value.setEffort(vel);
				} else {
					// not in list -> set to zero
					value.setEffort(0.);
				}
				msg.getBody()->getStabilizerEffort()->addElement(value);
			}
		}
		sendJausMessage(msg, p_remote_addr);
//      // request QueryJointEffort
//      QueryStabilizerEffort query;
//      sendJausMessage(query, p_remote_addr);
	}
}

void StabilizerDriverClient_ReceiveFSM::event(JausAddress sender, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata)
{
	ReportStabilizerPosition report;
	report.decode(reportdata);
	Receive::Body::ReceiveRec transport_data;
	transport_data.setSrcSubsystemID(sender.getSubsystemID());
	transport_data.setSrcNodeID(sender.getNodeID());
	transport_data.setSrcComponentID(sender.getComponentID());
	reportStabilizerPositionAction(report, transport_data);
}

int StabilizerDriverClient_ReceiveFSM::getNameIndexFromJointState(const sensor_msgs::msg::JointState::SharedPtr joint_state, std::string name)
{
	int result = -1;
	for (unsigned int index = 0; index < joint_state->name.size(); index++) {
		if (joint_state->name[index].compare(name) == 0) {
			result = index;
			break;
		}
	}
	return result;
}



}
