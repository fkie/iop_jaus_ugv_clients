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

#include <fkie_iop_ocu_slavelib/Slave.h>
#include <fkie_iop_component/iop_config.h>

#include "urn_jaus_jss_ugv_StabilizerDriverClient/StabilizerDriverClient_ReceiveFSM.h"

#include <ros/console.h>
#include <JausUtils.h>
#include <algorithm> // for std::find
#include <boost/algorithm/string.hpp>
#include <iterator>
#include <stdlib.h>
#include <string>     // std::string, std::stoi


using namespace JTS;
using namespace iop::ocu;


namespace urn_jaus_jss_ugv_StabilizerDriverClient
{



StabilizerDriverClient_ReceiveFSM::StabilizerDriverClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new StabilizerDriverClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pManagementClient_ReceiveFSM = pManagementClient_ReceiveFSM;
	p_has_access = false;
	p_by_query = false;
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
	iop::Config cfg("~StabilizerDriverClient");
	cfg.param("hz", p_hz, p_hz, false, false);
	// get the ROS parameter
	p_names.clear();
	p_stabilizer.clear();
	p_efforts.clear();
	p_positions.clear();
	XmlRpc::XmlRpcValue v;
	cfg.param("joint_names", v, v);
	for(unsigned int i = 0; i < v.size(); i++) {
		p_names.push_back(v[i]);
	}
	// subscribe to ROS joint state commands
	p_sub_jointstates = cfg.subscribe<sensor_msgs::JointState>("cmd_joint_states", 1, &StabilizerDriverClient_ReceiveFSM::pRosCmdJointState, this);
	p_sub_cmd_vel = cfg.subscribe<std_msgs::Float64MultiArray>("flipper_velocity_controller/command", 1, &StabilizerDriverClient_ReceiveFSM::pRosCmdVelocity, this);
	p_pub_jointstates = cfg.advertise<sensor_msgs::JointState>("joint_states", 1, true);

	// initialize the control layer, which handles the access control staff
	Slave &slave = Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:ugv:StabilizerDriver", 1, 0);
}

void StabilizerDriverClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:ugv:StabilizerDriver") == 0) {
		p_remote_addr = component;
		p_has_access = true;
	} else {
		ROS_WARN_STREAM("[StabilizerDriverClient] unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void StabilizerDriverClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	p_remote_addr = component;
}

void StabilizerDriverClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_has_access = false;
	p_remote_addr = JausAddress(0);
}

void StabilizerDriverClient_ReceiveFSM::create_events(std::string service_uri, JausAddress component, bool by_query)
{
	p_by_query = by_query;
	p_valid_capabilities = false;
	p_query_timer = p_nh.createTimer(ros::Duration(1), &StabilizerDriverClient_ReceiveFSM::pQueryCallback, this);
}

void StabilizerDriverClient_ReceiveFSM::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
	p_query_timer.stop();
	if (!by_query) {
		ROS_INFO_NAMED("StabilizerDriverClient", "cancel EVENT for stabilizer position by %d.%d.%d",
				component.getSubsystemID(), component.getNodeID(), component.getComponentID());
		pEventsClient_ReceiveFSM->cancel_event(*this, component, p_query_position);
	}
	p_valid_capabilities = false;
	p_stabilizer.clear();
	p_efforts.clear();
	p_positions.clear();
}

void StabilizerDriverClient_ReceiveFSM::pQueryCallback(const ros::TimerEvent& event)
{
	if (p_remote_addr.get() != 0) {
		if (!p_valid_capabilities) {
			QueryStabilizerCapabilities query_cap_msg;
			sendJausMessage(query_cap_msg, p_remote_addr);
		} else {
			sendJausMessage(p_query_position, p_remote_addr);
		}
	}
}

void StabilizerDriverClient_ReceiveFSM::reportStabilizerCapabilitiesAction(ReportStabilizerCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	ROS_DEBUG_NAMED("StabilizerDriverClient", "reportStabilizerCapabilitiesAction from %d.%d.%d",
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
	sensor_msgs::JointState rosmsg;
	rosmsg.header.stamp = ros::Time::now();
	std::map<unsigned char, std::string>::iterator st_it;
	for (st_it = p_stabilizer.begin(); st_it != p_stabilizer.end(); ++st_it) {
		rosmsg.name.push_back(st_it->second);
		rosmsg.velocity.push_back(0.);
		rosmsg.position.push_back(0.);
	}
	p_pub_jointstates.publish(rosmsg);
	p_valid_capabilities = true;
	// create a new query message with all id
	p_query_position = QueryStabilizerPosition();
	for (std::map<unsigned char, std::string>::iterator it = p_stabilizer.begin(); it != p_stabilizer.end(); it++) {
		QueryStabilizerPosition::Body::StabilizerID::QueryStabilizerRec st_rec;
		st_rec.setStabilizerID(it->first);
		p_query_position.getBody()->getStabilizerID()->addElement(st_rec);
	}
	// create event or timer for queries
	if (p_remote_addr.get() != 0) {
		if (p_by_query) {
			p_query_timer.stop();
			if (p_hz > 0) {
				ROS_INFO_NAMED("StabilizerDriverClient", "create QUERY timer to get stabilizer position from %s", p_remote_addr.str().c_str());
				p_query_timer = p_nh.createTimer(ros::Duration(1.0 / p_hz), &StabilizerDriverClient_ReceiveFSM::pQueryCallback, this);
			} else {
				ROS_WARN_NAMED("StabilizerDriverClient", "invalid hz %.2f for QUERY timer to get stabilizer position from %s", p_hz, p_remote_addr.str().c_str());
			}
		} else {
			ROS_INFO_NAMED("StabilizerDriverClient", "create EVENT to get stabilizer position from %s", p_remote_addr.str().c_str());
			pEventsClient_ReceiveFSM->create_event(*this, p_remote_addr, p_query_position, p_hz);
		}
	}
}

void StabilizerDriverClient_ReceiveFSM::reportStabilizerEffortAction(ReportStabilizerEffort msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	ROS_DEBUG_NAMED("StabilizerDriverClient", "reportStabilizerEffortAction from %d.%d.%d",
					subsystem_id, node_id, component_id);
	sensor_msgs::JointState rosmsg;
	rosmsg.header.stamp = ros::Time::now();
	ReportStabilizerEffort::Body::StabilizerEffort *efforts = msg.getBody()->getStabilizerEffort();
	for (unsigned int index = 0; index < efforts->getNumberOfElements(); index++) {
		unsigned char stabilizer_id = efforts->getElement(index)->getStabilizerID();
		rosmsg.name.push_back(p_stabilizer[stabilizer_id]);
		double effort = efforts->getElement(index)->getEffort();
		rosmsg.velocity.push_back((int)(effort * 100) / 100.);
	}
	p_pub_jointstates.publish(rosmsg);

}

void StabilizerDriverClient_ReceiveFSM::reportStabilizerPositionAction(ReportStabilizerPosition msg, Receive::Body::ReceiveRec transportData)
{
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	ROS_DEBUG_NAMED("StabilizerDriverClient", "reportStabilizerPositionAction from %d.%d.%d",
					subsystem_id, node_id, component_id);
	// publish to ROS
	sensor_msgs::JointState rosmsg;
	rosmsg.header.stamp = ros::Time::now();
	ReportStabilizerPosition::Body::StabilizerPosition *efforts = msg.getBody()->getStabilizerPosition();
	for (unsigned int index = 0; index < efforts->getNumberOfElements(); index++) {
		unsigned char stabilizer_id = efforts->getElement(index)->getStabilizerID();
		rosmsg.name.push_back(p_stabilizer[stabilizer_id]);
		double position = efforts->getElement(index)->getPosition();
		rosmsg.position.push_back((int)(position * 100) / 100.);
	}
	p_pub_jointstates.publish(rosmsg);
}


void StabilizerDriverClient_ReceiveFSM::pRosCmdJointState(const sensor_msgs::JointState::ConstPtr& joint_state)
{
	if (p_remote_addr.get() != 0) {
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

void StabilizerDriverClient_ReceiveFSM::pRosCmdVelocity(const std_msgs::Float64MultiArray::ConstPtr& cmd_vel)
{
	if (p_remote_addr.get() != 0) {
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

int StabilizerDriverClient_ReceiveFSM::getNameIndexFromJointState(const sensor_msgs::JointState::ConstPtr& joint_state, std::string name)
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



};
