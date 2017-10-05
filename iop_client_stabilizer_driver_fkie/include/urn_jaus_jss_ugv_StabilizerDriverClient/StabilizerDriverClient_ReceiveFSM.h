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

#ifndef STABILIZERDRIVERCLIENT_RECEIVEFSM_H
#define STABILIZERDRIVERCLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_ugv_StabilizerDriverClient/Messages/MessageSet.h"
#include "urn_jaus_jss_ugv_StabilizerDriverClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControlClient/AccessControlClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_ManagementClient/ManagementClient_ReceiveFSM.h"


#include "StabilizerDriverClient_ReceiveFSM_sm.h"

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <vector>
#include <map>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <iop_ocu_slavelib_fkie/SlaveHandlerInterface.h>
#include <iop_events_fkie/EventHandlerInterface.h>


namespace urn_jaus_jss_ugv_StabilizerDriverClient
{

class DllExport StabilizerDriverClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface, public iop::EventHandlerInterface
{
public:
	StabilizerDriverClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM);
	virtual ~StabilizerDriverClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void reportStabilizerCapabilitiesAction(ReportStabilizerCapabilities msg, Receive::Body::ReceiveRec transportData);
	virtual void reportStabilizerEffortAction(ReportStabilizerEffort msg, Receive::Body::ReceiveRec transportData);
	virtual void reportStabilizerPositionAction(ReportStabilizerPosition msg, Receive::Body::ReceiveRec transportData);

	/// EventHandlerInterface Methods
	void event(JausAddress reporter, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata);

	/// SlaveHandlerInterface Methods
	void control_allowed(std::string service_uri, JausAddress component, unsigned char authority);
	void enable_monitoring_only(std::string service_uri, JausAddress component);
	void access_deactivated(std::string service_uri, JausAddress component);
	void create_events(std::string service_uri, JausAddress component, bool by_query=false);
	void cancel_events(std::string service_uri, JausAddress component, bool by_query=false);
	/// Guard Methods

	StabilizerDriverClient_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;
	urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM;

	JausAddress p_remote_addr;
	bool p_has_access;
	ros::NodeHandle p_nh;
	ros::Timer p_query_timer;
	ros::Subscriber p_sub_jointstates;
	ros::Subscriber p_sub_cmd_vel;
	ros::Publisher p_pub_jointstates;
	QueryStabilizerPosition p_query_position;
	QueryStabilizerEffort p_query_effort;
	std::vector<std::string> p_names;
	std::map<unsigned char, std::string> p_stabilizer;
	std::map<unsigned char, float> p_efforts;
	std::map<unsigned char, float> p_positions;
	bool p_by_query;
	bool p_valid_capabilities;

	void pRosCmdJointState(const sensor_msgs::JointState::ConstPtr& joint_state);
	void pRosCmdVelocity(const std_msgs::Float64MultiArray::ConstPtr& cmd_vel);
	int getNameIndexFromJointState(const sensor_msgs::JointState::ConstPtr& joint_state, std::string name);
	void pQueryCallback(const ros::TimerEvent& event);

};

};

#endif // STABILIZERDRIVERCLIENT_RECEIVEFSM_H
