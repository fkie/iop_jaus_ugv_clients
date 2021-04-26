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
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>
#include <vector>
#include <map>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <fkie_iop_ocu_slavelib/SlaveHandlerInterface.h>
#include <fkie_iop_events/EventHandlerInterface.h>


namespace urn_jaus_jss_ugv_StabilizerDriverClient
{

class DllExport StabilizerDriverClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface, public iop::EventHandlerInterface
{
public:
	StabilizerDriverClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~StabilizerDriverClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

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
	urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM;
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;

	JausAddress p_remote_addr;
	bool p_has_access;
	iop::Timer p_query_timer;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr p_sub_jointstates;
	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr p_sub_cmd_vel;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr p_pub_jointstates;
	QueryStabilizerPosition p_query_position;
	QueryStabilizerEffort p_query_effort;
	std::vector<std::string> p_names;
	std::map<unsigned char, std::string> p_stabilizer;
	std::map<unsigned char, float> p_efforts;
	std::map<unsigned char, float> p_positions;
	bool p_by_query;
	bool p_valid_capabilities;
	double p_hz;

	void pRosCmdJointState(const sensor_msgs::msg::JointState::SharedPtr joint_state);
	void pRosCmdVelocity(const std_msgs::msg::Float64MultiArray::SharedPtr cmd_vel);
	int getNameIndexFromJointState(const sensor_msgs::msg::JointState::SharedPtr joint_state, std::string name);
	void pQueryCallback();

};

}

#endif // STABILIZERDRIVERCLIENT_RECEIVEFSM_H
