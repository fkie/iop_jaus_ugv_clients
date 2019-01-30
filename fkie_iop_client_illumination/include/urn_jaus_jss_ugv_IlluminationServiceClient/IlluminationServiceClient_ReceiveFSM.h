

#ifndef ILLUMINATIONSERVICECLIENT_RECEIVEFSM_H
#define ILLUMINATIONSERVICECLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_ugv_IlluminationServiceClient/Messages/MessageSet.h"
#include "urn_jaus_jss_ugv_IlluminationServiceClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControlClient/AccessControlClient_ReceiveFSM.h"

#include <std_msgs/Bool.h>
#include <fkie_iop_ocu_slavelib/SlaveHandlerInterface.h>
#include <fkie_iop_events/EventHandlerInterface.h>
#include <fkie_iop_client_illumination/IlluminatorClientList.h>

#include "IlluminationServiceClient_ReceiveFSM_sm.h"

namespace urn_jaus_jss_ugv_IlluminationServiceClient
{

class DllExport IlluminationServiceClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface, public iop::EventHandlerInterface
{
public:
	IlluminationServiceClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM);
	virtual ~IlluminationServiceClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void handleReportIlluminationConfigurationAction(ReportIlluminationConfiguration msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportIlluminationStateAction(ReportIlluminationState msg, Receive::Body::ReceiveRec transportData);


	/// EventHandlerInterface Methods
	void event(JausAddress reporter, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata);

	/// SlaveHandlerInterface Methods
	void control_allowed(std::string service_uri, JausAddress component, unsigned char authority);
	void enable_monitoring_only(std::string service_uri, JausAddress component);
	void access_deactivated(std::string service_uri, JausAddress component);
	void create_events(std::string service_uri, JausAddress component, bool by_query=false);
	void cancel_events(std::string service_uri, JausAddress component, bool by_query=false);

	/// Guard Methods



	IlluminationServiceClient_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;

	iop::IlluminatorClientList p_illuminator_list;
	QueryIlluminationState p_query_states;
	JausAddress p_remote_addr;
	bool p_has_access;
	ros::NodeHandle p_nh;
	ros::Timer p_query_timer;
	bool p_by_query;
	bool p_valid_configuration;
	double p_hz;

	void pQueryCallback(const ros::TimerEvent& event);
	void p_cmd_callback(SetIlluminationState cmd);

};

};

#endif // ILLUMINATIONSERVICECLIENT_RECEIVEFSM_H
