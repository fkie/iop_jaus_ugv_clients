

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

#include "IlluminationServiceClient_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>
#include <std_msgs/msg/bool.hpp>
#include <fkie_iop_ocu_slavelib/SlaveHandlerInterface.h>
#include <fkie_iop_events/EventHandlerInterface.h>
#include <fkie_iop_client_illumination/IlluminatorClientList.h>

namespace urn_jaus_jss_ugv_IlluminationServiceClient
{

class DllExport IlluminationServiceClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface, public iop::EventHandlerInterface
{
public:
	IlluminationServiceClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~IlluminationServiceClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

	/// Action Methods
	virtual void handleReportIlluminationConfigurationAction(ReportIlluminationConfiguration msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportIlluminationStateAction(ReportIlluminationState msg, Receive::Body::ReceiveRec transportData);


	/// EventHandlerInterface Methods
	void event(JausAddress reporter, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata);

	/// SlaveHandlerInterface Methods
	void register_events(JausAddress remote_addr, double hz);
	void unregister_events(JausAddress remote_addr);
	void send_query(JausAddress remote_addr);
	void stop_query(JausAddress remote_addr);

	/// Guard Methods



	IlluminationServiceClient_ReceiveFSMContext *context;

protected:

	/// References to parent FSMs
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;

	iop::IlluminatorClientList p_illuminator_list;
	QueryIlluminationState p_query_states;
	bool p_valid_configuration;
	double p_hz;

	void p_cmd_callback(SetIlluminationState cmd);

};

}

#endif // ILLUMINATIONSERVICECLIENT_RECEIVEFSM_H
