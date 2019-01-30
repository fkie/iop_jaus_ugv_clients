

#ifndef POWERPLANTMANAGERCLIENT_RECEIVEFSM_H
#define POWERPLANTMANAGERCLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_ugv_PowerPlantManagerClient/Messages/MessageSet.h"
#include "urn_jaus_jss_ugv_PowerPlantManagerClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControlClient/AccessControlClient_ReceiveFSM.h"

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <fkie_iop_ocu_slavelib/SlaveHandlerInterface.h>
#include <fkie_iop_events/EventHandlerInterface.h>

#include "PowerPlantManagerClient_ReceiveFSM_sm.h"

namespace urn_jaus_jss_ugv_PowerPlantManagerClient
{

class DllExport PowerPlantManagerClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface, public iop::EventHandlerInterface
{
public:
	PowerPlantManagerClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM);
	virtual ~PowerPlantManagerClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void handleReportPowerPlantCapabilitiesAction(ReportPowerPlantCapabilities msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportPowerPlantConfigurationAction(ReportPowerPlantConfiguration msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportPowerPlantStatusAction(ReportPowerPlantStatus msg, Receive::Body::ReceiveRec transportData);


	/// EventHandlerInterface Methods
	void event(JausAddress reporter, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata);

	/// SlaveHandlerInterface Methods
	void control_allowed(std::string service_uri, JausAddress component, unsigned char authority);
	void enable_monitoring_only(std::string service_uri, JausAddress component);
	void access_deactivated(std::string service_uri, JausAddress component);
	void create_events(std::string service_uri, JausAddress component, bool by_query=false);
	void cancel_events(std::string service_uri, JausAddress component, bool by_query=false);

	/// Guard Methods



	PowerPlantManagerClient_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;

	QueryPowerPlantStatus p_query_states;
	JausAddress p_remote_addr;
	bool p_has_access;
	ros::NodeHandle p_nh;
	ros::Timer p_query_timer;
	bool p_by_query;
	bool p_valid_configuration;
	double p_hz;

	bool p_battery_valid;
	int p_battery_id;
	ros::Publisher p_pub_battery_voltage;
	ros::Publisher p_pub_battery_capacity_percent;
	void pQueryCallback(const ros::TimerEvent& event);

};

};

#endif // PowerPlantManagerCLIENT_RECEIVEFSM_H
