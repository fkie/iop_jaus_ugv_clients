

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

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <fkie_iop_ocu_slavelib/SlaveHandlerInterface.h>
#include <fkie_iop_events/EventHandlerInterface.h>

#include "PowerPlantManagerClient_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>

namespace urn_jaus_jss_ugv_PowerPlantManagerClient
{

class DllExport PowerPlantManagerClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface, public iop::EventHandlerInterface
{
public:
	PowerPlantManagerClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~PowerPlantManagerClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

	/// Action Methods
	virtual void handleReportPowerPlantCapabilitiesAction(ReportPowerPlantCapabilities msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportPowerPlantConfigurationAction(ReportPowerPlantConfiguration msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportPowerPlantStatusAction(ReportPowerPlantStatus msg, Receive::Body::ReceiveRec transportData);


	/// EventHandlerInterface Methods
	void event(JausAddress reporter, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata);

	/// SlaveHandlerInterface Methods
	void register_events(JausAddress remote_addr, double hz);
	void unregister_events(JausAddress remote_addr);
	void send_query(JausAddress remote_addr);
	void stop_query(JausAddress remote_addr);

	/// Guard Methods



	PowerPlantManagerClient_ReceiveFSMContext *context;

protected:

	/// References to parent FSMs
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;

	QueryPowerPlantStatus p_query_power_plant_status;
	bool p_valid_configuration;
	double p_hz;

	bool p_battery_valid;
	int p_battery_id;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr p_pub_battery_voltage;
	rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr p_pub_battery_capacity_percent;

};

}

#endif // PowerPlantManagerCLIENT_RECEIVEFSM_H
