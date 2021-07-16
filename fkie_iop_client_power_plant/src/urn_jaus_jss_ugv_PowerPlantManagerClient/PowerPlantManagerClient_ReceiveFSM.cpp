

#include "urn_jaus_jss_ugv_PowerPlantManagerClient/PowerPlantManagerClient_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>



using namespace JTS;
using namespace iop::ocu;

namespace urn_jaus_jss_ugv_PowerPlantManagerClient
{



PowerPlantManagerClient_ReceiveFSM::PowerPlantManagerClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: SlaveHandlerInterface(cmp, "PowerPlantManagerClient", 1.0),
  logger(cmp->get_logger().get_child("PowerPlantManagerClient"))
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new PowerPlantManagerClient_ReceiveFSMContext(*this);

	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
	p_valid_configuration = false;
	p_hz = 1.0;
	p_battery_valid = false;
	p_battery_id = 0;
}



PowerPlantManagerClient_ReceiveFSM::~PowerPlantManagerClient_ReceiveFSM()
{
	delete context;
}

void PowerPlantManagerClient_ReceiveFSM::setupNotifications()
{
	pAccessControlClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_PowerPlantManagerClient_ReceiveFSM_Receiving_Ready", "AccessControlClient_ReceiveFSM");
	pAccessControlClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_PowerPlantManagerClient_ReceiveFSM_Receiving_Ready", "AccessControlClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving_Ready", "PowerPlantManagerClient_ReceiveFSM");
	registerNotification("Receiving", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving", "PowerPlantManagerClient_ReceiveFSM");
}


void PowerPlantManagerClient_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "PowerPlantManagerClient");
	cfg.declare_param<double>("hz", p_hz, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"Sets how often the reports are requested. If use_queries is True hz must be greather then 0. In this case each time a Query message is sent to get a report. If use_queries is False an event is created to get Reports. In this case 0 disables the rate and an event of type on_change will be created.",
		"Default: 1.0");
	cfg.param("hz", p_hz, p_hz, false);
	// initialize the control layer, which handles the access control staff
	this->set_rate(p_hz);
	this->set_supported_service(*this, "urn:jaus:jss:ugv:PowerPlantManager", 1, 0);
	this->set_event_name("powerplant capabilities");
	this->set_query_before_event(true, 1.0);
}

void PowerPlantManagerClient_ReceiveFSM::register_events(JausAddress remote_addr, double hz)
{
	pEventsClient_ReceiveFSM->create_event(*this, remote_addr, p_query_power_plant_status, p_hz);
}

void PowerPlantManagerClient_ReceiveFSM::unregister_events(JausAddress remote_addr)
{
	pEventsClient_ReceiveFSM->cancel_event(*this, remote_addr, p_query_power_plant_status);
	stop_query(remote_addr);
}

void PowerPlantManagerClient_ReceiveFSM::send_query(JausAddress remote_addr)
{
	if (!p_valid_configuration) {
		QueryPowerPlantCapabilities query_cap_msg;
		sendJausMessage(query_cap_msg, remote_addr);
	} else {
		sendJausMessage(p_query_power_plant_status, remote_addr);
	}
}

void PowerPlantManagerClient_ReceiveFSM::stop_query(JausAddress remote_addr)
{
	p_valid_configuration = false;
	p_battery_valid = false;
	this->set_event_name("powerplant capabilities");
	this->set_query_before_event(true, 1.0);
}

void PowerPlantManagerClient_ReceiveFSM::handleReportPowerPlantCapabilitiesAction(ReportPowerPlantCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	p_valid_configuration = true;
	unsigned int cap_size = msg.getBody()->getPowerPlantCapabilitiesList()->getNumberOfElements();
	for (unsigned int capidx = 0; capidx < cap_size; capidx++) {
		ReportPowerPlantCapabilities::body::powerPlantCapabilitiesList::powerPlantCapabilitiesSeq* capsec;
		capsec = msg.getBody()->getPowerPlantCapabilitiesList()->getElement(capidx);
		if (capsec->getPowerPlantCapabilitiesVar()->getFieldValue() == 2) {
			// it is battery
			iop::Config cfg(cmp, "PowerPlantManagerClient");
			int ppid = capsec->getPowerPlantDescRec()->getPowerPlantID();
			p_battery_id = ppid;
			std::stringstream ss;
			ss << (int)ppid;
			std::string idstr("powerplant_");
			idstr += ss.str();
			p_battery_valid = true;
			p_pub_battery_voltage = cfg.create_publisher<std_msgs::msg::Float32>(idstr + "/voltage", 5);
			p_pub_battery_capacity_percent = cfg.create_publisher<std_msgs::msg::Int8>(idstr + "/capacity_percent", 5);
			// TODO: implement more then one power plant
			break;
		}
	}
	// create event or timer for queries
	// force event for request sensor data
	this->set_event_name("power plant states");
	this->set_query_before_event(false);
}

void PowerPlantManagerClient_ReceiveFSM::handleReportPowerPlantConfigurationAction(ReportPowerPlantConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	RCLCPP_DEBUG(logger, "handle ReportPowerPlantConfiguration from %s not implemented", p_remote_addr.str().c_str());
}

void PowerPlantManagerClient_ReceiveFSM::handleReportPowerPlantStatusAction(ReportPowerPlantStatus msg, Receive::Body::ReceiveRec transportData)
{
	if (p_battery_valid) {
		unsigned int cap_size = msg.getBody()->getPowerPlantStatusList()->getNumberOfElements();
		for (unsigned int capidx = 0; capidx < cap_size; capidx++) {
			ReportPowerPlantStatus::body::powerPlantStatusList::powerPlantStatus* capsec;
			capsec = msg.getBody()->getPowerPlantStatusList()->getElement(capidx);
			if (capsec->getPowerPlantDescRec()->getPowerPlantID() == p_battery_id) {
				if (capsec->getPowerPlantStatusVar()->getFieldValue() == 2) {
					unsigned int bat_count = capsec->getPowerPlantStatusVar()->getBatteryStatus()->getNumberOfElements();
					for (unsigned int batidx = 0; batidx < bat_count; batidx++) {
						ReportPowerPlantStatus::body::powerPlantStatusList::powerPlantStatus::powerPlantStatusVar::batteryStatus::batteryStatusRec* batrec;
						batrec = capsec->getPowerPlantStatusVar()->getBatteryStatus()->getElement(batidx);
						auto ros_msg_volt = std_msgs::msg::Float32();
						ros_msg_volt.data = batrec->getVoltage();
						p_pub_battery_voltage->publish(ros_msg_volt);
						auto ros_msg_pc = std_msgs::msg::Int8();
						ros_msg_pc.data = batrec->getPercentChargeRemaining();
						p_pub_battery_capacity_percent->publish(ros_msg_pc);
						// TODO: implement more then one power plant
						break;
					}
				}
				break;
			}
		}
	}
}

void PowerPlantManagerClient_ReceiveFSM::event(JausAddress sender, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata)
{
	ReportPowerPlantStatus report;
	report.decode(reportdata);
	Receive::Body::ReceiveRec transport_data;
	transport_data.setSrcSubsystemID(sender.getSubsystemID());
	transport_data.setSrcNodeID(sender.getNodeID());
	transport_data.setSrcComponentID(sender.getComponentID());
	handleReportPowerPlantStatusAction(report, transport_data);
}


}
