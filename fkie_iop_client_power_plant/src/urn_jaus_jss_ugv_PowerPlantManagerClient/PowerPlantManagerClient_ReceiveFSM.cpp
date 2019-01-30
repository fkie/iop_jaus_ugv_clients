

#include "urn_jaus_jss_ugv_PowerPlantManagerClient/PowerPlantManagerClient_ReceiveFSM.h"

#include <fkie_iop_ocu_slavelib/Slave.h>
#include <fkie_iop_component/iop_config.h>



using namespace JTS;
using namespace iop::ocu;

namespace urn_jaus_jss_ugv_PowerPlantManagerClient
{



PowerPlantManagerClient_ReceiveFSM::PowerPlantManagerClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new PowerPlantManagerClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	p_has_access = false;
	p_by_query = false;
	p_valid_configuration = false;
	p_hz = 0.0;
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
	iop::Config cfg("~PowerPlantManagerClient");
	cfg.param("hz", p_hz, p_hz, false, false);

	// initialize the control layer, which handles the access control staff
	Slave &slave = Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:ugv:PowerPlantManager", 1, 0);
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
			iop::Config cfg("~PowerPlantManagerClient");
			int ppid = capsec->getPowerPlantDescRec()->getPowerPlantID();
			p_battery_id = ppid;
			std::stringstream ss;
			ss << (int)ppid;
			std::string idstr("powerplant_");
			idstr += ss.str();
			p_battery_valid = true;
			p_pub_battery_voltage = cfg.advertise<std_msgs::Float32>(idstr + "/voltage", 5);
			p_pub_battery_capacity_percent = cfg.advertise<std_msgs::Int8>(idstr + "/capacity_percent", 5);
			// TODO: implement more then one power plant
			break;
		}
	}
	// create event or timer for queries
	p_query_timer.stop();
	if (p_remote_addr.get() != 0) {
		if (p_by_query) {
			if (p_hz > 0) {
				ROS_INFO_NAMED("PowerPlantManagerClient", "create QUERY timer to get power plant states from %s", p_remote_addr.str().c_str());
				p_query_timer = p_nh.createTimer(ros::Duration(1.0 / p_hz), &PowerPlantManagerClient_ReceiveFSM::pQueryCallback, this);
			} else {
				ROS_WARN_NAMED("PowerPlantManagerClient", "invalid hz %.2f for QUERY timer to get power plant states from %s", p_hz, p_remote_addr.str().c_str());
			}
		} else {
			ROS_INFO_NAMED("PowerPlantManagerClient", "create EVENT to get power plant states from %s", p_remote_addr.str().c_str());
			pEventsClient_ReceiveFSM->create_event(*this, p_remote_addr, p_query_states, p_hz);
		}
	}
}

void PowerPlantManagerClient_ReceiveFSM::handleReportPowerPlantConfigurationAction(ReportPowerPlantConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	ROS_DEBUG_NAMED("PowerPlantManagerClient", "handle ReportPowerPlantConfiguration from %s not implemented", p_remote_addr.str().c_str());
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
						std_msgs::Float32 ros_msg_volt;
						ros_msg_volt.data = batrec->getVoltage();
						p_pub_battery_voltage.publish(ros_msg_volt);
						std_msgs::Int8 ros_msg_pc;
						ros_msg_pc.data = batrec->getPercentChargeRemaining();
						p_pub_battery_capacity_percent.publish(ros_msg_pc);
						// TODO: implement more then one power plant
						break;
					}
				}
				break;
			}
		}
	}
}

void PowerPlantManagerClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:ugv:PowerPlantManager") == 0) {
		p_remote_addr = component;
		p_has_access = true;
	} else {
		ROS_WARN_STREAM("[PowerPlantManagerClient] unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void PowerPlantManagerClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	p_remote_addr = component;
}

void PowerPlantManagerClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_has_access = false;
	p_remote_addr = JausAddress(0);
}

void PowerPlantManagerClient_ReceiveFSM::create_events(std::string service_uri, JausAddress component, bool by_query)
{
	p_by_query = by_query;
	p_query_timer = p_nh.createTimer(ros::Duration(1), &PowerPlantManagerClient_ReceiveFSM::pQueryCallback, this);
}

void PowerPlantManagerClient_ReceiveFSM::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
	p_query_timer.stop();
	if (!by_query) {
		ROS_INFO_NAMED("IlluminationClient", "cancel EVENT for power plant state by %s", component.str().c_str());
		pEventsClient_ReceiveFSM->cancel_event(*this, component, p_query_states);
	}
	p_valid_configuration = false;
	p_battery_valid = false;
}

void PowerPlantManagerClient_ReceiveFSM::pQueryCallback(const ros::TimerEvent& event)
{
	if (p_remote_addr.get() != 0) {
		if (!p_valid_configuration) {
			QueryPowerPlantCapabilities query_cap_msg;
			sendJausMessage(query_cap_msg, p_remote_addr);
		} else {
			sendJausMessage(p_query_states, p_remote_addr);
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


};
