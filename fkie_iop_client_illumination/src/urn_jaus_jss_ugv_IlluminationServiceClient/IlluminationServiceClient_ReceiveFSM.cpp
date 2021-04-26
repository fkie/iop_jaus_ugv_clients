

#include "urn_jaus_jss_ugv_IlluminationServiceClient/IlluminationServiceClient_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>
#include <fkie_iop_ocu_slavelib/Slave.h>



using namespace JTS;
using namespace iop::ocu;

namespace urn_jaus_jss_ugv_IlluminationServiceClient
{



IlluminationServiceClient_ReceiveFSM::IlluminationServiceClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: logger(cmp->get_logger().get_child("IlluminationServiceClient")),
  p_query_timer(std::chrono::milliseconds(100), std::bind(&IlluminationServiceClient_ReceiveFSM::pQueryCallback, this), false),
  p_illuminator_list(cmp)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new IlluminationServiceClient_ReceiveFSMContext(*this);

	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
	p_has_access = false;
	p_by_query = false;
	p_valid_configuration = false;
	p_hz = 0.0;
}



IlluminationServiceClient_ReceiveFSM::~IlluminationServiceClient_ReceiveFSM()
{
	delete context;
}

void IlluminationServiceClient_ReceiveFSM::setupNotifications()
{
	pAccessControlClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_IlluminationServiceClient_ReceiveFSM_Receiving_Ready", "AccessControlClient_ReceiveFSM");
	pAccessControlClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_IlluminationServiceClient_ReceiveFSM_Receiving_Ready", "AccessControlClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving_Ready", "IlluminationServiceClient_ReceiveFSM");
	registerNotification("Receiving", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving", "IlluminationServiceClient_ReceiveFSM");

}


void IlluminationServiceClient_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "IlluminationServiceClient");
	cfg.declare_param<double>("hz", p_hz, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"Sets how often the reports are requested. If use_queries is True hz must be greather then 0. In this case each time a Query message is sent to get a report. If use_queries is False an event is created to get Reports. In this case 0 disables the rate and an event of type on_change will be created.",
		"Default: 0.0");
	cfg.param("hz", p_hz, p_hz, false);

	// initialize the control layer, which handles the access control staff
	auto slave = Slave::get_instance(cmp);
	slave->add_supported_service(*this, "urn:jaus:jss:ugv:IlluminationService", 1, 1);
	p_illuminator_list.set_cmd_callback(&IlluminationServiceClient_ReceiveFSM::p_cmd_callback, this);
}

void IlluminationServiceClient_ReceiveFSM::handleReportIlluminationConfigurationAction(ReportIlluminationConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	p_illuminator_list.apply_configuration_report(msg);
	p_valid_configuration = true;
	// create event or timer for queries
	p_query_timer.stop();
	if (p_remote_addr.get() != 0) {
		if (p_by_query) {
			if (p_hz > 0) {
				RCLCPP_INFO(logger, "create QUERY timer to get illumination states from %s", p_remote_addr.str().c_str());
				p_query_timer.set_rate(p_hz);
				p_query_timer.start();
			} else {
				RCLCPP_WARN(logger, "invalid hz %.2f for QUERY timer to get illumination states from %s", p_hz, p_remote_addr.str().c_str());
			}
		} else {
			RCLCPP_INFO(logger, "create EVENT to get illumination states from %s", p_remote_addr.str().c_str());
			pEventsClient_ReceiveFSM->create_event(*this, p_remote_addr, p_query_states, p_hz);
		}
	}
}

void IlluminationServiceClient_ReceiveFSM::handleReportIlluminationStateAction(ReportIlluminationState msg, Receive::Body::ReceiveRec transportData)
{
	RCLCPP_DEBUG(logger, "apply illumination states from %s", transportData.getAddress().str().c_str());
	p_illuminator_list.apply_state_report(msg);
}

void IlluminationServiceClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:ugv:IlluminationService") == 0) {
		p_remote_addr = component;
		p_has_access = true;
	} else {
		RCLCPP_WARN(logger, "unexpected control allowed for %s received, ignored!", service_uri.c_str());
	}
}

void IlluminationServiceClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	p_remote_addr = component;
}

void IlluminationServiceClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_has_access = false;
	p_remote_addr = JausAddress(0);
}

void IlluminationServiceClient_ReceiveFSM::create_events(std::string service_uri, JausAddress component, bool by_query)
{
	p_by_query = by_query;
	p_query_timer.set_rate(1);
	p_query_timer.start();
}

void IlluminationServiceClient_ReceiveFSM::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
	p_query_timer.stop();
	if (!by_query) {
		RCLCPP_INFO(logger, "cancel EVENT for illumination state by %s", component.str().c_str());
		pEventsClient_ReceiveFSM->cancel_event(*this, component, p_query_states);
	}
	p_valid_configuration = false;
	p_illuminator_list.apply_configuration_report(ReportIlluminationConfiguration());
}

void IlluminationServiceClient_ReceiveFSM::pQueryCallback()
{
	if (p_remote_addr.get() != 0) {
		if (!p_valid_configuration) {
			QueryIlluminationConfiguration query_cfg_msg;
			sendJausMessage(query_cfg_msg, p_remote_addr);
		} else {
			sendJausMessage(p_query_states, p_remote_addr);
		}
	}
}

void IlluminationServiceClient_ReceiveFSM::event(JausAddress sender, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata)
{
	ReportIlluminationState report;
	report.decode(reportdata);
	Receive::Body::ReceiveRec transport_data;
	transport_data.setSrcSubsystemID(sender.getSubsystemID());
	transport_data.setSrcNodeID(sender.getNodeID());
	transport_data.setSrcComponentID(sender.getComponentID());
	handleReportIlluminationStateAction(report, transport_data);
}

void IlluminationServiceClient_ReceiveFSM::p_cmd_callback(SetIlluminationState cmd)
{
	if (p_remote_addr.get() != 0 && p_has_access) {
		sendJausMessage(cmd, p_remote_addr);
	}
}

}
