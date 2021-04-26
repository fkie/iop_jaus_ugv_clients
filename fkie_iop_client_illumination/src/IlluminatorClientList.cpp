/**
ROS/IOP Bridge
Copyright (c) 2018 Fraunhofer

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

#include <diagnostic_msgs/msg/key_value.hpp>
#include <fkie_iop_client_illumination/IlluminatorClientList.h>
#include <fkie_iop_component/iop_config.hpp>

using namespace iop;


IlluminatorClientList::IlluminatorClientList(std::shared_ptr<iop::Component> cmp)
{
	this->cmp = cmp;
	p_initialized = false;
	iop::Config cfg(cmp, "IlluminationClient");
	p_pub_diagnostic = cfg.create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(std::string("illuminator_states"), 10);
}

IlluminatorClientList::~IlluminatorClientList()
{
	p_clear_map();
}

void IlluminatorClientList::apply_configuration_report(urn_jaus_jss_ugv_IlluminationServiceClient::ReportIlluminationConfiguration config)
{
	lock_type lock(p_mutex);
	p_clear_map();
	bool supported = config.getBody()->getIlluminatorTypes()->getTypes()->getHeadlights() > 0;
	p_illuminator_map["Headlights"] = new IlluminatorClient(cmp, supported, "Headlights");
	p_illuminator_map["Headlights"]->set_cmd_callback(&IlluminatorClientList::p_illuminator_cmd_callback, this);
	supported = config.getBody()->getIlluminatorTypes()->getTypes()->getLeftTurnSignal() > 0;
	p_illuminator_map["LeftTurnSignal"]= new IlluminatorClient(cmp, supported, "LeftTurnSignal");
	p_illuminator_map["LeftTurnSignal"]->set_cmd_callback(&IlluminatorClientList::p_illuminator_cmd_callback, this);
	supported = config.getBody()->getIlluminatorTypes()->getTypes()->getRightTurnSignal() > 0;
	p_illuminator_map["RightTurnSignal"] = new IlluminatorClient(cmp, supported, "RightTurnSignal");
	p_illuminator_map["RightTurnSignal"]->set_cmd_callback(&IlluminatorClientList::p_illuminator_cmd_callback, this);
	supported = config.getBody()->getIlluminatorTypes()->getTypes()->getRunningLights() > 0;
	p_illuminator_map["RunningLights"] = new IlluminatorClient(cmp, supported, "RunningLights");
	p_illuminator_map["RunningLights"]->set_cmd_callback(&IlluminatorClientList::p_illuminator_cmd_callback, this);
	supported = config.getBody()->getIlluminatorTypes()->getTypes()->getBrakeLights() > 0;
	p_illuminator_map["BrakeLights"] = new IlluminatorClient(cmp, supported, "BrakeLights");
	p_illuminator_map["BrakeLights"]->set_cmd_callback(&IlluminatorClientList::p_illuminator_cmd_callback, this);
	supported = config.getBody()->getIlluminatorTypes()->getTypes()->getBackupLights() > 0;
	p_illuminator_map["BackupLights"] = new IlluminatorClient(cmp, supported, "BackupLights");
	p_illuminator_map["BackupLights"]->set_cmd_callback(&IlluminatorClientList::p_illuminator_cmd_callback, this);
	supported = config.getBody()->getIlluminatorTypes()->getTypes()->getVisibleLightSource() > 0;
	p_illuminator_map["VisibleLightSource"] = new IlluminatorClient(cmp, supported, "VisibleLightSource");
	p_illuminator_map["VisibleLightSource"]->set_cmd_callback(&IlluminatorClientList::p_illuminator_cmd_callback, this);
	supported = config.getBody()->getIlluminatorTypes()->getTypes()->getIRLightSource() > 0;
	p_illuminator_map["IRLightSource"] = new IlluminatorClient(cmp, supported, "IRLightSource");
	p_illuminator_map["IRLightSource"]->set_cmd_callback(&IlluminatorClientList::p_illuminator_cmd_callback, this);
	supported = config.getBody()->getIlluminatorTypes()->getTypes()->getVariableLight1() > 0;
	p_illuminator_map["VariableLight1"] = new IlluminatorClient(cmp, supported, "VariableLight1");
	p_illuminator_map["VariableLight1"]->set_cmd_callback(&IlluminatorClientList::p_illuminator_cmd_callback, this);
	supported = config.getBody()->getIlluminatorTypes()->getTypes()->getVariableLight2() > 0;
	p_illuminator_map["VariableLight2"] = new IlluminatorClient(cmp, supported, "VariableLight2");
	p_illuminator_map["VariableLight2"]->set_cmd_callback(&IlluminatorClientList::p_illuminator_cmd_callback, this);
	supported = config.getBody()->getIlluminatorTypes()->getTypes()->getVariableLight3() > 0;
	p_illuminator_map["VariableLight3"] = new IlluminatorClient(cmp, supported, "VariableLight3");
	p_illuminator_map["VariableLight3"]->set_cmd_callback(&IlluminatorClientList::p_illuminator_cmd_callback, this);
	supported = config.getBody()->getIlluminatorTypes()->getTypes()->getVariableLight4() > 0;
	p_illuminator_map["VariableLight4"] = new IlluminatorClient(cmp, supported, "VariableLight4");
	p_illuminator_map["VariableLight4"]->set_cmd_callback(&IlluminatorClientList::p_illuminator_cmd_callback, this);
	supported = config.getBody()->getIlluminatorTypes()->getTypes()->getHighBeams() > 0;
	p_illuminator_map["HighBeams"] = new IlluminatorClient(cmp, supported, "HighBeams");
	p_illuminator_map["HighBeams"]->set_cmd_callback(&IlluminatorClientList::p_illuminator_cmd_callback, this);
	supported = config.getBody()->getIlluminatorTypes()->getTypes()->getParkingLights() > 0;
	p_illuminator_map["ParkingLights"] = new IlluminatorClient(cmp, supported, "ParkingLights");
	p_illuminator_map["ParkingLights"]->set_cmd_callback(&IlluminatorClientList::p_illuminator_cmd_callback, this);
	supported = config.getBody()->getIlluminatorTypes()->getTypes()->getFogLights() > 0;
	p_illuminator_map["FogLights"] = new IlluminatorClient(cmp, supported, "FogLights");
	p_illuminator_map["FogLights"]->set_cmd_callback(&IlluminatorClientList::p_illuminator_cmd_callback, this);
	supported = config.getBody()->getIlluminatorTypes()->getTypes()->getHazardLights() > 0;
	p_illuminator_map["HazardLights"] = new IlluminatorClient(cmp, supported, "HazardLights");
	p_illuminator_map["HazardLights"]->set_cmd_callback(&IlluminatorClientList::p_illuminator_cmd_callback, this);
	p_initialized = true;
}

void IlluminatorClientList::p_clear_map() {
	lock_type lock(p_mutex);
	std::map<std::string, iop::IlluminatorClient*>::iterator it;
	for (it = p_illuminator_map.begin(); it != p_illuminator_map.end(); ++it) {
		delete it->second;
	}
	p_illuminator_map.clear();
}

void IlluminatorClientList::apply_state_report(urn_jaus_jss_ugv_IlluminationServiceClient::ReportIlluminationState states)
{
	lock_type lock(p_mutex);
	urn_jaus_jss_ugv_IlluminationServiceClient::ReportIlluminationState report;
	if (p_initialized) {
		urn_jaus_jss_ugv_IlluminationServiceClient::ReportIlluminationState::body::illuminationRec::illumination &state = *(states.getBody()->getIlluminationRec()->getIllumination());
		p_illuminator_map["Headlights"]->set_state(state.getHeadlights());
		p_illuminator_map["LeftTurnSignal"]->set_state(state.getLeftTurnSignal());
		p_illuminator_map["RightTurnSignal"]->set_state(state.getRightTurnSignal());
		p_illuminator_map["RunningLights"]->set_state(state.getRunningLights());
		p_illuminator_map["BrakeLights"]->set_state(state.getBrakeLights());
		p_illuminator_map["BackupLights"]->set_state(state.getBackupLights());
		p_illuminator_map["VisibleLightSource"]->set_state(state.getVisibleLightSource());
		p_illuminator_map["IRLightSource"]->set_state(state.getIRLightSource());
		p_illuminator_map["VariableLight1"]->set_state(state.getVariableLight1());
		p_illuminator_map["VariableLight2"]->set_state(state.getVariableLight2());
		p_illuminator_map["VariableLight3"]->set_state(state.getVariableLight3());
		p_illuminator_map["VariableLight4"]->set_state(state.getVariableLight4());
		p_illuminator_map["HighBeams"]->set_state(state.getHighBeams());
		p_illuminator_map["ParkingLights"]->set_state(state.getParkingLights());
		p_illuminator_map["FogLights"]->set_state(state.getFogLights());
		p_illuminator_map["HazardLights"]->set_state(state.getHazardLights());
		auto ros_msg = diagnostic_msgs::msg::DiagnosticStatus();
		ros_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
		std::map<std::string, iop::IlluminatorClient*>::iterator it;
		for (it = p_illuminator_map.begin(); it != p_illuminator_map.end(); ++it) {
			if (it->second->is_supported()) {
				auto entry = diagnostic_msgs::msg::KeyValue();
				entry.key = it->second->get_ros_key();
				entry.value = it->second->get_state_str();
				ros_msg.values.push_back(entry);
			}
		}
		p_pub_diagnostic->publish(ros_msg);
	}
}

urn_jaus_jss_ugv_IlluminationServiceClient::SetIlluminationState IlluminatorClientList::get_state_cmd()
{
	lock_type lock(p_mutex);
	urn_jaus_jss_ugv_IlluminationServiceClient::SetIlluminationState cmd;
	if (p_initialized) {
		cmd.getBody()->getIlluminationRec()->getIllumination()->setHeadlights(p_get_illuminator_state("Headlights"));
		cmd.getBody()->getIlluminationRec()->getIllumination()->setLeftTurnSignal(p_get_illuminator_state("LeftTurnSignal"));
		cmd.getBody()->getIlluminationRec()->getIllumination()->setRightTurnSignal(p_get_illuminator_state("RightTurnSignal"));
		cmd.getBody()->getIlluminationRec()->getIllumination()->setRunningLights(p_get_illuminator_state("RunningLights"));
		cmd.getBody()->getIlluminationRec()->getIllumination()->setBrakeLights(p_get_illuminator_state("BrakeLights"));
		cmd.getBody()->getIlluminationRec()->getIllumination()->setBackupLights(p_get_illuminator_state("BackupLights"));
		cmd.getBody()->getIlluminationRec()->getIllumination()->setVisibleLightSource(p_get_illuminator_state("VisibleLightSource"));
		cmd.getBody()->getIlluminationRec()->getIllumination()->setIRLightSource(p_get_illuminator_state("IRLightSource"));
		cmd.getBody()->getIlluminationRec()->getIllumination()->setVariableLight1(p_get_illuminator_state("VariableLight1"));
		cmd.getBody()->getIlluminationRec()->getIllumination()->setVariableLight2(p_get_illuminator_state("VariableLight2"));
		cmd.getBody()->getIlluminationRec()->getIllumination()->setVariableLight3(p_get_illuminator_state("VariableLight3"));
		cmd.getBody()->getIlluminationRec()->getIllumination()->setVariableLight4(p_get_illuminator_state("VariableLight4"));
		cmd.getBody()->getIlluminationRec()->getIllumination()->setHighBeams(p_get_illuminator_state("HighBeams"));
		cmd.getBody()->getIlluminationRec()->getIllumination()->setParkingLights(p_get_illuminator_state("ParkingLights"));
		cmd.getBody()->getIlluminationRec()->getIllumination()->setFogLights(p_get_illuminator_state("FogLights"));
		cmd.getBody()->getIlluminationRec()->getIllumination()->setHazardLights(p_get_illuminator_state("HazardLights"));
	}
	return cmd;
}

void IlluminatorClientList::p_illuminator_cmd_callback(std::string iop_key, bool state)
{
	lock_type lock(p_mutex);
	if (p_cmd_callback) {
		p_cmd_callback(get_state_cmd());
	}
}

jUnsignedInteger IlluminatorClientList::p_get_illuminator_state(std::string iop_key)
{
	jUnsignedInteger result = 0;
	iop::IlluminatorClient *il = p_illuminator_map[iop_key];
	if (il->is_supported() && il->get_state()) {
		result = 1;
	}
	return result;
}
