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


#include <fkie_iop_client_illumination/IlluminatorClient.h>
#include <fkie_iop_component/iop_config.h>

using namespace iop;

std::map<std::string, std::string> make_iop_ros_map() {
	std::map<std::string, std::string> result;
	result["Headlights"] = "head_lights";
	result["LeftTurnSignal"]= "left_turn_signal";
	result["RightTurnSignal"] = "right_turn_signal";
	result["RunningLights"] = "running_lights";
	result["BrakeLights"] = "brake_lights";
	result["BackupLights"] = "backup_lights";
	result["VisibleLightSource"] = "visible_light_source";
	result["IRLightSource"] = "ir_light_source";
	result["VariableLight1"] = "variable_light_1";
	result["VariableLight2"] = "variable_light_2";
	result["VariableLight3"] = "variable_light_3";
	result["VariableLight4"] = "variable_light_4";
	result["HighBeams"] = "high_beams";
	result["ParkingLights"] = "parking_lights";
	result["FogLights"] = "fog_lights";
	result["HazardLights"] = "hazard_lights";
	return result;
}

std::map<std::string, std::string> make_ros_iop_map() {
	std::map<std::string, std::string> result;
	result["head_lights"] = "Headlights";
	result["left_turn_signal"]= "LeftTurnSignal";
	result["right_turn_signal"] = "RightTurnSignal";
	result["running_lights"] = "RunningLights";
	result["brake_lights"] = "BrakeLights";
	result["backup_lights"] = "BackupLights";
	result["visible_light_source"] = "VisibleLightSource";
	result["ir_light_source"] = "IRLightSource";
	result["variable_light_1"] = "VariableLight1";
	result["variable_light_2"] = "VariableLight2";
	result["variable_light_3"] = "VariableLight3";
	result["variable_light_4"] = "VariableLight4";
	result["high_beams"] = "HighBeams";
	result["parking_lights"] = "ParkingLights";
	result["fog_lights"] = "FogLights";
	result["hazard_lights"] = "HazardLights";
	return result;
}

std::map<std::string, std::string> IlluminatorClient::p_iop_ros_map = make_iop_ros_map();
std::map<std::string, std::string> IlluminatorClient::p_ros_iop_map = make_ros_iop_map();

IlluminatorClient::IlluminatorClient(){
	p_supported = false;
	p_state = false;
	p_state_str = "OFF";
}

IlluminatorClient::IlluminatorClient(bool supported, std::string iop_key)
{
	p_supported = supported;
	p_state = false;
	p_state_str = "OFF";
	if (supported) {
		try {
			p_iop_key = iop_key;
			p_ros_key = get_ros_key(iop_key);
			iop::Config cfg("~IlluminationClient");
			p_sub_cmd = cfg.subscribe<std_msgs::Bool>(std::string("illuminator/cmd_") + p_ros_key, 10, &IlluminatorClient::p_ros_cmd_callback, this);
			p_pub_state = cfg.advertise<std_msgs::Bool>(std::string("illuminator/") + p_ros_key, 10, true);
		} catch (std::exception &e) {
			p_supported = false;
			ROS_WARN_NAMED("IlluminationClient", "init of illumination '%s' failed: %s", iop_key.c_str(), e.what());
		}
	}
}

IlluminatorClient::~IlluminatorClient()
{
	p_cmd_callback.clear();
}

void IlluminatorClient::init(std::string ros_key, std::string state, std::string diagnostic_key)
{
	init(ros_key, state.compare("ON") == 0, diagnostic_key);
}

void IlluminatorClient::init(std::string ros_key, bool state, std::string diagnostic_key)
{
	try {
		p_iop_key = get_iop_key(ros_key);
		p_ros_key = ros_key;
		p_diagnostic_key = diagnostic_key;
		iop::Config cfg("~IlluminationClient");
		p_sub_cmd = cfg.subscribe<std_msgs::Bool>(std::string("illuminator/cmd_") + p_ros_key, 10, &IlluminatorClient::p_ros_cmd_callback, this);
		p_pub_state = cfg.advertise<std_msgs::Bool>(std::string("illuminator/") + p_ros_key, 10, true);
		p_supported = true;
	} catch (std::exception &e) {
		ROS_WARN_NAMED("IlluminationClient", "init of illumination '%s' failed: %s", ros_key.c_str(), e.what());
	}
}

bool IlluminatorClient::is_supported()
{
	return p_supported;
}

bool IlluminatorClient::get_state()
{
	return p_state;
}

std::string IlluminatorClient::get_state_str()
{
	return p_state_str;
}

bool IlluminatorClient::set_state(bool state)
{
	bool result = false;
	if (is_supported()) {
		result = true;
		p_state = state;
		std_msgs::Bool msg;
		msg.data = state;
		p_pub_state.publish(msg);
		if (p_state) {
			p_state_str = "ON";
		} else {
			p_state_str = "OFF";
		}
	}
	return result;
}

std::string IlluminatorClient::get_iop_key(std::string ros_key)
{
	std::map<std::string, std::string>::iterator it = p_ros_iop_map.find(ros_key);
	if (it != p_ros_iop_map.end()) {
		return it->second;
	}
	throw std::invalid_argument(std::string("invalid ros key for illuminator, got: ") + ros_key.c_str());
}

std::string IlluminatorClient::get_ros_key(std::string iop_key)
{
	std::map<std::string, std::string>::iterator it = p_iop_ros_map.find(iop_key);
	if (it != p_iop_ros_map.end()) {
		return it->second;
	}
	throw std::invalid_argument(std::string("invalid iop key for illuminator, got: ") + iop_key.c_str());
}

bool IlluminatorClient::operator==(IlluminatorClient &value)
{
	return (p_ros_key.compare(value.p_ros_key) == 0) && (p_iop_key.compare(value.p_iop_key) == 0);
}

bool IlluminatorClient::operator!=(IlluminatorClient &value)
{
	return !(*this == value);
}

void IlluminatorClient::p_ros_cmd_callback(const std_msgs::Bool::ConstPtr& state)
{
	if (!p_cmd_callback.empty() && is_supported()) {
		p_state = state->data;
		p_cmd_callback(p_iop_key, p_state);
		if (p_state) {
			p_state_str = "ON";
		} else {
			p_state_str = "OFF";
		}
	}
}
