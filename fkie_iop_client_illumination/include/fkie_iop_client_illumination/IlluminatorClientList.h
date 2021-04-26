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


#ifndef ILLUMINATORCLIENTLIST_H
#define ILLUMINATORCLIENTLIST_H

#include <mutex>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <fkie_iop_component/iop_component.hpp>
#include "urn_jaus_jss_ugv_IlluminationServiceClient/Messages/ReportIlluminationState.h"
#include "urn_jaus_jss_ugv_IlluminationServiceClient/Messages/ReportIlluminationConfiguration.h"
#include "urn_jaus_jss_ugv_IlluminationServiceClient/Messages/SetIlluminationState.h"
#include "IlluminatorClient.h"

namespace iop
{

class IlluminatorClientList
{
public:
	IlluminatorClientList(std::shared_ptr<iop::Component> cmp);
	~IlluminatorClientList();
	void apply_configuration_report(urn_jaus_jss_ugv_IlluminationServiceClient::ReportIlluminationConfiguration config);
	void apply_state_report(urn_jaus_jss_ugv_IlluminationServiceClient::ReportIlluminationState states);

	urn_jaus_jss_ugv_IlluminationServiceClient::SetIlluminationState get_state_cmd();
	template<class T>
	void set_cmd_callback(void(T::*handler)(urn_jaus_jss_ugv_IlluminationServiceClient::SetIlluminationState), T*obj) {
		p_cmd_callback = std::bind(handler, obj, std::placeholders::_1);
	}

protected:
	std::shared_ptr<iop::Component> cmp;
	bool p_initialized;
	std::map<std::string, iop::IlluminatorClient*> p_illuminator_map;
	rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr p_pub_diagnostic;
	std::function<void (urn_jaus_jss_ugv_IlluminationServiceClient::SetIlluminationState)> p_cmd_callback;
	typedef std::recursive_mutex mutex_type;
	typedef std::unique_lock<mutex_type> lock_type;
	mutable mutex_type p_mutex;

	void p_clear_map();
	jUnsignedInteger p_get_illuminator_state(std::string iop_key);
	jUnsignedInteger p_get_illuminator_support(std::string iop_key);
	void p_illuminator_cmd_callback(std::string iop_key, bool state);
	void p_ros_diagnostic_callback(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr state);
};

}

#endif
