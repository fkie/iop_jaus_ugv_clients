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


#ifndef ILLUMINATORCLIENT_H
#define ILLUMINATOR_H


#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace iop
{

class IlluminatorClient
{
public:
	static std::string get_iop_key(std::string ros_key);
	static std::string get_ros_key(std::string iop_key);

	IlluminatorClient();
	IlluminatorClient(bool supported, std::string iop_key);
	~IlluminatorClient();

	void init(std::string ros_key, std::string state, std::string diagnostic_key="");
	void init(std::string ros_key, bool state, std::string diagnostic_key="");
	template<class T>
	void set_cmd_callback(void(T::*handler)(std::string iop_key, bool state), T*obj) {
		p_cmd_callback = boost::bind(handler, obj, _1, _2);
		std::cout << "set callback" << p_cmd_callback << ", " << !p_cmd_callback.empty() << std::endl;
	}
	void set_state_callback();
	bool is_valid();
	bool is_supported();
	bool get_state();
	std::string get_state_str();
	bool set_state(bool state);
	std::string get_iop_key() { return p_iop_key; }
	std::string get_ros_key() { return p_ros_key; }
	bool operator==(IlluminatorClient &value);
	bool operator!=(IlluminatorClient &value);

protected:
	static std::map<std::string, std::string> p_iop_ros_map;
	static std::map<std::string, std::string> p_ros_iop_map;
	std::string p_ros_key;
	std::string p_iop_key;
	std::string p_diagnostic_key;
	bool p_supported;
	bool p_state;
	std::string p_state_str;
	ros::Subscriber p_sub_cmd;
	ros::Publisher p_pub_state;
	boost::function<void (std::string iop_key, bool state)> p_cmd_callback;

	void p_ros_cmd_callback(const std_msgs::Bool::ConstPtr& state);

};

};

#endif
