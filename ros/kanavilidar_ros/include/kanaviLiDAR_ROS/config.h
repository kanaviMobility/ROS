#ifndef __CONFIG_H__
#define __CONFIG_H__

/**
 * @file config.h
 * @author twchong (twchong@kanavi-mobility.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

//boost LIB
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>	//using .ini file
#include <boost/filesystem.hpp>

#include "kanaviLiDAR_ros.h"
#include <fstream>

typedef struct iniconfig
{
	iniconfig(){
		ip_ = "";
		port_ = 0;
		m_ip_ = "";
		fixed_ = "";
		topic_ = "";
		h_reverse = false;
		sensor_ip_ = "";
	}
	/* data */
	std::string ip_;			//local ip
	int port_;					//lcal port
	std::string m_ip_;			//udp multicast group ip
	std::string fixed_;			//rviz fixed frame name
	std::string topic_;			//rviz topic name
	bool h_reverse;				//rviz horizontal reverse visualization
	std::string sensor_ip_;		//set LiDAR IP
}iniConfig;

class node_config
{
public:
	node_config(/* args */);
	~node_config();

	iniConfig getConfig(const std::string &name);
	void saveConfig(const std::string &name, const iniConfig &data);

private:
	std::string getStr(boost::property_tree::ptree pt, const std::string &config_);
	int getInt(boost::property_tree::ptree pt, const std::string &config_);
	bool getBool(boost::property_tree::ptree pt, const std::string &config_);

	template <typename T>
	void setCofig(boost::property_tree::ptree &pt, const string & key, const T & value){
		printf("setconfig : %s\n", key.c_str());
		 pt.put(key, value);
			// pt.add(key, value);
		}

	std::string getResultFromCMD(std::string com);

	/* data */
};

node_config::node_config(/* args */)
{
}

node_config::~node_config()
{
}

/**
 * @brief get configuration information to config file
 * 
 * @param name 			file name
 * @return iniConfig 	config information
 */
iniConfig node_config::getConfig(const std::string &name)
{
	iniConfig st;
	boost::property_tree::ptree pt;
	if(!name.empty())	//check name string empty?
	{
		try
		{
			/* code */
			boost::property_tree::ini_parser::read_ini(name, pt);		//read file

			st.ip_ = getStr(pt, KANAVI::ROS::CONFIG::NODE_CONFIG 	//get IP
					+ KANAVI::ROS::CONFIG::IP);
			st.port_ = getInt(pt, KANAVI::ROS::CONFIG::NODE_CONFIG 	//get port
					+ KANAVI::ROS::CONFIG::PORT);
			st.m_ip_ = getStr(pt, KANAVI::ROS::CONFIG::NODE_CONFIG 	//get multicast ip
					+ KANAVI::ROS::CONFIG::MultiCast);
			st.topic_ = getStr(pt, KANAVI::ROS::CONFIG::NODE_CONFIG //get topic name
					+ KANAVI::ROS::CONFIG::TOPIC);
			st.fixed_ = getStr(pt, KANAVI::ROS::CONFIG::NODE_CONFIG //get fixed name
					+ KANAVI::ROS::CONFIG::FIXED_FRAME);
			st.sensor_ip_ = getStr(pt, KANAVI::ROS::CONFIG::NODE_CONFIG 	//get sensor ip
					+ KANAVI::ROS::CONFIG::SENSOR_IP);
			st.h_reverse = getBool(pt, KANAVI::ROS::CONFIG::NODE_CONFIG 	//get information of horizontal reverse
					+ KANAVI::ROS::CONFIG::HORIZONTAL_REVERSE);
		}
		catch(const boost::property_tree::ptree_error &e)
		{
			std::cerr << e.what() << '\n';
		}
	}
	return st;
}

void node_config::saveConfig(const std::string &name, const iniConfig &data)
{
	boost::property_tree::ptree pt;

	std::string cmd = getResultFromCMD("cd " + INI_PATH + "; pwd");		//get absolute path using CMD

	cmd.erase(std::find(cmd.begin(), cmd.end(), '\n'));					//delete '\n' char
	std::string fullp = cmd + "/" + name;


	std::fstream fs;
	fs.open(fullp, std::ios::out);										// file open
	fs.close();															// file close

	printf("CHECK Config File Name : %s\n", fullp.c_str());

	if(!data.ip_.empty())	//set IP 
	{
		printf("check IP set?\n");
		setCofig(pt, KANAVI::ROS::CONFIG::NODE_CONFIG 
					+ KANAVI::ROS::CONFIG::IP, data.ip_);
	}
	if(data.port_ != 0)		//set port
	{
		setCofig(pt, KANAVI::ROS::CONFIG::NODE_CONFIG 
					+ KANAVI::ROS::CONFIG::PORT, data.port_);
	}
	if(!data.m_ip_.empty())	//set multicast ip
	{
		setCofig(pt, KANAVI::ROS::CONFIG::NODE_CONFIG 
					+ KANAVI::ROS::CONFIG::MultiCast, data.m_ip_);
	}
	if(!data.topic_.empty())	//set topic name
	{
		setCofig(pt, KANAVI::ROS::CONFIG::NODE_CONFIG 
					+ KANAVI::ROS::CONFIG::TOPIC, data.topic_);
	}
	if(!data.fixed_.empty())	//set fixed name
	{
		setCofig(pt, KANAVI::ROS::CONFIG::NODE_CONFIG 
					+ KANAVI::ROS::CONFIG::FIXED_FRAME, data.fixed_);
	}
	if(!data.sensor_ip_.empty())	//set lidar sensor IP
	{
		setCofig(pt, KANAVI::ROS::CONFIG::NODE_CONFIG 
					+ KANAVI::ROS::CONFIG::SENSOR_IP, data.sensor_ip_);
	}
	if(data.h_reverse)				//set horizontal reverse checked
	{
		setCofig(pt, KANAVI::ROS::CONFIG::NODE_CONFIG 
					+ KANAVI::ROS::CONFIG::HORIZONTAL_REVERSE, data.h_reverse);
	}

	boost::property_tree::write_ini(fullp, pt);		//write .ini file.
}

/**
 * @brief get string inform from config file.
 * 
 * @param pt 		boost::property_tree::ptree
 * @param config_ 	config name
 * @return std::string 	get value from config file
 */
std::string node_config::getStr(boost::property_tree::ptree pt, const std::string &config_)
{
	std::string str = pt.get<std::string>(config_, "");

	return str;
}

/**
 * @brief get int inform from config file.
 * 
 * @param pt  		boost::property_tree::ptree
 * @param config_ 	config name
 * @return int 		get value from config file
 */
int node_config::getInt(boost::property_tree::ptree pt, const std::string &config_)
{
	int val = pt.get<int>(config_, -1);

	return val;
}

/**
 * @brief get bool inform from config file.
 * 
 * @param pt  		boost::property_tree::ptree
 * @param config_ 	config name
 * @return true 
 * @return false 
 */
bool node_config::getBool(boost::property_tree::ptree pt, const std::string &config_)
{
	bool checked = pt.get<bool>(config_, false);

	return checked;
}

/**
 * @brief get string from CMD
 * 
 * @param com 	command
 * @return std::string 
 */
std::string node_config::getResultFromCMD(std::string com)
{
	std::string result;
	FILE* stream;
	const int maxBuffer = 256; 			// 버퍼의 크기는 적당하게
	char buffer[maxBuffer];
	com.append(" 2>&1"); 				// 표준에러를 표준출력으로 redirect

	stream = popen(com.c_str(), "r"); 	// 주어진 command를 shell로 실행하고 파이프 연결 (fd 반환)
	if (stream) 
	{
		while (fgets(buffer, maxBuffer, stream) != NULL) 
		{
			result.append(buffer); 		// fgets: fd (stream)를 길이 (maxBuffer)만큼 읽어 버퍼 (buffer)에 저장
		}
		pclose(stream); 				// close pipe
	}
	return result;
}

#endif // __CONFIG_H__