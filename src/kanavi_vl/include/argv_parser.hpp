#ifndef __ARGV_PARSER_H__
#define __ARGV_PARSER_H__

// Copyright (c) 2025, Kanavi Mobility
// All rights reserved.
//
// This file is part of the ROS1/ROS2 Hybrid Build Project.
// Licensed under the BSD 3-Clause License.
// You may obtain a copy of the License at the root of this repository (LICENSE file).

/**
 * @file argv_parser.hpp
 * @author twchong (twchong@kanavi-mobility.com)
 * @brief for parse intput ARGV
 * @version 0.1
 * @date 2024-11-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <iostream>
#include "common.h"
#include <string>

/**
 * @brief define user structure for argv
 * 
 */
struct argvContainer
{
	std::string local_ip;		// local IP address
	int port;					// port number
	bool checked_multicast;		// checked multicast
	std::string multicast_ip;	// multicast IP address
	std::string topicName;		// ROS Node topic Name
	std::string fixedName;		// ROS Node Fixed Name
	
	argvContainer(){
		// set defalut Values
		local_ip = KANAVI::COMMON::default_local_IP;
		port = KANAVI::COMMON::default_port_num;
		multicast_ip = KANAVI::COMMON::default_multicast_IP;
		checked_multicast = false;
		topicName = KANAVI::COMMON::ROS_TOPIC_NAME;
		fixedName = KANAVI::COMMON::ROS_FIXED_NAME;
	}
};


/**
 * @class argv_parser
 * @brief Parses command-line arguments to extract program configurations and options.
 */
class argv_parser
{
private:
	/* data */
	argvContainer argvResult;

	// funcs.
/**
 * @brief Parses command-line arguments.
 * @return void
 * @param &argc_ The number of command-line arguments.
 * @param **argv_ The array of argument strings passed to the program.
 */
	void parseArgv(const int &argc_, char **argv_);
	
	// Vars.

public:
	argv_parser(const int &argc_, char **argv_)
	{
		parseArgv(argc_, argv_);
	}
	~argv_parser(){}

/**
 * @brief Returns the parsed command-line parameters.
 * @return argvContainer An object containing extracted arguments such as IP address, port, topic name, etc.
 */
	argvContainer getParameters();
};

/**
 * @brief Parses command-line arguments.
 * @return void
 * @param &argc_ The number of command-line arguments.
 * @param **argv_ The array of argument strings passed to the program.
 */
inline void argv_parser::parseArgv(const int &argc_, char **argv_)
{
	if(argc_  == 0)
	{
		printf("Active Defalut Mode");
		argvResult.checked_multicast = true;
	}

	for(int i=0; i<argc_; i++)
	{
		if(!strcmp(argv_[i], KANAVI::ROS::PARAMETER_IP.c_str()))		// check ARGV - IP & port num.
		{
			argvResult.local_ip = argv_[i+1];
			argvResult.port = atoi(argv_[i+2]);
		}
		else if(!strcmp(argv_[i], KANAVI::ROS::PARAMETER_Multicast.c_str()))	// check ARGV - udp multicast ip
		{
			argvResult.checked_multicast = true;
			argvResult.multicast_ip = argv_[i+1];
		}
		else if(!strcmp(argv_[i], KANAVI::ROS::PARAMETER_FIXED.c_str()))							// check ARGV - ROS Fixed name
		{
			argvResult.fixedName = argv_[i+1];
		}
		else if(!strcmp(argv_[i], KANAVI::ROS::PARAMETER_TOPIC.c_str()))							// check ARGV - ROS topic name
		{
			argvResult.topicName = argv_[i+1];
		}
	}

}

/**
 * @brief Returns the parsed command-line parameters.
 * @return argvContainer An object containing extracted arguments such as IP address, port, topic name, etc.
 */
inline argvContainer argv_parser::getParameters()
{
	return argvResult;;
}

#endif // __ARGV_PARSER_H__