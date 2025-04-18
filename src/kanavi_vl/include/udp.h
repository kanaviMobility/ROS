#ifndef __UDP_H__
#define __UDP_H__

// Copyright (c) 2025, Kanavi Mobility
// All rights reserved.
//
// This file is part of the ROS1/ROS2 Hybrid Build Project.
// Licensed under the BSD 3-Clause License.
// You may obtain a copy of the License at the root of this repository (LICENSE file).

/**
 * @file udp.h
 * @author twchong (twchong@kanavi-mobility.com)
 * @brief define UDP Functions
 * @version 0.1
 * @date 2024-11-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <iostream>
#include <string>
#include <string.h>
#include <stdio.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>     // close
#include <vector>

#include <cassert>
#include <algorithm>
#include <iterator>

#define MAX_BUF_SIZE 65000

/**
 * @class kanavi_udp
 * @brief [TODO] Describe the purpose of class kanavi_udp
 */
class kanavi_udp
{
private:
	/* data */

	//SECTION -- FUNCS.
/**
 * @brief [TODO] Describe the function init
 * @return int [description]
 * @param &ip_ [description]
 * @param &port_ [description]
 * @param "224.0.0.5" [description]
 * @param false [description]
 */
	int init(const std::string &ip_, const int &port_, std::string multicast_ip_ = "224.0.0.5", bool multi_checked_ = false);

/**
 * @brief [TODO] Describe the function check_udp_buf_size
 * @return void [description]
 */
	void check_udp_buf_size();

	//!SECTION --------

	//SECTION -- VARS.
	struct sockaddr_in g_udpAddr;
	struct sockaddr_in g_senderAddr;
	int g_udpSocket;

	struct ip_mreq multi_Addr;

	u_char g_udp_buf[MAX_BUF_SIZE];

	//!SECTION --------
public:
	kanavi_udp(const std::string &local_ip_, const int &port_, const std::string &multicast_ip_);
	kanavi_udp(const std::string &local_ip_, const int &port_);
	~kanavi_udp();

/**
 * @brief [TODO] Describe the function getData
 * @return std::vector<u_char> [description]
 */
	std::vector<u_char> getData();

/**
 * @brief [TODO] Describe the function sendData
 * @return void [description]
 * @param data_ [description]
 */
	void sendData(std::vector<u_char> data_);

/**
 * @brief [TODO] Describe the function connect
 * @return int [description]
 */
	int connect();
/**
 * @brief [TODO] Describe the function disconnect
 * @return int [description]
 */
	int disconnect();
};
#endif // __UDP_H__