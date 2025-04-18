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
 * @brief Provides UDP socket communication functionality including multicast support for Kanavi sensors.
 *
 * This class handles socket setup, data transmission, and reception over UDP.
 * It supports both unicast and multicast communication modes.
 */
class kanavi_udp
{
private:
	/* data */

	//SECTION -- FUNCS.
/**
 * @brief Initializes the UDP socket with given IP, port, and optional multicast settings.
 * 
 * @param ip_ Local IP address to bind the socket.
 * @param port_ Port number for UDP communication.
 * @param multicast_ip_ (Optional) Multicast group IP address. Default is "224.0.0.5".
 * @param multi_checked_ Whether to enable multicast reception.
 * @return 0 if successful, or error code otherwise.
 */
	int init(const std::string &ip_, const int &port_, std::string multicast_ip_ = "224.0.0.5", bool multi_checked_ = false);

/**
 * @brief Checks and logs the current receive buffer size for the UDP socket.
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
/**
 * @brief Constructor for initializing UDP communication with local and multicast IP.
 * 
 * @param local_ip_ Local IP address to bind.
 * @param port_ UDP port number.
 * @param multicast_ip_ Multicast IP address to join.
 */
	kanavi_udp(const std::string &local_ip_, const int &port_, const std::string &multicast_ip_);
	kanavi_udp(const std::string &local_ip_, const int &port_);
	~kanavi_udp();

/**
 * @brief Receives a UDP packet and returns the raw data.
 * 
 * @return Vector of received bytes.
 */
	std::vector<u_char> getData();

/**
 * @brief Sends a UDP packet to the configured address (Not Used).
 * 
 * @param data_ Byte buffer to send.
 */
	void sendData(std::vector<u_char> data_);

/**
 * @brief Establishes the UDP socket connection.
 * 
 * @return 0 if successful, or error code otherwise.
 */
	int connect();
	
/**
 * @brief Closes the UDP socket connection.
 * 
 * @return 0 if successful, or error code otherwise.
 */
	int disconnect();
};
#endif // __UDP_H__