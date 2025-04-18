#ifndef __KANAVI_LIDAR_H__
#define __KANAVI_LIDAR_H__

// Copyright (c) 2025, Kanavi Mobility
// All rights reserved.
//
// This file is part of the ROS1/ROS2 Hybrid Build Project.
// Licensed under the BSD 3-Clause License.
// You may obtain a copy of the License at the root of this repository (LICENSE file).

/**
 * @file kanavi_lidar.h
 * @author twchong (twchong@kanavi-mobility.com)
 * @brief define processing Function for kanavi-mobility LiDAR VL-series Raw data
 * @version 0.1
 * @date 2024-11-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <iostream>
#include <vector>
#include "common.h"
typedef struct kanavi_datagram{
	// LiDAR Model
	int model;
	// vertical FoV
	double v_fov;
	// vertical resolution
	double v_resolution;
	// horizontal FoV
	double h_fov;
	// horizontal resolution
	double h_resolution;
	// check data input End.
	bool checked_end;
	// check lidar sensor IP
	std::string lidar_ip;
	// buf : raw
	std::vector< std::vector<u_char> > raw_buf;
	// buf : Length
	std::vector< std::vector<float> > len_buf;
	// raw data size
	size_t input_packet_size;

	kanavi_datagram(){
	}
	kanavi_datagram(int model_){
		model = model_;
		switch(model)
		{
		case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R2:
			v_fov = KANAVI::COMMON::SPECIFICATION::R2::VERTICAL_FoV;
			v_resolution = KANAVI::COMMON::SPECIFICATION::R2::VERTICAL_RESOLUTION;
			h_fov = KANAVI::COMMON::SPECIFICATION::R2::HORIZONTAL_FoV;
			h_resolution = KANAVI::COMMON::SPECIFICATION::R2::HORIZONTAL_RESOLUTION;
			input_packet_size = KANAVI::COMMON::SPECIFICATION::R2::RAW_TOTAL_SIZE;
			raw_buf.resize(KANAVI::COMMON::SPECIFICATION::R2::VERTICAL_CHANNEL);
			len_buf.resize(KANAVI::COMMON::SPECIFICATION::R2::VERTICAL_CHANNEL);
			break;
		case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R4:
			v_fov = KANAVI::COMMON::SPECIFICATION::R4::VERTICAL_FoV;
			v_resolution = KANAVI::COMMON::SPECIFICATION::R4::VERTICAL_RESOLUTION;
			h_fov = KANAVI::COMMON::SPECIFICATION::R4::HORIZONTAL_FoV;
			h_resolution = KANAVI::COMMON::SPECIFICATION::R4::HORIZONTAL_RESOLUTION;
			input_packet_size = KANAVI::COMMON::SPECIFICATION::R4::RAW_TOTAL_SIZE;
			raw_buf.resize(KANAVI::COMMON::SPECIFICATION::R4::VERTICAL_CHANNEL);
			len_buf.resize(KANAVI::COMMON::SPECIFICATION::R4::VERTICAL_CHANNEL);
			break;
		case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R270:
			v_fov = KANAVI::COMMON::SPECIFICATION::R270::VERTICAL_FoV;
			v_resolution = KANAVI::COMMON::SPECIFICATION::R270::VERTICAL_RESOLUTION;
			h_fov = KANAVI::COMMON::SPECIFICATION::R270::HORIZONTAL_FoV;
			h_resolution = KANAVI::COMMON::SPECIFICATION::R270::HORIZONTAL_RESOLUTION;
			input_packet_size = KANAVI::COMMON::SPECIFICATION::R270::RAW_TOTAL_SIZE;
			raw_buf.resize(KANAVI::COMMON::SPECIFICATION::R270::VERTICAL_CHANNEL);
			len_buf.resize(KANAVI::COMMON::SPECIFICATION::R270::VERTICAL_CHANNEL);
			break;
		}
	}

}kanaviDatagram;

namespace KANAVI
{
	namespace PROCESS
	{
		namespace InputMode
		{
			const int FAIL = -1;
			const int OnGoing = 0;
			const int SUCCESS = 1;
		} // namespace InputMode
	}
}

/**
 * @class kanavi_lidar
 * @brief [TODO] Describe the purpose of class kanavi_lidar
 */
class kanavi_lidar
{
private:
	// FUNCTIONS----
/**
 * @brief [TODO] Describe the function classification
 * @return int [description]
 * @param &data [description]
 */
	int classification(const std::vector<u_char> &data);
/**
 * @brief [TODO] Describe the function checkDataInputEnd
 * @return int [description]
 * @param &data [description]
 */
	int checkDataInputEnd(const std::vector<u_char> &data);
/**
 * @brief [TODO] Describe the function checkChannel
 * @return int [description]
 * @param &data [description]
 */
	int checkChannel(const std::vector<u_char> &data);
/**
 * @brief [TODO] Describe the function parse
 * @return void [description]
 * @param &data [description]
 */
	void parse(const std::vector<u_char> &data);

/**
 * @brief [TODO] Describe the function r2
 * @return void [description]
 * @param &input [description]
 * @param *output [description]
 */
	void r2(const std::vector<u_char> &input, kanaviDatagram *output);
/**
 * @brief [TODO] Describe the function r4
 * @return void [description]
 * @param &input [description]
 * @param *output [description]
 */
	void r4(const std::vector<u_char> &input, kanaviDatagram *output);
/**
 * @brief [TODO] Describe the function r270
 * @return void [description]
 * @param &input [description]
 * @param *output [description]
 */
	void r270(const std::vector<u_char> &input, kanaviDatagram *output);
/**
 * @brief [TODO] Describe the function parseLength
 * @return void [description]
 * @param &input [description]
 * @param *output [description]
 * @param ch [description]
 */
	void parseLength(const std::vector<u_char> &input, kanaviDatagram *output, int ch);
	// !FUNTCIONS---

	/* data */
	kanaviDatagram *datagram_;
	std::vector<u_char> temp_buf_;	// for complete packet
	bool checked_onGoing;

	bool checked_pares_end;
	bool checked_ch_r4[4];

public:
	kanavi_lidar(int model_);
	~kanavi_lidar();

/**
 * @brief [TODO] Describe the function process
 * @return int [description]
 * @param &data [description]
 */
	int process(const std::vector<u_char> &data);

/**
 * @brief [TODO] Describe the function getLiDARModel
 * @return std::string [description]
 */
	std::string getLiDARModel();

/**
 * @brief [TODO] Describe the function checkedProcessEnd
 * @return bool [description]
 */
	bool checkedProcessEnd();

/**
 * @brief [TODO] Describe the function getDatagram
 * @return kanaviDatagram [description]
 */
	kanaviDatagram getDatagram();

};


#endif // __KANAVI_LIDAR_H__