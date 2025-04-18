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
 * @brief Handles LiDAR data parsing and processing for different Kanavi LiDAR models (R2, R4, R270).
 *
 * This class is responsible for receiving raw LiDAR data packets, validating them,
 * and converting them into structured data formats such as kanaviDatagram.
 * It supports model-specific parsing logic and provides access to processed results.
 */

class kanavi_lidar
{
private:
	// FUNCTIONS----
/**
 * @brief Classifies the incoming raw LiDAR data.
 * @param data The raw data buffer received from the LiDAR sensor using UDP.
 * @return Integer representing the classification result or model type.
 */
	int classification(const std::vector<u_char> &data);
/**
 * @brief Checks whether the full LiDAR data packet has been received.
 * @param data The data buffer to be checked.
 * @return 1 if input is complete; 0 otherwise.
 */
	int checkDataInputEnd(const std::vector<u_char> &data);
/**
 * @brief Validates the channel information in the LiDAR data packet.
 * @param data The data buffer to be validated.
 * @return 1 if channel is valid; -1 if error.
 */
	int checkChannel(const std::vector<u_char> &data);
/**
 * @brief Parses the input LiDAR data according to the internal model.
 * @param data Raw data to parse.
 */
	void parse(const std::vector<u_char> &data);

/**
 * @brief Parses R2-model LiDAR data into structured format.
 * @param input Raw input data.
 * @param output Output structure to hold parsed data.
 */
	void r2(const std::vector<u_char> &input, kanaviDatagram *output);
/**
 * @brief Parses R4-model LiDAR data into structured format.
 * @param input Raw input data.
 * @param output Output structure to hold parsed data.
 */
	void r4(const std::vector<u_char> &input, kanaviDatagram *output);
/**
 * @brief Parses R270-model LiDAR data into structured format.
 * @param input Raw input data.
 * @param output Output structure to hold parsed data.
 */
	void r270(const std::vector<u_char> &input, kanaviDatagram *output);
/**
 * @brief Parses the length section of the data for a given channel.
 * @param input Raw input data.
 * @param output Output datagram structure.
 * @param ch Channel index to parse.
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
/**
 * @brief Constructor for kanavi_lidar class with specific model.
 * @param model_ Integer model identifier (e.g., R2, R4, R270).
 */
	kanavi_lidar(int model_);
	~kanavi_lidar();

/**
 * @brief Processes the input data and performs parsing and validation.
 * @param data The input LiDAR data to process.
 * @return Status code after processing.
 */
	int process(const std::vector<u_char> &data);

/**
 * @brief Returns the model name as a string.
 * @return LiDAR model (e.g., "R2", "R4", "R270").
 */
	std::string getLiDARModel();

/**
 * @brief Checks whether the LiDAR data processing is complete.
 * @return true if the full packet has been processed.
 */
	bool checkedProcessEnd();

/**
 * @brief Retrieves the parsed datagram result.
 * @return Parsed LiDAR data in kanaviDatagram format.
 */
	kanaviDatagram getDatagram();

};


#endif // __KANAVI_LIDAR_H__