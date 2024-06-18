/*
 * Copyright (c) 2022, Kanavi-Mobility.co.,ltd.
 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __KANAVILIDAR_ROS_H__
#define __KANAVILIDAR_ROS_H__

/**
 * @file kanaviLiDAR_ros.h
 * @author twchong (twchong@kanavi-mobility.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <ros/ros.h>
#include <iostream>

#include <time.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>
#include <Eigen/Eigen>
#include <array>
#include <typeinfo>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include "KANAVI_LIB/include/header.h"	//lidar process
#include "KANAVI_LIB/include/lidar_spec.h"	//lidar process
#include "KANAVI_LIB/processor/kanaviProcessor.h"
#include "KANAVI_LIB/UDP/Udp.h"

using namespace std;

namespace KANAVI{
	namespace ROS{
		namespace CONFIG{
			const std::string NODE_CONFIG 			= "CONFIG.";
			const std::string IP					= "local_IP";
			const std::string PORT					= "local_PORT";
			const std::string MultiCast				= "MULTICAST_IP";
			const std::string TOPIC					= "TOPIC";
			const std::string FIXED_FRAME			= "FIXED_FRAME";
			const std::string SENSOR_IP				= "SENSOR_IP";
			const std::string HORIZONTAL_REVERSE	= "HORIZONTAL_REVERSE";
		}
	}
}

const std::string INI_PATH="~/catkin_ws/src/kanavilidar_ros/config/";


// struct EIGEN_ALIGN16 Point {
// 	PCL_ADD_POINT4D;				//PointXYZRGB
// 	float intensity;
// 	uint32_t t;
// 	uint16_t reflectivity;
// 	uint8_t ring;  
// 	uint16_t ambient;
// 	uint32_t range; 
// 	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// };	// point format to publish 

// POINT_CLOUD_REGISTER_POINT_STRUCT(Point,
// 	(float, x, x)
// 	(float, y, y)
// 	(float, z, z)
// 	(float, intensity, intensity)
// 	// use std::uint32_t to avoid conflicting with pcl::uint32_t
// 	(std::uint32_t, t, t)
// 	(std::uint16_t, reflectivity, reflectivity)
// 	(std::uint8_t, ring, ring)
// 	(std::uint16_t, ambient, ambient)
// 	(std::uint32_t, range, range)
// )

// POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZRGB,
// 	(float, x, x)
// 	(float, y, y)
// 	(float, z, z)
// 	(uint32_t, rgba, rgba)
// )

#endif // __KANAVILIDAR_ROS_H__