#ifndef __KANAVI_NODE_H__
#define __KANAVI_NODE_H__

// Copyright (c) 2025, Kanavi Mobility
// All rights reserved.
//
// This file is part of the ROS1/ROS2 Hybrid Build Project.
// Licensed under the BSD 3-Clause License.
// You may obtain a copy of the License at the root of this repository (LICENSE file).

/**
 * @file kanavi_node.h
 * @author twchong (twchong@kanavi-mobility.com)
 * @brief Header for Kanavi LiDAR ROS2 Node
 * @version 0.1
 * @date 2024-11-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/impl/angles.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.h>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <string>

#include "argv_parser.hpp"
#include "udp.h"
#include "kanavi_lidar.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
using namespace std::chrono_literals;  // "10ms"와 같은 단위 사용을 위해 필요

/**
 * @class kanavi_node
 * @brief [TODO] Describe the purpose of class kanavi_node
 */
class kanavi_node : public rclcpp::Node
{
private:

	//SECTION - FUNCS.

/**
 * @brief [TODO] Describe the function helpAlarm
 * @return void [description]
 */
	void helpAlarm();

/**
 * @brief [TODO] Describe the function receiveData
 * @return void [description]
 */
	void receiveData();

/**
 * @brief [TODO] Describe the function endProcess
 * @return void [description]
 */
	void endProcess();

/**
 * @brief [TODO] Describe the function log_set_parameters
 * @return void [description]
 */
	void log_set_parameters();

/**
 * @brief [TODO] Describe the function length2PointCloud
 * @return void [description]
 * @param datagram [description]
 */
	void length2PointCloud(kanaviDatagram datagram);

/**
 * @brief [TODO] Describe the function generatePointCloud
 * @return void [description]
 * @param &datagram [description]
 * @param &cloud_ [description]
 */
	void generatePointCloud(const kanaviDatagram &datagram, PointCloudT &cloud_);

/**
 * @brief [TODO] Describe the function length2point
 * @return PointT [description]
 * @param len [description]
 * @param v_sin [description]
 * @param v_cos [description]
 * @param h_sin [description]
 * @param h_cos [description]
 */
	PointT length2point(float len, float v_sin, float v_cos, float h_sin, float h_cos);

/**
 * @brief [TODO] Describe the function rotateAxisZ
 * @return void [description]
 * @param cloud [description]
 * @param angle [description]
 */
	void rotateAxisZ(PointCloudT::Ptr cloud, float angle);
	
/**
 * @brief [TODO] Describe the function HSV2RGB
 * @return void [description]
 * @param *fR [description]
 * @param *fG [description]
 * @param *fB [description]
 * @param fH [description]
 * @param fS [description]
 * @param fV [description]
 */
	void HSV2RGB(float *fR, float *fG, float *fB, float fH, float fS, float fV);

/**
 * @brief [TODO] Describe the function publish_pointcloud
 * @return void [description]
 * @param cloud_ [description]
 */
	void publish_pointcloud(PointCloudT::Ptr cloud_);
	// need process...
	
	//!SETCION

	/* data */
	//SECTION - Variables

	// argv parse Class
	std::unique_ptr<argv_parser> m_argv;
	
	// Network
	std::string local_ip_;
	int port_;
	std::string multicast_ip_;

	// ROS
	std::string topicName_;
	std::string fixedName_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
	// timer for RECV
	rclcpp::TimerBase::SharedPtr timer_;

	// flags
	bool checked_multicast_;
	bool checked_help_;

	// rotate angle
	float rotate_angle;

	// datagram
	kanaviDatagram g_datagram;

	// sin, cos value for calculate Angle
	std::vector<float> v_sin;
	std::vector<float> v_cos;
	std::vector<float> h_sin;
	std::vector<float> h_cos;

	// pcl point cloud 
	PointCloudT::Ptr g_pointcloud;

	// UDP network
	std::unique_ptr<kanavi_udp> m_udp;

	// LiDAR Processor
	std::unique_ptr<kanavi_lidar> m_process;

	//!SECTION	

public:
	kanavi_node(const std::string &node_, int &argc_, char **argv_);
	~kanavi_node();

/**
 * @brief [TODO] Describe the function calculateAngular
 * @return void [description]
 * @param model [description]
 */
	void calculateAngular(int model);

	// void publishing();
};
#endif // __KANAVI_NODE_H__