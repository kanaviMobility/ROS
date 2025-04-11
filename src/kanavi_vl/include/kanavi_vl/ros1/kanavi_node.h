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
 * @brief Header for Kanavi LiDAR ROS1 Node
 * @version 0.1
 * @date 2024-11-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <ros/ros.h>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>

#include <chrono>
#include <string>

#include "argv_parser.hpp"

#include "udp.h"

#include <kanavi_lidar.h>	// for LiDAR data processing

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std::chrono_literals;  // "10ms"와 같은 단위 사용을 위해 필요

class kanavi_node
{
private:

	//SECTION - FUNCS.

	void helpAlarm();

	void receiveData();

	void endProcess();

	void log_set_parameters();

	void length2PointCloud(kanaviDatagram datagram);

	void calculateAngular(int model);

	void generatePointCloud(const kanaviDatagram &datagram, PointCloudT &cloud_);

	PointT length2point(float len, float v_sin, float v_cos, float h_sin, float h_cos);

	void HSV2RGB(float *fR, float *fG, float *fB, float fH, float fS, float fV);
	
	sensor_msgs::PointCloud2 cloud_to_cloud_msg(int ww, int hh, const pcl::PointCloud<pcl::PointXYZRGB>& cloud, int timestamp, const std::string& frame);

	void rotateAxisZ(PointCloudT::Ptr cloud, float angle);

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
	ros::NodeHandle nh_;
	ros::Publisher publisher_;

	// timer for RECV
	ros::Timer timer_;

	// flags
	bool checked_multicast_;
	bool checked_help_;

	// UDP network
	std::unique_ptr<kanavi_udp> m_udp;

	// LiDAR data processing Class
	std::unique_ptr<kanavi_lidar> kanavi_;

	// sin, cos value for calculate Angle
	std::vector<float> v_sin;
	std::vector<float> v_cos;
	std::vector<float> h_sin;
	std::vector<float> h_cos;

	// pcl point cloud 
	PointCloudT::Ptr g_pointcloud;

	// rotate Angle
	float rotate_angle;

	//!SECTION	

public:
	kanavi_node(const std::string &node_, int &argc_, char **argv_);
	~kanavi_node();

	// void publishing();
	void run();
};
#endif // __KANAVI_NODE_H__