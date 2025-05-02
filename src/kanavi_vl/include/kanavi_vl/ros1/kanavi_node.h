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

/**
 * @class kanavi_node
 * @brief ROS1-compatible LiDAR interface for Kanavi sensors.
 *
 * This class handles UDP communication with the LiDAR sensor, parses the incoming data,
 * converts it into a ROS-compatible PointCloud2 message, and publishes it to a topic.
 */
class kanavi_node
{
private:

	//SECTION - FUNCS.

/**
 * @brief Prints help message for ROS1 node command-line usage.
 */
	void helpAlarm();

/**
 * @brief Receives LiDAR data from the UDP socket and processes it.
 */
	std::vector<u_char> receiveDatagram();

/**
 * @brief Ends the ROS1 node operation and releases resources.
 */
	void endProcess();

/**
 * @brief Configures ROS1 log parameters such as log level and output behavior.
 */
	void log_set_parameters();

/**
 * @brief Converts raw datagram into an internal point cloud representation.
 * @param datagram Parsed datagram from LiDAR sensor.
 */
	void length2PointCloud(kanaviDatagram datagram);

/**
 * @brief Calculates angular resolution and spacing for a specific LiDAR model.
 * @param model LiDAR model identifier.
 */
	void calculateAngular(int model);

/**
 * @brief Converts a kanaviDatagram into a PCL-compatible point cloud.
 * @param datagram Parsed kanaviDatagram.
 * @param cloud_ Output point cloud.
 */
	void generatePointCloud(const kanaviDatagram &datagram, PointCloudT &cloud_);

/**
 * @brief Converts a length measurement and trigonometric values to a 3D point.
 * @param len Distance measurement.
 * @param v_sin Vertical sine.
 * @param v_cos Vertical cosine.
 * @param h_sin Horizontal sine.
 * @param h_cos Horizontal cosine.
 * @return Computed 3D point(XYZRGB).
 */
	PointT length2point(float len, float v_sin, float v_cos, float h_sin, float h_cos);

/**
 * @brief Converts HSV color to RGB color.
 * @param fR Pointer to resulting red value.
 * @param fG Pointer to resulting green value.
 * @param fB Pointer to resulting blue value.
 * @param fH Hue component.
 * @param fS Saturation component.
 * @param fV Value component.
 */
	void HSV2RGB(float *fR, float *fG, float *fB, float fH, float fS, float fV);
	
/**
 * @brief Converts a PCL point cloud to ROS1 PointCloud2 message format.
 * 
 * @param ww Width of the point cloud.
 * @param hh Height of the point cloud.
 * @param cloud Input point cloud (XYZRGB).
 * @param timestamp Time stamp to include in message.
 * @param frame Coordinate frame ID.
 * @return ROS1 PointCloud2 message.
 */
	sensor_msgs::PointCloud2 cloud_to_cloud_msg(int ww, int hh, const pcl::PointCloud<pcl::PointXYZRGB>& cloud, int timestamp, const std::string& frame);

/**
 * @brief Rotates the point cloud around the Z-axis by a given angle.
 * @param cloud Input/output point cloud.
 * @param angle Rotation angle in radians.
 */
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
/**
 * @brief Rotates the entire point cloud around the Z-axis.
 * @param cloud Point cloud to rotate.
 * @param angle Rotation angle (radians).
 */
	kanavi_node(const std::string &node_, int &argc_, char **argv_);
	~kanavi_node();

/**
 * @brief Main loop to receive, process, and publish LiDAR point cloud in ROS1.
 */
	void run();
};
#endif // __KANAVI_NODE_H__