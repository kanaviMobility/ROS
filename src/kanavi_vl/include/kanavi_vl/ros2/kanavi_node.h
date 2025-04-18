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
 * @brief ROS2 node wrapper for Kanavi LiDAR sensor integration.
 *
 * This class manages receiving LiDAR data via UDP, parsing it into structured
 * data formats, converting it into PCL point clouds, and publishing to ROS topics.
 */
class kanavi_node : public rclcpp::Node
{
private:

	//SECTION - FUNCS.

/**
 * @brief Prints help information for using command-line arguments.
 */
	void helpAlarm();

/**
 * @brief Receives UDP data from the LiDAR and initiates parsing.
 */
	void receiveData();

/**
 * @brief Finalizes the node process and cleans up resources.
 */
	void endProcess();

/**
 * @brief Sets up the logging parameters for the ROS2 node.
 */
	void log_set_parameters();


/**
 * @brief Converts raw datagram into an internal point cloud representation.
 * @param datagram Parsed datagram from LiDAR sensor.
 */
	void length2PointCloud(kanaviDatagram datagram);

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
 * @brief Rotates the point cloud around the Z-axis by a given angle.
 * @param cloud Input/output point cloud.
 * @param angle Rotation angle in radians.
 */
	void rotateAxisZ(PointCloudT::Ptr cloud, float angle);
	

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
 * @brief Publishes the given point cloud to a ROS2 topic.
 * @param cloud_ Point cloud to publish.
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
/**
 * @brief Constructor for kanavi_node, sets up the ROS2 node.
 * @param node_ Node name.
 * @param argc_ Argument count.
 * @param argv_ Argument values.
 */
	kanavi_node(const std::string &node_, int &argc_, char **argv_);
	~kanavi_node();

/**
 * @brief Calculates angular resolution and alignment based on LiDAR model.
 * @param model LiDAR model type (e.g., R2, R4, R270).
 */
	void calculateAngular(int model);

	// void publishing();
};
#endif // __KANAVI_NODE_H__