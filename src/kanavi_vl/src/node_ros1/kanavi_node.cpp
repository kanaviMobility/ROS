#include "ros1/kanavi_node.h"

/**
 * @brief Construct a new kanavi node::kanavi node object
 *
 * @param node_ Node name
 * @param argc_ argc
 * @param argv_ argv
 */
kanavi_node::kanavi_node(const std::string &node_, int &argc_, char **argv_)
{
	checked_multicast_ = false;
	checked_help_ = false;

	// check help
	for (int i = 0; i < argc_; i++)
	{
		if (!strcmp(argv_[i], "-h")) // output help command
		{
			helpAlarm();
			checked_help_ = true;

			timer_ = nh_.createTimer(ros::Duration(1), std::bind(&kanavi_node::endProcess, this));
			// break;
			exit(1);
		}
	}

	if (!checked_help_)
	{
		// parse PARAMETERS
		m_argv = std::make_unique<argv_parser>(argc_, argv_);

		// get PARAMETERS
		argvContainer argvs = m_argv->getParameters();

		// set PARAMETERS
		local_ip_ = argvs.local_ip;
		port_ = argvs.port;
		multicast_ip_ = argvs.multicast_ip;
		topicName_ = argvs.topicName;
		fixedName_ = argvs.fixedName;
		checked_multicast_ = argvs.checked_multicast;

		log_set_parameters();

		// init UDP network -- multicast Mode
		// SETCTION
		// NEED Uncast mode & Multicast Mode
		//! SETCION
		if(!checked_multicast_)
		{
			m_udp = std::make_unique<kanavi_udp>(local_ip_, port_);
		}
		else
		{
			m_udp = std::make_unique<kanavi_udp>(local_ip_, port_, multicast_ip_);
		}

		// check model using node name
		int model_ = -1;
		if (!strcmp("r270", node_.c_str()))
		{
			model_ = KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R270;
			rotate_angle = KANAVI::COMMON::SPECIFICATION::R270::BASE_ZERO_ANGLE;
		}
/**
 * @brief [TODO] Describe the function if
 * @return else [description]
 * @param !strcmp("r4" [description]
 * @param node_.c_str( [description]
 */
		else if (!strcmp("r4", node_.c_str()))
		{
			model_ = KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R4;
			rotate_angle = KANAVI::COMMON::SPECIFICATION::R4::BASE_ZERO_ANGLE;
		}
/**
 * @brief [TODO] Describe the function if
 * @return else [description]
 * @param !strcmp("r2" [description]
 * @param node_.c_str( [description]
 */
		else if (!strcmp("r2", node_.c_str()))
		{
			model_ = KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R2;
			rotate_angle = KANAVI::COMMON::SPECIFICATION::R2::BASE_ZERO_ANGLE;
		}

		if (model_ < 0)
		{
			return;
		}

		calculateAngular(model_);

		// init LiDAR processor
		kanavi_ = std::make_unique<kanavi_lidar>(model_);

		// init
		// auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
		publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topicName_, 1);

		// init. point cloud
		g_pointcloud.reset(new PointCloudT);
	}
}

/**
 * @brief Destroy the kanavi node::kanavi node object
 *
 */
kanavi_node::~kanavi_node()
{
	m_udp->disconnect();
}

/**
 * @brief when command '-h' input, Display Help Strings
 *
 */
void kanavi_node::helpAlarm()
{
	printf("[HELP]============ \n"
		   "%s : set Network Infromation\n"
		   "\t ex) %s [ip] [port] \n"
		   "%s : set multicast & IP\n"
		   "\t ex) %s [ip]\n"
		   "%s : set fixed frame Name for rviz\n"
		   "%s : set topic name for rviz\n",
		   KANAVI::ROS::PARAMETER_IP.c_str(), KANAVI::ROS::PARAMETER_IP.c_str(), KANAVI::ROS::PARAMETER_Multicast.c_str(), KANAVI::ROS::PARAMETER_Multicast.c_str(), KANAVI::ROS::PARAMETER_FIXED.c_str(), KANAVI::ROS::PARAMETER_TOPIC.c_str());
}

/**
 * @brief receive Data From UDP using Timer
 *
 */
void kanavi_node::receiveData()
{
	// recv data using udp
	std::vector<u_char> buf_ = m_udp->getData();

	// process data

	// check input data size
	if (!buf_.empty())
	{
		// input data processing
		kanavi_->process(buf_);
	}
}

/**
 * @brief Node Process END checker
 *
 */
void kanavi_node::endProcess()
{
	// Clean up resources safely
	this->timer_.stop();
	// Signal to stop spinning
	ros::shutdown();
}

/**
 * @brief output Log for LiDAR & ROS Node Information
 *
 */
void kanavi_node::log_set_parameters()
{
RCLCPP_INFO(node_->get_logger(), "---------KANAVI ROS2------------");
RCLCPP_INFO(node_->get_logger(), "Local IP :\t%s", local_ip_.c_str());
RCLCPP_INFO(node_->get_logger(), "Port Num. :\t%d", port_);
	if (checked_multicast_)
	{
RCLCPP_INFO(node_->get_logger(), "Multicast IP :\t%s", multicast_ip_.c_str());
	}
RCLCPP_INFO(node_->get_logger(), "Fixed Frame Name :\t%s", fixedName_.c_str());
RCLCPP_INFO(node_->get_logger(), "Topic Name :\t%s", topicName_.c_str());
RCLCPP_INFO(node_->get_logger(), "--------------------------------");
}

void kanavi_node::run()
{
/**
 * @brief [TODO] Describe the function rate
 * @return ros::Rate [description]
 * @param 30 [description]
 */
	ros::Rate rate(30);
	// SECTION - Init LiDAR
	// active UDP RECV using timer
	// timer_ = nh_.createTimer(ros::Duration(1), std::bind(&kanavi_node::receiveData, this));

	int udp_return = m_udp->connect();
RCLCPP_INFO(node_->get_logger(), "UDP CONNECT %d", udp_return);

	timer_.start();
	//! SECTION

	// SECTION - RUN ROS Node
	while (ros::ok())
	{
		// recv data from UDP
RCLCPP_INFO(node_->get_logger(), "[NODE] REC DATA");
		receiveData();

		// get Point Cloud from Lidar processor
RCLCPP_INFO(node_->get_logger(), "[NODE] CHECK Process END");
		if (kanavi_->checkedProcessEnd())
		{

			// datagram Length -> pointcloud
			length2PointCloud(kanavi_->getDatagram());

			// rotate Center
			rotateAxisZ(g_pointcloud, rotate_angle);

			// streaming..
RCLCPP_INFO(node_->get_logger(), "[NODE] PULISHING");
			publisher_.publish(cloud_to_cloud_msg(g_pointcloud->width,
												  g_pointcloud->height,
												  *g_pointcloud,
												  100,
												  fixedName_));

			g_pointcloud->clear();
		}
	}
	//! SECTION
}

void kanavi_node::length2PointCloud(kanaviDatagram datagram)
{

	// generate Point Cloud
	generatePointCloud(datagram, *g_pointcloud);

	// printf("CHECK point CLoud NUM : %d\n", g_pointcloud->size());
}

void kanavi_node::calculateAngular(int model)
{
	switch (model)
	{
	case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R2:
		for (int i = 0; i < KANAVI::COMMON::SPECIFICATION::R2::VERTICAL_CHANNEL; i++)
		{
			v_sin.push_back(sin(DEG2RAD(KANAVI::COMMON::SPECIFICATION::R2::VERTICAL_RESOLUTION * i)));
			v_cos.push_back(cos(DEG2RAD(KANAVI::COMMON::SPECIFICATION::R2::VERTICAL_RESOLUTION * i)));
		}
		for (int i = 0; i < KANAVI::COMMON::SPECIFICATION::R2::HORIZONTAL_DATA_CNT; i++)
		{
			h_sin.push_back(sin(DEG2RAD(KANAVI::COMMON::SPECIFICATION::R2::HORIZONTAL_RESOLUTION * i)));
			h_cos.push_back(cos(DEG2RAD(KANAVI::COMMON::SPECIFICATION::R2::HORIZONTAL_RESOLUTION * i)));
		}
		break;
	case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R4:
		for (int i = 0; i < KANAVI::COMMON::SPECIFICATION::R4::VERTICAL_CHANNEL; i++)
		{
			v_sin.push_back(sin(DEG2RAD(KANAVI::COMMON::SPECIFICATION::R4::VERTICAL_RESOLUTION * i)));
			v_cos.push_back(cos(DEG2RAD(KANAVI::COMMON::SPECIFICATION::R4::VERTICAL_RESOLUTION * i)));
		}

		for (int i = 0; i < KANAVI::COMMON::SPECIFICATION::R4::HORIZONTAL_DATA_CNT; i++)
		{
			h_sin.push_back(sin(DEG2RAD(KANAVI::COMMON::SPECIFICATION::R4::HORIZONTAL_RESOLUTION * i)));
			h_cos.push_back(cos(DEG2RAD(KANAVI::COMMON::SPECIFICATION::R4::HORIZONTAL_RESOLUTION * i)));
		}
		break;
	case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R270:
		for (int i = 0; i < KANAVI::COMMON::SPECIFICATION::R270::HORIZONTAL_DATA_CNT; i++)
		{
			h_sin.push_back(sin(DEG2RAD(KANAVI::COMMON::SPECIFICATION::R270::HORIZONTAL_RESOLUTION * i)));
			h_cos.push_back(cos(DEG2RAD(KANAVI::COMMON::SPECIFICATION::R270::HORIZONTAL_RESOLUTION * i)));
		}
		break;
	}
}

void kanavi_node::generatePointCloud(const kanaviDatagram &datagram, PointCloudT &cloud_)
{
	switch (datagram.model)
	{
	case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R2:
		for (int ch = 0; ch < KANAVI::COMMON::SPECIFICATION::R2::VERTICAL_CHANNEL; ch++)
		{
			for (int i = 0; i < KANAVI::COMMON::SPECIFICATION::R2::HORIZONTAL_DATA_CNT; i++)
			{
				cloud_.push_back(length2point(datagram.len_buf[ch][i], v_sin[ch], v_cos[ch], h_sin[i], h_cos[i]));
			}
		}
		break;
	case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R4:
		for (int ch = 0; ch < KANAVI::COMMON::SPECIFICATION::R4::VERTICAL_CHANNEL; ch++)
		{
			for (int i = 0; i < KANAVI::COMMON::SPECIFICATION::R4::HORIZONTAL_DATA_CNT; i++)
			{
				cloud_.push_back(length2point(datagram.len_buf[ch][i], v_sin[ch], v_cos[ch], h_sin[i], h_cos[i]));
			}
		}
		break;
	case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R270:

		for (int i = 0; i < KANAVI::COMMON::SPECIFICATION::R270::HORIZONTAL_DATA_CNT; i++)
		{
			cloud_.push_back(length2point(datagram.len_buf[0][i], 0, 1, h_sin[i], h_cos[i]));
		}

		break;
	}
}

PointT kanavi_node::length2point(float len, float v_sin, float v_cos, float h_sin, float h_cos)
{
	pcl::PointXYZRGB p_;

	p_.x = len * v_cos * h_cos;
	p_.y = len * v_cos * h_sin;
	p_.z = len * v_sin;

	float r, g, b;
	HSV2RGB(&r, &g, &b, len * 20, 1.0, 1.0); // convert hsv to rgb
	p_.r = r * 255;
	p_.g = g * 255;
	p_.b = b * 255;

	return p_;
}

void kanavi_node::HSV2RGB(float *fR, float *fG, float *fB, float fH, float fS, float fV)
{
	float fC = fV * fS;
	float fHPrime = fmod(fH / 60.0, 6);
	float fX = fC * (1 - fabs(fmod(fHPrime, 2) - 1));
	float fM = fV - fC;

	if (0 <= fHPrime && fHPrime < 1)
	{
		*fR = fC;
		*fG = fX;
		*fB = 0;
	}
/**
 * @brief [TODO] Describe the function if
 * @return else [description]
 * @param 2 [description]
 */
	else if (0 <= fHPrime && fHPrime < 2)
	{
		*fR = fX;
		*fG = fC;
		*fB = 0;
	}
/**
 * @brief [TODO] Describe the function if
 * @return else [description]
 * @param 3 [description]
 */
	else if (0 <= fHPrime && fHPrime < 3)
	{
		*fR = 0;
		*fG = fC;
		*fB = fX;
	}
/**
 * @brief [TODO] Describe the function if
 * @return else [description]
 * @param 4 [description]
 */
	else if (0 <= fHPrime && fHPrime < 4)
	{
		*fR = 0;
		*fG = fX;
		*fB = fC;
	}
/**
 * @brief [TODO] Describe the function if
 * @return else [description]
 * @param 5 [description]
 */
	else if (0 <= fHPrime && fHPrime < 5)
	{
		*fR = fX;
		*fG = 0;
		*fB = fC;
	}
/**
 * @brief [TODO] Describe the function if
 * @return else [description]
 * @param 6 [description]
 */
	else if (0 <= fHPrime && fHPrime < 6)
	{
		*fR = fC;
		*fG = 0;
		*fB = fX;
	}
	else
	{
		*fR = 0;
		*fG = 0;
		*fB = 0;
	}
}

sensor_msgs::PointCloud2 kanavi_node::cloud_to_cloud_msg(int ww, int hh, const pcl::PointCloud<pcl::PointXYZRGB> &cloud, int timestamp, const std::string &frame)
{
	sensor_msgs::PointCloud2 msg{};
	pcl::toROSMsg(cloud, msg);

	// msg.header.stamp.fromNSec(100000000);
	msg.header.stamp = ros::Time::now();

	msg.header.frame_id = frame;

	return msg;
}

void kanavi_node::rotateAxisZ(PointCloudT::Ptr cloud, float angle)
{
	float rad = pcl::deg2rad(angle);
	Eigen::Matrix4f m_ = Eigen::Matrix4f::Identity();
	m_(0, 0) = cos(rad);
	m_(0, 1) = -sin(rad);
	m_(1, 0) = sin(rad);
	m_(1, 1) = cos(rad);

	pcl::transformPointCloud(*cloud, *cloud, m_);
}