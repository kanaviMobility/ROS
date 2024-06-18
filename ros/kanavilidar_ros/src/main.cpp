#include "../include/kanaviLiDAR_ROS/kanaviLiDAR_ros.h"
#include "../include/kanaviLiDAR_ROS/kanavi_converter.h"
#include <pcl/common/common.h>
#include <pcl/common/impl/angles.hpp>
#include <pcl/common/transforms.h>

#include <stdlib.h>
#include "../include/kanaviLiDAR_ROS/config.h"

void printBuf(lidarDatagram dg)
{
	int line = 0;
	switch(dg.LiDAR_Model)
	{
	case KANAVI::MODEL::LiDAR::VL_AS16:
		for(int ch=0; ch<KANAVI::VL_AS16::SPECIFICATION::VERTICAL_CHANNEL; ch++)
		{
			for(int i=0; i<KANAVI::VL_AS16::SPECIFICATION::HORIZONTAL_DATA_CNT; i++)
			{
				printf("[%.2d][%.4d] : %.4d [cm] ", ch, dg.vl_as16.RAWdata_Angle[i], 
											dg.vl_as16.RAWdata_RadialDistance[ch][i]);
				line++;
				if(line ==5)
				{
					printf("\n");
					line = 0;
				}
			}
			printf("\n");
			line = 0;
		}
		break;
	case KANAVI::MODEL::LiDAR::VL_R002IF01:
		for(int ch=0; ch<2; ch++)
		{
			for(int i=0; i<KANAVI::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_DATA_CNT; i++)
			{
				printf("[%.1d][%.3d] : %f [m] ", ch, i, dg.industrial_Length[ch][i]);
				line++;
				if(line ==5)
				{
					printf("\n");
					line = 0;
				}
			}
			printf("\n");
			line = 0;
		}
		break;
	}
}

void helpAlarm()
{
	printf("[HELP]============ "
		"-i / -I : set Network Infromation\n"
		"\t ex) -i [ip] [port] \n"
		"-m / -M: set multicast & IP\n"
		"\t ex) -multicast [ip]\n"
		"-fixed : set fixed frame Name for rviz\n"
		"-topic : set topic name for rviz\n"
		"-s : set input LiDAR IP address\n"
		"\t ex) -s 192.168.123.99\n"
		"-hr : horizontal Reverse\n"
		"\t ex) -hr\n"
		"-fl : config file load\n"
		"\t ex) -fl ~/catkin_ws/src/kanavilidar_ros/config/[fileName].ini\n"
		"-fs : to save configuration using ini File\n"
		"\t ex) -i 192.168.xxx.xxx 5000 -fs [fileName].ini\n"
		"-axes : to set basic axes\n"
		"\t opions================\n "
		"\t -axes 1 \n"
		"\t\t\t\tz   y\t\t\t\n"
		"\t\t\t\t|  7\t\t\t\t\n"
		"\t\t\t\t| /\t\t\t\t\n"
		"\t\t\t\t|/\t\t\t\t	\n"
		"\t\t\t\t+----------->x\t\n"
		"\t -axes 2 \n"
		"\t\t\t\tz   x\t\t\t\t\n"
		"\t\t\t\t|  7\t\t\t\t\n"
		"\t\t\t\t| /\t\t\t\t\n"
		"\t\t\t\t|/\t\t\t\t\n"
		" y<------------+\t\t\t\t\n"
		"[VL-AS16] UDP Mode : Unicast\n"
		"          Base IP  : 192.168.123.99, 5000"
		"[Industrial] UDP Mode : Multicast\n"
		"             Base IP  : 192.168.123.99, 5000, 224.0.0.5\n");
}

void dpNowCondition(std::string lip, int port, std::string gip, bool multi, std::string frame_name)
{
	if(multi)
	{
		printf("[*][argv SET]========\n"
			"\tLocal IP\t: %s\n"
			"\tPort\t: %i\n"
			"\tMultiCast IP\t: %s\n"
			"\tFixed Frame\t: %s\n", lip.c_str(), port, gip.c_str(), frame_name.c_str());
	}
	else
	{
		printf("[*][argv SET]========\n"
			"\tLocal IP\t: %s\n"
			"\tPort\t: %i\n"
			"\tFixed Frame\t: %s\n", lip.c_str(), port, frame_name.c_str());
	}
}

sensor_msgs::PointCloud2 cloud_to_cloud_msg(int ww, int hh, const pcl::PointCloud<pcl::PointXYZRGB>& cloud, int timestamp, const std::string& frame) 
{
	sensor_msgs::PointCloud2 msg{};
	pcl::toROSMsg(cloud, msg);

	//msg.header.stamp.fromNSec(100000000);
	msg.header.stamp = ros::Time::now();

	msg.header.frame_id = frame;

	return msg;
}   // convert cloud_msg to ROS

/**
* @mainpage main.cpp
* @brief    LiDAR 프로세서 동작을 확인하기 위한 main
* @details  kanaviLidarProcessor를 통한 데이터 처리를 확인
*/
int main(int argc, char* argv[])
{

	/* ---define ROS------*/
	ros::init(argc, argv, "Kanavi_cloud_node");	//ROS init
	ros::Publisher lidar_pub;						//publisher define
	ros::NodeHandle nh;								//노드 핸들러 define
	// ros::Rate loop_rate(15); 						//30Hz - freq.
	std::string name_fixedFrame = "map";
	std::string name_topic = "/points";
	bool checked_h_reverse = false;
	std::string config_file;						//get config ini file name
	bool checked_iniSave = false;
	bool checked_iniLoad = false;

	/*---define UDP communcation Var.*/
	kanaviUDP *m_udp = new kanaviUDP;		// kanavi UDP processor

	std::string udpIP = "192.168.123.99";			// local UDP ethernet Port IP
	std::string g_udpIP = "224.0.0.5";				// UDP multicast Group IP
	int udpPort = 5000;								// UDP Port Num
	bool setMulticast = false;						// check UDP Multicast FUNC.

	bool checked_sensorIP = false;					//check transmit LiDAR sensor IP
	std::string sensor_IP = "192.168.123.200";		//set transmit LiDAR sensor IP

	kanaviLidarProcessor *m_KanaviLidar;		// kanavi LiDAR processor
	m_KanaviLidar = new kanaviLidarProcessor;
	kanavi_converter *m_convertor = new kanavi_converter;

	node_config *m_config	= new node_config;		// configuration file
	iniConfig configList;							//about configuration structure

	float z_rotat_angle = 0;

	int axesMode = 1;								// axes mode select

	//check argv -- 인자 확인
	if(argc == 0)
	{
		helpAlarm();
		return 0;
	}
	else
	{
		for(int i=0; i<argc; i++)
		{
			if(!strcmp(argv[i], "-i") || !strcmp(argv[i], "-I"))		// check ARGV - IP & port num.
			{
				udpIP = argv[i+1];
				udpPort = atoi(argv[i+2]);
			}
			else if(!strcmp(argv[i], "-m") || !strcmp(argv[i], "-M"))	// check ARGV - udp multicast ip
			{
				setMulticast = true;
				g_udpIP = argv[i+1];
			}
			else if(!strcmp(argv[i], "-fixed"))							// check ARGV - ROS Fixed name
			{
				name_fixedFrame = argv[i+1];
			}
			else if(!strcmp(argv[i], "-topic"))							// check ARGV - ROS topic name
			{
				name_topic = argv[i+1];
			}
			else if(!strcmp(argv[i], "-s"))								// check ARGV - sensor IP
			{
				sensor_IP = argv[i+1];
				checked_sensorIP = true;
			}
			else if(!strcmp(argv[i], "-hr"))							//check ARGV - horizontal reverse
			{
				checked_h_reverse = true;
			}
			else if(!strcmp(argv[i], "-fl"))							// check ARGV - load config file
			{
				if(i+1 >= argc)
				{
					perror("Please Insert Config File(.ini)\n");
					return -1;
				}
				config_file = argv[i+1];
				checked_iniLoad = true;
				if(config_file.empty())
				{
					perror("Please Insert Config File(.ini)\n");
					return -1;
				}
			}
			else if(!strcmp(argv[i], "-fs"))							// check ARGV - save configuratoin
			{
				config_file = argv[i+1];
				checked_iniSave = true;
				if(config_file.empty())
				{
					perror("Please Insert Config File name(.ini)\n");
					return -1;
				}
			}
			else if(!strcmp(argv[i], "-axes"))
			{
				axesMode = atoi(argv[i+1]);
			}
			else if(!strcmp(argv[i], "-h"))								//output help command
			{
				helpAlarm();
				return 0;
			}
		}
	}

	//read config files(.ini).
	if(checked_iniLoad)
	{
		configList = m_config->getConfig(config_file);

		udpIP	= configList.ip_;						// load ip
		udpPort	= configList.port_;						// load port num.

		g_udpIP	= configList.m_ip_;						// load multicast ip
		if(!g_udpIP.empty())							// if multicast ip is not Null
			setMulticast = true;						// set Multicast On

		name_topic 		= configList.topic_;			// load topic name
		name_fixedFrame	= configList.fixed_;			// load fixed name
		
		sensor_IP = configList.sensor_ip_;				// load sensor IP
		if(!sensor_IP.empty())							// if sensor ip is not Null
			checked_sensorIP = true;					// set to check sensor IP
		
		checked_h_reverse = configList.h_reverse;		// load horizontal reverse

		// print load configuration
		printf("*Load Config File==================\n ");
		printf("\t Local IP\t: %s\n", udpIP.c_str());
		printf("\t Local port\t: %d\n", udpPort);
		printf("\t multicast group IP\t: %s\n", g_udpIP.c_str());
		printf("\t Fixed Frame\t: %s\n", name_fixedFrame.c_str());
		printf("\t Topic\t: %s\n", name_topic.c_str());
		printf("\t Sensor IP\t: %s\n", sensor_IP.c_str());
		printf("\t horizontal Reverse\t: %d\n", checked_h_reverse);
	}

	//generate config file(.ini)
	if(checked_iniSave)
	{
		printf("Save Config...\n");
		configList.ip_ = udpIP;
		configList.port_ = udpPort;
		if(setMulticast)
			configList.m_ip_ = g_udpIP;
		configList.topic_ = name_topic;
		configList.fixed_ = name_fixedFrame;
		if(checked_sensorIP)
			configList.sensor_ip_ = sensor_IP;
		configList.h_reverse = checked_h_reverse;

		m_config->saveConfig(config_file, configList);
		printf("**Generate Config File %s\n", (INI_PATH + config_file).c_str());
	}

	sleep(3);

	lidar_pub = nh.advertise<sensor_msgs::PointCloud2>(name_topic, 1);	// "/test_points"에 대한 메시지 구성을 PointCloud2로 하여 보낼것을 master에게 알림

	//set UDP config
	if(setMulticast)
	{
		printf("*set Multicast\n");
		m_udp->setMulticast(g_udpIP, setMulticast);		//set mutlicast
	}
	m_udp->InitUDP(udpIP, udpPort);							//set basic udp config.

	if(checked_sensorIP)
	{
		m_udp->setLidarIP(sensor_IP);
	}

	if(!m_udp->connect())									//check udp connect
	{
		perror("UDP connect error\n");
		return -1;
	}

	std::vector<u_char> recv_buf;		//udp recv buf;
	lidarDatagram datagram;			//lidar total buf struct
	Eigen::Matrix4f transform_Z = Eigen::Matrix4f::Identity();	// idle vector using point cloud rotation

	clock_t start,end;					// check processing time
	double result;						// check processing time
	start = clock();

	while (ros::ok())	//ros loop....
	{
		start = clock();
		// printf("loop\n");
		dpNowCondition(udpIP, udpPort, g_udpIP, setMulticast, name_fixedFrame);		//output Configuration condition

		int cnt =0;
		recv_buf = m_udp->getData();												//get LiDAR data using udp
		printf("recv_buf size %d\n", recv_buf.size());

		datagram = m_KanaviLidar->process(recv_buf);								//processing, get datagram
		end = clock();
		result = (double)(end - start);
		cout << "Stage 1 Time : "<< ((result)/CLOCKS_PER_SEC) * 1000 << " microseconds" << endl;

		switch(m_KanaviLidar->getLiDARModel())										//check lidar model for point cloud rotation
		{
		case static_cast<int>(KANAVI::MODEL::LiDAR::VL_AS16) :
			z_rotat_angle = 17.5;
			break;
		case KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::MODEL::VL_R002IF01:
			z_rotat_angle = 30;
			break;
		case KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::MODEL::VL_R001IK02:
			z_rotat_angle = -60;
			break;
		case KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::MODEL::VL_R004IK02:
			z_rotat_angle = 35;
			break;
		default:
			break;
		}

		//rotation Z-axes
		float rad = pcl::deg2rad(z_rotat_angle);
		transform_Z(0,0) = cos(rad);
		transform_Z(0,1) = -sin(rad);
		transform_Z(1,0) = sin(rad);
		transform_Z(1,1) = cos(rad);

		if(datagram.PARA_Input_END)								// All data processing END.
		{
			// printf("DISPLAY....\n");
			m_convertor->setReverse(checked_h_reverse);			// check horizontal angle Reverse
			m_convertor->setDatagram(datagram);					// convert Length array to Point Cloud
			m_convertor->setaxesMode(axesMode);					// set axes mode in 3D visualization
			PointCloudT cloud = m_convertor->getPointCloud();	// get pcl::PointCloud<pcl::PointXYZRGB>
			printf("point size : %d\n", cloud.size());
			
			//rotation Z
			pcl::transformPointCloud(cloud, cloud, transform_Z);	//rotation Point Cloud

			lidar_pub.publish(cloud_to_cloud_msg(cloud.width, cloud.height, cloud, 100, name_fixedFrame));		//broadcast MSG using ROS
		}
		end = clock();
		result = (double)(end - start);
		cout << "Stage 2 Time : "<< ((result)/CLOCKS_PER_SEC) * 1000 << " microseconds" << endl;
		// printBuf(datagram);								//dp datagram
		datagram.clear();

		ros::spinOnce();		//update
		// loop_rate.sleep();

		end = clock();
		result = (double)(end - start);
		cout << "Processing Time : "<< ((result)/CLOCKS_PER_SEC)<<" seconds" << endl;
		cout << "Processing Time : "<< ((result)/CLOCKS_PER_SEC) * 1000 << " microseconds" << endl;
	}

	delete m_udp;
	delete m_convertor;
	delete m_KanaviLidar;
	
	return 0;
}
