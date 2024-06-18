#include <processor/kanaviProcessor.h>
#include <UDP/kanavi_udp.h>
#include <include/header.h>
#include <thread>
#include <pthread.h>
#include <sstream>
//time
#include <time.h>
#include <sys/timeb.h>
#include <unistd.h>

#define OPENCV_		//opencv output visualization

#ifdef OPENCV_
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define DEG2RAD(deg)	(deg * CV_PI / 180.0)

#endif

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

#ifdef OPENCV_

void HSV2RGB(float *fR, float *fG, float *fB, float fH, float fS, float fV)
{
    float fC = fV * fS;
    float fHPrime = fmod(fH / 60.0, 6);
    float fX = fC * (1 - fabs(fmod(fHPrime, 2) - 1));
    float fM = fV - fC;

    if(0 <= fHPrime && fHPrime < 1)
    {
      *fR = fC;
      *fG = fX;
      *fB = 0;
    }
    else if(0 <= fHPrime && fHPrime < 2)
    {
      *fR = fX;
      *fG = fC;
      *fB = 0;
    }
    else if(0 <= fHPrime && fHPrime < 3)
    {
      *fR = 0;
      *fG = fC;
      *fB = fX;
    }
    else if(0 <= fHPrime && fHPrime < 4)
    {
      *fR = 0;
      *fG = fX;
      *fB = fC;
    }
    else if(0 <= fHPrime && fHPrime < 5)
    {
      *fR = fX;
      *fG = 0;
      *fB = fC;
    }
    else if(0 <= fHPrime && fHPrime < 6)
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

PointXYZ length2Point(float len, float v_sin, float v_cos, float h_sin, float h_cos)
{
	PointXYZ p;

	p.x = len * v_cos * h_cos;
	p.y = len * v_cos * h_sin;
	p.z = len * v_sin;

	return p;
}

cv::Mat convertMat(lidarDatagram dg)
{
	std::vector<PointXYZ> points;

	//calculate resolution value(cos, sin)
	std::vector<float> v_sin, v_cos;
	std::vector<float> h_sin, h_cos;
	switch(dg.LiDAR_Model)
	{
	case KANAVI::MODEL::LiDAR::VL_AS16:
		for(int i=0; i<16; i++)
		{
			v_sin.push_back( sin(DEG2RAD(KANAVI::VL_AS16::SPECIFICATION::VERTICAL_RESOLUTION * (7 - i))) );
			v_cos.push_back( cos(DEG2RAD(KANAVI::VL_AS16::SPECIFICATION::VERTICAL_RESOLUTION * (7 - i))) );
		}
		for(int i=0; i<KANAVI::VL_AS16::SPECIFICATION::HORIZONTAL_DATA_CNT; i++)
		{
			h_sin.push_back( sin(DEG2RAD(KANAVI::VL_AS16::SPECIFICATION::HORIZONTAL_RESOLUTION * i)) );
			h_cos.push_back( cos(DEG2RAD(KANAVI::VL_AS16::SPECIFICATION::HORIZONTAL_RESOLUTION * i)) );
		}
		break;
	case KANAVI::MODEL::LiDAR::VL_R002IF01:
		for(int i=0; i<2; i++)
		{
			v_sin.push_back( sin(DEG2RAD(KANAVI::INDUSTRIAL::SPECIFICATION::R2::VERTICAL_RESOLUTION * i)) );
			v_cos.push_back( cos(DEG2RAD(KANAVI::INDUSTRIAL::SPECIFICATION::R2::VERTICAL_RESOLUTION * i)) );
		}
		for(int i=0; i<KANAVI::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_DATA_CNT; i++)
		{
			h_sin.push_back( sin(DEG2RAD(KANAVI::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_RESOLUTION * i)) );
			h_cos.push_back( cos(DEG2RAD(KANAVI::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_RESOLUTION * i)) );
		}
		break;
	}

	//convert 3D point
	switch(dg.LiDAR_Model)
	{
	case KANAVI::MODEL::LiDAR::VL_AS16:
		for(int ch=0; ch<16; ch++)
		{
			for(int i=0; i<KANAVI::VL_AS16::SPECIFICATION::HORIZONTAL_DATA_CNT; i++)
			{
				points.push_back(length2Point(dg.RAWdata_RadialDistance[ch][i]/100.0, v_sin[ch], v_cos[ch],
											h_sin[i], h_cos[i]));
			}
		}
		break;
	case KANAVI::MODEL::LiDAR::VL_R002IF01:
		for(int ch=0; ch<2; ch++)
		{
			for(int i=0; i<KANAVI::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_DATA_CNT; i++)
			{
				points.push_back(length2Point(dg.industrial_Length[ch][i], v_sin[ch], v_cos[ch],
											h_sin[i], h_cos[i]));
			}
		}
		break;
	}

	//convert 2D
	cv::Mat f(500, 500, CV_8UC3, cv::Scalar(255,255,255));
	cv::line(f, cv::Point(0, 250), cv::Point(500,250), cv::Scalar(0,0,0), 2);
	cv::line(f, cv::Point(250, 0), cv::Point(250,500), cv::Scalar(0,0,0), 2);
	int center = 250;
	float r,g,b;

	float ratio = 10.0;

	for(int i=0; i<points.size(); i++)
	{
		HSV2RGB(&r,&g,&b, points[i].length() * 20, 1.0,1.0);
		cv::circle(f, cv::Point(center + points[i].x*ratio, center + points[i].y*ratio), 2, cv::Scalar(g*255, b*255, r*255), -1);
	}

	return f;
}
#endif

/**
* @mainpage main.cpp
* @brief    LiDAR 프로세서 동작을 확인하기 위한 main
* @details  kanaviLidarProcessor를 통한 데이터 처리를 확인
*/
int main(int argc, char* argv[])
{
	kanaviLidarProcessor *m_KanaviLidar;
	m_KanaviLidar = new kanaviLidarProcessor;
	kanaviUDP *m_udp = new kanaviUDP;

	std::string udpIP = "192.168.100.99";
	int udpPort = 5000;

	bool setMulticast = false;

#ifdef OPENCV_
	cv::Mat outputM;
#endif

    //check argv -- 인자 확인
	if(argc == 0)
	{
		printf("-i [ip] [port]\n");
		printf("-m : multicast");
		printf("VL-AS16 is unicast\n");
		printf("R2 is Multicast\n");
	}
	else
	{
		for(int i=0; i<argc; i++)
		{
			if(!strcmp(argv[i], "-i") || !strcmp(argv[i], "-I"))
			{
				udpIP = argv[i+1];
				udpPort = atoi(argv[i+2]);
			}
			if(!strcmp(argv[i], "-m") || !strcmp(argv[i], "-M"))
			{
				setMulticast = true;
			}
		}
	}

	if(setMulticast)
	{
		printf("set Multicast\n");
		m_udp->setMulticast("224.0.0.5", setMulticast);
	}
	m_udp->InitUDP(udpIP, udpPort);

	printf("[MAIN] UDP set %s, %d\n", udpIP.c_str(), udpPort);

	if(!m_udp->connect())
	{
		perror("UDP connect error\n");
		return -1;
	}

	std::vector<u_char> recv_buf;	//udp recv buf;
	lidarDatagram datagram;

	while(1)
	{
		// printf("wait....\n");
		int cnt =0;
		recv_buf = m_udp->getData();
		// printf("check input buf : %d\n", recv_buf.size());

		datagram = m_KanaviLidar->process(recv_buf);
		// printf("Model : %d\n", datagram.LiDAR_Model);
#ifdef OPENCV_
		//output
		// printf("bool check : %d\n" , datagram.PARA_Input_END);
		if(datagram.PARA_Input_END)
		{
			outputM = convertMat(datagram);
			cv::imshow("demo", outputM);
			if(cv::waitKey(33) == 'c')
			{
				break;
			}
		}
#else
		printBuf(datagram);
#endif
	}

	return 0;
}
