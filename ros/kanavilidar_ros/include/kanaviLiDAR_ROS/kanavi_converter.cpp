#include "kanavi_converter.h"

kanavi_converter::kanavi_converter(/* args */)
{
	g_checked_HorizontalReverse = false;
	gaxesMode = 1;
}

kanavi_converter::~kanavi_converter()
{
}

/**
 * @brief calculate speed of objects that are detected by LiDAR Model
 *
 * @param len - Distance between objects and LiDAR
 */
 
/*void kanavi_converter::calculateSpeed(float len)
{
    auto now = std::chrono::steady_clock::now();
    if (!first_measurement)
    {
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_measurement_time).count();
        if(duration > 0)
        {
            float speed = (len - last_len) * 1000.0 / duration;
            std::cout << "Speed: " << speed << " units/s" << std::endl;
        }
        else
        {
            std::cout << "Time interval is too short for speed calculation" << std::endl;
        }
    }
    last_measurement_time = now;
    last_len = len;
    first_measurement = false;
}*/

/**
 * @brief calculate of angular value using to convert length to 3D point.
 *
 * @param model LiDAR Model
 */ 
void kanavi_converter::calculateAngular(int model)
{
	switch (model)
	{
	case KANAVI::MODEL::LiDAR::VL_AS16:
		for (int i = 0; i < 16; i++)
		{
			v_sin.push_back(sin(DEG2RAD(KANAVI::VL_AS16::SPECIFICATION::VERTICAL_RESOLUTION * (7 - i))));
			v_cos.push_back(cos(DEG2RAD(KANAVI::VL_AS16::SPECIFICATION::VERTICAL_RESOLUTION * (7 - i))));
		}
		if (g_checked_HorizontalReverse)
		{
			for (int i = 0; i < KANAVI::VL_AS16::SPECIFICATION::HORIZONTAL_DATA_CNT; i++)
			{
				h_sin.push_back(sin(DEG2RAD(KANAVI::VL_AS16::SPECIFICATION::HORIZONTAL_RESOLUTION * i)));
				h_cos.push_back(cos(DEG2RAD(KANAVI::VL_AS16::SPECIFICATION::HORIZONTAL_RESOLUTION * i)));
			}
		}
		else
		{
			// for(int i=0; i<KANAVI::VL_AS16::SPECIFICATION::HORIZONTAL_DATA_CNT; i++)
			for (int i = KANAVI::VL_AS16::SPECIFICATION::HORIZONTAL_DATA_CNT - 1; i >= 0; i--)
			{
				h_sin.push_back(sin(DEG2RAD(KANAVI::VL_AS16::SPECIFICATION::HORIZONTAL_RESOLUTION * i)));
				h_cos.push_back(cos(DEG2RAD(KANAVI::VL_AS16::SPECIFICATION::HORIZONTAL_RESOLUTION * i)));
			}
		}
		break;
	case KANAVI::MODEL::LiDAR::VL_R002IK01:
		for (int i = 0; i < KANAVI::INDUSTRIAL::SPECIFICATION::R2::VERTICAL_CHANNEL; i++)
		{
			v_sin.push_back(sin(DEG2RAD(KANAVI::INDUSTRIAL::SPECIFICATION::R2::VERTICAL_RESOLUTION * i)));
			v_cos.push_back(cos(DEG2RAD(KANAVI::INDUSTRIAL::SPECIFICATION::R2::VERTICAL_RESOLUTION * i)));
		}
		if (g_checked_HorizontalReverse)
		{
			for (int i = KANAVI::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_DATA_CNT - 1; i >= 0; i--)
			{
				h_sin.push_back(sin(DEG2RAD(KANAVI::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_RESOLUTION * i)));
				h_cos.push_back(cos(DEG2RAD(KANAVI::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_RESOLUTION * i)));
			}
		}
		else
		{
			for (int i = 0; i < KANAVI::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_DATA_CNT; i++)
			{
				h_sin.push_back(sin(DEG2RAD(KANAVI::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_RESOLUTION * i)));
				h_cos.push_back(cos(DEG2RAD(KANAVI::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_RESOLUTION * i)));
			}
		}
		break;
	/*case KANAVI::MODEL::LiDAR::VL_R001IK02:
		if (g_checked_HorizontalReverse)
		{
			for (int i = KANAVI::INDUSTRIAL::SPECIFICATION::R300::HORIZONTAL_DATA_CNT - 1; i >= 0; i--)
			{
				h_sin.push_back(sin(DEG2RAD(KANAVI::INDUSTRIAL::SPECIFICATION::R300::HORIZONTAL_RESOLUTION * i)));
				h_cos.push_back(cos(DEG2RAD(KANAVI::INDUSTRIAL::SPECIFICATION::R300::HORIZONTAL_RESOLUTION * i)));
			}
		}
		else
		{
			for (int i = 0; i < KANAVI::INDUSTRIAL::SPECIFICATION::R300::HORIZONTAL_DATA_CNT; i++)
			{
				h_sin.push_back(sin(DEG2RAD(KANAVI::INDUSTRIAL::SPECIFICATION::R300::HORIZONTAL_RESOLUTION * i)));
				h_cos.push_back(cos(DEG2RAD(KANAVI::INDUSTRIAL::SPECIFICATION::R300::HORIZONTAL_RESOLUTION * i)));
			}
		}
		break;*/
	case KANAVI::MODEL::LiDAR::VL_R004IK01:
		for (int i = 0; i < KANAVI::INDUSTRIAL::SPECIFICATION::R4::VERTICAL_CHANNEL; i++)
		{
			v_sin.push_back(sin(DEG2RAD(KANAVI::INDUSTRIAL::SPECIFICATION::R4::VERTICAL_RESOLUTION * i)));
			v_cos.push_back(cos(DEG2RAD(KANAVI::INDUSTRIAL::SPECIFICATION::R4::VERTICAL_RESOLUTION * i)));
		}
		if (g_checked_HorizontalReverse)
		{
			for (int i = KANAVI::INDUSTRIAL::SPECIFICATION::R4::HORIZONTAL_DATA_CNT - 1; i >= 0; i--)
			{
				h_sin.push_back(sin(DEG2RAD(KANAVI::INDUSTRIAL::SPECIFICATION::R4::HORIZONTAL_RESOLUTION * i)));
				h_cos.push_back(cos(DEG2RAD(KANAVI::INDUSTRIAL::SPECIFICATION::R4::HORIZONTAL_RESOLUTION * i)));
			}
		}
		else
		{
			for (int i = 0; i < KANAVI::INDUSTRIAL::SPECIFICATION::R4::HORIZONTAL_DATA_CNT; i++)
			{
				h_sin.push_back(sin(DEG2RAD(KANAVI::INDUSTRIAL::SPECIFICATION::R4::HORIZONTAL_RESOLUTION * i)));
				h_cos.push_back(cos(DEG2RAD(KANAVI::INDUSTRIAL::SPECIFICATION::R4::HORIZONTAL_RESOLUTION * i)));
			}
		}
	case KANAVI::MODEL::LiDAR::VL_R001IK03: // Modified part
		if (g_checked_HorizontalReverse)
		{
			for (int i = KANAVI::INDUSTRIAL::SPECIFICATION::R270::HORIZONTAL_DATA_CNT - 1; i >= 0; i--)
			{
				h_sin.push_back(sin(DEG2RAD(KANAVI::INDUSTRIAL::SPECIFICATION::R270::HORIZONTAL_RESOLUTION * i)));
				h_cos.push_back(cos(DEG2RAD(KANAVI::INDUSTRIAL::SPECIFICATION::R270::HORIZONTAL_RESOLUTION * i)));
			}
		}
		else
		{
			for (int i = 0; i < KANAVI::INDUSTRIAL::SPECIFICATION::R270::HORIZONTAL_DATA_CNT; i++)
			{
				h_sin.push_back(sin(DEG2RAD(KANAVI::INDUSTRIAL::SPECIFICATION::R270::HORIZONTAL_RESOLUTION * i)));
				h_cos.push_back(cos(DEG2RAD(KANAVI::INDUSTRIAL::SPECIFICATION::R270::HORIZONTAL_RESOLUTION * i)));
			}
		}
		break;
	}
}

/**
 * @brief generate Point Cloud using LiDAR protocol datagram
 *
 * @param datagram 	lidar data buf
 * @param cloud_ 	point cloud
 */
void kanavi_converter::generatePointCloud(const lidarDatagram &datagram, PointCloudT &cloud_)
{	
	// std::cout << "MODEL: " << datagram.LiDAR_Model << std::endl;
	switch (datagram.LiDAR_Model) // check lidar model
	{
	case KANAVI::MODEL::LiDAR::VL_AS16:		
		for (int ch = 0; ch < KANAVI::VL_AS16::SPECIFICATION::VERTICAL_CHANNEL; ch++)
		{
			for (int i = 0; i < KANAVI::VL_AS16::SPECIFICATION::HORIZONTAL_DATA_CNT; i++)
			{
				// printf("%d %d\n", ch, i);
				cloud_(i, ch) = length2point(datagram.vl_as16.RAWdata_RadialDistance[ch][i] / 100.0,
											 v_sin[ch], v_cos[ch], h_sin[i], h_cos[i]); // convert length to point coordinate
			}
		}
		break;
	case KANAVI::MODEL::LiDAR::VL_R002IK01:		//R2
		for (int ch = 0; ch < KANAVI::INDUSTRIAL::SPECIFICATION::R2::VERTICAL_CHANNEL; ch++)
		{
			// Left for further testing. (24/03/20)
			// std::cout << "Vertical Angle [" << ch << "]: Sin = " << v_sin[ch] << ", Cos = " << v_cos[ch] << std::endl;
			
			for (int i = 0; i < KANAVI::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_DATA_CNT; i++)
			{
			    // Left below commented codes for debugging PCs info and distance info. (24/03/19)
			    
			    // std::cout << "Horizontal Angle [" << i << "]: Sin = " << h_sin[i] << ", Cos = " << h_cos[i] << std::endl;
			    // std::cout << "Length [" << ch << "]["<< i << "]: " << datagram.industrial_Length[ch][i] << std::endl; 
			    
				cloud_(i, ch) = length2point(datagram.industrial_Length[ch][i], v_sin[ch], v_cos[ch], h_sin[i], h_cos[i]); // convert length to point coordinate
				// Left for further testing. (24/03/20)
				// std::cout << "PC width: " << cloud_.width << std::endl;
				// std::cout << "PC height: " << cloud_.height << std::endl;
				// std::cout << "PC size: " << cloud_.size() << std::endl;
				// std::cout << "Is Dense? " << (cloud_.is_dense ? "true" : "false") << std::endl;
			}
		}
		// Left for further testing. (24/03/20)
		/*for (size_t i = 0; i < cloud_.size(); ++i)
		{
		    std::cout << " Point #" << i << ": " << cloud_.points[i].x << ", " << cloud_.points[i].y << ", " << cloud_.points[i].z << std::endl;
		}*/
		break;
	/*case KANAVI::MODEL::LiDAR::VL_R001IK02:		//R300
		for (int i = 0; i < KANAVI::INDUSTRIAL::SPECIFICATION::R300::HORIZONTAL_DATA_CNT; i++)
		{
			cloud_(i, 0) = length2point(datagram.industrial_Length[0][i], 0, 1, h_sin[i], h_cos[i]); // convert length to point coordinate
		}
		break;*/
	
	// Todo - Modify R4 logic similar to R2. (24/03/19)
	
	case KANAVI::MODEL::LiDAR::VL_R004IK01:		//R4
		// printf("convert Length to Point3D %d %d %d %d\n", v_sin.size(), v_cos.size(), h_sin.size(), h_cos.size());
		for (int ch = 0; ch < KANAVI::INDUSTRIAL::SPECIFICATION::R4::VERTICAL_CHANNEL; ch++)
		{
			for (int i = 0; i < KANAVI::INDUSTRIAL::SPECIFICATION::R4::HORIZONTAL_DATA_CNT; i++)
			{
			    // std::cout << "Length [" << ch << "]["<< i << "]: " << datagram.industrial_Length[ch][i] << std::endl;
				cloud_(i, ch) = length2point(datagram.industrial_Length[ch][i], v_sin[ch], v_cos[ch], h_sin[i], h_cos[i]); // convert length to point coordinate
			}
		}
		break;
	case KANAVI::MODEL::LiDAR::VL_R001IK03:		//R270 // Modified part
		for (int i = 0; i < KANAVI::INDUSTRIAL::SPECIFICATION::R270::HORIZONTAL_DATA_CNT; i++)
		{
			// Left for further testing. (24/03/20)
			// pcl::PointXYZRGB point = length2point(datagram.industrial_Length[0][i], 0, 1, h_sin[i], h_cos[i]);
			// cloud_(i, 0) = point;
			// printf("Point[%d]: X :%f, Y :%f, Z :%f, R :%d, G :%d, B :%d\n",
				// i, point.x, point.y, point.z, point.r, point.g, point.b);
			cloud_(i, 0) = length2point(datagram.industrial_Length[0][i], 0, 1, h_sin[i], h_cos[i]); // convert length to point coordinate
		}
		break;
	}
}

/**
 * @brief convert length to 3D point coordinate
 *
 * @param len 		length
 * @param v_sin 	vertical sin value
 * @param v_cos 	vertical cos value
 * @param h_sin 	horizontal sin value
 * @param h_cos 	horizontal cos value
 * @return pcl::PointXYZRGB 	point value
 */
pcl::PointXYZRGB kanavi_converter::length2point(float len, float v_sin, float v_cos, float h_sin, float h_cos)
{
	pcl::PointXYZRGB p_;
		
	switch(gaxesMode)
	{
		case 1:
		p_.x = len * v_cos * h_cos;
		p_.y = len * v_cos * h_sin;
		p_.z = len * v_sin;
		break;
		case 2:
		p_.x = len * v_cos * h_sin;
		p_.y = -(len * v_cos * h_cos);
		p_.z = len * v_sin;
		break;
	}

	float r, g, b;
	HSV2RGB(&r, &g, &b, len * 20, 1.0, 1.0); // convert hsv to rgb
	p_.r = r * 255;
	p_.g = g * 255;
	p_.b = b * 255;

	return p_;
}

/**
 * @brief convert HSV to RGB color
 *
 * @param fR 	return Red color value (0.0 ~ 1.0)
 * @param fG 	return Green color value (0.0 ~ 1.0)
 * @param fB 	return Blue color value (0.0 ~ 1.0)
 * @param fH 	Hue of HSV value (0 to 360)
 * @param fS 	Saturation of HSV value (0.0 ~ 1.0)
 * @param fV 	Value of HSV value (0.0 ~ 1.0)
 */
void kanavi_converter::HSV2RGB(float *fR, float *fG, float *fB, float fH, float fS, float fV)
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
	else if (0 <= fHPrime && fHPrime < 2)
	{
		*fR = fX;
		*fG = fC;
		*fB = 0;
	}
	else if (0 <= fHPrime && fHPrime < 3)
	{
		*fR = 0;
		*fG = fC;
		*fB = fX;
	}
	else if (0 <= fHPrime && fHPrime < 4)
	{
		*fR = 0;
		*fG = fX;
		*fB = fC;
	}
	else if (0 <= fHPrime && fHPrime < 5)
	{
		*fR = fX;
		*fG = 0;
		*fB = fC;
	}
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

/**
 * @brief set datagram of total lidar buf
 *
 * @param datagram
 */
void kanavi_converter::setDatagram(const lidarDatagram &datagram)
{
	float hfov = 0, vfov = 0;
	float hres = 0, vres = 0;
	
	switch (datagram.LiDAR_Model) // check lidar model and set specification
	{
	case KANAVI::MODEL::LiDAR::VL_AS16:
		hfov = KANAVI::VL_AS16::SPECIFICATION::HORIZONTAL_FoV;
		vfov = KANAVI::VL_AS16::SPECIFICATION::VERTICAL_FoV;
		cloud = PointCloudT(KANAVI::VL_AS16::SPECIFICATION::HORIZONTAL_DATA_CNT,
							KANAVI::VL_AS16::SPECIFICATION::VERTICAL_CHANNEL);
		break;
	case KANAVI::MODEL::LiDAR::VL_R002IK01:
		hfov = KANAVI::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_FoV;
		vfov = KANAVI::INDUSTRIAL::SPECIFICATION::R2::VERTICAL_FoV;
		hres = KANAVI::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_RESOLUTION;
		vres = KANAVI::INDUSTRIAL::SPECIFICATION::R2::VERTICAL_RESOLUTION;
		cloud = PointCloudT(KANAVI::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_DATA_CNT,
							KANAVI::INDUSTRIAL::SPECIFICATION::R2::VERTICAL_CHANNEL);
		break;
	/*case KANAVI::MODEL::LiDAR::VL_R001IK02:
		hfov = KANAVI::INDUSTRIAL::SPECIFICATION::R300::HORIZONTAL_FoV;
		vfov = KANAVI::INDUSTRIAL::SPECIFICATION::R300::VERTICAL_FoV;
		cloud = PointCloudT(KANAVI::INDUSTRIAL::SPECIFICATION::R300::HORIZONTAL_DATA_CNT,
							KANAVI::INDUSTRIAL::SPECIFICATION::R300::VERTICAL_CHANNEL);
		break;*/
	case KANAVI::MODEL::LiDAR::VL_R004IK01:
		hfov = KANAVI::INDUSTRIAL::SPECIFICATION::R4::HORIZONTAL_FoV;
		vfov = KANAVI::INDUSTRIAL::SPECIFICATION::R4::VERTICAL_FoV;
		hres = KANAVI::INDUSTRIAL::SPECIFICATION::R4::HORIZONTAL_RESOLUTION;
		vres = KANAVI::INDUSTRIAL::SPECIFICATION::R4::VERTICAL_RESOLUTION;
		cloud = PointCloudT(KANAVI::INDUSTRIAL::SPECIFICATION::R4::HORIZONTAL_DATA_CNT,
							KANAVI::INDUSTRIAL::SPECIFICATION::R4::VERTICAL_CHANNEL);
		break;
	case KANAVI::MODEL::LiDAR::VL_R001IK03: // Modified part
		hfov = KANAVI::INDUSTRIAL::SPECIFICATION::R270::HORIZONTAL_FoV;
		vfov = KANAVI::INDUSTRIAL::SPECIFICATION::R270::VERTICAL_FoV;
		hres = KANAVI::INDUSTRIAL::SPECIFICATION::R270::HORIZONTAL_RESOLUTION;
		vres = KANAVI::INDUSTRIAL::SPECIFICATION::R270::VERTICAL_RESOLUTION;
		cloud = PointCloudT(KANAVI::INDUSTRIAL::SPECIFICATION::R270::HORIZONTAL_DATA_CNT,
							KANAVI::INDUSTRIAL::SPECIFICATION::R270::VERTICAL_CHANNEL);
		break;
	}
	printf("\n[*]Sensor Specification---------\n");
	printf("\tHorizontal FoV\t\t: %f\n"
			"\tHorizontal Resol\t: %f\n"
			"\tVertical FoV\t\t: %f\n"
			"\tVertical Resol\t\t: %f\n",
			hfov, hres, vfov, vres);

	// calculate sin,cos
	if (!checked_setAngular)
	{
		calculateAngular(datagram.LiDAR_Model);
		checked_setAngular = true;
	}
	// calculate point coordinates
	generatePointCloud(datagram, cloud);
}

/**
 * @brief set horizontal reverse
 *
 * @param checked
 */
void kanavi_converter::setReverse(const bool &checked)
{
	g_checked_HorizontalReverse = checked;
}

/**
 * @brief get Point Cloud
 *
 * @return PointCloudT 	pcl::PointCloud<pcl::PointXYZRGB>
 */
PointCloudT kanavi_converter::getPointCloud()
{
	return cloud;
}

/**
 * @brief set visualization basic axes
 * 
 * @param mode 
 */
void kanavi_converter::setaxesMode(const int &mode)
{
	gaxesMode = mode;
}
