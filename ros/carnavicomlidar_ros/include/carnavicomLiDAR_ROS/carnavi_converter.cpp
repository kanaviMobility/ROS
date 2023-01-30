#include "carnavi_converter.h"

carnavi_converter::carnavi_converter(/* args */)
{
	g_checked_HorizontalReverse = false;
}

carnavi_converter::~carnavi_converter()
{
}

/**
 * @brief calculate of angular value using to convert length to 3D point.
 *
 * @param model LiDAR Model
 */
void carnavi_converter::calculateAngular(int model)
{
	switch (model)
	{
	case CARNAVICOM::MODEL::LiDAR::VL_AS16:
		for (int i = 0; i < 16; i++)
		{
			v_sin.push_back(sin(DEG2RAD(CARNAVICOM::VL_AS16::SPECIFICATION::VERTICAL_RESOLUTION * (7 - i))));
			v_cos.push_back(cos(DEG2RAD(CARNAVICOM::VL_AS16::SPECIFICATION::VERTICAL_RESOLUTION * (7 - i))));
		}
		if (g_checked_HorizontalReverse)
		{
			for (int i = 0; i < CARNAVICOM::VL_AS16::SPECIFICATION::HORIZONTAL_DATA_CNT; i++)
			{
				h_sin.push_back(sin(DEG2RAD(CARNAVICOM::VL_AS16::SPECIFICATION::HORIZONTAL_RESOLUTION * i)));
				h_cos.push_back(cos(DEG2RAD(CARNAVICOM::VL_AS16::SPECIFICATION::HORIZONTAL_RESOLUTION * i)));
			}
		}
		else
		{
			// for(int i=0; i<CARNAVICOM::VL_AS16::SPECIFICATION::HORIZONTAL_DATA_CNT; i++)
			for (int i = CARNAVICOM::VL_AS16::SPECIFICATION::HORIZONTAL_DATA_CNT - 1; i >= 0; i--)
			{
				h_sin.push_back(sin(DEG2RAD(CARNAVICOM::VL_AS16::SPECIFICATION::HORIZONTAL_RESOLUTION * i)));
				h_cos.push_back(cos(DEG2RAD(CARNAVICOM::VL_AS16::SPECIFICATION::HORIZONTAL_RESOLUTION * i)));
			}
		}
		break;
	case CARNAVICOM::MODEL::LiDAR::VL_R002IF01:
		for (int i = 0; i < 2; i++)
		{
			v_sin.push_back(sin(DEG2RAD(CARNAVICOM::INDUSTRIAL::SPECIFICATION::R2::VERTICAL_RESOLUTION * i)));
			v_cos.push_back(cos(DEG2RAD(CARNAVICOM::INDUSTRIAL::SPECIFICATION::R2::VERTICAL_RESOLUTION * i)));
		}
		if (g_checked_HorizontalReverse)
		{
			for (int i = CARNAVICOM::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_DATA_CNT - 1; i >= 0; i--)
			{
				h_sin.push_back(sin(DEG2RAD(CARNAVICOM::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_RESOLUTION * i)));
				h_cos.push_back(cos(DEG2RAD(CARNAVICOM::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_RESOLUTION * i)));
			}
		}
		else
		{
			for (int i = 0; i < CARNAVICOM::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_DATA_CNT; i++)
			{
				h_sin.push_back(sin(DEG2RAD(CARNAVICOM::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_RESOLUTION * i)));
				h_cos.push_back(cos(DEG2RAD(CARNAVICOM::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_RESOLUTION * i)));
			}
		}
		break;
	case CARNAVICOM::MODEL::LiDAR::VL_R001IK02:
		if (g_checked_HorizontalReverse)
		{
			for (int i = CARNAVICOM::INDUSTRIAL::SPECIFICATION::R300::HORIZONTAL_DATA_CNT - 1; i >= 0; i--)
			{
				h_sin.push_back(sin(DEG2RAD(CARNAVICOM::INDUSTRIAL::SPECIFICATION::R300::HORIZONTAL_RESOLUTION * i)));
				h_cos.push_back(cos(DEG2RAD(CARNAVICOM::INDUSTRIAL::SPECIFICATION::R300::HORIZONTAL_RESOLUTION * i)));
			}
		}
		else
		{
			for (int i = 0; i < CARNAVICOM::INDUSTRIAL::SPECIFICATION::R300::HORIZONTAL_DATA_CNT; i++)
			{
				h_sin.push_back(sin(DEG2RAD(CARNAVICOM::INDUSTRIAL::SPECIFICATION::R300::HORIZONTAL_RESOLUTION * i)));
				h_cos.push_back(cos(DEG2RAD(CARNAVICOM::INDUSTRIAL::SPECIFICATION::R300::HORIZONTAL_RESOLUTION * i)));
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
void carnavi_converter::generatePointCloud(const carnaviDatagram &datagram, PointCloudT &cloud_)
{
	switch (datagram.LiDAR_Model) // check lidar model
	{
	case CARNAVICOM::MODEL::LiDAR::VL_AS16:		
		for (int ch = 0; ch < CARNAVICOM::VL_AS16::SPECIFICATION::VERTICAL_CHANNEL; ch++)
		{
			for (int i = 0; i < CARNAVICOM::VL_AS16::SPECIFICATION::HORIZONTAL_DATA_CNT; i++)
			{
				// printf("%d %d\n", ch, i);
				cloud_(i, ch) = length2point(datagram.vl_as16.RAWdata_RadialDistance[ch][i] / 100.0,
											 v_sin[ch], v_cos[ch], h_sin[i], h_cos[i]); // convert length to point coordinate
			}
		}
		break;
	case CARNAVICOM::MODEL::LiDAR::VL_R002IF01:		//R2
		for (int ch = 0; ch < CARNAVICOM::INDUSTRIAL::SPECIFICATION::R2::VERTICAL_CHANNEL; ch++)
		{
			for (int i = 0; i < CARNAVICOM::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_DATA_CNT; i++)
			{
				cloud_(i, ch) = length2point(datagram.industrial_Length[ch][i], v_sin[ch], v_cos[ch], h_sin[i], h_cos[i]); // convert length to point coordinate
			}
		}
		break;
	case CARNAVICOM::MODEL::LiDAR::VL_R001IK02:		//R300
		for (int i = 0; i < CARNAVICOM::INDUSTRIAL::SPECIFICATION::R300::HORIZONTAL_DATA_CNT; i++)
		{
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
pcl::PointXYZRGB carnavi_converter::length2point(float len, float v_sin, float v_cos, float h_sin, float h_cos)
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
void carnavi_converter::HSV2RGB(float *fR, float *fG, float *fB, float fH, float fS, float fV)
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
void carnavi_converter::setDatagram(const carnaviDatagram &datagram)
{
	float hfov = 0, vfov = 0;
	switch (datagram.LiDAR_Model) // check lidar model and set specification
	{
	case CARNAVICOM::MODEL::LiDAR::VL_AS16:
		hfov = CARNAVICOM::VL_AS16::SPECIFICATION::HORIZONTAL_FoV;
		vfov = CARNAVICOM::VL_AS16::SPECIFICATION::VERTICAL_FoV;
		cloud = PointCloudT(CARNAVICOM::VL_AS16::SPECIFICATION::HORIZONTAL_DATA_CNT,
							CARNAVICOM::VL_AS16::SPECIFICATION::VERTICAL_CHANNEL);
		break;
	case CARNAVICOM::MODEL::LiDAR::VL_R002IF01:
		hfov = CARNAVICOM::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_FoV;
		vfov = CARNAVICOM::INDUSTRIAL::SPECIFICATION::R2::VERTICAL_FoV;
		cloud = PointCloudT(CARNAVICOM::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_DATA_CNT,
							CARNAVICOM::INDUSTRIAL::SPECIFICATION::R2::VERTICAL_CHANNEL);
		break;
	case CARNAVICOM::MODEL::LiDAR::VL_R001IK02:
		hfov = CARNAVICOM::INDUSTRIAL::SPECIFICATION::R300::HORIZONTAL_FoV;
		vfov = CARNAVICOM::INDUSTRIAL::SPECIFICATION::R300::VERTICAL_FoV;
		cloud = PointCloudT(CARNAVICOM::INDUSTRIAL::SPECIFICATION::R300::HORIZONTAL_DATA_CNT,
							CARNAVICOM::INDUSTRIAL::SPECIFICATION::R300::VERTICAL_CHANNEL);
		break;
	}
	printf("\n[*]Sensor Spcification---------\n");
	printf("\tHorizontal FoV\t\t: %f\n"
		   "\tHorizontal Resol\t: %f\n"
		   "\tVertical FoV\t\t: %f\n"
		   "\tVertical Resol\t\t: %f\n",
		   hfov, datagram.PARA_Horizontal_Resolution, vfov, datagram.PARA_Vertical_Resolution);

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
void carnavi_converter::setReverse(const bool &checked)
{
	g_checked_HorizontalReverse = checked;
}

/**
 * @brief get Point Cloud
 *
 * @return PointCloudT 	pcl::PointCloud<pcl::PointXYZRGB>
 */
PointCloudT carnavi_converter::getPointCloud()
{
	return cloud;
}