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

#ifndef HEADER_H_H
#define HEADER_H_H

/**
 * @file header.h
 * @author twchong (twchong@kanavi-mobility.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <iostream>
#include <sstream>
#include <math.h>
#include <time.h>
#include <chrono>

#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <sys/un.h>
#include <stdint.h>

#include "lidar_spec.h"

#define BUFFER_SIZE 65000

#define INDUSTRAL_MAX_CH	KANAVI::INDUSTRIAL::SPECIFICATION::R4::VERTICAL_CHANNEL
#define INDUSTRAL_MAX_CNT	KANAVI::INDUSTRIAL::SPECIFICATION::R4::HORIZONTAL_DATA_CNT
// #define INDUSTRAL_MAX_CNT_R4       KANAVI::INDUSTRIAL::SPECIFICATION::R4::HORIZONTAL_DATA_CNT

typedef struct pointXYZ{
    float x;
    float y;
    float z;

    pointXYZ()
    {
        //initialize
        x = 0;
        y = 0;
        z = 0;
    }

    float length(){
        return sqrt(pow(x,2) + pow(y,2) + pow(z,2));
    }
    void clear(){
        x = 0;
        y = 0;
        z = 0;
    }
}PointXYZ;

typedef struct vl_as16_
{
	uint8_t     HEADER_SerialID[4]; //[0~3]
    uint16_t    HEADER_StartAngle;  //[4~5]
    uint16_t    HEADER_EndAngle;    //[6~7]
    uint16_t    HEADER_DataLength;  //[8~9]
    uint8_t     HEADER_DataType;    //[10]
    uint8_t     HEADER_NTPTime[8];  //[11~18]
    uint8_t     HEADER_reserved1;   //[19]

    int		RAWdata_Angle[KANAVI::VL_AS16::SPECIFICATION::HORIZONTAL_DATA_CNT];
	int		RAWdata_RadialDistance[16][KANAVI::VL_AS16::SPECIFICATION::HORIZONTAL_DATA_CNT];
	int		RAWdata_Intensity[16][KANAVI::VL_AS16::SPECIFICATION::HORIZONTAL_DATA_CNT];
    int		RAWdata_reserved[KANAVI::VL_AS16::SPECIFICATION::HORIZONTAL_DATA_CNT];

    int                     OBJ_CNT;
    std::vector<uint8_t>	OBJ_ID;
    std::vector<float>		OBJ_xMax;
    std::vector<float>		OBJ_yMax;  //rewrite -> 2byte
    std::vector<float>		OBJ_zMax;
    std::vector<float>		OBJ_xMin;
    std::vector<float>		OBJ_yMin;      //rewrite -> 2byte
    std::vector<float>		OBJ_zMin;
    std::vector<uint8_t>	OBJ_Classification;
    std::vector<uint8_t>	OBJ_Status;
    std::vector<uint8_t>	OBJ_Relative_Velocity;
    std::vector<uint8_t>	OBJ_Relative_Acceleration;
    std::vector<uint16_t>	OBJ_Relative_Angle;
    std::vector<uint16_t>	OBJ_Relative_YawRate;
    std::vector<uint8_t>	OBJ_Current_Age;
    std::vector<uint8_t>	OBJ_Prediction_Age;

    int8_t			ErrorNWarning_Internal_Temp;
    uint8_t			ErrorNWarning_Motor_Status;
    uint8_t			ErrorNWarning_APD_Voltage;
    uint8_t			ErrorNWarning_PnT_Status;
    uint8_t			ErrorNWarning_Reserved1;
    uint8_t			ErrorNWarning_CANID00;
    uint8_t			ErrorNWarning_CANID01;
    uint8_t			ErrorNWarning_AliveCNT;

	/* data */
	vl_as16_(){
		HEADER_SerialID[0] = 1; //[0~3]
        HEADER_SerialID[1] = 2; //[0~3]
        HEADER_SerialID[2] = 3; //[0~3]
        HEADER_SerialID[3] = 4; //[0~3]
        HEADER_StartAngle = 0;  //[4~5]
        HEADER_EndAngle = 1159;    //[6~7]
        HEADER_DataLength = 18544;  //[8~9]
        HEADER_DataType = 1;    //[10]
        for(int i=0; i<8; i++)
        {
            HEADER_NTPTime[i] = 0;
        }
        HEADER_reserved1 = 0;   //[19]
        OBJ_CNT = 0;

        ErrorNWarning_Internal_Temp=0;
        ErrorNWarning_Motor_Status=0;
        ErrorNWarning_APD_Voltage=0;
        ErrorNWarning_PnT_Status=0;
        ErrorNWarning_Reserved1=0;
        ErrorNWarning_CANID00=0;
        ErrorNWarning_CANID01=0;
        ErrorNWarning_AliveCNT=0;
	}

	void clear() {
		memset(RAWdata_Angle, 0, KANAVI::VL_AS16::SPECIFICATION::HORIZONTAL_DATA_CNT);
        memset(RAWdata_reserved, 0, KANAVI::VL_AS16::SPECIFICATION::HORIZONTAL_DATA_CNT);
		for(int i=0; i<16; i++)
		{
			memset(RAWdata_RadialDistance[i], 0, KANAVI::VL_AS16::SPECIFICATION::HORIZONTAL_DATA_CNT);
			memset(RAWdata_Intensity[i], 0, KANAVI::VL_AS16::SPECIFICATION::HORIZONTAL_DATA_CNT);
		}

        OBJ_ID.clear();
        OBJ_xMax.clear();
        OBJ_yMax.clear();
        OBJ_zMax.clear();
        OBJ_xMin.clear();
        OBJ_yMin.clear();
        OBJ_zMin.clear();

        OBJ_Classification.clear();
        OBJ_Status.clear();
        OBJ_Relative_Velocity.clear();
        OBJ_Relative_Acceleration.clear();
        OBJ_Relative_Angle.clear();
        OBJ_Relative_YawRate.clear();
        OBJ_Current_Age.clear();
        OBJ_Prediction_Age.clear();
	}

}VL_AS16_Datagram;


typedef struct lidar_datagram {
    int         LiDAR_Model;								// lidar model
    double      PARA_Vertical_Resolution;					// vertical resolution
    double      PARA_Horizontal_Resolution;					// horizontal resolution
    int         PARA_Start_Angle;							// start angle : 0
    int         PARA_End_Angle;								// end angle : num. of Data
    bool        PARA_Input_END;								// check all data input
    /*  ************************
     *          S16 : 0
     * ***************************/
    std::string LiDAR_IP;									// VL-AS16
    
	VL_AS16_Datagram vl_as16;

	// industrial
	float industrial_Length[INDUSTRAL_MAX_CH][INDUSTRAL_MAX_CNT];	// industrial LiDAR buf

    lidar_datagram() {
        LiDAR_Model = 0x06; // When change this value to the value corresponding to VL-R270, it doesn't create point clouds.
        PARA_Vertical_Resolution = KANAVI::INDUSTRIAL::SPECIFICATION::R4::VERTICAL_RESOLUTION;
        PARA_Horizontal_Resolution = KANAVI::INDUSTRIAL::SPECIFICATION::R4::HORIZONTAL_RESOLUTION;
        PARA_Start_Angle    = 0;
        PARA_End_Angle      = 1159;
        PARA_Input_END      = true; // Also, when change this to True for generation point clouds, it doesn't create point clouds.
        /*  ************************
         *          S3 : 1
         *          S16 : 2
         * ***************************/
        // LiDAR_IP = "192.168.1.80";
        LiDAR_IP = "192.168.123.200";
    }

    void clear()
    {
		/*for(int i=0; i<INDUSTRAL_MAX_CH; i++)
		{
			memset(industrial_Length[i], 0, INDUSTRAL_MAX_CNT);
		}
		vl_as16.clear();*/

        PARA_Input_END = true;
    }

}lidarDatagram;

#endif // HEADER_H_H
