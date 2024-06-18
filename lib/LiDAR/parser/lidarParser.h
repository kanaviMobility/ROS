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

#ifndef PARSER_H
#define PARSER_H

/**
 * @file lidarParser.h
 * @author twchong (twchong@kanavi-mobility.com)
 * @brief to classify and parse Kanavi-Mobility LiDAR sensor.
 * @version 0.1
 * @date 2022-07-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../../include/lidar_spec.h"
#include "../../include/header.h"
#include "../VL_AS16/vl_as16.h"
#include "../industrial/industrial.h"
#include <cmath>
#include <float.h>

class lidarParser
{
public:
    lidarParser();

    bool setData(const std::vector<u_char> &data);
    int getLidarModel();

    void getLiDARdatagram(lidarDatagram &datagram);

    size_t getRawDataSize();
    std::vector<u_char> getRawData();

    void setParsingMode(int mode);

private:
    //function
    int classificationModel(const std::vector<u_char> &data);
    void parsing_VLAS16(const std::vector<u_char> &data);

    //accumulate
    bool accumulateData_VLAS16(const std::vector<u_char> &data);
	bool accumulateData_industrial(const std::vector<u_char> &data, int model);

	bool process_R2(const std::vector<u_char> &data);
	bool process_R300(const std::vector<u_char> &data);
	bool process_R4(const std::vector<u_char> &data);
	bool process_R270(const std::vector<u_char> &data); // Modified part

    //parsing -- method
	void parsingRawData(const std::vector<u_char> &data, lidarDatagram &datagram);
	void parsingRawData_industrial(const std::vector<u_char> &data, lidarDatagram &datagram);

    //variable
    std::string g_LiDARModel;
    lidarDatagram protocolDatagram;

    //lidar data buffer
    std::vector<u_char> g_lidarBuffer;
    std::vector<u_char> previous_lidarBuffer;
    int g_bufSize;

    //detect lidar protocol start & end
    bool m_detect_Start;
    bool m_detect_End;

    bool g_checkModel;

    //each lidar processor
    VL_AS16 *m_vlas16_Processor;

	industrialLiDAR *m_industrial;

    //angle sin&cos value
    double vertical_SIN[16];
    double vertical_COS[16];
    double horizontal_SIN[1160];
    double horizontal_COS[1160];

    //set previous datagram & partial update
    lidarDatagram previousDatagram;
    bool g_checked_preDatagramSet;
    int g_cntForAllData;

    int partial_window[50][50];
    int g_datswit;
};

#endif // PARSER_H
