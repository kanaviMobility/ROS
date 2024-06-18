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

#ifndef VL_AS16_H
#define VL_AS16_H

/**
 * @file vl_as16.h
 * @author twchong (twchong@kanavi-mobility.com)
 * @brief to processing Kanavi-Mobility LiDAR sensor of VL-AS16
 * @version 0.1
 * @date 2022-07-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../../include/lidar_spec.h"
#include "../../include/header.h"
class VL_AS16{

public:
    VL_AS16();
	~VL_AS16();

    void processor(const std::vector<u_char> &data, lidarDatagram &datagram);

private:

    void sortData(const std::vector<u_char> &data, lidarDatagram &protocol);
    void sortData_SET02(const std::vector<u_char> &data, lidarDatagram &protocol);
    void sortData_SET03(const std::vector<u_char> &data, lidarDatagram &protocol);

    void sortLength(const std::vector<u_char> &data, lidarDatagram &protocol);
    void sortOBJ(const std::vector<u_char> &data, lidarDatagram &protocol, const size_t &startPos);
    void sortEnW(const std::vector<u_char> &data, lidarDatagram &protocol);

    int FUNC_HEXtoDEC(u_char up, u_char down);
    int FUNC_HEX2DEX_Length(u_char up, u_char down);
    float FUNC_HEX2DEX_objSize(u_char up, u_char down);

    uint16_t each8to16(u_char up, u_char down);

	const int SIZE_HEAD2LENGTH = static_cast<int>(KANAVI::VL_AS16::PROTOCOL_SIZE::HEAD)
							+ static_cast<int>(KANAVI::VL_AS16::PROTOCOL_SIZE::TARGET_ID)
							+ static_cast<int>(KANAVI::VL_AS16::PROTOCOL_SIZE::SOURCE_ID)
							+ static_cast<int>(KANAVI::VL_AS16::PROTOCOL_SIZE::COMMAND)
							+ static_cast<int>(KANAVI::VL_AS16::PROTOCOL_SIZE::PARAMETER)
							+ static_cast<int>(KANAVI::VL_AS16::PROTOCOL_SIZE::DATALENGTH);	//14
};

#endif // VL_AS16_H
