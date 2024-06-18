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

#ifndef __INDUSTRIAL_H__
#define __INDUSTRIAL_H__

/**
 * @file industrial.h
 * @author twchong (twchong@kanavi-mobility.com)
 * @brief processing Kanavi-Mobility Industrial LiDAR sensor
 * @version 0.1
 * @date 2022-07-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../../include/lidar_spec.h"
#include "../../include/header.h"

class industrialLiDAR
{
private:
	/* data */
	void parseLength(const std::vector<u_char> &input, lidarDatagram &output, int ch);
	void R2(const std::vector<u_char> &input, lidarDatagram &output);
	void R300(const std::vector<u_char> &input, lidarDatagram &output);
	void R4(const std::vector<u_char> &input, lidarDatagram &output);
	void R270(const std::vector<u_char> &input, lidarDatagram &output); // Modified part

public:
	industrialLiDAR(/* args */);
	~industrialLiDAR();

	void process(const std::vector<u_char> &input, lidarDatagram &output);

private:
	bool g_checked_CH[INDUSTRAL_MAX_CH];
};

#endif // __INDUSTRIAL_H__
