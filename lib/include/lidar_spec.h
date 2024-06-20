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

#ifndef __LIDAR_SPEC_H__
#define __LIDAR_SPEC_H__

/**
 * @file lidar_spec.h
 * @author twchong (twchong@kanavi-mobility.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

namespace KANAVI {
	enum PARSE_MODE{
		LENGTH,
		COORDINATE
	};

	namespace MODEL{
		enum LiDAR{
			VL_AS16			= 0x00,
			VL_L001IK01		= 0x01,		/* 1ch laser range finder */
			VL_R001IK01		= 0x02,		/* 1ch 120 scanning LiDAR */
			VL_R002IK01		= 0x03,		/* 2ch 120 scanning LiDAR (R2) */
			VL_R001IK02		= 0x04,		/* 1ch 300 scanning LiDAR (R300) */
			VL_R004IK01		= 0x06,		/* 4ch 90 scanning LiDAR ?(S3) */
			VL_R001IK03		= 0x07			/* 1ch 270 scanning LiDAR (R270) */ // Modified part
		};
	}

	namespace VL_AS16 {
		namespace SPECIFICATION {
			const double HORIZONTAL_FoV			= 145;
			const double HORIZONTAL_RESOLUTION	= 0.125;
			const int HORIZONTAL_DATA_CNT		= static_cast<int>(HORIZONTAL_FoV/HORIZONTAL_RESOLUTION);
			const double VERTICAL_FoV			= 9.6;
			const double VERTICAL_RESOLUTION	= 0.6;
			const int VERTICAL_CHANNEL			= static_cast<int>(VERTICAL_FoV / VERTICAL_RESOLUTION);
		}

		namespace PROTOCOL_VALUE {
			enum class HEAD{
				UPPER = 0xFA,
				LOWER = 0x0A
			};

			enum class TAIL{
				UPPER = 0xF5,
				LOWER = 0x05
			};

			namespace COMMAND {
				/* define... */
			}

			namespace PARAMETER {
				/* defone... */
			}

			namespace DATA_PACKET {
				enum HEADER{
					SERIAL_ID_0 = 1,
					SERIAL_ID_1 = 2,
					SERIAL_ID_2 = 3,
					SERIAL_ID_3 = 4
				};
			}
		}

		namespace PROTOCOL_POS {
			enum HEADER{
				HEAD		= 0,
				TARGET_ID	= 2,
				SOURCE_ID	= 4,
				COMMAND		= 6,
				PARAMETER	= 8,
				DATALENGTH	= 10
			};

			enum TAIL{
				CHECKSUM		= 4,	/* total_size - this */
				TAIL			= 2		/* total_size - this */
			};

			namespace DATA_PACKET {
				enum HEADER{
					SERIAL_ID = 14,
					START_ANGLE = 18,
					END_ANGLE	= 20,
					DATA_LENGTH	= 22,
					DATA_TYPE	= 24,
					NTP_TIME	= 25,
					RESERVED	= 33
				};
			}

			const int OBJECT_DATA_START = 59194;
		}

		namespace PROTOCOL_SIZE {
			const int HEAD			= 2;
			const int TARGET_ID		= 2;
			const int SOURCE_ID		= 2;
			const int COMMAND		= 2;
			const int PARAMETER		= 2;
			const int DATALENGTH	= 4;
			const int CHECKSUM		= 2;
			const int TAIL			= 2;

			const int HEADER_SIZE = 20;
			const int RAWDATA_SIZE = 59160;

			namespace DATA_PACKET {
				enum class HEADER{
					SERIAL_ID = 4,
					START_ANGLE = 2,
					END_ANGLE	= 2,
					DATA_LENGTH	= 2,
					DATA_TYPE	= 1,
					NTP_TIME	= 8,
					RESERVED	= 1
				};

				enum class RAW{
					ANGLE = 2,
					DISTANCE = 32,
					DISTANCE_EACH = 2,
					INTENSITY = 16,
					INTENSITY_EACH = 1,
					RESERVED = 1
				};

				enum class OBJECT{
					ID = 1,
					COORDINATE_MAX	= 2,
					COORDINATE_MIN	= 2,
					COORDINATE_TOTAL= 12,
					CLASSIFICATION	= 1,
					STAUS			= 1,
					RELATIVE_VELOCITY	= 1,
					RELATIVE_ACCEL		= 1,
					RELATIVE_ANGLE		= 2,
					RELATIVE_YAWRATE	= 2,
					CURRENT_AGE			= 1,
					PREDICTION_AGE		= 1
				};

				enum class ERRORnWARN{
					INTERNAL_TEMP	= 1,
					MOTOR_STATUS	= 1,
					APD_VOLTAGE		= 1,
					POWER_STATUS	= 1,
					RESERVED		= 1,
					CAN_ID			= 2,
					AVLIVE_CNT		= 1
				};
			}
		}
	}

	namespace INDUSTRIAL{
		namespace SPECIFICATION {
			namespace R2{
				const double	HORIZONTAL_FoV			= 120;
				const double	HORIZONTAL_RESOLUTION	= 0.25;
				const int		HORIZONTAL_DATA_CNT		= static_cast<int>(HORIZONTAL_FoV/HORIZONTAL_RESOLUTION);
				const double	VERTICAL_FoV			= 3;
				const double	VERTICAL_RESOLUTION		= 1.5;
				const int		VERTICAL_CHANNEL		= static_cast<int>(VERTICAL_FoV / VERTICAL_RESOLUTION);
			}

			namespace R4 {
				const double	HORIZONTAL_FoV			= 100;
				const double	HORIZONTAL_RESOLUTION	= 0.25;
				const int		HORIZONTAL_DATA_CNT		= static_cast<int>(HORIZONTAL_FoV/HORIZONTAL_RESOLUTION);
				const double	VERTICAL_FoV			= 3.2;
				const double	VERTICAL_RESOLUTION		= 1.07;
				const int		VERTICAL_CHANNEL		= 4;
			}
			
			// Modified part
			namespace R270 {
				const double	HORIZONTAL_FoV			= 270;
				const double	HORIZONTAL_RESOLUTION	= 0.25;
				const int		HORIZONTAL_DATA_CNT		= static_cast<int>(HORIZONTAL_FoV/HORIZONTAL_RESOLUTION);
				const double	VERTICAL_FoV			= 1;
				const double	VERTICAL_RESOLUTION		= 1;
				const int		VERTICAL_CHANNEL		= static_cast<int>(VERTICAL_FoV / VERTICAL_RESOLUTION);
			}
		}

		namespace R2 {
			namespace PROTOCOL_VALUE {
			}
			namespace PROTOCOL_POS {
			}
			namespace PROTOCOL_SIZE {
				const int TOTAL = 969;
			}
		}

		namespace R4 {
			namespace PROTOCOL_VALUE {
			}
			namespace PROTOCOL_POS {
			}
			namespace PROTOCOL_SIZE {
				const int TOTAL = 809;
			}
		}

		// Modified part
		namespace R270 { 
			namespace PROTOCOL_VALUE {
			}
			namespace PROTOCOL_POS {
			}
			namespace PROTOCOL_SIZE {
				const int TOTAL = 2169;
			}
		}
		namespace COMMON
		{
			namespace PROTOCOL_VALUE
			{
				const int HEADER		= 0xFA;

				enum MODEL {
					VL_L001IK01		= 0x01,		/* 1ch laser range finder */
					VL_R001IK01		= 0x02,		/* 1ch 120 scanning LiDAR */
					VL_R002IK01		= 0x03,		/* 2ch 120 scanning LiDAR (R2) */
					VL_R001IK02		= 0x04,		/* 1ch 300 scanning LiDAR (R300) */
					VL_R004IK01		= 0x06,		/* 4ch 90 scanning LiDAR ?(S3) */
					VL_R001IK03		= 0x07			/* 1ch 270 scanning LiDAR (R270)*/ // Modified part
				};

				namespace COMMAND {
					enum MODE{
						ENGINEER_DEBUG		= 0xDB,
						FIRMWARE_BOOTLOAD	= 0xFB,
						CONFIG_SET			= 0xCF,
						NANK				= 0xF0,
						DISTANCE_DATA		= 0xDD
					};

					namespace PARAMETER{
						enum class RECV{
							DEFAULT_CONFIG		= 0x01,
							LOAD_EXIST_SET_DATA	= 0x11,
							HFoV				= 0x21,
							OUTPUT_CHANNEL		= 0x31,
							USER_AREA			= 0x41,
							OBJECT_SIZE			= 0x51,
							PULSE_OUTPUT_TIME	= 0x61,
							CONFIG				= 0x71,
							MAX_DETECTION_DISTANCE	= 0x81,
							PULSE_ACTIVE_STATE	= 0xB1,
							SET_NETWORK_IP		= 0xC1,
							GET_NETWORK_IP		= 0xD1,
							SET_TEACHING_MODE	= 0xE1,
							GET_TEACHING_AREA	= 0xF1
						};

						enum class REQ{
							DEFAULT_CONFIG		= 0x00,
							LOAD_EXIST_SET_DATA	= 0x10,
							HFoV				= 0x20,
							OUTPUT_CHANNEL		= 0x30,
							USER_AREA			= 0x40,
							OBJECT_SIZE			= 0x50,
							PULSE_OUTPUT_TIME	= 0x60,
							CONFIG				= 0x70,
							MAX_DETECTION_DISTANCE	= 0x80,
							PULSE_ACTIVE_STATE	= 0xB0,
							SET_NETWORK_IP		= 0xC0,
							GET_NETWORK_IP		= 0xD0,
							SET_TEACHING_MODE	= 0xE0,
							GET_TEACHING_AREA	= 0xF0
						};
					}
				}

				const int NAK = 0xF0;

				enum CHANNEL{
					CHANNEL_0	= 0xC0,
					CHANNEL_1	= 0xC1,
					CHANNEL_2	= 0xC2,
					CHANNEL_3	= 0xC3
				};

				namespace DATA
				{
					namespace RESPONSE {
						enum SUCCESS{
							SUCCESS = 0x01
						};
						enum DETECTION{	/* 1byte Bit Value */
							isAreaSet_EN	= 1, /* 0th bit */
							isAreaSet_UN	= 0, /* 0th bit */
							OUTPUT_PIN1_EN	= 1, /* 1th bit */
							OUTPUT_PIN1_UN	= 0, /* 1th bit */
							OUTPUT_PIN2_EN	= 1, /* 2th bit */
							OUTPUT_PIN2_UN	= 0, /* 2th bit */
							AreaDetect1_EN	= 1, /* 3th bit, detect output in area 1 */
							AreaDetect1_UN	= 0, /* 3th bit, not detect in area 1  */
							AreaDetect2_EN	= 1, /* 4th bit, detect output in area 2 */
							AreaDetect2_UN	= 0, /* 4th bit, not detect in area 2  */
							AreaDetect3_EN	= 1, /* 5th bit, detect output in area 3 */
							AreaDetect3_UN	= 0, /* 5th bit, not detect in area 3  */
							AreaDetect4_EN	= 1, /* 6th bit, detect output in area 4 */
							AreaDetect4_UN	= 0, /* 6th bit, not detect in area 4  */
							AreaDetect5_EN	= 1, /* 7th bit, detect output in area 5 */
							AreaDetect5_UN	= 0, /* 7th bit, not detect in area 5  */
						};

						enum CONFIG{
							GENERAL_PURPOSE			= 0x00,
							PLATFORM_SCREEN_DOOR	= 0x01,
							TRAFFIC_LIGHT			= 0x02,
							SMART_TOLLING			= 0x03,
							SECURITY				= 0x04
						};

						enum GET_TEACHING_AREA{
							THEACHING	= 0x01,
							NOT			= 0x00
						};
					}

					namespace REQUEST
					{
						namespace SET_DEFAULT_CONFIG {
							const int SET = 0xDF;
						}

						enum EXIST_AREA_CONFIG{
							REQUEST = 0xED
						};

						enum PULSE_OUTPUT_MODE{
							DEAULT = 0x00,
							ms_100 = 0x01,
							ms_200 = 0x02,
							ms_300 = 0x03,
							ms_400 = 0x04,
							ms_500 = 0x05,
							ms_600 = 0x06,
							ms_700 = 0x07,
							ms_800 = 0x08,
							ms_900 = 0x09,
							ms_1000 = 0x0A
						};

						enum CONFIG{
							STATE_REQUEST = 0xAE
						};

						enum class PULSE_ACTIVE_STATE{
							ACTIVE_LOW = 0x00,
							ACTIVE_HIGH = 0x01
						};

						enum SET_PULSEPIN_OUTPUT{
							MODE_STANDARD	= 0x00,
							MODE_DUAL		= 0x01
						};

						enum class SET_SELF_CHECK_ACTIVE_STATE{
							ACTIVE_LOW	= 0x00,
							ACTIVE_HIGH	= 0x01
						};

						const int GET_NETWORK = 0xAA;

						namespace SET_TEACHING_MODE
						{
							enum class SET_TEACHING
							{
								_DELETE	= 0x00,
								SET		= 0x01
							};

							enum class SET_TEACHING_MAGIN
							{
								DELETE_ = 0x00
							};
						}

						const int GET_TEACHING_AREA = 0xEF;
					}
				}
			}

			namespace PROTOCOL_POS {
				const int HEADER		= 0;
				const int PRODUCT_LINE	= 1;
				const int ID			= 2;
				const int COMMAND		= 3;
				enum class COMMAND{
					MODE		= 3,
					PARAMETER	= 4
				};

				const int DATALENGTH	= 5;
				const int RAWDATA_START	= 7;
				const int CHECKSUM		= 1;	/*total_size - this*/

				namespace RESPONSE {
					enum SET_DEFAULT_CONFIG{
						SET = 5
					};

					enum EXIST_AREA_CONFIG {	/* continue distance data packet */
						OUTPUT_CHANNEL			= 0,
						SELF_CHECK_ACTIVE_STATE	= 1,
						PULSE_ACTIVE_STATE		= 2,
						PULSE_OUTPUT_MODE		= 3,
						PULSE_PIN_MODE			= 4,
						PULSE_PIN_CHANNEL		= 5,
						START_ANGLE				= 6,
						FINISH_ANGLE			= 8,
						MIN_DISTANCE			= 10,
						MAX_DISTANCE			= 11,
						OBJECT_SIZE				= 12,
						QUANTITY_AREA			= 13,
					};

					enum CONFIG{
						FW_VERSION = 5,
						HW_VERSION = 8,
						END_TARGET = 11
					};

					enum GET_NETWORK{
						IP_ADDRESS	= 5,
						MAC_ADDRESS = 9,
						SUBNETMASK	= 15,
						GATEWAY		= 19,
						PORT		= 23
					};

					enum GET_TEACHING_AREA{
						THEACHING_OR_NOT = 5
					};
				}

				namespace REQUEST {
					enum EXIST_AREA_CONFIG{
						REQUEST = 5
					};
					enum SET_FOV{
						START_ANGLE = 5,
						FINISH_ANGLE = 7
					};

					enum PULSE_OUTPUT_MODE{
						SET = 5
					};

					enum CONFIG{
						STATE_REQUEST = 5
					};

					enum SET_MAXMIN_DISTANCE{
						MAX = 5,
						MIN = 6
					};

					const int PULSE_ACTIVE_STATE = 5;

					enum SET_PULSEPIN_OUTPUT{
						MODE = 5,
						CHANNEL = 6
					};

					const int SET_SELF_CHECK_ACTIVE_STATE = 5;

					enum SET_NETWORK{
						IP_ADDRESS	= 5,
						MAC_ADDRESS = 9,
						SUBNETMASK	= 15,
						GATEWAY		= 19,
						PORT		= 23
					};

					const int GET_NETWORK = 5;

					enum SET_TEACHING_MODE{
						SET_TEACHING = 5,
						SET_TEACHING_MAGIN = 6
					};

					const int GET_TEACHING_AREA = 5;
				}
			}

			namespace PROTOCOL_SIZE {
				const int HEADER		= 1;
				const int PRODUCT_LINE	= 1;
				const int ID			= 1;
				const int COMMAND		= 2;
				const int DATALENGTH	= 2;
				const int CHECKSUM		= 2;

				const int DISTANCE_D	= 1;
				const int DISTANCE_F	= 1;
				const int DITECTION		= 1;

				namespace RESPONSE {
					enum EXIST_AREA_CONFIG {
						TOTAL					= 15, /* size is increase by COORDINATE_ */
						OUTPUT_CHANNEL			= 1,
						SELF_CHECK_ACTIVE_STATE	= 1,
						PULSE_ACTIVE_STATE		= 1,
						PULSE_OUTPUT_MODE		= 1,
						PULSE_PIN_MODE			= 1,
						PULSE_PIN_CHANNEL		= 1,
						START_ANGLE				= 2,
						FINISH_ANGLE			= 2,
						MIN_DISTANCE			= 1,
						MAX_DISTANCE			= 1,
						OBJECT_SIZE				= 1,
						QUANTITY_AREA			= 1,
						QUANTITY_POINT			= 1,
						COORDINATE_X_DECIMAL	= 1,
						COORDINATE_X_FLOAT		= 1,
						COORDINATE_Y_DECIMAL	= 1,
						COORDINATE_Y_FLOAT		= 1
					};

					enum CONFIG{
						FW_VERSION = 3,
						HW_VERSION = 3,
						END_TARGET = 1
					};

					enum GET_NETWORK{
						IP_ADDRESS	= 4,
						MAC_ADDRESS = 6,
						SUBNETMASK	= 4,
						GATEWAY		= 4,
						PORT		= 2
					};

					enum GET_TEACHING_AREA{
						THEACHING_OR_NOT = 1
					};
				}

				namespace REQUEST {
					enum class SET_DEFAULT_CONFIG{
						SET = 1
					};

					enum EXIST_AREA_CONFIG{
						REQUEST = 1
					};
					enum SET_FOV{
						START_ANGLE = 2,
						FINISH_ANGLE = 2
					};

					enum class PULSE_OUTPUT_MODE{
						SET = 1
					};

					enum CONFIG{
						STATE_REQUEST = 1
					};

					enum SET_MAXMIN_DISTANCE{
						MAX = 1,
						MIN = 1
					};

					const int PULSE_ACTIVE_STATE = 1;

					enum SET_PULSEPIN_OUTPUT{
						MODE = 1,
						CHANNEL = 1
					};

					const int SET_SELF_CHECK_ACTIVE_STATE = 1;

					enum SET_NETWORK{
						IP_ADDRESS	= 4,
						MAC_ADDRESS = 6,
						SUBNETMASK	= 4,
						GATEWAY		= 4,
						PORT		= 2
					};

					const int GET_NETWORK = 1;

					enum SET_TEACHING_MODE{
						SET_TEACHING = 1,
						SET_TEACHING_MAGIN = 1
					};

					const int GET_TEACHING_AREA = 1;
				}
			}
		}
	}
}

#endif // __LIDAR_SPEC_H__
