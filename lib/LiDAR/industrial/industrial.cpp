#include "industrial.h"

industrialLiDAR::industrialLiDAR(/* args */)
{
	for(int i=0; i<INDUSTRAL_MAX_CH; i++)
	{
		g_checked_CH[i] = false;
	}
}

industrialLiDAR::~industrialLiDAR()
{
}

/**
 * @brief to process Kanavi-Mobility Industrial LiDAR sensor
 * 
 * @param input 	datagram of LiDAR raw data
 * @param output 	protocol structure of Kanavi-Mobility LiDAR sensor
 */
void industrialLiDAR::process(const std::vector<u_char> &input, lidarDatagram &output)
{
	//sort length
	if(input.size() > 0)
	{
		switch(input[KANAVI::INDUSTRIAL::COMMON::PROTOCOL_POS::PRODUCT_LINE])
		{
		case KANAVI::MODEL::LiDAR::VL_R002IK01:
			R2(input, output);
			break;
		/*case KANAVI::MODEL::LiDAR::VL_R001IK02:
			R300(input, output);
			break;*/
		case KANAVI::MODEL::LiDAR::VL_R004IK01:
			R4(input, output);
			break;
		case KANAVI::MODEL::LiDAR::VL_R001IK03: // Modified part
			R270(input, output);
			break;
		}
	}
}

/**
 * @brief to parse raw data to length value
 * 
 * @param input 	datagram of LiDAR raw data
 * @param output 	protocol structure of Kanavi-Mobility LiDAR sensor
 * @param ch 		now length data's channel
 */
void industrialLiDAR::parseLength(const std::vector<u_char> &input, lidarDatagram &output, int ch)
{
	int start = KANAVI::INDUSTRIAL::COMMON::PROTOCOL_POS::RAWDATA_START;
	int end = input.size() - KANAVI::INDUSTRIAL::COMMON::PROTOCOL_SIZE::CHECKSUM;

	float up = 0;
	float low = 0;
	float len = 0;

	int cnt=0;
	for(int i=start; i<end; i+=2)
	{
		up = input[i];
		low = input[i+1];
		len = up + low/100;		//convert 2 byte to length[m]
		/*if(ch == 0)
		{
		    printf("[%d] index : %d, %f\n", ch, cnt, len);
		}*/
		output.industrial_Length[ch][cnt] = len;
		cnt++;
	}
}

/**
 * @brief to process Kanavi-Mobility industrial LiDAR sensor VL-R002IK01(R2)
 * 
 * @param input 	datagram of LiDAR raw data
 * @param output 	protocol structure of Kanavi-Mobility LiDAR sensor
 */
void industrialLiDAR::R2(const std::vector<u_char> &input, lidarDatagram &output)
{
	u_char mode = input[static_cast<int>(KANAVI::INDUSTRIAL::COMMON::PROTOCOL_POS::COMMAND::MODE)];
	u_char ch = input[static_cast<int>(KANAVI::INDUSTRIAL::COMMON::PROTOCOL_POS::COMMAND::PARAMETER)];
	
	// Left for further testing. (24/03/20)
	// printf("CH: %d\n", ch);
	// printf("Input[%d]: %d\n", static_cast<int>(KANAVI::INDUSTRIAL::COMMON::PROTOCOL_POS::COMMAND::PARAMETER), input[static_cast<int>(KANAVI::INDUSTRIAL::COMMON::PROTOCOL_POS::COMMAND::PARAMETER)]);

	if(mode == KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::COMMAND::MODE::DISTANCE_DATA)
	{
		// check ch 0 data input
		if(ch == KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::CHANNEL::CHANNEL_0)
		{
			printf("[industrial][input CH0]\n");
			// output.clear();
			//set specification
			{
				output.LiDAR_Model = KANAVI::MODEL::LiDAR::VL_R002IK01;
				output.PARA_Vertical_Resolution 
					= KANAVI::INDUSTRIAL::SPECIFICATION::R2::VERTICAL_RESOLUTION;
				output.PARA_Horizontal_Resolution
					= KANAVI::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_RESOLUTION;
				output.PARA_Start_Angle = 0;
				output.PARA_End_Angle 
					= KANAVI::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_DATA_CNT;
			}
				
			parseLength(input, output, static_cast<int>(ch & 0x0F));

			g_checked_CH[0] = true;
		}
		else if(ch == KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::CHANNEL::CHANNEL_1)	//check ch 1 data input
		{
			g_checked_CH[0] = true; // Add this logic for processing. (24/03/19)
			
			if(g_checked_CH[0])
			{
				parseLength(input, output, static_cast<int>(ch & 0x0F));	// convert byte to length
				g_checked_CH[1] = true;
			}
			else
			{
			    printf("CH0 data is not processed yet.\n");
			}
		}
		else
		{
		    printf("Invalid channel value: %d\n", ch);
		}
		
		if(g_checked_CH[0] && g_checked_CH[1]) // Add this logic for processing. (24/03/19)
		{
		    output.PARA_Input_END =true;
		}		
		else
		{
		    output.PARA_Input_END = false;
		}
	}
	else
	{
	    printf("Invalid distance data.\n");
	}
}

/**
 * @brief to process Kanavi-Mobility industrial LiDAR sensor VL-R001IK02(R300)
 * 
 * @param input		datagram of LiDAR raw data
 * @param output	protocol structure of Kanavi-Mobility LiDAR sensor
 */
/*void industrialLiDAR::R300(const std::vector<u_char> &input, lidarDatagram &output)
{
	u_char mode = input[static_cast<int>(KANAVI::INDUSTRIAL::COMMON::PROTOCOL_POS::COMMAND::MODE)];
	u_char ch = input[static_cast<int>(KANAVI::INDUSTRIAL::COMMON::PROTOCOL_POS::COMMAND::PARAMETER)];

	if(mode == KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::COMMAND::MODE::DISTANCE_DATA)
	{
		//check ch 0 data input
		if(ch == KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::CHANNEL::CHANNEL_0)
		{
			// output.clear();
			//set specification
			{
				output.LiDAR_Model = KANAVI::MODEL::LiDAR::VL_R001IK02;
				output.PARA_Vertical_Resolution 
					= KANAVI::INDUSTRIAL::SPECIFICATION::R300::VERTICAL_RESOLUTION;
				output.PARA_Horizontal_Resolution
					= KANAVI::INDUSTRIAL::SPECIFICATION::R300::HORIZONTAL_RESOLUTION;
				output.PARA_Start_Angle = 0;
				output.PARA_End_Angle 
					= KANAVI::INDUSTRIAL::SPECIFICATION::R300::HORIZONTAL_DATA_CNT;
			}
				
			parseLength(input, output, static_cast<int>(ch & 0x0F));	// convert byte to length
			output.PARA_Input_END = true;		//data input complete
		}
	}
}*/

/**
 * @brief to process Kanavi-Mobility industrial LiDAR sensor VL-R00(R4)
 * 
 * @param input 	datagram of LiDAR raw data
 * @param output 	protocol structure of Kanavi-Mobility LiDAR sensor
 */
 
 // Todo - Modify similar to R2 logic. (24/03/19)
 // Check calling parselength function exactly and see what it returns.
 // Check if it passes the condition statements properly.  
void industrialLiDAR::R4(const std::vector<u_char> &input, lidarDatagram &output)
{
	u_char mode = input[static_cast<int>(KANAVI::INDUSTRIAL::COMMON::PROTOCOL_POS::COMMAND::MODE)];
	u_char ch = input[static_cast<int>(KANAVI::INDUSTRIAL::COMMON::PROTOCOL_POS::COMMAND::PARAMETER)];
	
	printf("R4 processing...[%X]\n", ch);

	if(mode == KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::COMMAND::MODE::DISTANCE_DATA)
	{
		if(ch == KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::CHANNEL::CHANNEL_0)
		{
			{
				output.LiDAR_Model = KANAVI::MODEL::LiDAR::VL_R004IK01;
				output.PARA_Vertical_Resolution 
					= KANAVI::INDUSTRIAL::SPECIFICATION::R4::VERTICAL_RESOLUTION;
				output.PARA_Horizontal_Resolution
					= KANAVI::INDUSTRIAL::SPECIFICATION::R4::HORIZONTAL_RESOLUTION;
				output.PARA_Start_Angle = 0;
				output.PARA_End_Angle 
					= KANAVI::INDUSTRIAL::SPECIFICATION::R4::HORIZONTAL_DATA_CNT;
			}
			
			parseLength(input, output, static_cast<int>(ch & 0x0F));
			g_checked_CH[0] = true;
		}
		else if(ch == KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::CHANNEL::CHANNEL_1)	//check ch 1 data input
		{
			parseLength(input, output, static_cast<int>(ch & 0x0F));
			g_checked_CH[1] = true;
		}
		else if(ch == KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::CHANNEL::CHANNEL_2)	//check ch 2 data input
		{
			parseLength(input, output, static_cast<int>(ch & 0x0F));
			g_checked_CH[2] = true;
		}
		else if(ch == KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::CHANNEL::CHANNEL_3)	//check ch 3 data input
		{
			parseLength(input, output, static_cast<int>(ch & 0x0F));
			g_checked_CH[3] = true;
		}
		else
		{
		    printf("Invalid channel input.\n");
		}

		
		// parse data
		// parseLength(input, output, static_cast<int>(ch & 0x0F));

		if(g_checked_CH[0] && g_checked_CH[1] && g_checked_CH[2] && g_checked_CH[3])
		{
			output.PARA_Input_END = true;
			for(int i=0; i<INDUSTRAL_MAX_CH; i++)
			{
				g_checked_CH[i] = false;
			}
		}
		else
		{
		    output.PARA_Input_END = false;
		}
	}
	else
	{
	    printf("Invalid distance data.\n");
	}
}


void industrialLiDAR::R270(const std::vector<u_char> &input, lidarDatagram &output)
{
	u_char mode = input[static_cast<int>(KANAVI::INDUSTRIAL::COMMON::PROTOCOL_POS::COMMAND::MODE)];
	u_char ch = input[static_cast<int>(KANAVI::INDUSTRIAL::COMMON::PROTOCOL_POS::COMMAND::PARAMETER)];

	if(mode == KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::COMMAND::MODE::DISTANCE_DATA)
	{
		//check ch 0 data input
		if(ch == KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::CHANNEL::CHANNEL_0)
		{
			//output.clear();
			//set specification
			{
				output.LiDAR_Model = KANAVI::MODEL::LiDAR::VL_R001IK03; // Modified part
				output.PARA_Vertical_Resolution 
					= KANAVI::INDUSTRIAL::SPECIFICATION::R270::VERTICAL_RESOLUTION;
				output.PARA_Horizontal_Resolution
					= KANAVI::INDUSTRIAL::SPECIFICATION::R270::HORIZONTAL_RESOLUTION;
				output.PARA_Start_Angle = 0;
				output.PARA_End_Angle 
					= KANAVI::INDUSTRIAL::SPECIFICATION::R270::HORIZONTAL_DATA_CNT;
			}
				
			parseLength(input, output, static_cast<int>(ch & 0x0F));	// convert byte to length
			output.PARA_Input_END = true;		//data input complete
		}
	}
}
