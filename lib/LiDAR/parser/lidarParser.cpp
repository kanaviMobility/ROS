#include "lidarParser.h"
#define OUTPUT_FULL_DATA_CNT	5

lidarParser::lidarParser()
{
	// protocolDatagram = new lidarDatagram();
	g_LiDARModel = KANAVI::MODEL::VL_R004IK01;

	m_detect_Start = false;
	m_detect_End = false;

	g_checkModel = true;

	m_vlas16_Processor = new VL_AS16();
	m_industrial = new industrialLiDAR();


	g_checked_preDatagramSet = false;
	g_cntForAllData = 0;
	g_bufSize = 0;

	g_lidarBuffer.clear();
}

/**
 * @brief to set datagram of LiDAR raw data
 * 
 * @param data 		datagram of LiDAR raw data
 * @return true 	to classify the sensor and parse LiDAR raw data
 * @return false 	can not classify kind of LiDAR sensor
 */
bool lidarParser::setData(const std::vector<u_char> &data)
{
	if(g_checkModel)
	{
		protocolDatagram.LiDAR_Model = classificationModel(data);  //라이다 종류 확인
		if(protocolDatagram.LiDAR_Model != -1)	// not lidar data check
		{
			g_checkModel = false;
		}
		else
		{
			perror("CAN NOT Detected LiDAR Sensor...");
		}
		printf("model ? %d\n", protocolDatagram.LiDAR_Model);
	}

	switch (protocolDatagram.LiDAR_Model)
	{
	case KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::MODEL::VL_R004IK01:
		// break;
		printf("[lidarParser][setData] classification R4!\n");
		return accumulateData_industrial(data, protocolDatagram.LiDAR_Model);
	case KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::MODEL::VL_R002IK01:
		printf("[lidarParser][setData] classification R2!\n");
		return accumulateData_industrial(data, protocolDatagram.LiDAR_Model);
	//case KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::MODEL::VL_R001IK02:
		// break;
		//printf("[lidarParser][setData] classification R300!\n");
		//return accumulateData_industrial(data, protocolDatagram.LiDAR_Model);
	case KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::MODEL::VL_R001IK03: // Modified part
	        printf("[lidarParser][setData] classification R270!\n");
	        return accumulateData_industrial(data, protocolDatagram.LiDAR_Model);
	case static_cast<int>(KANAVI::MODEL::LiDAR::VL_AS16):
		/* code */
		printf("[lidarParser][setData] classification AS16!\n");
		return accumulateData_VLAS16(data);
	default:
		printf("[lidarParser][setData] WHAT?! UnKnown LiDAR Sensor %d\n", protocolDatagram.LiDAR_Model);
		return false;
	}

	return true;
}

/**
 * @brief return kanavi LiDAR model
 * 
 * @return int ref. KANAVI::MODEL::LiDAR
 */
int lidarParser::getLidarModel()
{
	return protocolDatagram.LiDAR_Model;
}


/**
 * @brief return the parsing result protocol structure
 * 
 * @param datagram ref. lidarDatagram structure
 */
void lidarParser::getLiDARdatagram(lidarDatagram &datagram)
{
	if(protocolDatagram.PARA_Input_END)	//check lidar data input end
	{
		datagram = protocolDatagram;
	}
}


/**
 * @brief to classify Kanavi-Mobility LiDAR model
 * 
 * @param data datagram of LiDAR raw data
 * @return int  ref. KANAVI::MODEL::LiDAR
 */
int lidarParser::classificationModel(const std::vector<u_char> &data)
{
	// printf("[parser][classification MODEL]\n");

	if((static_cast<int>(data[0]) == static_cast<int>(KANAVI::VL_AS16::PROTOCOL_VALUE::HEAD::LOWER))
		&& (static_cast<int>(data[1]) == static_cast<int>(KANAVI::VL_AS16::PROTOCOL_VALUE::HEAD::UPPER))
		&& (static_cast<int>(data[data.size() - 2]) == static_cast<int>(KANAVI::VL_AS16::PROTOCOL_VALUE::TAIL::LOWER))
		&& (static_cast<int>(data[data.size() - 1]) == static_cast<int>(KANAVI::VL_AS16::PROTOCOL_VALUE::TAIL::UPPER)))		// VL-AS16 header & tail detected
	{
		return static_cast<int>(KANAVI::MODEL::LiDAR::VL_AS16);
	}
	else if((static_cast<int>(data[0]) == static_cast<int>(KANAVI::VL_AS16::PROTOCOL_VALUE::HEAD::LOWER))
		&& (static_cast<int>(data[1]) == static_cast<int>(KANAVI::VL_AS16::PROTOCOL_VALUE::HEAD::UPPER)))					// VL-AS16 only header detected
	{
		return static_cast<int>(KANAVI::MODEL::LiDAR::VL_AS16);
	}

	if((data[KANAVI::INDUSTRIAL::COMMON::PROTOCOL_POS::HEADER] & 0xFF) 
		== KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::HEADER)																// industrial header detected
	{
		u_char indus_M = data[KANAVI::INDUSTRIAL::COMMON::PROTOCOL_POS::PRODUCT_LINE];
		switch(indus_M)
		{
		case KANAVI::MODEL::LiDAR::VL_R002IK01:
			return KANAVI::MODEL::LiDAR::VL_R002IK01;
		//case KANAVI::MODEL::LiDAR::VL_R001IK02:
			//return KANAVI::MODEL::LiDAR::VL_R001IK02;
		case KANAVI::MODEL::LiDAR::VL_R004IK01:
			return KANAVI::MODEL::LiDAR::VL_R004IK01;
		case KANAVI::MODEL::LiDAR::VL_R001IK03: // Modified part
		  return KANAVI::MODEL::LiDAR::VL_R001IK03;
		}
	}

	return -1;
}

/**
 * @brief to parse Kanavi-Mobility LiDAR raw data(VL-AS16)
 * 
 * @param data datagram of LiDAR raw data
 */
void lidarParser::parsing_VLAS16(const std::vector<u_char> &data)
{
//    g_lengthBuffer.clear();
	parsingRawData(data, protocolDatagram);
}

/**
 * @brief to check all Data input(VL-AS16)
 * 
 * @param data 	datagram of LiDAR raw data
 * @return true 	if datagram has header and tail
 * @return false 	if datagram has only header or tail
 */
bool lidarParser::accumulateData_VLAS16(const std::vector<u_char> &data)
{
	// header detected
	if((static_cast<int>(data[0]) == static_cast<int>(KANAVI::VL_AS16::PROTOCOL_VALUE::HEAD::LOWER))
		&& (static_cast<int>(data[1]) == static_cast<int>(KANAVI::VL_AS16::PROTOCOL_VALUE::HEAD::UPPER)))
	{
		// protocolDatagram.clear();
		g_lidarBuffer.clear();
		g_lidarBuffer = data;
		m_detect_Start  = true;
	}

	// tail detected
	if(static_cast<int>(data[data.size() - 2]) == static_cast<int>(KANAVI::VL_AS16::PROTOCOL_VALUE::TAIL::LOWER)
		&& static_cast<int>(data[data.size() - 1]) == static_cast<int>(KANAVI::VL_AS16::PROTOCOL_VALUE::TAIL::UPPER)	
		&& m_detect_Start)
	{
		for(int i=0; i<data.size(); i++)
		{
			g_lidarBuffer.push_back(data[i]);
		}
		m_detect_End = true;
	}

	if(m_detect_Start && m_detect_End )
	{
		//data output Sque.
		m_detect_Start = false;
		m_detect_End = false;
		
		//parsing
		parsing_VLAS16(g_lidarBuffer);

		previous_lidarBuffer.clear();
		previous_lidarBuffer = g_lidarBuffer;
		g_lidarBuffer.clear();

		m_detect_Start = false;
		m_detect_End = false;

		return true;
	}
	else
	{
		return false;
	}

}

/**
 * @brief  to check all Data input(industrial)
 * 
 * @param data 	datagram of LiDAR raw data
 * @param model 	industrial LiDAR model
 * @return true 	if datagram has header and tail
 * @return false 	if datagram has only header or tail
 */
bool lidarParser::accumulateData_industrial(const std::vector<u_char> &data, int model)
{
	//check data size 
	// u_char model = data[KANAVI::INDUSTRIAL::COMMON::PROTOCOL_POS::PRODUCT_LINE];
	switch(model)
	{
		case KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::MODEL::VL_R002IK01:	//R2
			return process_R2(data);
		//case KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::MODEL::VL_R001IK02:	//R300
			//return process_R300(data);
		case KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::MODEL::VL_R004IK01:	//R4
			return process_R4(data);
		case KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::MODEL::VL_R001IK03: //R270 // Modified part
		  return process_R270(data);
		default:
			return false;
	}
}

/**
 * @brief to check all Data input(industrial - VL_R002IK01(R2))
 * 
 * @param data 	datagram of LiDAR raw data
 * @return true 	if datagram has header and tail
 * @return false 	if datagram has only header or tail
 */
bool lidarParser::process_R2(const std::vector<u_char> &data) 
{
	if(data.size() < KANAVI::INDUSTRIAL::R2::PROTOCOL_SIZE::TOTAL)		// check data size
	{
		// return false;
		if(data[KANAVI::INDUSTRIAL::COMMON::PROTOCOL_POS::HEADER] 		// check header value
			== KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::HEADER)
		{
			m_detect_Start = true;											
			g_lidarBuffer.clear();											// buf clear
			g_lidarBuffer = data;
		}
		else
		{
			if(m_detect_Start)
			{
				for(size_t i=0; i<data.size(); i++)
				{
					g_lidarBuffer.push_back(data[i]);
				}
				m_detect_End = true;
			}
		}
	}
	else if(data.size() == KANAVI::INDUSTRIAL::R2::PROTOCOL_SIZE::TOTAL)
	{
		u_char ch = data[static_cast<int>(KANAVI::INDUSTRIAL::COMMON::PROTOCOL_POS::COMMAND::PARAMETER)];
		m_detect_Start = true;
		m_detect_End = true;
		g_lidarBuffer = data;
		protocolDatagram.PARA_Input_END = true;
	}

	if(m_detect_Start && m_detect_End)
	{
		parsingRawData_industrial(g_lidarBuffer, protocolDatagram);

		m_detect_Start = false;
		m_detect_End = false;
	}
	printf("check Return value : %d\n", protocolDatagram.PARA_Input_END);
	return protocolDatagram.PARA_Input_END;
}

/**
 * @brief to check all Data input(industrial - VL_R002IK01(R2))
 * 
 * @param data 	datagram of LiDAR raw data
 * @return true 	if datagram has header and tail
 * @return false 	if datagram has only header or tail
 */
/*bool lidarParser::process_R300(const std::vector<u_char> &data) 
{
	if(data.size() < KANAVI::INDUSTRIAL::R300::PROTOCOL_SIZE::TOTAL)
	{
		// return false;
		if(data[KANAVI::INDUSTRIAL::COMMON::PROTOCOL_POS::HEADER] 
			== KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::HEADER)
		{
			m_detect_Start = true;
			g_lidarBuffer.clear();
			g_lidarBuffer = data;
		}
		else
		{
			if(m_detect_Start)
			{
				for(size_t i=0; i<data.size(); i++)
				{
					g_lidarBuffer.push_back(data[i]);
				}
				m_detect_End = true;
			}
		}
	}
	else if(data.size() == KANAVI::INDUSTRIAL::R300::PROTOCOL_SIZE::TOTAL)
	{
		m_detect_Start = true;
		m_detect_End = true;
		g_lidarBuffer = data;
	}

	if(m_detect_Start
		&& m_detect_End
		&& g_lidarBuffer.size() ==  KANAVI::INDUSTRIAL::R300::PROTOCOL_SIZE::TOTAL)
	{
		parsingRawData_industrial(g_lidarBuffer, protocolDatagram);
		m_detect_Start = false;
		m_detect_End = false;
	}
	return protocolDatagram.PARA_Input_END;
}*/

/**
 * @brief 
 * 
 * @param data to check all Data input(industrial - VL_R004IK02(R4)
 * @return true 
 * @return false 
 */
bool lidarParser::process_R4(const std::vector<u_char> &data) 
{
	if(data.size() < KANAVI::INDUSTRIAL::R4::PROTOCOL_SIZE::TOTAL)
	{
		// return false;
		if(data[KANAVI::INDUSTRIAL::COMMON::PROTOCOL_POS::HEADER] 
			== KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::HEADER)
		{
			m_detect_Start = true;
			g_lidarBuffer.clear();
			g_lidarBuffer = data;
		}
		else
		{
			if(m_detect_Start)
			{
				for(size_t i=0; i<data.size(); i++)
				{
					g_lidarBuffer.push_back(data[i]);
				}
				m_detect_End = true;
			}
		}
	}
	else if(data.size() == KANAVI::INDUSTRIAL::R4::PROTOCOL_SIZE::TOTAL)
	{
		m_detect_Start = true;
		m_detect_End = true;
		g_lidarBuffer = data;
                /*for(size_t i=0; i<data.size(); i++)
                {
                    g_lidarBuffer.push_back(data[i]);
                }*/
		protocolDatagram.PARA_Input_END = true;
	}

	if(m_detect_Start
		&& m_detect_End
		&& g_lidarBuffer.size() ==  KANAVI::INDUSTRIAL::R4::PROTOCOL_SIZE::TOTAL)
	{
		parsingRawData_industrial(g_lidarBuffer, protocolDatagram);
		m_detect_Start = false;
		m_detect_End = false;
		protocolDatagram.PARA_Input_END = true;
	}
	
	printf("check Return value : %d\n", protocolDatagram.PARA_Input_END);
	return protocolDatagram.PARA_Input_END;
}

// Modified part
bool lidarParser::process_R270(const std::vector<u_char> &data) 
{
	if(data.size() < KANAVI::INDUSTRIAL::R270::PROTOCOL_SIZE::TOTAL)
	{
		// return false;
		if(data[KANAVI::INDUSTRIAL::COMMON::PROTOCOL_POS::HEADER] 
			== KANAVI::INDUSTRIAL::COMMON::PROTOCOL_VALUE::HEADER)
		{
			m_detect_Start = true;
			g_lidarBuffer.clear();
			g_lidarBuffer = data;
		}
		else
		{
			if(m_detect_Start)
			{
				std::vector<u_char>total_buffer;
				
				for(size_t i=0; i<data.size(); i++)
				{
					g_lidarBuffer.push_back(data[i]);
				}
				m_detect_End = true;
				//protocolDatagram.PARA_Input_END = true;
			}
		}
	}
	else if(data.size() == KANAVI::INDUSTRIAL::R270::PROTOCOL_SIZE::TOTAL)
	{
		m_detect_Start = true;
		m_detect_End = true;
		g_lidarBuffer = data;
		//protocolDatagram.PARA_Input_END = true;
	}

	if(m_detect_Start
		&& m_detect_End
		&& g_lidarBuffer.size() ==  KANAVI::INDUSTRIAL::R270::PROTOCOL_SIZE::TOTAL)
	{
		parsingRawData_industrial(g_lidarBuffer, protocolDatagram);
		m_detect_Start = false;
		m_detect_End = false;
	}
	return protocolDatagram.PARA_Input_END;
}

/**
 * @brief return LiDAR raw data size
 * 
 * @return size_t 
 */
size_t lidarParser::getRawDataSize()
{
	if(!m_detect_Start && !m_detect_End && previous_lidarBuffer.size() != 0)
	{
		return previous_lidarBuffer.size();
	}
	else
	{
		return 0;
	}
}

/**
 * @brief return LiDAR raw data
 * 
 * @return std::vector<u_char> 
 */
std::vector<u_char> lidarParser::getRawData()
{
	if(!m_detect_Start && !m_detect_End && previous_lidarBuffer.size() != 0)
	{
		return previous_lidarBuffer;
	}
	else
	{
	    return std::vector<u_char>();
	}
}

/**
 * @brief to parsing LiDAR raw data to Protocol structure(VL-AS16)
 * 
 * @param data 		datagram of LiDAR raw data
 * @param datagram 	ref. lidarDatagram structure
 */
void lidarParser::parsingRawData(const std::vector<u_char> &data, lidarDatagram &datagram)
{
	m_vlas16_Processor->processor(data, datagram);
}

/**
 * @brief to parsing LiDAR raw data to Protocol structure(industrial)
 * 
 * @param data 		datagram of LiDAR raw data
 * @param datagram 	ref. lidarDatagram structure
 */
void lidarParser::parsingRawData_industrial(const std::vector<u_char> &data, lidarDatagram &datagram)
{
	m_industrial->process(data, datagram);
}
