#include "kanaviProcessor.h"
#include <time.h>
#define CHECK_processingTime    1

kanaviLidarProcessor::kanaviLidarProcessor()
{
    m_datagram = new lidarDatagram();
    m_dataParser = new lidarParser();

	g_lidarModel  = 0;
}

kanaviLidarProcessor::~kanaviLidarProcessor()
{

}

/**
 * @brief to convert raw data to LiDAR protocol structure
 * 
 * @param data 				datagram of LiDAR raw data
 * @return lidarDatagram 	protocol structure of Kanavi-Mobility LiDAR sensor
 */
lidarDatagram kanaviLidarProcessor::process(const std::vector<u_char> &data)
{
	lidarDatagram datagram;
	if(m_dataParser->setData(data))
	{
		// printf("Parse ENd..get datagram...\n");
		m_dataParser->getLiDARdatagram(datagram);
	}

	return datagram;
}


/**
 * @brief return LiDAR Model
 * 
 * @return int ref. KANAVI::MODEL::LiDAR
 */
int kanaviLidarProcessor::getLiDARModel()
{
    return m_dataParser->getLidarModel();
}