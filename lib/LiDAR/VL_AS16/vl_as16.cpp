#include "vl_as16.h"

VL_AS16::VL_AS16()
{

}

/**
 * @brief to process Kanavi-Mobility LiDAR sensor of VL-AS16
 * 
 * @param data 		datagram of LiDAR raw data
 * @param datagram 	protocol structure of Kanavi-Mobility LiDAR sensor
 */
void VL_AS16::processor(const std::vector<u_char> &data, lidarDatagram &datagram)
{
    datagram.clear();

	datagram.LiDAR_Model = KANAVI::MODEL::LiDAR::VL_AS16;
	datagram.PARA_Vertical_Resolution 
		= KANAVI::VL_AS16::SPECIFICATION::VERTICAL_RESOLUTION;
	datagram.PARA_Horizontal_Resolution
		= KANAVI::VL_AS16::SPECIFICATION::HORIZONTAL_RESOLUTION;
	datagram.PARA_Start_Angle = 0;
	datagram.PARA_End_Angle 
		= KANAVI::VL_AS16::SPECIFICATION::HORIZONTAL_DATA_CNT;

    if(data.size() >= 59208)
    {
        //Dectect Send Protocol 1 -- RAW DATA + OBJ DATA
        // printf("[Processor_AS16][mode1]\n");
        sortData(data, datagram);
    }
    else if(data.size() == 59206)
    {
        //Dectect Send Protocol 1 -- RAW DATA Only
        // printf("[Processor_AS16][mode2]\n");
        sortData_SET02(data, datagram);
    }
}

/**
 * @brief to sort raw data with length and object to Protocol Structure
 * 
 * @param data 		datagram of LiDAR raw data
 * @param protocol 	protocol structure of Kanavi-Mobility LiDAR sensor
 */
void VL_AS16::sortData(const std::vector<u_char> &data, lidarDatagram &protocol)
{
    //length and Intensity
    sortLength(data, protocol);

    //Obj Data Processing
    size_t Obj_Start = static_cast<size_t>(KANAVI::VL_AS16::PROTOCOL_POS::OBJECT_DATA_START);
    sortOBJ(data, protocol, Obj_Start);

    //Error & Warning
    sortEnW(data, protocol);

    protocol.PARA_Input_END = true;
}

/**
 * @brief to sort only length raw data to Protocol Structure
 * 
 * @param data 		datagram of LiDAR raw data
 * @param protocol 	protocol structure of Kanavi-Mobility LiDAR sensor
 */
void VL_AS16::sortData_SET02(const std::vector<u_char> &data, lidarDatagram &protocol)
{
    //length and Intensity
    sortLength(data, protocol);

    //Error & Warning
    sortEnW(data, protocol);

    protocol.PARA_Input_END = true;
}

/**
 * @brief to sort only object raw data to Protocol Structure
 * 
 * @param data 		datagram of LiDAR raw data
 * @param protocol 	protocol structure of Kanavi-Mobility LiDAR sensor
 */
void VL_AS16::sortData_SET03(const std::vector<u_char> &data, lidarDatagram &protocol)
{
    size_t Obj_Start = static_cast<size_t>(KANAVI::VL_AS16::PROTOCOL_SIZE::HEADER_SIZE) + SIZE_HEAD2LENGTH;
    sortOBJ(data, protocol, Obj_Start);
    sortEnW(data, protocol);

    protocol.PARA_Input_END = true;
}


/**
 * @brief to sort raw data to Length value
 * 
 * @param data 		datagram of LiDAR raw data
 * @param protocol 	protocol structure of Kanavi-Mobility LiDAR sensor
 */
void VL_AS16::sortLength(const std::vector<u_char> &data, lidarDatagram &protocol)
{
    int start = static_cast<size_t>(SIZE_HEAD2LENGTH)
                    + static_cast<size_t>(KANAVI::VL_AS16::PROTOCOL_SIZE::HEADER_SIZE);
    int end = static_cast<size_t>(KANAVI::VL_AS16::PROTOCOL_SIZE::RAWDATA_SIZE)
                + (static_cast<size_t>(SIZE_HEAD2LENGTH)
                + static_cast<size_t>(KANAVI::VL_AS16::PROTOCOL_SIZE::HEADER_SIZE));

    int pos = 0;
	// clock_t cl_start = clock();
    for (int i = start; i < end; i += 51)
    {
        protocol.vl_as16.RAWdata_Angle[pos] = FUNC_HEXtoDEC(data[i+1], data[i]);
		
		protocol.vl_as16.RAWdata_RadialDistance[0][pos] = FUNC_HEX2DEX_Length(data[i+3], data[i+2]);
		protocol.vl_as16.RAWdata_RadialDistance[1][pos] = FUNC_HEX2DEX_Length(data[i + 5], data[i + 4]);
		protocol.vl_as16.RAWdata_RadialDistance[2][pos] = FUNC_HEX2DEX_Length(data[i + 7], data[i + 6]);
		protocol.vl_as16.RAWdata_RadialDistance[3][pos] = FUNC_HEX2DEX_Length(data[i + 9], data[i + 8]);
		protocol.vl_as16.RAWdata_RadialDistance[4][pos] = FUNC_HEX2DEX_Length(data[i + 11], data[i + 10]);
		protocol.vl_as16.RAWdata_RadialDistance[5][pos] = FUNC_HEX2DEX_Length(data[i + 13], data[i + 12]);
		protocol.vl_as16.RAWdata_RadialDistance[6][pos] = FUNC_HEX2DEX_Length(data[i + 15], data[i + 14]);
		protocol.vl_as16.RAWdata_RadialDistance[7][pos] = FUNC_HEX2DEX_Length(data[i + 17], data[i + 16]);
		protocol.vl_as16.RAWdata_RadialDistance[8][pos] = FUNC_HEX2DEX_Length(data[i + 19], data[i + 18]);
		protocol.vl_as16.RAWdata_RadialDistance[9][pos] = FUNC_HEX2DEX_Length(data[i + 21], data[i + 20]);
		protocol.vl_as16.RAWdata_RadialDistance[10][pos] = FUNC_HEX2DEX_Length(data[i + 23], data[i + 22]);
		protocol.vl_as16.RAWdata_RadialDistance[11][pos] = FUNC_HEX2DEX_Length(data[i + 25], data[i + 24]);
		protocol.vl_as16.RAWdata_RadialDistance[12][pos] = FUNC_HEX2DEX_Length(data[i + 27], data[i + 26]);
		protocol.vl_as16.RAWdata_RadialDistance[13][pos] = FUNC_HEX2DEX_Length(data[i + 29], data[i + 28]);
		protocol.vl_as16.RAWdata_RadialDistance[14][pos] = FUNC_HEX2DEX_Length(data[i + 31], data[i + 30]);
		protocol.vl_as16.RAWdata_RadialDistance[15][pos] = FUNC_HEX2DEX_Length(data[i + 33], data[i + 32]);

        protocol.vl_as16.RAWdata_Intensity[0][pos] = static_cast<int>(data[i + 34]);
        protocol.vl_as16.RAWdata_Intensity[1][pos] = static_cast<int>(data[i + 35]);
        protocol.vl_as16.RAWdata_Intensity[2][pos] = static_cast<int>(data[i + 36]);
        protocol.vl_as16.RAWdata_Intensity[3][pos] = static_cast<int>(data[i + 37]);
        protocol.vl_as16.RAWdata_Intensity[4][pos] = static_cast<int>(data[i + 38]);
        protocol.vl_as16.RAWdata_Intensity[5][pos] = static_cast<int>(data[i + 39]);
        protocol.vl_as16.RAWdata_Intensity[6][pos] = static_cast<int>(data[i + 40]);
        protocol.vl_as16.RAWdata_Intensity[7][pos] = static_cast<int>(data[i + 41]);
        protocol.vl_as16.RAWdata_Intensity[8][pos] = static_cast<int>(data[i + 42]);
        protocol.vl_as16.RAWdata_Intensity[9][pos] = static_cast<int>(data[i + 43]);
        protocol.vl_as16.RAWdata_Intensity[10][pos] = static_cast<int>(data[i + 44]);
        protocol.vl_as16.RAWdata_Intensity[11][pos] = static_cast<int>(data[i + 45]);
        protocol.vl_as16.RAWdata_Intensity[12][pos] = static_cast<int>(data[i + 46]);
        protocol.vl_as16.RAWdata_Intensity[13][pos] = static_cast<int>(data[i + 47]);
        protocol.vl_as16.RAWdata_Intensity[14][pos] = static_cast<int>(data[i + 48]);
        protocol.vl_as16.RAWdata_Intensity[15][pos] = static_cast<int>(data[i + 49]);

        protocol.vl_as16.RAWdata_reserved[pos] = static_cast<int>(data[i + 50]);

        pos++;
    }

	// clock_t cl_end = clock() - cl_start;
	// printf("[VL-AS16.cpp] CHECKED [sortLength] FUNC : %fms\n", (double)((double)cl_end/CLOCKS_PER_SEC) * 1000);
}


/**
 * @brief to sort raw data to object data
 * 
 * @param data 		datagram of LiDAR raw data
 * @param protocol 	protocol structure of Kanavi-Mobility LiDAR sensor
 * @param startPos 	object raw data start position
 */
void VL_AS16::sortOBJ(const std::vector<u_char> &data, lidarDatagram &protocol, 
					const size_t &startPos)
{
    size_t endPoint = data.size() -4;
    size_t Obj_END;

    int objCntCheck=0;
    protocol.vl_as16.OBJ_CNT = static_cast<int>(data[startPos]);

    if(protocol.vl_as16.OBJ_CNT%2 == 0)
    {
        Obj_END = endPoint-1 - 8; // entire Data size - error&warning Size
    }
    else
    {
        Obj_END = endPoint - 8; // entire Data size - error&warning Size
    }

    if(protocol.vl_as16.OBJ_CNT > 0 && protocol.vl_as16.OBJ_CNT < 100)
    {
        for(size_t i=startPos+1; i<Obj_END; i+=23)
        {
            if(objCntCheck < protocol.vl_as16.OBJ_CNT)
            {
                protocol.vl_as16.OBJ_ID.push_back(data[i]);
                protocol.vl_as16.OBJ_xMax.push_back((FUNC_HEX2DEX_objSize(data[i+2], data[i+1])));
                protocol.vl_as16.OBJ_yMax.push_back((FUNC_HEX2DEX_objSize(data[i+4], data[i+3])));
                protocol.vl_as16.OBJ_zMax.push_back((FUNC_HEX2DEX_objSize(data[i+6], data[i+5])));

                protocol.vl_as16.OBJ_xMin.push_back((FUNC_HEX2DEX_objSize(data[i+8], data[i+7])));
                protocol.vl_as16.OBJ_yMin.push_back((FUNC_HEX2DEX_objSize(data[i+10], data[i+9])));
                protocol.vl_as16.OBJ_zMin.push_back((FUNC_HEX2DEX_objSize(data[i+12], data[i+11])));

                protocol.vl_as16.OBJ_Classification.push_back(data[i+13]);
                protocol.vl_as16.OBJ_Status.push_back(data[i+14]);
                protocol.vl_as16.OBJ_Relative_Velocity.push_back(data[i+15]);
                protocol.vl_as16.OBJ_Relative_Acceleration.push_back(data[i+16]);
                protocol.vl_as16.OBJ_Relative_Angle.push_back(each8to16(data[i+18], data[i+17]));
                protocol.vl_as16.OBJ_Relative_YawRate.push_back(each8to16(data[i+20], data[i+19]));
                protocol.vl_as16.OBJ_Current_Age.push_back(data[i+21]);
                protocol.vl_as16.OBJ_Prediction_Age.push_back(data[i+22]);
            }
            objCntCheck++;
        }
    }
}

/**
 * @brief to sort raw data to error & warning information
 * 
 * @param data 		datagram of LiDAR raw data
 * @param protocol 	protocol structure of Kanavi-Mobility LiDAR sensor
 */
void VL_AS16::sortEnW(const std::vector<u_char> &data, lidarDatagram &protocol)
{
    size_t pos;
    if(protocol.vl_as16.OBJ_CNT%2 == 0)
    {
        pos = data.size()-4 -1;
    }
    else
    {
        pos = data.size()-4;
    }

    protocol.vl_as16.ErrorNWarning_Internal_Temp = data[pos-8];
    protocol.vl_as16.ErrorNWarning_Motor_Status = data[pos-7];
    protocol.vl_as16.ErrorNWarning_APD_Voltage = data[pos-6];
    protocol.vl_as16.ErrorNWarning_PnT_Status = data[pos-5];
    protocol.vl_as16.ErrorNWarning_Reserved1 = data[pos-4];
    protocol.vl_as16.ErrorNWarning_CANID00 = data[pos-3];
    protocol.vl_as16.ErrorNWarning_CANID01 = data[pos-2];
    protocol.vl_as16.ErrorNWarning_AliveCNT = data[pos-1];
}

/**
 * @brief convert hex to dec
 * 
 * @param up 	high byte
 * @param down 	low byte
 * @return int 	dec
 */
int VL_AS16::FUNC_HEXtoDEC(u_char up, u_char down)
{
    int output;
    output = static_cast<int>(up) * 256 + static_cast<int>(down);
    return output;
}

/**
 * @brief convert hex to dec(length[cm])
 * 
 * @param up 	high byte
 * @param down 	low byte
 * @return int 	length[cm]
 */
int VL_AS16::FUNC_HEX2DEX_Length(u_char up, u_char down)
{
    int output;
    output = static_cast<int>(up) * 256 + static_cast<int>(down);
	if(output > 20000)
		output = 0;
    return output;
}

/**
 * @brief convert hex to dec(object pos)
 * 
 * @param up 		high byte
 * @param down 		low byte
 * @return float 
 */
float VL_AS16::FUNC_HEX2DEX_objSize(u_char up, u_char down)
{
    float output;
    int sum;

    sum = static_cast<int>(up) * 256 + static_cast<int>(down);
    if(sum > 15000)
    {
        int dum = static_cast<int>(0x100000000 - static_cast<int16_t>(sum));
        output = -static_cast<float>(dum) / 100;
    }
    else
        output = static_cast<float>(sum) / 100;

    return output;
}

/**
 * @brief convert 8byte to 16byte
 * 
 * @param up 			high byte
 * @param down 			low byte
 * @return uint16_t 
 */
uint16_t VL_AS16::each8to16(u_char up, u_char down)
{
    uint16_t output;
    output = static_cast<uint16_t>(static_cast<int>(up) * 256 + static_cast<int>(down));
    return output;
}

