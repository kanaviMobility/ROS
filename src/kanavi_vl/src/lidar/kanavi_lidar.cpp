#include "kanavi_lidar.h"

/**
 * @brief Construct a new kanavi lidar::kanavi lidar object
 *
 * @param model_ LiDAR Model ref include/common.h
 */
kanavi_lidar::kanavi_lidar(int model_)
{
	datagram_ = new kanaviDatagram(model_);
	checked_onGoing = false;
	checked_pares_end = false;
	memset(checked_ch_r4, false, 4);
}

/**
 * @brief Destroy the kanavi lidar::kanavi lidar object
 *
 */
kanavi_lidar::~kanavi_lidar()
{
}

/**
 * @brief check LiDAR Sensor
 *
 * @param data
 * @return int
 */
int kanavi_lidar::classification(const std::vector<u_char> &data)
{
	if ((data[KANAVI::COMMON::PROTOCOL_POS::HEADER] & 0xFF) == KANAVI::COMMON::PROTOCOL_VALUE::HEADER) // industrial header detected
	{
		u_char indus_M = data[KANAVI::COMMON::PROTOCOL_POS::PRODUCT_LINE];
		switch (indus_M)
		{
		case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R2:
			printf("********R2**********\n");
			return KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R2;
		case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R4:
			printf("********R4**********\n");
			return KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R4;
		case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R270:
			printf("********R270**********\n");
			return KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R270;
		default:
			return -1;
		}
	}

	return -1;
}

int kanavi_lidar::checkDataInputEnd(const std::vector<u_char> &data)
{
	// hedaer checker
	bool checked_h = false;

	// header value Check in data
	if ((data[KANAVI::COMMON::PROTOCOL_POS::HEADER] & 0xFF) == KANAVI::COMMON::PROTOCOL_VALUE::HEADER)
	{
		checked_h = true;
		checked_pares_end = false;
	}
	else
	{
		// SECTION
		//! SECTION
	}

	// check input header parts
	size_t base_buf_size = 0;
	int ch_ = 0;
	ch_ = checkChannel(data);

	if (checked_h)
	{
		switch (datagram_->model)
		{
		case KANAVI::COMMON::PROTOCOL_VALUE::R2:
			base_buf_size = KANAVI::COMMON::SPECIFICATION::R2::RAW_TOTAL_SIZE;
			break;
		case KANAVI::COMMON::PROTOCOL_VALUE::R4:
			base_buf_size = KANAVI::COMMON::SPECIFICATION::R4::RAW_TOTAL_SIZE;
			break;
		case KANAVI::COMMON::PROTOCOL_VALUE::R270:
			base_buf_size = KANAVI::COMMON::SPECIFICATION::R270::RAW_TOTAL_SIZE;
			break;
		default:
			break;
		}

		if (datagram_->model == KANAVI::COMMON::PROTOCOL_VALUE::R270)
		{
			// r270은 데이터 버퍼 크기에 의해
			// 데이터가 나누어서 입력되므로
			// 이를 구분해서 받을 필요가 았음
			if (data.size() < base_buf_size)
			{
				return KANAVI::PROCESS::InputMode::OnGoing;
			}
			else if (data.size() == base_buf_size)
			{
				return KANAVI::PROCESS::InputMode::SUCCESS;
			}
		}
		else
		{
			if (data.size() == base_buf_size)
			{
				return KANAVI::PROCESS::InputMode::SUCCESS;
			}
		}
	}

	return KANAVI::PROCESS::InputMode::FAIL;
}

int kanavi_lidar::checkChannel(const std::vector<u_char> &data)
{
	return 0;
}

int kanavi_lidar::process(const std::vector<u_char> &data)
{
	printf("---------KANAVI PROCESS------------\n");
	std::vector<u_char> buf_;

	// check data Input end.
	if (!checked_onGoing)
	{
		int checked_input_end = checkDataInputEnd(data);
		// check data input END
		if (checked_input_end == KANAVI::PROCESS::InputMode::FAIL)
		{
			return checked_input_end;
		}
		else if (checked_input_end == KANAVI::PROCESS::InputMode::OnGoing)
		{
			// it's R270 Mode
			checked_onGoing = true;
			temp_buf_ = data;
			return checked_input_end;
		}
		// when Model is not R270,
		// and KANAVI::PROCESS::InputMode::SUCCESS return get!
		buf_ = data;
	}
	else
	{

		// sum temp_bud + now data
		buf_ = temp_buf_;
		for (int i = 0; i < data.size(); i++)
		{
			buf_.push_back(data[i]);
		}
	}

	// check Model
	int check_model = classification(buf_);
	if (check_model == -1)
	{
		perror("LiDAR Classification Error!");
		return -1;
	}

	// check Model, one more time
	if (check_model != datagram_->model)
	{
		perror("LiDAR Model not Matched");
		return -1;
	}

	// SECTION---process LiDAR parse
	parse(buf_);
	//! SECTIOn

	// init Vars.
	temp_buf_.clear();
	checked_onGoing = false;
}

std::string kanavi_lidar::getLiDARModel()
{
	return std::string();
}

void kanavi_lidar::parse(const std::vector<u_char> &data)
{
	switch (datagram_->model)
	{
	case KANAVI::COMMON::PROTOCOL_VALUE::R2:
		r2(data, datagram_);
		break;
	case KANAVI::COMMON::PROTOCOL_VALUE::R4:
		r4(data, datagram_);
		break;
	case KANAVI::COMMON::PROTOCOL_VALUE::R270:
		r270(data, datagram_);
		break;
	}
}

void kanavi_lidar::r2(const std::vector<u_char> &input, kanaviDatagram *output)
{
	// u_char mode = input[static_cast<int>(KANAVI::COMMON::PROTOCOL_POS::COMMAND::MODE)];
	// u_char ch = input[static_cast<int>(KANAVI::COMMON::PROTOCOL_POS::COMMAND::PARAMETER)];

	// if (mode == KANAVI::COMMON::PROTOCOL_VALUE::COMMAND::MODE::DISTANCE_DATA)
	// {
	// 	// check ch 0 data input
	// 	if (ch == KANAVI::COMMON::PROTOCOL_VALUE::CHANNEL::CHANNEL_0)
	// 	{
	// 		// set specification
	// 		{
	// 			output->v_resolution = KANAVI::COMMON::SPECIFICATION::R2::VERTICAL_RESOLUTION;
	// 			output->h_resolution = KANAVI::COMMON::SPECIFICATION::R2::HORIZONTAL_RESOLUTION;
	// 		}
	// 	}
	// 	parseLength(input, output, static_cast<int>(ch & 0x0F)); // convert byte to length
	// }
}

void kanavi_lidar::r4(const std::vector<u_char> &input, kanaviDatagram *output)
{
	u_char mode = input[static_cast<int>(KANAVI::COMMON::PROTOCOL_POS::COMMAND::MODE)];
	u_char ch = input[static_cast<int>(KANAVI::COMMON::PROTOCOL_POS::COMMAND::PARAMETER)];

	if (mode == KANAVI::COMMON::PROTOCOL_VALUE::COMMAND::MODE::DISTANCE_DATA)
	{
		// check ch 0 data input
		if (ch == KANAVI::COMMON::PROTOCOL_VALUE::CHANNEL::CHANNEL_0)
		{
			// set specification
			{
				output->v_resolution = KANAVI::COMMON::SPECIFICATION::R4::VERTICAL_RESOLUTION;
				output->h_resolution = KANAVI::COMMON::SPECIFICATION::R4::HORIZONTAL_RESOLUTION;
			}
		}
		// printf("[LiDAR] Process CH : %d\n", ch & 0x0F);
		checked_ch_r4[ch & 0x0F] = true;
		parseLength(input, output, static_cast<int>(ch & 0x0F)); // convert byte to length

		if (ch == KANAVI::COMMON::PROTOCOL_VALUE::CHANNEL::CHANNEL_3)
		{
			// checked_onGoing = true;
			if (checked_ch_r4[0] && checked_ch_r4[1] && checked_ch_r4[2] && checked_ch_r4[3])
			{
				memset(checked_ch_r4, false, 4);
				checked_pares_end = true;
			}
			else
			{
				memset(checked_ch_r4, false, 4);
				printf("[LiDAR] Align ERROR#####\n");
			}
		}
	}
}

void kanavi_lidar::r270(const std::vector<u_char> &input, kanaviDatagram *output)
{
	u_char mode = input[static_cast<int>(KANAVI::COMMON::PROTOCOL_POS::COMMAND::MODE)];
	u_char ch = input[static_cast<int>(KANAVI::COMMON::PROTOCOL_POS::COMMAND::PARAMETER)];

	if (mode == KANAVI::COMMON::PROTOCOL_VALUE::COMMAND::MODE::DISTANCE_DATA)
	{
		// check ch 0 data input
		if (ch == KANAVI::COMMON::PROTOCOL_VALUE::CHANNEL::CHANNEL_0)
		{
			// set specification
			{
				output->v_resolution = KANAVI::COMMON::SPECIFICATION::R270::VERTICAL_RESOLUTION;
				output->h_resolution = KANAVI::COMMON::SPECIFICATION::R270::HORIZONTAL_RESOLUTION;
			}

			parseLength(input, output, static_cast<int>(ch & 0x0F)); // convert byte to length
			checked_pares_end = true;
		}
	}
}

void kanavi_lidar::parseLength(const std::vector<u_char> &input, kanaviDatagram *output, int ch)
{
	std::vector<float> len_;

	int start = KANAVI::COMMON::PROTOCOL_POS::RAWDATA_START;
	int end = input.size() - KANAVI::COMMON::PROTOCOL_SIZE::CHECKSUM;

	float up = 0;
	float low = 0;
	float len = 0;

	for (int i = start; i < end; i += 2)
	{
		up = input[i];
		low = input[i + 1];
		len = up + low / 100; // convert 2 byte to length[m]
		len_.push_back(len);
	}
	output->len_buf[ch] = len_;
	// checked_pares_end = true;
}

bool kanavi_lidar::checkedProcessEnd()
{
	return checked_pares_end;
}

kanaviDatagram kanavi_lidar::getDatagram()
{
	return *datagram_;
}

//TODO - TEST
int kanavi_lidar::process2(const std::vector<u_char> &data)
{
	printf("---------KANAVI PROCESS------------\n");

	std::unique_ptr<FrameAssembler> assembler;

	if (datagram_->model == KANAVI::COMMON::PROTOCOL_VALUE::R4)
	{
		assembler = std::make_unique<R4Assembler>();
	}
	else if (datagram_->model == KANAVI::COMMON::PROTOCOL_VALUE::R270)
	{
		assembler = std::make_unique<R270Assembler>();
	}
	else
	{
		printf("Unsupported model: %d\n", datagram_->model);
		return -1;
	}

	if (assembler->addData(data))
	{
		auto datagram = assembler->toDatagram();
		// TODO: convert datagram to point cloud and publish
		assembler->reset();
	}

	return 0;
}