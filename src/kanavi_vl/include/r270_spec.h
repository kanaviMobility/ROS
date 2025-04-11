#ifndef __R270_SPEC_H__
#define __R270_SPEC_H__

// Copyright (c) 2025, Kanavi Mobility
// All rights reserved.
//
// This file is part of the ROS1/ROS2 Hybrid Build Project.
// Licensed under the BSD 3-Clause License.
// You may obtain a copy of the License at the root of this repository (LICENSE file).

#include "common.h"

namespace KANAVI {
	namespace R4 {
		const double	HORIZONTAL_FoV			= 270;
		const double	HORIZONTAL_RESOLUTION	= 0.25;
		const int		HORIZONTAL_DATA_CNT		= static_cast<int>(HORIZONTAL_FoV-HORIZONTAL_RESOLUTION);
		const double	VERTICAL_FoV			= 1;
		const double	VERTICAL_RESOLUTION		= 1;
		const int		VERTICAL_CHANNEL		= static_cast<int>(VERTICAL_FoV / VERTICAL_RESOLUTION);
		const int		DATASIZE				= 2169;
	}
}

#endif // __R270_SPEC_H__