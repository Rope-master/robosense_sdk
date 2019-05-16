/**
 *****************************************************************************
 * COPYRIGHT STATEMENT
 * Copyright (c) 2018, robosense Co.,Ltd. - www.robosense.ai
 * All Rights Reserved.
 *
 * You can not use, copy or spread without official authorization.
 *****************************************************************************
 *
 * Author: robosense Perception Group
 * Version: 2.0.0
 * Date: 2018.6
 *
 * DESCRIPTION
 *
 * robosense built-in data structs type —— user defined configuration params container.
 *
 */

#ifndef ROBOSENSE_CONFIGURATION_TYPE_H
#define ROBOSENSE_CONFIGURATION_TYPE_H

#include "rs_perception/common/base/basic_types.h"

namespace robosense {
namespace perception {

/**
 * @brief tracker info container struct
 */
class alignas(16) RoboUsrConfig{

	public:

	RoboUsrConfig();
	RoboUsrConfig(const RoboUsrConfig &pa);
	RoboUsrConfig &operator=(const RoboUsrConfig &pa);

	void load(const std::string &filename);
	void save(const std::string &filename);

	//lidar
	LidarType lidar_type;


	int work_mode;
	int detection_mode;

	//user defined args
	ObjectLimit obj_limit;

	//detection args
	bool use_geo_filter;
	bool use_auto_align;

	Range2D ground_range;
	Range2D detect_range;
	Range2D kernel_range;
	Range2D ignore_range;

	float detect_thd;
	int min_pts_num;
	bool is_upper_detect;

	//tracking args
	bool use_tracking;
	float predict_time;
	int history_num;

	//classify
	bool use_classification;
	bool with_enhance; //分类增强
};


class alignas(16) RoboLidarConfig{

	public:
	RoboLidarConfig();
	RoboLidarConfig(const RoboLidarConfig &pa);
	RoboLidarConfig &operator=(const RoboLidarConfig &pa);

	Pose lidar_mount;

	void load(const std::string &filename);};


}
}
#endif //ROBOSENSE_CONFIGURATION_TYPE_H
