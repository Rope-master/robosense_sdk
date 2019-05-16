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
 * robosense ground detection initializer module.
 *
 */

#ifndef ROBOSENSE_COMMON_GEOMETRY_GROUND_INITIALIZER_H
#define ROBOSENSE_COMMON_GEOMETRY_GROUND_INITIALIZER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "rs_perception/common/base/basic_types.h"

namespace robosense {
namespace perception {

template<typename PointT>
class alignas(16) GroundInitializer {

	public:

	typedef pcl::PointCloud<PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	typedef typename PointCloud::ConstPtr PointCloudConstPtr;

	typedef std::shared_ptr<GroundInitializer<PointT> > Ptr;
	typedef std::shared_ptr<const GroundInitializer<PointT> > ConstPtr;

	GroundInitializer(const float& lidar_height = 1.9f, const Range2D& range = Range2D(-20.f, 20.f, -5.f, 5.f));

	bool groundInitializer(PointCloudConstPtr in_cloud_ptr, Eigen::VectorXf &plane_model, float &estimate_lidar_height);

	private:

	class GroundInitializerInternal;
	GroundInitializerInternal* Internal;

};

}
}

#endif //ROBOSENSE_COMMON_GEOMETRY_GROUND_INITIALIZER_H
