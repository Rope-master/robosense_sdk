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
 * robosense basic geometry module.
 *
 */

#ifndef ROBOSENSE_EGO_MOTION_COMP_H
#define ROBOSENSE_EGO_MOTION_COMP_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace robosense {
namespace perception {

template<typename PointT>
struct alignas(16) EgoMotionComp {

	typedef pcl::PointCloud <PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	typedef typename PointCloud::ConstPtr PointCloudConstPtr;

	typedef std::shared_ptr<EgoMotionComp<PointT> > Ptr;
	typedef std::shared_ptr<const EgoMotionComp<PointT> > ConstPtr;

	EgoMotionComp();

	void egoMotionCompensation(PointCloudConstPtr raw_cloud, PointCloudPtr corrected_cloud, const float &vel,
	                                  const float &angular_vel);

	void sync(const double& timestamp);

	private:

	class EgoMotionCompInternal;
	EgoMotionCompInternal* Internal;

};

}
}
#endif //PROJECT_EGO_MOTION_COMP_H
