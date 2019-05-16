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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include "rs_perception/common/base/basic_types.h"

#ifndef ROBOSENSE_GEOBASE_H
#define ROBOSENSE_GEOBASE_H


namespace robosense {
namespace perception {

template<typename PointT>
struct alignas(16) GeoUtil {


	typedef pcl::PointCloud<PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	typedef typename PointCloud::ConstPtr PointCloudConstPtr;


	static float computeDisOfPoints(PointT start, PointT end);

	static float computeDis2Line(PointT start, PointT end, PointT test_point);

	static Eigen::Vector3f calcCloudBarycenter(PointCloudConstPtr cloud);

	static std::vector<float> calcDistriFeature(PointCloudConstPtr cloud, Eigen::Vector3i bin_size = Eigen::Vector3i(8,8,8));

	static bool isInrange2D(const PointT& p, const Range2D& range);
	static bool isInrange3D(const PointT& p, const Range3D& range);


	/**
	 * geometry transformes
	 * */

	static Eigen::Matrix4f calcRotationMatrix(const Eigen::Vector3f &before_vector, const Eigen::Vector3f &after_vector);

	static Eigen::Matrix4f calcRotationMatrix(const float &angle, const Eigen::Vector3f &axis_vector);

	static Eigen::Matrix4f calcRotationMatrix(const Eigen::Vector3f &angle);

	static Eigen::Matrix4f calcTransformMatrix(const Pose &pose);

	static Eigen::Matrix4f calcTransformMatrix(const Eigen::Vector3f &t);

	static Eigen::Matrix4f calcTransformMatrix(const Eigen::Vector3f &before_vector, const Eigen::Vector3f &after_vector,
	                                    const Eigen::Vector3f &trans);


};



}  // robosense
}
#endif //ROBOSENSE_GEOBASE_H
