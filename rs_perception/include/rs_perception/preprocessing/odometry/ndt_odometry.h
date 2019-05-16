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
 * robosense local odometry estimation module.
 *
 */

#ifndef ROBOSENSE_NDT_ODOMETRY_H
#define ROBOSENSE_NDT_ODOMETRY_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace robosense {
namespace perception {

template<typename PointT>
class alignas(16) NdtOdometry {
  public:

	typedef pcl::PointCloud<PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	typedef typename PointCloud::ConstPtr PointCloudConstPtr;

	typedef std::shared_ptr<NdtOdometry<PointT> > Ptr;
	typedef std::shared_ptr<const NdtOdometry<PointT> > ConstPtr;


	NdtOdometry();


  bool insertCloud(PointCloudConstPtr in_cloud_ptr);

  void getAlignLocalMap(PointCloudPtr map_cloud_ptr);

  bool getAlignPreFrame(const int &idx, PointCloudPtr align_cloud_ptr);

  bool getPreGlobalPose(const int &idx, Eigen::Matrix4f &global_pose);

  bool setAlignNum(const int &num = 5);

  bool setMinShift(const float &shift = 0.1);

  bool setVoxelSize(const float &size = 2.);

	private:

	class NdtOdometryInternal;
	NdtOdometryInternal* Internal;


};

}
}

#endif //ROBOSENSE_NDT_ODOMETRY_H
