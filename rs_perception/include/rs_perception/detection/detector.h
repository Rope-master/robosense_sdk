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
 * robosense detection module.
 *
 */

#ifndef ROBOSENSE_DETECTION_DETECTOR_H
#define ROBOSENSE_DETECTION_DETECTOR_H


#include "rs_perception/common/base/configuration_type.h"
#include <pcl/PointIndices.h>

namespace robosense {
namespace perception {

template<typename PointT>
class alignas(16) Detector {
  public:
  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;
  typedef std::shared_ptr<Detector<PointT> > Ptr;
  typedef std::shared_ptr<const Detector<PointT> > ConstPtr;


  Detector(const RoboUsrConfig &usr_config);


  void detector(const PointCloudConstPtr in_cloud_ptr, const float &estimate_lidar_height,
                std::vector<PointCloudPtr> &clusters, PointCloudPtr ground_cloud_ptr, PointCloudPtr obstacle_cloud_ptr);

	void getGndIndices(pcl::PointIndicesPtr& ground_idx_ptr) const;

	void getObjIndices(pcl::PointIndicesPtr& obstacle_idx_ptr) const;


	private:

	class DetectorInternal;
	DetectorInternal* Internal;

};

}
}

#endif //ROBOSENSE_DETECTION_DETECTOR_H
