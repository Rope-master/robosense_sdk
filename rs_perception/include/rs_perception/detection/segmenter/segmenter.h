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
#ifndef ROBOSENSE_DETECTION_SEGMENTER_H
#define ROBOSENSE_DETECTION_SEGMENTER_H

#include "rs_perception/common/base/configuration_type.h"

namespace robosense {
namespace perception {


template<typename PointT>
class alignas(16) Segmenter {

  public:

  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

  typedef std::shared_ptr<Segmenter<PointT> > Ptr;
  typedef std::shared_ptr<const Segmenter<PointT> > ConstPtr;

  Segmenter(const RoboUsrConfig &usr_config);

  void segmenter(const PointCloudConstPtr in_cloud_ptr, const float &estimate_lidar_height,
                 std::vector<PointCloudPtr> &out_objects);

	private:

	class SegmenterInternal;
	SegmenterInternal* Internal;

};

}
}

#endif //ROBOSENSE_DETECTION_SEGMENTER_H
