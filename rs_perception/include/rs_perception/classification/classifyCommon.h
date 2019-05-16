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
 * robosense classification module, for common processing of object classification.
 *
 */

#ifndef ROBOSENCE_CLASSIFY_COMMON_H
#define ROBOSENCE_CLASSIFY_COMMON_H

#include "rs_perception/common/base/basic_types.h"
#include "rs_perception/common/base/object.h"

namespace robosense {
namespace perception {

template<typename PointT>
class ClassifyUtil {
  public:

  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

  typedef std::shared_ptr<ClassifyUtil<PointT> > Ptr;
  typedef std::shared_ptr<const ClassifyUtil<PointT> > ConstPtr;

  ClassifyUtil() = default;

	static void enhanceClassifier(std::vector<typename Object<PointT>::Ptr>& obj_list, const float &estimate_lidar_height);

  static void enhanceClassifier(typename Object<PointT>::Ptr in_percept, const float &estimate_lidar_height);
};


}
}
#endif