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
 * robosense classification module, for object recognition.
 *
 */

#ifndef CLASSIFIER_1_H
#define CLASSIFIER_1_H

#include "rs_perception/common/base/basic_types.h"
#include "rs_perception/classification/classifyCommon.h"

namespace robosense {
namespace perception {
/**
 * @brief The Classifier Class with Method 1.
 */

template<typename PointT>
class Classifier1 {

  public:

  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

  typedef std::shared_ptr<Classifier1<PointT> > Ptr;
  typedef std::shared_ptr<const Classifier1<PointT> > ConstPtr;

  Classifier1();

  void classify(std::vector<typename Object<PointT>::Ptr>& obj_list);

  void classifyObject(typename Object<PointT>::Ptr obj);

  void freshLidarHeight(const float &estimate_lidar_height);

  bool loadModel(const std::string& modelFile);


  private:

	class Classifier1Internal;
	Classifier1Internal* Internal;

};

}
}

#endif
