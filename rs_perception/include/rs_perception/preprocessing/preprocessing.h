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
 * robosense preprocessing module, including calibration according mounting args etc.
 *
 */


#ifndef ROBOSENSE_PREPROCESS_H
#define ROBOSENSE_PREPROCESS_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace robosense {
namespace perception {
/**
 * @brief preprocessing module
 */

template <typename PointT>
class alignas(16) PreProcess {

  public:

	typedef pcl::PointCloud<PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	typedef typename PointCloud::ConstPtr PointCloudConstPtr;

	typedef std::shared_ptr<PreProcess<PointT> > Ptr;
	typedef std::shared_ptr<const PreProcess<PointT> > ConstPtr;

  PreProcess();

  bool preProcess(PointCloudConstPtr pcloud_in, PointCloudPtr pcloud_out);

  private:

};
}
}


#endif //ROBOSENSE_PREPROCESS_H


/**
 * @brief preprocessing module
 * @defgroup preprocessing
 */