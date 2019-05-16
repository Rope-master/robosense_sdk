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
 * robosense classification module.
 *
 */

#ifndef ROBO_FEAT_DESCRIPTOR_H
#define ROBO_FEAT_DESCRIPTOR_H

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include "rs_perception/common/base/basic_types.h"
#include "rs_perception/common/base/object.h"

namespace robosense {
namespace perception {
/**
 * @brief The robosense 3d descriptor module
 */

template<typename PointT>
class RoboFeature {
	public:

		typedef pcl::PointCloud<PointT> PointCloud;
		typedef typename PointCloud::Ptr PointCloudPtr;
		typedef typename PointCloud::ConstPtr PointCloudConstPtr;

		typedef std::shared_ptr<RoboFeature<PointT> > Ptr;
		typedef std::shared_ptr<const RoboFeature<PointT> > ConstPtr;

		RoboFeature(const float &estimate_lidar_height = 1.9f);

		~RoboFeature();

		void computeFeature(typename Object<PointT>::ConstPtr object);

		void getFeature(std::vector<float> &feature);

	private:

		class RoboFeatureInternal;
		RoboFeatureInternal* Internal;
};

}
}

#endif // ROBO_FEAT_DESCRIPTOR_H
