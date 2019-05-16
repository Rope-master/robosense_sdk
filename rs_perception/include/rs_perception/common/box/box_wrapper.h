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
 * robosense box wrappering module.
 *
 */

#ifndef ROBOSENSE_BOX_WRAPER_H
#define ROBOSENSE_BOX_WRAPER_H

#include "rs_perception/common/base/object.h"
#include "rs_perception/common/base/configuration_type.h"

namespace robosense {
namespace perception {

template<typename PointT>
class alignas(16) BoxWrapper
{

	public:

	typedef pcl::PointCloud<PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	typedef typename PointCloud::ConstPtr PointCloudConstPtr;

	typedef std::shared_ptr<BoxWrapper<PointT> > Ptr;
	typedef std::shared_ptr<const BoxWrapper<PointT> > ConstPtr;

	BoxWrapper(const RoboUsrConfig& args);

	void wrapBox(const std::vector<PointCloudPtr> &segment_vec, std::vector<typename Object<PointT>::Ptr > &cluster_vec);

	void segmentFilter(std::vector<PointCloudPtr> &raw_segments);
	void mergePriorBox(std::vector<typename Object<PointT>::Ptr> &cluster_vec, const std::vector<BBox>& prior_boxes);
	void mergePriorBox(std::vector<PointCloudPtr> &segments_vec, const std::vector<BBox>& prior_boxes);
	void mergeBox(std::vector<typename Object<PointT>::Ptr> &cluster_vec);
	void objectFilter(std::vector<typename Object<PointT>::Ptr> &cluster_vec, bool with_mode);

	void signature(std::vector<typename Object<PointT>::Ptr > &cluster_vec, float timestamp, int mode);

	void freshLidarHeight(const float &estimate_lidar_height);
	void refineObject(std::vector<typename Object<PointT>::Ptr> &obj_list);



	private:

	class BoxWrapperInternal;
	BoxWrapperInternal* Internal;
};


}
}

#endif //ROBOSENSE_BOX_WRAPER_H
