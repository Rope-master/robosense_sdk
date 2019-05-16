
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
 * robosense box module, for object bounding box calculation.
 *
 */

#ifndef ROBOSENSE_BOXER_H
#define ROBOSENSE_BOXER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "rs_perception/common/base/basic_types.h"
#include "rs_perception/common/base/object.h"

namespace robosense {
namespace perception {

/**
 * @brief calculate bounding-box for pointcloud cluster
 */
template<typename PointT>
class BBoxBuilder {

	public:

	typedef pcl::PointCloud<PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	typedef typename PointCloud::ConstPtr PointCloudConstPtr;

	typedef std::shared_ptr<BBoxBuilder<PointT> > Ptr;
	typedef std::shared_ptr<const BBoxBuilder<PointT> > ConstPtr;

	BBoxBuilder();

	// build box for all
	void build(std::vector<typename Object<PointT>::Ptr>& objects);

	// build box for one
	void buildObject(typename Object<PointT>::Ptr& obj);

	private:

	class BBoxBuilderInternal;
	BBoxBuilderInternal* Internal;

};


}
}

#endif