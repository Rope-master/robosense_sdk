//
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
 * robosense multi-lidar pointcloud fusion model.
 *
 */

#ifndef ROBOSENSE_MULTILIDARMERGER_H
#define ROBOSENSE_MULTILIDARMERGER_H

#include "rs_perception/common/base/geo_base.h"

namespace robosense {
namespace perception {

template<typename PointT>
class alignas(16) MultiLidarMerger {
	public:

	typedef pcl::PointCloud<PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	typedef typename PointCloud::ConstPtr PointCloudConstPtr;

	typedef std::shared_ptr<MultiLidarMerger<PointT> > Ptr;
	typedef std::shared_ptr<const MultiLidarMerger<PointT> > ConstPtr;

	MultiLidarMerger(const std::string &xml_path, const int &lateral_dev_num,
	                 const Range3D &range = Range3D(-5, 5, -4, 4, -3, 2));


	bool mergeLidars(PointCloudConstPtr anchor_cloud, std::vector<PointCloudConstPtr> lateral_cloud_vec,
	                 PointCloudPtr merge_cloud, PointCloudPtr total_cloud);


	private:

	class MultiLidarMergerInternal;
	MultiLidarMergerInternal* Internal;

};


}
}



#endif //PROJECT_MULTILIDARMERGER_H
