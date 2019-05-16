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
 * robosense freespace module, for free space detection
 *
 */

#ifndef ROBOSENSE_MAP_FREE_SPACE_H
#define ROBOSENSE_MAP_FREE_SPACE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace robosense {
namespace perception {
/** @brief Class for free space filter
	*/
template<typename PointT>
class MapFreeSpace {
  public:

  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

  typedef std::shared_ptr<MapFreeSpace<PointT> > Ptr;
  typedef std::shared_ptr<const MapFreeSpace<PointT> > ConstPtr;

  MapFreeSpace();

  bool setMapLabel(const std::string &map_label_path);

  bool
  mapFreeSpace(PointCloudConstPtr in_cloud_ptr, const Eigen::Matrix4f &global_trans_mat, PointCloudPtr roi_cloud_ptr,
               int mode = 0);

  bool mapFreeSpace(PointCloudConstPtr in_cloud_ptr, const Eigen::Matrix4f &global_trans_mat,
                    std::vector<int> &free_space_indices, int mode = 0);

  bool isInROI(const PointT& tmp_pt) const;

  private:

  class MapFreeSpaceInternal;
  MapFreeSpaceInternal* Internal;

};
}
}

#endif //ROBOSENSE_MAP_FREE_SPACE_H

/**
 * @brief free_space module
 * @defgroup free_space
 */
