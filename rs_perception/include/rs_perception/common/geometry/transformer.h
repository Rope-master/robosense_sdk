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
 * robosense geometry tranformation module.
 *
 */

#ifndef ROBOSENSE_TRANSFORMER_H
#define ROBOSENSE_TRANSFORMER_H

#include "rs_perception/common/base/basic_types.h"
#include "rs_perception/common/base/object.h"
#include "rs_perception/common/base/configuration_type.h"

#define WITH_SORTED 0

namespace robosense {
namespace perception {

template<typename PointT>
class alignas(16) Transformer {

	public:

	typedef pcl::PointCloud<PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	typedef typename PointCloud::ConstPtr PointCloudConstPtr;

	typedef std::shared_ptr<Transformer<PointT> > Ptr;
	typedef std::shared_ptr<const Transformer<PointT> > ConstPtr;

	Transformer(const RoboLidarConfig &lidar_config, const RoboUsrConfig &args, const std::string &roi_map_path = "");

	bool preprocess(PointCloudConstPtr in_cloud_ptr, PointCloudPtr out_cloud_ptr);

	bool initCalibration(PointCloudConstPtr in_cloud_ptr, PointCloudPtr out_cloud_ptr);

	bool transFromLidar2Vehicle(PointCloudConstPtr in_cloud_ptr, PointCloudPtr out_cloud_ptr);

	bool transFromVehicle2Lidar(PointCloudConstPtr in_cloud_ptr, PointCloudPtr out_cloud_ptr);

	bool autoAlign(PointCloudConstPtr in_cloud_ptr, PointCloudPtr align_cloud_ptr, bool use_auto_align = true);


	bool transPerceptResult(const std::vector<typename Object<PointT>::Ptr> &percept_list,
	                        std::vector<typename Object<PointT>::Ptr> &percept_list_out, const Eigen::Matrix4f& trans_mat);

	bool transPerceptResult(std::vector<typename Object<PointT>::Ptr> &percept_list, const Eigen::Matrix4f& trans_mat);

	bool applyRange(PointCloudConstPtr in_cloud, PointCloudPtr out_cloud, bool use_range = false);

	bool applyRoi(PointCloudConstPtr in_cloud, PointCloudPtr out_cloud, bool use_roi = false);

	void setVehicle2GlobalMat(const Eigen::Matrix4f &vehicle2global_mat = Eigen::Matrix4f::Identity());

	bool isInValidRange(const PointT &p) const;
	bool isInValidRange2D(const PointT &p) const;
	bool isInValidROI(const PointT &p) const;
	bool isInValidRange2DAndROI(const PointT &p) const;
	bool isInValidRangeAndROI(const PointT &p) const;

	float getLidarHightEstimate() const ;

	Eigen::Matrix4f getAutoAlignMatrix() const;

	private:

	class TransformerInternal;
	TransformerInternal* Internal;

};


}
}


#endif //ROBOSENSE_TRANSFORMER_H
