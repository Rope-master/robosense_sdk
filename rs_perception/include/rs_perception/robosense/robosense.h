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
 * robosense sdk proxy module, integrated entrance for all functional modules
 * including detection, tracking and classification, etc.
 *
 */

#ifndef ROBOSENSE_ALL_H
#define ROBOSENSE_ALL_H

#include "rs_perception/common/base/object.h"
#include <pcl/PointIndices.h>

namespace robosense {
namespace perception {
/**
 * @brief integrated entrance for sdk
 */
template<typename PointT>
class alignas(16) RobosenseALL {

	public:

	typedef pcl::PointCloud<PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	typedef typename PointCloud::ConstPtr PointCloudConstPtr;

	typedef std::shared_ptr<RobosenseALL<PointT> > Ptr;
	typedef std::shared_ptr<const RobosenseALL<PointT> > ConstPtr;

	/**
	 * @brief constructor
	 * @param lidar_config_path lidar mounting calibration file, usually named as "align.txt"
	 * @param usr_config_path user-defined xml configuration file, usually named as "perception_args_xx.xml"
	 * @param class_model_path model pass for classificaiton
	 * @param deep_model_path  deeplearning model path for detection
	 * @param roi_map_path  roi filtering mask data path, if roi filtering is enabled.
	 */
	RobosenseALL(const std::string &lidar_config_path, const std::string &usr_config_path,
	             const std::string &class_model_path = "", const std::string &roi_map_path = "");

	~RobosenseALL();

	std::string getVersion();

	/**
	 * @brief The Main Proxy class API for Robosense perception SDK, all of the perception function can be entered by the class.
	 * @param in_cloud_ptr input pointcloud for perception
	 * @param timestamp timestamp of the input pointcloud
	 * @param v2g_mat vehicle (lidar) local coordinate to global coordinate transformation.
	 */
	void
	perception(PointCloudConstPtr in_cloud_ptr, double timestamp, Eigen::Matrix4f v2g_mat = Eigen::Matrix4f::Identity());

	/**
	 * @brief Get ground points
	 * @param ground_cloud_ptr
	 */
	void getGroundPoints(PointCloudPtr& ground_cloud_ptr) const;

	/**
	 * @brief Get ground points's indices instead
	 * @param ground_idx_ptr
	 */
	void getGndIndices(pcl::PointIndicesPtr& ground_idx_ptr) const;

	/**
	 * @brief Get non-ground points
	 * @param object_cloud_ptr
	 */
	void getObjectPoints(PointCloudPtr& object_cloud_ptr) const;
	/**
	 * @brief Get non-ground points's indices instead
	 * @param obstacle_idx_ptr
	 */
	void getObjIndices(pcl::PointIndicesPtr& obstacle_idx_ptr) const;

	/**
	 * @brief Get perception zone filtered pointcloud, the filtering include detection range and roi mask (if roi is enabled).
	 * This can be used to debug which zone the perception SDK is working on.
	 * @param free_cloud_ptr
	 */
	void getCloudFreeSpace(PointCloudPtr& free_cloud_ptr) const;

	/**
	 * @brief Get the percepted result: object list
	 * @param percept_result percepted result
	 * @param trans_mat transformation matrix used to decide which coordinate the percepted results are refered, i.e., When 'trans_mat'
	 * is identity, means the results is according to global coordinate by default, if you want get the result according to lidar coordinate,
	 * you should give a inverse matrix of 'v2g_mat' (see perception api above).
	 */
	void getPeceptResults(std::vector<typename Object<PointT>::Ptr> &percept_result,
												const Eigen::Matrix4f& trans_mat = Eigen::Matrix4f::Identity()) const;


	private:

	class RobosenseALLInternal;
	RobosenseALLInternal* Internal;

};


}
}

#endif //ROBOSENSE_ALL_H

