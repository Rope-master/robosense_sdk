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
 * robosense user display debug module.
 *
 */

#ifndef ROBOSENSE_DRAWRVIZ_H
#define ROBOSENSE_DRAWRVIZ_H

#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <tf/tf.h>
#include <time.h>
#include <pcl_ros/point_cloud.h>
#include "rs_perception/common/base/basic_types.h"
#include "rs_perception/common/base/object.h"

namespace robosense {
namespace perception {

template<typename PointT>
class alignas(16) DrawRviz {
	public:

	typedef pcl::PointCloud<PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	typedef typename PointCloud::ConstPtr PointCloudConstPtr;

	typedef std::shared_ptr<DrawRviz<PointT> > Ptr;
	typedef std::shared_ptr<const DrawRviz<PointT> > ConstPtr;

	DrawRviz();

	void show_car_self(const ros::Publisher &pub_car_info, const std_msgs::Header &_header, Eigen::VectorXf car_pos);

	void show_cluster(const ros::Publisher &pub_cluster_cloud, const std_msgs::Header &_header,
	                  const std::vector<typename Object<PointT>::Ptr> &clusters);

	void show_trajectory(const ros::Publisher &pub_trajectory, const std_msgs::Header &_header,
	                  const std::vector<typename Object<PointT>::Ptr> &clusters);

	void show_percept(const ros::Publisher &pub_percept_info, const std_msgs::Header &_header,
	                  const std::vector<typename Object<PointT>::Ptr> &percept_list);

	void draw_box(const typename Object<PointT>::Ptr& obj, const int &marker_id, visualization_msgs::Marker &marker, float alpha,
	              float scale = 1.0);

	void draw_cube(const typename Object<PointT>::Ptr& obj, const int &marker_id, visualization_msgs::Marker &marker, float alpha,
	               float scale = 1.0);

	void draw_text(const Eigen::Vector3f &pos, const std::string &info, const int &marker_id, visualization_msgs::Marker &marker,
	               float alpha = 1.0);

	void draw_track_arrow(const typename Object<PointT>::Ptr& obj, const int &marker_id, visualization_msgs::Marker &marker,
		float alpha = 1.0);

	pcl::PCLHeader convertROSHeader2PCLHeader(const std_msgs::Header&_header);

	std_msgs::Header convertPCLHeader2ROSHeader(const pcl::PCLHeader &_header);

	void generateColors(std::vector<Eigen::Vector3f> &colors, int num);
};

}
}
#endif //ROBOSENSE_DRAWRVIZ_H