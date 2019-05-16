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
 * robosense base module, for detection.
 *
 */

#ifndef ROBOSENSE_OBJECT_H
#define ROBOSENSE_OBJECT_H

#include "rs_perception/common/base/basic_types.h"
#include <boost/circular_buffer.hpp>

namespace robosense {
namespace perception {

template<typename PointT>
struct alignas(16) Object {

	typedef pcl::PointCloud<PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	typedef typename PointCloud::ConstPtr PointCloudConstPtr;

	typedef std::shared_ptr<Object<PointT> > Ptr;
	typedef std::shared_ptr<const Object<PointT> > ConstPtr;

	Object();
	Object(PointCloudPtr pcloud);

	//-------------functions-----------------
	//deep copy
	void clone(const Object<PointT> &m);

	//debug output to console
	void info() const;

	//output to file
	void print(const std::string& filename) const;

	//object transform
	void transform(typename Object<PointT>::Ptr object_trans, const Eigen::Matrix4f &trans_mat) const;
	void transform(const Eigen::Matrix4f &trans_mat);

	//----debug test----------
	double timestamp = 0.f;

	//---------------------object level-----------------------
	int mode = -1;

	//basic info
	int id = -1;
	PointCloudPtr cloud;
	Polygon polygon;

	BBox box;

	float length = 0.f;
	float width = 0.f;
	float height = 0.f;

	Eigen::Vector3f center = Eigen::Vector3f::Zero();
	Eigen::Vector3f direction = Eigen::Vector3f(1, 0, 0);
	float yaw = 0.f;

	Eigen::Vector3f nearest_point = Eigen::Vector3f::Zero();

	//---------------------tracking info-------------------------
	bool is_tracked = false;
	int tracker_id = -1;
	Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
	Eigen::Vector3f acceleration = Eigen::Vector3f::Zero();
	Eigen::Matrix3f velocity_uncertainty = Eigen::Matrix3f::Identity()*5;//
	float angle_velocity = 0.f;
	float association_score = 0.f;
	float tracker_robustness = 0.f;
	double visible_track_time = 0.f;
	double track_time = 0.f;
	boost::circular_buffer<Eigen::Vector3f> trajectory;
    boost::circular_buffer<Eigen::Vector3f> history_velocity;
	ObjectType track_type = UNKNOW;

	//---------------------classification info-------------------
	ObjectType type = UNKNOW;
	float type_confidence = 0.f;
	std::vector<float> type_probs;
	bool is_background = false;

};

}

}


#endif //ROBOSENSE_OBJECT_H
