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
 * robosense tracker module, for fast multi-objects tracking.
 *
 */

#ifndef ROBOSENSE_TRACK_OBJECT_H
#define ROBOSENSE_TRACK_OBJECT_H

#include "rs_perception/common/base/basic_types.h"
#include "rs_perception/common/base/object.h"
#include <boost/circular_buffer.hpp>

namespace robosense {
namespace perception {

template<typename PointT>
struct alignas(16) TrackObject {

	typedef pcl::PointCloud<PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	typedef typename PointCloud::ConstPtr PointCloudConstPtr;

	typedef std::shared_ptr<TrackObject<PointT> > Ptr;
	typedef std::shared_ptr<const TrackObject<PointT> > ConstPtr;

	TrackObject();
	TrackObject(typename Object<PointT>::Ptr& obj);

	void clone(const TrackObject<PointT>& t);

	//object should be tracked
	typename Object<PointT>::Ptr object;

	BBox track_box;

	int tracker_id = -1;

	Eigen::Vector3f barycenter = Eigen::Vector3f::Zero();

	// bbox info
	Eigen::Vector3f center = Eigen::Vector3f::Zero();
	Eigen::Vector3f direction = Eigen::Vector3f(1, 0, 0);
	Eigen::Vector3f size = Eigen::Vector3f::Zero();

	std::vector<float> distribute_feature;

	// states
	Eigen::Vector3f track_pos = Eigen::Vector3f::Zero();
	Eigen::Vector3f velocity = Eigen::Vector3f::Zero();

	Eigen::Vector3f acceleration = Eigen::Vector3f::Zero();
	Eigen::Matrix3f velocity_uncertainty = Eigen::Matrix3f::Identity()*5;

	boost::circular_buffer<Eigen::Vector3f> trajectory;
    boost::circular_buffer<Eigen::Vector3f> history_velocity;

	float angle_velocity = 0.f;
	float association_score = 0.f;
    float tracker_robustness = 0.f;

	double visible_track_time = 0.0;
	double track_time = 0.0;

	int track_frame_cnt = 0;

	ObjectType track_type = UNKNOW;

	bool is_tracked = false;

};

}
}
#endif //ROBOSENSE_TRACK_OBJECT_H
