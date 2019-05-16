/**
 *****************************************************************************
 * COPYRIGHT STATEMENT
 * Copyright (c) 2018, Robosense Co.,Ltd. - www.robosense.ai
 * All Rights Reserved.
 *
 * You can not use, copy or spread without official authorization.
 *****************************************************************************
 *
 * Author: Robosense Perception Group
 * Version: 2.0.0
 * Date: 2018.6
 *
 * DESCRIPTION
 *
 * Robosense communication module, for output perception result to user computer.
 *
 */

#ifndef ROBOSENSE_COMM_UTIL_H
#define ROBOSENSE_COMM_UTIL_H

#include "rs_perception/common/base/object.h"
#include "perceptron.pb.h"

namespace robosense {
namespace perception {


template<typename PointT>
void serialize(const typename Object<PointT>::Ptr&obj, Perceptron &pb_obj) {

	pb_obj.set_timestamp(obj->timestamp);
	pb_obj.set_id(obj->tracker_id);

	Point3 *obj_location = pb_obj.mutable_location();
	obj_location->set_x(obj->center(0));
	obj_location->set_y(obj->center(1));
	obj_location->set_z(obj->center(2));

	pb_obj.set_length(obj->length);
	pb_obj.set_width(obj->width);
	pb_obj.set_height(obj->height);

	Point3 *obj_dir = pb_obj.mutable_direction();
	obj_dir->set_x(obj->direction(0));
	obj_dir->set_y(obj->direction(1));
	obj_dir->set_z(obj->direction(2));

	pb_obj.set_yaw(obj->yaw);

	Point3 *obj_nearest_p = pb_obj.mutable_nearest_point();
	obj_nearest_p->set_x(obj->nearest_point(0));
	obj_nearest_p->set_y(obj->nearest_point(1));
	obj_nearest_p->set_z(obj->nearest_point(2));

	pb_obj.set_is_tracking_success(obj->is_tracked);

	Point3 *obj_vel = pb_obj.mutable_velocity();
	obj_vel->set_x(obj->velocity(0));
	obj_vel->set_y(obj->velocity(1));
	obj_vel->set_z(obj->velocity(2));

	Point3 *obj_acc = pb_obj.mutable_acceleration();
	obj_acc->set_x(obj->acceleration(0));
	obj_acc->set_y(obj->acceleration(1));
	obj_acc->set_z(obj->acceleration(2));

	pb_obj.set_angular_velocity(obj->angle_velocity);
	pb_obj.set_association_score(obj->association_score);
	pb_obj.set_tracker_robustness(obj->tracker_robustness);
	pb_obj.set_tracking_time(obj->track_time);
	pb_obj.set_visible_tracking_time(obj->visible_track_time);

	for (int i = 0; i < obj->history_velocity.size(); ++i) {
		Point3 *p = pb_obj.add_history_velocity();
		p->set_x(obj->history_velocity[i](0));
		p->set_y(obj->history_velocity[i](1));
		p->set_z(obj->history_velocity[i](2));
	}

	pb_obj.set_track_type(static_cast<Perceptron::Type>(obj->track_type));

	pb_obj.set_type(static_cast<Perceptron::Type>(obj->type));
	pb_obj.set_type_confidence(obj->type_confidence);

	for (int i = 0; i < obj->polygon.size(); ++i) {
		Point3 *p = pb_obj.add_polygon_point();
		p->set_x(obj->polygon.points[i].x);
		p->set_y(obj->polygon.points[i].y);
		p->set_z(obj->polygon.points[i].z);
	}

	for (int i = 0; i < obj->trajectory.size(); ++i) {
		Point3 *p = pb_obj.add_trajectory();
		p->set_x(obj->trajectory[i](0));
		p->set_y(obj->trajectory[i](1));
		p->set_z(obj->trajectory[i](2));
	}

	//	for (int i = 0; i < obj->cloud->size(); ++i) {
	//		Point3* p = pb_obj.add_cloud_point();
	//		p->set_x(obj->cloud->points[i].x);
	//		p->set_y(obj->cloud->points[i].y);
	//		p->set_z(obj->cloud->points[i].z);
	//	}

}

template<typename PointT>
void deserialize(const Perceptron &pb_obj, typename Object<PointT>::Ptr& obj) {

	obj->timestamp = pb_obj.timestamp();
	obj->tracker_id = pb_obj.id();

	obj->center(0) = pb_obj.location().x();
	obj->center(1) = pb_obj.location().y();
	obj->center(2) = pb_obj.location().z();

	obj->length = pb_obj.length();
	obj->width = pb_obj.width();
	obj->height = pb_obj.height();

	obj->direction(0) = pb_obj.direction().x();
	obj->direction(1) = pb_obj.direction().y();
	obj->direction(2) = pb_obj.direction().z();

	obj->yaw = pb_obj.yaw();

	obj->nearest_point(0) = pb_obj.nearest_point().x();
	obj->nearest_point(1) = pb_obj.nearest_point().y();
	obj->nearest_point(2) = pb_obj.nearest_point().z();

	obj->is_tracked = pb_obj.is_tracking_success();

	obj->velocity(0) = pb_obj.velocity().x();
	obj->velocity(1) = pb_obj.velocity().y();
	obj->velocity(2) = pb_obj.velocity().z();

	obj->acceleration(0) = pb_obj.acceleration().x();
	obj->acceleration(1) = pb_obj.acceleration().y();
	obj->acceleration(2) = pb_obj.acceleration().z();

	obj->angle_velocity = pb_obj.angular_velocity();
	obj->association_score = pb_obj.association_score();
	obj->tracker_robustness = pb_obj.tracker_robustness();
	obj->track_time = pb_obj.tracking_time();
	obj->visible_track_time = pb_obj.visible_tracking_time();

	obj->history_velocity.clear();
	for (int i = 0; i < pb_obj.history_velocity_size(); ++i) {
		const auto &p = pb_obj.history_velocity(i);
		obj->history_velocity.push_back(Eigen::Vector3f(p.x(), p.y(), p.z()));
	}

	obj->track_type = static_cast<ObjectType>(pb_obj.track_type());

	obj->type = static_cast<ObjectType>(pb_obj.type());
	obj->type_confidence = pb_obj.type_confidence();

	obj->polygon.clear();
	for (int i = 0; i < pb_obj.polygon_point_size(); ++i) {
		const auto &p = pb_obj.polygon_point(i);
		PolygonType point;
		point.x = p.x();
		point.y = p.y();
		point.z = p.z();
		obj->polygon.points.emplace_back(point);
	}

	obj->trajectory.clear();
	for (int i = 0; i < pb_obj.trajectory_size(); ++i) {
		const auto &p = pb_obj.trajectory(i);
		obj->trajectory.push_back(Eigen::Vector3f(p.x(), p.y(), p.z()));
	}

	//	obj->cloud->clear();
	//	for (int i = 0; i < pb_obj.cloud_point_size(); ++i) {
	//		const auto& p = pb_obj.cloud_point(i);
	//		PointT point;
	//		point.x = p.x();
	//		point.y = p.y();
	//		point.z = p.z();
	//		obj->cloud->points.emplace_back(point);
	//	}

}

}
}


#endif //ROBOSENSE_COMM_UTIL_H
