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
 * robosense communication module for ros.
 *
 */

#include "rs_perception/common/base/basic_types.h"
#include "rs_perception/PerceptionListMsg.h"
#include "std_msgs/Header.h"

#ifndef ROBOSENSE_GEN_ROS_MSGS_H
#define ROBOSENSE_GEN_ROS_MSGS_H

namespace robosense {
namespace perception {

template<typename PointT>
rs_perception::PerceptionListMsg
genRosMessages(const std::vector<typename Object<PointT>::Ptr> &objects, const Pose& pose, const std_msgs::Header &header) {
	rs_perception::PerceptionListMsg percept_list;

	percept_list.header = header;
	percept_list.pose.pos.x = pose.pos(0);
    percept_list.pose.pos.y = pose.pos(1);
    percept_list.pose.pos.z = pose.pos(2);
    percept_list.pose.angle.x = pose.angle(0);
    percept_list.pose.angle.y = pose.angle(1);
    percept_list.pose.angle.z = pose.angle(2);

	for (int i = 0; i < objects.size(); ++i) {
		const typename Object<PointT>::Ptr &obj = objects[i];

		rs_perception::PerceptionMsg per;

		per.location.x = obj->center(0);
		per.location.y = obj->center(1);
		per.location.z = obj->center(2);

		per.direction.x = obj->direction(0);
		per.direction.y = obj->direction(1);
		per.direction.z = obj->direction(2);

		per.yaw = obj->yaw;

		per.length = obj->length;
		per.width = obj->width;
		per.height = obj->height;

		per.nearest_point.x = obj->nearest_point(0);
		per.nearest_point.y = obj->nearest_point(1);
		per.nearest_point.z = obj->nearest_point(2);

		std::vector<Eigen::Vector3f> corners;
		obj->box.corners(corners);
        for (int k = 0; k < 8; ++k) {
            geometry_msgs::Point vec;
            vec.x = corners[k](0);
            vec.y = corners[k](1);
            vec.z = corners[k](2);
            per.box_corners.push_back(vec);
        }

		per.is_tracking_success = obj->is_tracked;
		per.id = obj->tracker_id;
		per.association_score = obj->association_score;
		per.tracker_robustness = obj->tracker_robustness;

		per.velocity.x = obj->velocity(0);
		per.velocity.y = obj->velocity(1);
		per.velocity.z = obj->velocity(2);;

		per.acceleration.x = obj->acceleration(0);
		per.acceleration.y = obj->acceleration(1);
		per.acceleration.z = obj->acceleration(2);

		per.angular_velocity = obj->angle_velocity;
		per.tracking_time = obj->track_time;
		per.visible_tracking_time = obj->visible_track_time;

        for (int k = 0; k < obj->history_velocity.size(); ++k) {
            geometry_msgs::Vector3 vec;
            vec.x = obj->history_velocity[k](0);
            vec.y = obj->history_velocity[k](1);
            vec.z = obj->history_velocity[k](2);
            per.history_velocity.push_back(vec);
        }

        for (int k = 0; k < obj->trajectory.size(); ++k) {
            geometry_msgs::Point vec;
            vec.x = obj->trajectory[k](0);
            vec.y = obj->trajectory[k](1);
            vec.z = obj->trajectory[k](2);
            per.trajectory.push_back(vec);
        }

		per.type = static_cast<int>(obj->type);
		per.type_confidence = obj->type_confidence;
		per.is_background = obj->is_background;

        for (int k = 0; k < obj->polygon.size(); ++k) {
            geometry_msgs::Point vec;
            vec.x = obj->polygon.points[k].x;
            vec.y = obj->polygon.points[k].y;
            vec.z = obj->polygon.points[k].z;
            per.polygon_point.push_back(vec);
        }

//		for (int k = 0; k < obj->cloud->size(); ++k) {
//			geometry_msgs::Point vec;
//			vec.x = obj->cloud->points[k].x;
//			vec.y = obj->cloud->points[k].y;
//			vec.z = obj->cloud->points[k].z;
//			per.cloud.push_back(vec);
//		}

		percept_list.perceptions.push_back(per);

	}

	return percept_list;
}

}
}

#endif //ROBOSENSE_GEN_ROS_MSGS_H
