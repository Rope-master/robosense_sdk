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

#include "display/robo_draw_rviz.h"

#define USE_ClUSER_COLOR 0
#define TEXT_SCALE (0.8)

namespace robosense {
namespace perception {

static int marker_per_num = 0;
static std::vector<Eigen::Vector3f> colors_;

template<typename PointT>
DrawRviz<PointT>::DrawRviz() {

	generateColors(colors_, 100);

}



template<typename PointT>
pcl::PCLHeader DrawRviz<PointT>::convertROSHeader2PCLHeader(const std_msgs::Header &_header) {
	pcl::PCLHeader header;
	header.seq = _header.seq;
	header.stamp = pcl_conversions::toPCL(_header.stamp);
	header.frame_id = _header.frame_id;

	return header;
}


template<typename PointT>
std_msgs::Header DrawRviz<PointT>::convertPCLHeader2ROSHeader(const pcl::PCLHeader &_header) {
	std_msgs::Header header;
	header.seq = _header.seq;
	header.stamp = pcl_conversions::fromPCL(_header.stamp);
	header.frame_id = _header.frame_id;

	return header;
}


template<typename PointT>
void DrawRviz<PointT>::generateColors(std::vector<Eigen::Vector3f> &colors, int num) {
	colors.clear();
	colors.resize(num);
	for (int i = 0; i < num; ++i) {
		Eigen::Vector3f color;
		color(0) = rand() % 255;
		color(1) = rand() % 255;
		color(2) = rand() % 255;

		color += Eigen::Vector3f(60, 60, 60);

		color(0) = color(0) < 255 ? color(0) : 255;
		color(1) = color(1) < 255 ? color(1) : 255;
		color(2) = color(2) < 255 ? color(2) : 255;

		colors[i] = color;
	}
}


template<typename PointT>
void DrawRviz<PointT>::show_car_self(const ros::Publisher &pub_car_info, const std_msgs::Header &_header,
                                     Eigen::VectorXf car_pos) {
	visualization_msgs::MarkerArrayPtr marker_array(new visualization_msgs::MarkerArray);

	visualization_msgs::Marker marker_car;
	marker_car.header = _header;
	marker_car.id = 0;
	marker_car.ns = "car_vel_info";
	marker_car.scale.x = TEXT_SCALE * 1.1;
	marker_car.scale.y = TEXT_SCALE * 1.1;
	marker_car.scale.z = TEXT_SCALE * 1.1;
	marker_car.color.r = 1.0;
	marker_car.color.g = 0.7;
	marker_car.color.a = 1.0;

	Eigen::Vector3f self_positon = car_pos.head(3);
	Eigen::Vector3f self_angle = car_pos.segment(3, 3);
	Eigen::Vector3f self_velocity = car_pos.tail(3);

	std::string info_self = num2str<float>(self_velocity.norm() * 3.6f, 1) + "km/h";
	draw_text(self_positon, info_self, 0, marker_car, 1.0);
	marker_array->markers.push_back(marker_car);

	visualization_msgs::Marker marker_yaw_dir;

	marker_yaw_dir.pose.position.x = self_positon(0);
	marker_yaw_dir.pose.position.y = self_positon(1);
	marker_yaw_dir.pose.position.z = self_positon(2);
	marker_yaw_dir.header = _header;
	marker_yaw_dir.id = 0;
	marker_yaw_dir.ns = "car_dir_info";
	marker_yaw_dir.scale.x = 2.0;
	marker_yaw_dir.scale.y = 0.15;
	marker_yaw_dir.scale.z = 0.15;
	marker_yaw_dir.color.r = 1.0;
	marker_yaw_dir.color.g = 0.0;
	marker_yaw_dir.color.b = 1.0;
	marker_yaw_dir.color.a = 1.0;
	marker_yaw_dir.type = visualization_msgs::Marker::ARROW;
	marker_yaw_dir.action = visualization_msgs::Marker::ADD;
	tf::Quaternion quat = tf::createQuaternionFromRPY(self_angle(0), self_angle(1), self_angle(2));
	tf::quaternionTFToMsg(quat, marker_yaw_dir.pose.orientation);

	marker_array->markers.push_back(marker_yaw_dir);


	pub_car_info.publish(marker_array);
}

template<typename PointT>
void DrawRviz<PointT>::show_percept(const ros::Publisher &pub_percept_info, const std_msgs::Header &_header,
                                    const std::vector<typename Object<PointT>::Ptr> &percept_list) {
	visualization_msgs::MarkerArrayPtr marker_array(new visualization_msgs::MarkerArray);
	visualization_msgs::Marker marker_box;
	visualization_msgs::Marker marker_cube;
	visualization_msgs::Marker marker_text_box;
	visualization_msgs::Marker marker_text_label;
	visualization_msgs::Marker marker_text_track;
	visualization_msgs::Marker marker_track_arrow;

	Eigen::Vector3f colors[5];
	colors[0] = Eigen::Vector3f(0.6, 0.5, 0.6);//淡紫色 background
	colors[1] = Eigen::Vector3f(1, 1, 0);//黄色 pedestrian
	colors[2] = Eigen::Vector3f(0, 1, 1);//青色 bike
	colors[3] = Eigen::Vector3f(0.5, 0, 0);//红色 car
	colors[4] = Eigen::Vector3f(0, 0, 1);//蓝色 truck

	std::string info[5] = {"unknow", "ped", "bike", "car", "truck"};

	marker_text_box.header = _header;
	marker_text_box.ns = "box_info";
	marker_text_box.scale.x = TEXT_SCALE;
	marker_text_box.scale.y = TEXT_SCALE;
	marker_text_box.scale.z = TEXT_SCALE;
	marker_text_box.color.r = 0.f;
	marker_text_box.color.g = 0.6f;
	marker_text_box.color.b = 0.7f;
	marker_text_box.color.a = 1.f;

	marker_text_track.header = _header;
	marker_text_track.ns = "track_info";
	marker_text_track.scale.x = TEXT_SCALE;
	marker_text_track.scale.y = TEXT_SCALE;
	marker_text_track.scale.z = TEXT_SCALE;
	marker_text_track.color.r = 0.f;
	marker_text_track.color.g = 0.7f;
	marker_text_track.color.b = 0.6f;
	marker_text_track.color.a = 1.f;

	marker_text_label.header = _header;
	marker_text_label.ns = "label_info";
	marker_text_label.scale.x = TEXT_SCALE;
	marker_text_label.scale.y = TEXT_SCALE;
	marker_text_label.scale.z = TEXT_SCALE;
	marker_text_label.color.a = 1.0;

	marker_track_arrow.header = _header;
	marker_track_arrow.ns = "velocity_dir";
	marker_track_arrow.color.r = 0.f;
	marker_track_arrow.color.g = 0.7f;
	marker_track_arrow.color.b = 0.3f;
	marker_track_arrow.color.a = 0.8f;

	marker_box.header = _header;
	marker_box.ns = "box";
	marker_box.color.r = colors[0](0);
	marker_box.color.g = colors[0](1);
	marker_box.color.b = colors[0](2);
	marker_box.scale.x = marker_box.scale.y = marker_box.scale.z = 0.03;

	marker_cube.header = _header;
	marker_cube.ns = "cube";

	//	std::string link_mode[4] ={"Bary", "Center", "S-center", "Corner"};

	int marker_id = 0;
	for (int i = 0; i < percept_list.size(); ++i) {
		const typename Object<PointT>::Ptr& perceptron = percept_list[i];

		//-------------------------------box----------------------------
		draw_box(perceptron, marker_id, marker_box, 0.5, 1.0);

		Eigen::Vector3f nearest = perceptron->nearest_point;
		std::string text_box =
			num2str<float>(nearest.norm(), 1) + " (" + num2str<float>(perceptron->center(0), 1) + " " +
			num2str<float>(perceptron->center(1), 1) + " " + num2str<float>(perceptron->center(2), 1) + ")";

		Eigen::Vector3f pos0 = perceptron->center;
		pos0(2) += perceptron->height * 0.5f + 0.2f;
		draw_text(pos0, text_box, marker_id, marker_text_box, 1.0);

		marker_array->markers.push_back(marker_box);
		marker_array->markers.push_back(marker_text_box);
		
		//--------------------------------tracking------------------------------
		float velocity = perceptron->velocity.norm();
		bool is_valid_vel = velocity > 0.3f;
		draw_track_arrow(perceptron, marker_id, marker_track_arrow, is_valid_vel ? 0.8 : 0);

		Eigen::Vector3f pos1 = perceptron->center;
		pos1(2) += perceptron->height * 0.5f + 0.7f;

		float angle_vel = perceptron->angle_velocity / PI_OVER_180;

		std::string text_track =
			"<" + num2str<int>(perceptron->tracker_id, 0) + ">" + num2str<float>(velocity * 3.6f, 1) + "km/h"
//			+ "--> " + num2str<float>(perceptron->yaw/PI_OVER_180, 1);
			+" >>" + num2str<float>(perceptron->association_score, 3);

//			+" >> " + num2str<float>(perceptron->angle_velocity/PI_OVER_180, 2); //association_score

		//num2str<float>(angle_vel, 1);
		//+ " / " + num2str<float>(perceptron->sequence_robustness, 3);
		draw_text(pos1, text_track, marker_id, marker_text_track, is_valid_vel ? 0.8 : 0);

		marker_array->markers.push_back(marker_track_arrow);
		marker_array->markers.push_back(marker_text_track);


		//--------------------------------classification---------------------------
		int type = perceptron->type;
		Eigen::Vector3f color = colors[type];
		bool is_bgd = (type == 0);
		//cube
		marker_cube.color.r = color(0);
		marker_cube.color.g = color(1);
		marker_cube.color.b = color(2);

		draw_cube(perceptron, marker_id, marker_cube, is_bgd ? 0.3 : 0.6, 1.0);

		marker_text_label.color.r = color(0);
		marker_text_label.color.g = color(1);
		marker_text_label.color.b = color(2);

		Eigen::Vector3f pos2 = perceptron->center;
		pos2(2) += perceptron->height * 0.5f + 1.2f;

		std::string text_label = info[type] + " >>" + num2str<float>(perceptron->type_confidence, 2);

		draw_text(pos2, text_label, marker_id, marker_text_label, is_bgd ? 0 : 0.95);
		
		marker_array->markers.push_back(marker_cube);
		marker_array->markers.push_back(marker_text_label);

		marker_id++;
	}

	if (marker_id < marker_per_num) {
		int k = 0;
		for (int i = marker_id; i < marker_per_num; ++i) {
			marker_box.id = marker_id + k;
			marker_box.color.a = 0.f;
			marker_array->markers.push_back(marker_box);

			marker_cube.id = marker_id + k;
			marker_cube.color.a = 0.f;
			marker_array->markers.push_back(marker_cube);

			marker_text_box.id = marker_id + k;
			marker_text_box.color.a = 0.f;
			marker_array->markers.push_back(marker_text_box);

			marker_text_track.id = marker_id + k;
			marker_text_track.color.a = 0.f;
			marker_array->markers.push_back(marker_text_track);

			marker_track_arrow.id = marker_id + k;
			marker_track_arrow.color.a = 0.f;
			marker_array->markers.push_back(marker_track_arrow);

			marker_text_label.id = marker_id + k;
			marker_text_label.color.a = 0.f;
			marker_array->markers.push_back(marker_text_label);

			k++;
		}
	}

	marker_per_num = marker_id;
	pub_percept_info.publish(marker_array);

}


template<typename PointT>
void DrawRviz<PointT>::show_cluster(const ros::Publisher &pub_cluster_cloud, const std_msgs::Header &_header,
                                    const std::vector<typename Object<PointT>::Ptr> &clusters) {

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

#if !USE_ClUSER_COLOR
	Eigen::Vector3f colors[5];
	colors[0] = Eigen::Vector3f(0.6, 0.5, 0.6);//淡紫色 background
	colors[1] = Eigen::Vector3f(1, 1, 0);//黄色 pedestrian
	colors[2] = Eigen::Vector3f(0, 1, 1);//青色 bike
	colors[3] = Eigen::Vector3f(0.5, 0, 0);//红色 car
	colors[4] = Eigen::Vector3f(0, 0, 1);//蓝色 truck
#endif

	for (int i = 0; i < clusters.size(); ++i) {
#if !USE_ClUSER_COLOR
		Eigen::Vector3f color = colors[clusters[i]->type];
#endif

#if 1 //画点云
		for (int j = 0; j < clusters[i]->cloud->size(); ++j) {
			const PointT& tmp_point = clusters[i]->cloud->points[j];
			pcl::PointXYZRGB tmp_point_rgb;
			tmp_point_rgb.x = tmp_point.x;
			tmp_point_rgb.y = tmp_point.y;
			tmp_point_rgb.z = tmp_point.z;
#else //画多边形
		for (int j = 0; j < clusters[i]->polygon.size(); ++j) {
			const PolygonType& tmp_point = clusters[i]->polygon.points[j];
			pcl::PointXYZRGB tmp_point_rgb;
			tmp_point_rgb.x = tmp_point.x;
			tmp_point_rgb.y = tmp_point.y;
			tmp_point_rgb.z = tmp_point.z;
#endif

#if USE_ClUSER_COLOR
			tmp_point_rgb.r = (uint8_t)colors_[i % 100](0);
			tmp_point_rgb.g = (uint8_t)colors_[i % 100](1);
			tmp_point_rgb.b = (uint8_t)colors_[i % 100](2);
#else
			tmp_point_rgb.r = (uint8_t) (color(0) * 255);
			tmp_point_rgb.g = (uint8_t) (color(1) * 255);
			tmp_point_rgb.b = (uint8_t) (color(2) * 255);
#endif
			cluster_cloud_ptr->push_back(tmp_point_rgb);
		}

	}

	cluster_cloud_ptr->header = convertROSHeader2PCLHeader(_header);
	pub_cluster_cloud.publish(cluster_cloud_ptr);
}


template<typename PointT>
void DrawRviz<PointT>::show_trajectory(const ros::Publisher &pub_trajectory, const std_msgs::Header &_header,
                     const std::vector<typename Object<PointT>::Ptr> &clusters){

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr trajectory_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

#if !USE_ClUSER_COLOR
	Eigen::Vector3f colors[5];
	colors[0] = Eigen::Vector3f(0.6, 0.5, 0.6);//淡紫色 background
	colors[1] = Eigen::Vector3f(1, 1, 0);//黄色 pedestrian
	colors[2] = Eigen::Vector3f(0, 1, 1);//青色 bike
	colors[3] = Eigen::Vector3f(0.5, 0, 0);//红色 car
	colors[4] = Eigen::Vector3f(0, 0, 1);//蓝色 truck
#endif

	for (int i = 0; i < clusters.size(); ++i) {
		const typename Object<PointT>::Ptr& obj = clusters[i];

#if !USE_ClUSER_COLOR
		Eigen::Vector3f color = colors[obj->type];
#endif

		for (int j = 0; j < obj->trajectory.size(); ++j) {
			pcl::PointXYZRGB tmp_point_rgb;
			tmp_point_rgb.x = obj->trajectory[j][0];
			tmp_point_rgb.y = obj->trajectory[j][1];
			tmp_point_rgb.z = obj->trajectory[j][2];


#if USE_ClUSER_COLOR
			tmp_point_rgb.r = (uint8_t)colors_[i % 100](0);
			tmp_point_rgb.g = (uint8_t)colors_[i % 100](1);
			tmp_point_rgb.b = (uint8_t)colors_[i % 100](2);
#else
			tmp_point_rgb.r = (uint8_t) (color(0) * 255);
			tmp_point_rgb.g = (uint8_t) (color(1) * 255);
			tmp_point_rgb.b = (uint8_t) (color(2) * 255);
#endif
			trajectory_cloud_ptr->push_back(tmp_point_rgb);
		}

	}

	trajectory_cloud_ptr->header = convertROSHeader2PCLHeader(_header);
	pub_trajectory.publish(trajectory_cloud_ptr);
}


template<typename PointT>
void DrawRviz<PointT>::draw_cube(const typename Object<PointT>::Ptr& obj, const int &marker_id, visualization_msgs::Marker &marker,
                                 float alpha, float scale) {
	marker.id = marker_id;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;

	BBox box;
	box.center = obj->center;
	box.size = Eigen::Vector3f(obj->length, obj->width, obj->height);
	box.heading = obj->direction;
	box.angle = obj->yaw;

	float box_size = box.volume();
	if (box_size > 0) {
		marker.color.a = alpha;

		BBox box_s = box.scale(scale);

		marker.pose.position.x = box_s.center(0);
		marker.pose.position.y = box_s.center(1);
		marker.pose.position.z = box_s.center(2);

		marker.scale.x = box_s.size(0);
		marker.scale.y = box_s.size(1);
		marker.scale.z = box_s.size(2);

		tf::Quaternion quat = tf::createQuaternionFromYaw(box.angle);
		tf::quaternionTFToMsg(quat, marker.pose.orientation);

	} else {
		marker.color.a = 0;
	}

}


template<typename PointT>
void DrawRviz<PointT>::draw_box(const typename Object<PointT>::Ptr& obj, const int &marker_id, visualization_msgs::Marker &marker,
                                float alpha, float scale) {
	marker.id = marker_id;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = visualization_msgs::Marker::ADD;

	BBox box;
	box.center = obj->center;
	box.size = Eigen::Vector3f(obj->length, obj->width, obj->height);
	box.heading = obj->direction;
	box.angle = obj->yaw;

	float box_size = box.volume();
	if (box_size > 0) {

		tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, 0);
		tf::quaternionTFToMsg(quat, marker.pose.orientation);

		marker.color.a = alpha;
		std::vector<geometry_msgs::Point> cub_points;

		std::vector<Eigen::Vector3f> corners;
		box.corners(corners);

		for (int i = 0; i < 8; ++i) {
			geometry_msgs::Point pts;
			pts.x = corners[i](0);
			pts.y = corners[i](1);
			pts.z = corners[i](2);
			cub_points.push_back(pts);
		}


		marker.points.push_back(cub_points[0]);
		marker.points.push_back(cub_points[1]);
		marker.points.push_back(cub_points[1]);
		marker.points.push_back(cub_points[2]);
		marker.points.push_back(cub_points[2]);
		marker.points.push_back(cub_points[3]);
		marker.points.push_back(cub_points[3]);
		marker.points.push_back(cub_points[0]);
		// horizontal high points for lines
		marker.points.push_back(cub_points[4]);
		marker.points.push_back(cub_points[5]);
		marker.points.push_back(cub_points[5]);
		marker.points.push_back(cub_points[6]);
		marker.points.push_back(cub_points[6]);
		marker.points.push_back(cub_points[7]);
		marker.points.push_back(cub_points[7]);
		marker.points.push_back(cub_points[4]);
		// vertical points for lines
		marker.points.push_back(cub_points[0]);
		marker.points.push_back(cub_points[4]);
		marker.points.push_back(cub_points[1]);
		marker.points.push_back(cub_points[5]);
		marker.points.push_back(cub_points[2]);
		marker.points.push_back(cub_points[6]);
		marker.points.push_back(cub_points[3]);
		marker.points.push_back(cub_points[7]);
	} else {
		marker.color.a = 0;
	}
}

template<typename PointT>
void DrawRviz<PointT>::draw_track_arrow(const typename Object<PointT>::Ptr& obj, const int &marker_id,
                                        visualization_msgs::Marker &marker, float alpha) {
	marker.id = marker_id;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;

	BBox box;
	box.center = obj->center;
	box.size = Eigen::Vector3f(obj->length, obj->width, obj->height);
	box.heading = obj->direction;
	box.angle = obj->yaw;

	float box_size = obj->box.volume();
	if (box_size > 0) {
		marker.color.a = alpha;


		float arrow_length = obj->velocity.norm();
		float main_direction = atan2f(obj->velocity(1), obj->velocity(0));

		marker.scale.x = sqrtf(arrow_length + 1.0) - 1.0;
		if (obj->type == 0) {
			marker.scale.y = 0.1;
			marker.scale.z = 0.1;
		} else {
			marker.scale.y = 0.2;
			marker.scale.z = 0.2;
		}

		tf::Quaternion quat = tf::createQuaternionFromRPY(0., 0., main_direction);
		tf::quaternionTFToMsg(quat, marker.pose.orientation);
		marker.pose.position.x = obj->center(0);
		marker.pose.position.y = obj->center(1);
		marker.pose.position.z = obj->center(2);

	} else {
		marker.color.a = 0;
	}
}


template<typename PointT>
void DrawRviz<PointT>::draw_text(const Eigen::Vector3f &pos, const std::string &info, const int &marker_id,
                                 visualization_msgs::Marker &marker, float alpha) {
	marker.id = marker_id;
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = pos(0);
	marker.pose.position.y = pos(1);
	marker.pose.position.z = pos(2);

	tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, 0);
	tf::quaternionTFToMsg(quat, marker.pose.orientation);

	marker.color.a = alpha;

	marker.text = info;
}

template
class DrawRviz<pcl::PointXYZI>;

}
}
