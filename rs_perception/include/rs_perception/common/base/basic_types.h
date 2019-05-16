
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
 * robosense built-in data structs type definition module.
 *
 */

#ifndef ROBOSENSE_BASIC_TYPES_H
#define ROBOSENSE_BASIC_TYPES_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <iomanip>

//#define RESET   "\033[0m"
//#define BLACK   "\033[30m"      /* Black */
//#define RED     "\033[31m"      /* Red */
//#define GREEN   "\033[32m"      /* Green */
//#define YELLOW  "\033[33m"      /* Yellow */
//#define BLUE    "\033[34m"      /* Blue */
//#define MAGENTA "\033[35m"      /* Magenta */
//#define CYAN    "\033[36m"      /* Cyan */
//#define WHITE   "\033[37m"      /* White */
//#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
//#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
//#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
//#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
//#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
//#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
//#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
//#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

#define COUT(X) std::cout <<std::setiosflags(std::ios::fixed)<< X << "\033[3m" << "\r" << std::flush << std::endl
#define COUTR(X) std::cout <<std::setiosflags(std::ios::fixed)<< "\033[1;31m " << X << "\033[0m" << "\r" << std::flush << std::endl
#define COUTG(X) std::cout <<std::setiosflags(std::ios::fixed)<< "\033[1;32m " << X << "\033[0m" << "\r" << std::flush << std::endl
#define COUTY(X) std::cout <<std::setiosflags(std::ios::fixed)<< "\033[1;33m " << X << "\033[0m" << "\r" << std::flush << std::endl
#define COUTB(X) std::cout <<std::setiosflags(std::ios::fixed)<< "\033[1;34m " << X << "\033[0m" << "\r" << std::flush << std::endl
#define COUTP(X) std::cout <<std::setiosflags(std::ios::fixed)<< "\033[1;35m " << X << "\033[0m" << "\r" << std::flush << std::endl

namespace robosense {
namespace perception {

//define some common value
const double PI_OVER_180 = (0.0174532925);
const double RS_EPS = (1e-6);
const double RS_MAX = (1e6);
const double RS_MIN = (-1e6);
const size_t HISTORY_BUFF_SIZE = 15;

typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<Point>::Ptr PointCloudPtr;
typedef pcl::PointCloud<Point>::ConstPtr PointCloudConstPtr;

enum ObjectType { UNKNOW = 0, PED = 1, BIC = 2, CAR = 3, TRUCK_BUS = 4, MAX_OBJ_TYPE_NUM = 5 };

typedef pcl::PointXYZ PolygonType;
typedef pcl::PointCloud<PolygonType> Polygon;


template<typename T>
std::string num2str(T num, int precision) {
	std::stringstream ss;
	ss.setf(std::ios::fixed, std::ios::floatfield);
	ss.precision(precision);
	std::string st;
	ss << num;
	ss >> st;

	return st;
}


/**
 * @brief Range definitions for 2D
 */
struct alignas(16) Range2D
{
	Range2D() = default;

	Range2D(float x_min, float x_max, float y_min, float y_max);

	Range2D(const Range2D &r);

	Range2D &operator=(const Range2D &r);

	float xmin = 0, xmax = 0, ymin = 0, ymax = 0;
};

/**
 * @brief Range definitions for 3D
 */
struct alignas(16) Range3D
{
	Range3D() = default;

	Range3D(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);

	Range3D(const Range3D &r);

	Range3D &operator=(const Range3D &r);

	float xmin = 0, xmax = 0, ymin = 0, ymax = 0, zmin = 0, zmax = 0;
};


struct alignas(16) BBox {

	typedef std::shared_ptr<BBox> Ptr;
	typedef std::shared_ptr<const BBox> ConstPtr;

	BBox() = default;

	BBox(const Eigen::Vector3f &center, const Eigen::Vector3f &size, const Eigen::Vector3f &dir);
	BBox(const Eigen::Vector3f &center, const Eigen::Vector3f &size, const float &ang);

	float area() const;
	float volume() const;
	float ratio() const;
	bool isNull() const;
	bool isEqual(const BBox& box) const;

	void corners(std::vector<Eigen::Vector3f>& corners) const;
	Eigen::Vector3f anchor() const;

	BBox transform(const Eigen::Matrix4f &trans_mat) const;
	BBox scale(float ratio) const;
	BBox resize(const Eigen::Vector3f& new_size, bool align = true) const;
	BBox trans(const Eigen::Vector3f& t) const;
	BBox rotate2d(float angle) const;
	BBox resetDir(const Eigen::Vector3f new_dir) const;
	BBox resetAngle(float angle) const;

	bool isInside(const Eigen::Vector3f point) const;
	int relationWith(const BBox &box) const;

	Eigen::Vector3f size = Eigen::Vector3f::Zero();
	Eigen::Vector3f center = Eigen::Vector3f::Zero();
	Eigen::Vector3f heading = Eigen::Vector3f(1,0,0);
	float angle = 0.f; /**< yaw angle of bbox direction, coincide with heading*/

};


/**
 * @brief pose
 */
struct alignas(16) Pose //
{
	Eigen::Vector3f pos; /**< translation, with meter unit*/
	Eigen::Vector3f angle; /**< rotation, with radian unit*/
};


/**
 * @brief lidar device type related parameters, mainly for rotated lidar
 */
struct alignas(16) LidarType //
{
	std::string name;/**< lidar type name*/
	float hori_resolution;/**< lidar angle resolution between points in same line, with degree unit*/
	float vert_resolution;/**< lidar angle resolution between lines, with degree unit*/
};


/**
 * @brief ObjectLimit parameters container, in order to filter out background and static objects so that the following tracking
 * and classification can focus on the foreground dynamic objects, the lidar assumed to be mounted horizontally.
 */
struct alignas(16) ObjectLimit
{
	float obj_max_height; /**<height threshold for an object's absolute height from the ground*/
	float obj_size_height; /**<height threshold for an object size to be detected*/
	float obj_size_length; /**<length threshold for an object size to be detected*/
	float obj_size_width; /**<width threshold for an object size to be detected*/
	float obj_length_width_ratio; /**<the ratio of height and length threshold for an object to be detected*/
	float obj_max_area; /**< possible vechile-like object area restriction*/
};


}
}

#endif //ROBOSENSE_BASIC_TYPES_H
