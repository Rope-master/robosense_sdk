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
 * robosense user utility module.
 *
 */


#ifndef ROBOSENSE_UTILITY_H
#define ROBOSENSE_UTILITY_H

#include <pcl_conversions/pcl_conversions.h>
#include "rs_perception/common/base/object.h"
#include <string>
#include <fstream>


namespace robosense {
namespace perception {


template<typename PointT>
class RobosenseUtil {

	public:

	typedef pcl::PointCloud<PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	typedef typename PointCloud::ConstPtr PointCloudConstPtr;

	typedef std::shared_ptr<RobosenseUtil<PointT> > Ptr;
	typedef std::shared_ptr<const RobosenseUtil<PointT> > ConstPtr;

	RobosenseUtil();

	/**
	 save the perception results obtained from SDK to txt file.
	 */
	static int print2file(const std::vector<typename Object<PointT>::Ptr> &object_in, const std::string& savePath,
	                 const std::string& saveName, int saveFrmNumber);


	private:

	static std::string obj_types_[5]; //= {"unKnow", "pedestrian", "nonMot", "smallMot", "bigMot"};

};



}
}

#endif // ROBOSENSE_UTILITY_H
