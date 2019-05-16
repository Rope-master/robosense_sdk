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

#ifndef ROBOSENSE_TRACKER_H
#define ROBOSENSE_TRACKER_H

#include "rs_perception/tracking/track_object.h"
#include <boost/circular_buffer.hpp>

namespace robosense {
namespace perception {


/**
 * @brief Tracker Class, the main class for tracking multi-objects.
 */
template<typename PointT>
class alignas(16) Tracker {

	public:

	typedef pcl::PointCloud<PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	typedef typename PointCloud::ConstPtr PointCloudConstPtr;

	typedef std::shared_ptr<Tracker<PointT> > Ptr;
	typedef std::shared_ptr<const Tracker<PointT> > ConstPtr;

	Tracker(double predict_time = 0, int history_buffer_size = 10, int eva_win_size = 8);

	void setTrackerParams(double predict_time = 0, int history_buffer_size = 10,
												int eva_win_size = 8);

	void resetKalman(const Eigen::Vector4f &process_noise_conv = Eigen::Vector4f(0.1, 0.1, 0.2, 0.2),
	                 const Eigen::Vector4f &measure_noise_conv = Eigen::Vector4f(0.3, 0.3, 0.6, 0.6));

	void tracking(std::vector<typename Object<PointT>::Ptr> &obj_list, double timestamp,
				  const Eigen::Matrix4f &l2g_trans, bool use_global_refine = false);

	void getHistoricalTrackInfos(std::vector<boost::circular_buffer<typename TrackObject<PointT>::Ptr> >& track_history);

	void getBeliefTrackerBoxes(std::vector<BBox> &tracker_boxes, const Eigen::Matrix4f &g2l_trans);

	private:

	class TrackerInternal;
	TrackerInternal* Internal;
};

}
}
#endif

