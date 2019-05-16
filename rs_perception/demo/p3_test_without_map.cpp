
/**
 * Example of Robosense SDK usage, bare use, without map and localization.
 */

#include <ros/package.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "rs_perception/robosense/robosense.h"
#include "rs_perception/module_manager/module_manager.h"
#include "rs_perception/common/geometry/multiLidarMerger.h"
#include "communication/communication.h"
#include "communication/communication2.h"
#include "display/robo_draw_rviz.h"
#include "ros_message/gen_ros_msgs.hpp"
#include "util/robosense_util.h"

#define DEBUGING 1
#define USE_PRINT 0
#define USE_COMM 1
#define COMM_MODE 0
#define WITH_DRAW 1

using namespace robosense::perception;

static int frame_id = 0;

// give the lisense key file and data path to module manager to authorization.
// Notice that this module statement must lay before all SDK module defined.
ModuleManager::Ptr module_manager;//
DrawRviz<Point>::Ptr dr;
RobosenseALL<Point>::Ptr robosense_all;
#if (COMM_MODE==0)
RobosenseConmunicater<Point>::Ptr communicator;
#elif (COMM_MODE == 1)
RobosenseConmunicater2<Point>::Ptr communicator;
#endif
RobosenseUtil<Point>::Ptr robo_tool;

MultiLidarMerger<Point>::Ptr merger;

ros::Publisher pub_ground;
ros::Publisher pub_no_ground;
ros::Publisher pub_percept;
ros::Publisher pub_car;
ros::Publisher pub_clusters_cloud;
ros::Publisher pub_trajectory_cloud;
ros::Publisher pub_ori_cloud;
ros::Publisher pub_percept_result;
ros::Publisher pub_free_cloud;

static std::string left_lidar_topic   ;
static std::string right_lidar_topic  ;
static std::string middle_lidar_topic ;

/**
 * @brief callback function: accept pointcloud data parsed by rslidar driver and process perception algorithm
 * @param pts_msg[in]: input pointcloud message obtained by rslidar driver.
 */
void fullscanCallback(const sensor_msgs::PointCloud2ConstPtr& left_rslidar_msg,  const sensor_msgs::PointCloud2ConstPtr& middle_rslidar_msg,
                      const sensor_msgs::PointCloud2ConstPtr& right_rslidar_msg)
{
#if DEBUGING
    COUT( "---------------------------" << frame_id << "-----------------------------" );
    clock_t timer_total = clock();
#endif
    
    //-----------step 1: translate the ROS PointCloud2 msg into pcl pointcloud which is the standard input format for our sdk.
	PointCloudPtr left_local_laser(new PointCloud);
	pcl::fromROSMsg(*left_rslidar_msg, *left_local_laser);

	PointCloudPtr right_local_laser(new PointCloud);
	pcl::fromROSMsg(*right_rslidar_msg, *right_local_laser);

	PointCloudPtr middle_merge_laser(new PointCloud);
	pcl::fromROSMsg(*middle_rslidar_msg, *middle_merge_laser);

	clock_t timer_tmp = clock();

	std::vector<PointCloudConstPtr> lateral_clouds;
	lateral_clouds.push_back(left_local_laser);
	lateral_clouds.push_back(right_local_laser);

	PointCloudPtr merge_laser(new PointCloud);
	PointCloudPtr total_laser(new PointCloud);

	merger->mergeLidars(middle_merge_laser, lateral_clouds, merge_laser, total_laser);
	std::cout << "merge_time: " << (double) (clock() - timer_total) / CLOCKS_PER_SEC << "s " << std::endl;

      //-----------step 2: calculate the transform matrix between vehicle coord and global coord
    PointCloudPtr object_cloud_ptr(new PointCloud);
    PointCloudPtr ground_cloud_ptr(new PointCloud);
    PointCloudPtr free_cloud_ptr(new PointCloud);

    std::vector<Object<Point>::Ptr> percept_vec_filtered;

    total_laser->header=middle_merge_laser->header;

    COUT(total_laser->width<<" "<<total_laser->height);

    //-----------step 3: call the SDK functions by a ensemble proxy entrance "robosense_all"
    double time_stamp = middle_rslidar_msg->header.stamp.toSec();
    robosense_all->perception(merge_laser, time_stamp);

    //get perception result
    robosense_all->getGroundPoints(ground_cloud_ptr);
    robosense_all->getCloudFreeSpace(free_cloud_ptr);
    robosense_all->getObjectPoints(object_cloud_ptr);
	//reture perception in global as default, if need to get results in lidar coordiante, please pass in a global-to-lidar trans matrix
	robosense_all->getPeceptResults(percept_vec_filtered, Eigen::Matrix4f::Identity());

#if USE_PRINT
	robo_tool->print2file(percept_vec_filtered, results_save_path, results_frame_prefix_name, frame_id);
#endif

#if DEBUGING
    COUTG( "frame total time: " << (double) (clock() - timer_total) / CLOCKS_PER_SEC << "s. " );
    timer_total = clock();
#endif

    //------------step 4: collect the perception results and display them by ros rviz
    //if you want to transport the perception results across computers, please enable the socket module

#if USE_COMM
    communicator->sendPerceptResults(percept_vec_filtered, frame_id, Pose());
#endif

    //	pcl::PCLHeader _header = middle_merge_laser->header;
    std_msgs::Header _header = middle_rslidar_msg->header;

    rs_perception::PerceptionListMsg percep_result_msg;
    percep_result_msg = genRosMessages<Point>(percept_vec_filtered, Pose(), _header);
    pub_percept_result.publish( percep_result_msg );

#if WITH_DRAW

    pub_ori_cloud.publish(total_laser);

    //publish ground and non-ground pointcloud
    ground_cloud_ptr->header = dr->convertROSHeader2PCLHeader(_header);
    pub_ground.publish(ground_cloud_ptr);

    object_cloud_ptr->header = dr->convertROSHeader2PCLHeader(_header);
    pub_no_ground.publish(object_cloud_ptr);

    free_cloud_ptr->header = dr->convertROSHeader2PCLHeader(_header);
    pub_free_cloud.publish(free_cloud_ptr);

    //============draw the infos in rviz======================
    Eigen::VectorXf car_pos = Eigen::VectorXf::Zero(9);

    //draw and display the perception results infos in rviz
    dr->show_car_self(pub_car, _header, car_pos);
    dr->show_cluster(pub_clusters_cloud, _header, percept_vec_filtered);
    dr->show_percept(pub_percept, _header, percept_vec_filtered);
		dr->show_trajectory(pub_trajectory_cloud, _header, percept_vec_filtered);

#if DEBUGING
    COUTG( "draw time: " << (double) (clock() - timer_total) / CLOCKS_PER_SEC << "s. " );
#endif

#endif

    frame_id++;
}


int main(int argc, char **argv)
{

    //check versions
    COUTG("OpenCV version: "<<CV_VERSION);
    COUTG("ROS version: "<<ROS_VERSION_MAJOR<<"."<<ROS_VERSION_MINOR<<"."<<ROS_VERSION_PATCH);
    COUTG("PCL version: "<<PCL_VERSION_PRETTY);
    COUTG("Boost version: "<<BOOST_LIB_VERSION);
    COUTG("Eigen version: "<<EIGEN_WORLD_VERSION<<"."<<EIGEN_MAJOR_VERSION<<"."<<EIGEN_MINOR_VERSION);

    ros::init(argc, argv, "test_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~"); //parameter node

    std::string results_save_path;
    std::string results_frame_prefix_name;
    std::string key_path, pcap_path, lidar_config_path, args_path, receiv_ip,
      multi_lidar_args_path, class_model_path, roi_map_path;
    int port, receiv_port;

    //obtain the global configuration parameters from launch file
    private_nh.param("pcap", pcap_path, std::string(""));
    private_nh.param("port", port, 6699);
    private_nh.param("lidar_config_path", lidar_config_path, std::string(""));
    private_nh.param("args_path", args_path, std::string(""));
    private_nh.param("receiv_ip", receiv_ip, std::string(""));
    private_nh.param("receiv_port", receiv_port, 60000);
    private_nh.param("roi_map_path", roi_map_path, std::string(""));

    // p3
    private_nh.getParam("multi_lidar_args_path", multi_lidar_args_path);

    //--------------------------load needed file-------------------------
    key_path = ros::package::getPath("rs_perception") + "/key/RobosensePerceptionKey.key";
    class_model_path = ros::package::getPath("rs_perception") + "/model/classification/cf-5class";
    results_save_path = ros::package::getPath("rs_perception") + "/data/percept_results";
    results_frame_prefix_name = "rs_percept_result_";

    //------------------------------------------init the moduels--------------------------------------------
    // Authorize the SDK, notice that this should be placed before any SDK module works.
    module_manager.reset(new ModuleManager(key_path, pcap_path, port));

#if (COMM_MODE==0)
    communicator.reset(new RobosenseConmunicater<Point>(receiv_port, receiv_ip));
#elif (COMM_MODE ==1)
    communicator.reset(new RobosenseConmunicater2<Point>(receiv_port, receiv_ip));
#endif
    communicator->m_initSender();

    robo_tool.reset(new RobosenseUtil<Point>());

    merger.reset(new MultiLidarMerger<Point>(multi_lidar_args_path,2,Range3D(-5,5,-4,4,-3, 2)));
    robosense_all.reset(new RobosenseALL<Point>(lidar_config_path, args_path, class_model_path, roi_map_path));

    dr.reset(new DrawRviz<Point>());

    // p3
    message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub_left(node, "/left/rslidar_points", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub_middle(node, "/middle/rslidar_points", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub_right(node, "/right/rslidar_points", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,sensor_msgs::PointCloud2,sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), points_sub_left,points_sub_middle, points_sub_right);
    sync.registerCallback(boost::bind(&fullscanCallback, _1, _2, _3));

    pub_percept_result = node.advertise<rs_perception::PerceptionListMsg>("rs_percept_result", 1);

#if WITH_DRAW
    pub_ground = node.advertise<sensor_msgs::PointCloud2>("ground", 1);
    pub_no_ground = node.advertise<sensor_msgs::PointCloud2>("non_ground", 1);
    pub_clusters_cloud = node.advertise<sensor_msgs::PointCloud2>("cluster", 1);
	pub_trajectory_cloud = node.advertise<sensor_msgs::PointCloud2>("trajectory", 1);
    pub_percept = node.advertise<visualization_msgs::MarkerArray>("rs_perception_info", 1);
    pub_car = node.advertise<visualization_msgs::MarkerArray>("car_info", 1);
    pub_ori_cloud = node.advertise<sensor_msgs::PointCloud2>("rslidar_points_fusion", 1);
    pub_free_cloud = node.advertise<sensor_msgs::PointCloud2>("free_space",1);
#endif

    ros::spin();

    return 0;
}


