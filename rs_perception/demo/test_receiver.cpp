
/**
 * Example of Robosense SDK usage for communication.
 * There are 2 ways of communication,
 * method 1: Obstacles in a frame sent one by one, need smaller IO band width;
 * method 2: All of the obstacles in a frame collected together and sent by once, need bigger IO bang width.
 * Default is method 1, you can rechoose it by reset the Macro COMM_MODE to 1.
 */

#include <ros/package.h>
#include <std_msgs/Float32.h>
#include "communication/communication.h"
#include "communication/communication2.h"
#include "display/robo_draw_rviz.h"
#include "util/robosense_util.h"

#define COMM_MODE 0
#define WITH_DEBUG 0 //when used online into real environment, please switch off

using namespace robosense::perception;

int main(int argc, char **argv)
{
  //check versions
  COUTG("OpenCV version: "<<CV_VERSION);
  COUTG("ROS version: "<<ROS_VERSION_MAJOR<<"."<<ROS_VERSION_MINOR<<"."<<ROS_VERSION_PATCH);
  COUTG("PCL version: "<<PCL_VERSION_PRETTY);
  COUTG("Boost version: "<<BOOST_LIB_VERSION);
  COUTG("Eigen version: "<<EIGEN_WORLD_VERSION<<"."<<EIGEN_MAJOR_VERSION<<"."<<EIGEN_MINOR_VERSION);

  ros::init(argc, argv, "receive_node");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~"); //parameter node

  DrawRviz<Point>::Ptr dr(new DrawRviz<Point>());
  ros::Publisher pub_percept = node.advertise<visualization_msgs::MarkerArray>("rs_perception_info", 10);
	ros::Publisher pub_trajectory_cloud = node.advertise<sensor_msgs::PointCloud2>("trajectory", 1);

#if (COMM_MODE == 0)
  RobosenseConmunicater<Point>::Ptr communicator(new RobosenseConmunicater<Point>(60000, "192.168.1.255"));
#elif (COMM_MODE == 1)
  RobosenseConmunicater2<Point>::Ptr communicator(new RobosenseConmunicater2<Point>(60000, "192.168.1.255"));
#endif

  communicator->m_initReciver();

  std_msgs::Header _header;
  _header.frame_id = "rslidar";

  int ret = -10;
  int failor_cnt = 0;

  while (true) {
    Pose pose;
#if WITH_DEBUG
    std::vector<Object<Point>::Ptr> percept_vec_filtered;
    ret = communicator->receivePerceptResults(percept_vec_filtered, pose);
    //draw and display the perception results infos in rviz
    dr->show_percept(pub_percept, _header, percept_vec_filtered);
    dr->show_trajectory(pub_trajectory_cloud, _header, percept_vec_filtered);
    if (ret == 0) {
      failor_cnt = 0;
      COUTG("Communication: receive sucded!");
    } else {
      failor_cnt++;
      COUTR("Communication: receive error!");
    }
#else
    std::vector<Perceptron> percept_vec_filtered;
    ret = communicator->receivePerceptResults(percept_vec_filtered, pose);

    if (ret != 0) {
      failor_cnt++;
      COUTR("Communication: receive error!");
      if (failor_cnt>1000) {
        COUTR("Communication: Connection has been interrupted!");
      }
    } else {
      failor_cnt = 0;
    }
#endif
  }

  return 0;
}


