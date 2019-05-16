/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver node for the Robosense 3D LIDARs.
 */
#include <ros/ros.h>
#include "rsdriver.h"
#include "std_msgs/String.h"

using namespace rslidar_driver;
volatile sig_atomic_t flag = 1;

static void my_handler(int sig)
{
  flag = 0;
}

int control_mode = 1;
int send_signal=1;
void controlCallback(const std_msgs::String::ConstPtr& msg)
{
    control_mode = 0;
    send_signal =1;
    ROS_INFO("pause pressed");
}
void continueCallback(const std_msgs::String::ConstPtr& msg)
{
    control_mode = 1;
    ROS_INFO("continue pressed");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rsdriver");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

ros::Subscriber control = node.subscribe("control_signal",1,controlCallback);
    ros::Subscriber control1 = node.subscribe("continual_signal",1,continueCallback);

  signal(SIGINT, my_handler);

  // start the driver
  rslidar_driver::rslidarDriver dvr(node, private_nh);
//  loop until shut down or end of file
//  while (ros::ok() && dvr.poll())
//  {
//    ros::spinOnce();
//  }
  
    while(ros::ok())
    {
        if(control_mode == 0)
        {
            if(send_signal == 1)
            {
                dvr.poll();
                send_signal = 0;
            }
        }
        else
        {
            dvr.poll();
        }
        ros::spinOnce();
    }

  return 0;
}
