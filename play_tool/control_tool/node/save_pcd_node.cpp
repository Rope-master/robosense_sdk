//
// Created by mjj on 17-7-12.
//

#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <queue>

std::queue<pcl::PointCloud<pcl::PointXYZI> > g_frames;
int g_max_frames = 10;
ros::Publisher pub_frame;
int g_index = 0;
std::string g_save_path;

template<typename T>
std::string num2str(T num)
{
    std::stringstream ss;
    std::string st;
    ss <<std::fixed<< num;
    ss >> st;
    return st;
}

void fullscanCallback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr msg){

    std::cout<<"save button pressed"<<std::endl;

    if (msg->header.seq == 0)return;
    if (msg->empty())return;

    while(g_frames.size() > g_max_frames){
        g_frames.pop();
    }
    g_frames.push(*msg);

}

void frameCallback(const std_msgs::String msg){
    if (g_frames.empty())
        return;
    std::string tmp_str = msg.data;
    if (tmp_str == "pre_frame"){
        std::cout<<"press pre_frame button"<<std::endl;
        g_index++;
        if (g_index >= g_frames.size() - 1){
            g_index = g_frames.size() - 1;
        }
    }else if(tmp_str == "next_frame"){
        std::cout<<"press next_frame button"<<std::endl;
        g_index--;
        if (g_index <= 0){
            g_index = 0;
        }
    }
    int pop_times = g_frames.size() - g_index - 1;
    std::queue<pcl::PointCloud<pcl::PointXYZI> > tmp_queue = g_frames;
    for (int i = 0; i < pop_times; ++i) {
        tmp_queue.pop();
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    *tmp_cloud_ptr = tmp_queue.front();
    tmp_cloud_ptr->header.seq = 0;
    pub_frame.publish(tmp_cloud_ptr);

    if (tmp_str == "save_frame"){
        std::cout<<"press save_frame button"<<std::endl;
        double timestamp = tmp_cloud_ptr->header.stamp;
        std::string tmp_file_path = g_save_path + num2str(timestamp) + ".pcd";
        pcl::io::savePCDFileBinary(tmp_file_path.c_str(), *tmp_cloud_ptr);
    }
}

void frameNumCallback(const std_msgs::String msg){
    std::string tmp_str = msg.data;
    g_max_frames = atoi(tmp_str.c_str());
}

void continueCallback(const std_msgs::String msg){
    g_index = 0;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "listen_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    ros::Subscriber sub_fullscan = node.subscribe("/rslidar_points",10,fullscanCallback);
    ros::Subscriber sub_framenode = node.subscribe("/frame",10,frameCallback);
    ros::Subscriber sub_framenumnode = node.subscribe("/frame_num",10,frameNumCallback);
    ros::Subscriber sub_clear = node.subscribe("/continual_signal",10,continueCallback);
    pub_frame = node.advertise<pcl::PointCloud<pcl::PointXYZI> >("/rslidar_points",1);

    std::string tmp_save_path = ros::package::getPath("control_tool") + "/save_data/";

    g_save_path = tmp_save_path;
    ros::spin();

    return 0;

}
