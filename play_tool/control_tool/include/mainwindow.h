//
// Created by mjj on 17-7-12.
//

#ifndef ROBOSENSE_PLAY_MAINWINDOW_H
#define ROBOSENSE_PLAY_MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
Q_OBJECT

public:
    explicit MainWindow(ros::NodeHandle node, QWidget *parent = 0);
    ~MainWindow();
public slots:
    //control_node
    void on_pushButton_next_clicked();
    void on_pushButton_continue_clicked();
    void on_pushButton_pre_frame_clicked();
    void on_pushButton_next_frame_clicked();
    void on_pushButton_save_frame_clicked();
    void spinBox_nFrame_valueChanged(int);
    //replay_node
    void on_pushButton_load_pcd_clicked();
    void on_pushButton_next_pcd_clicked();
    void on_pushButton_pre_pcd_clicked();
    void on_pushButton_show_pcd_clicked();
    void comboBox_pcd_changed(const QString &cur_pcd);
private:
    void publishString(const ros::Publisher& pub,const std::string& str);
    void read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type);
    void sort_filelists(std::vector<std::string>& filists,std::string type);
    void publishPCD();
    int findIndex(const std::vector<std::string>& file_lists,const std::string& cur_file_name);

    template<typename T>
    std::string num2str(T num) {
        std::stringstream ss;
        std::string st;
        ss << num;
        ss >> st;
        return st;
    }

    int max_save_num_;
    ros::Publisher pub_control_;
    ros::Publisher pub_continue_;
    ros::Publisher pub_frame_;
    ros::Publisher pub_max_frame_num_;
    ros::Publisher pub_pcd_;

    QString QProPath_;
    std::vector<std::string> pcd_filelists_;
    int cur_pcd_index_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_pcd_ptr_;
    pcl::PCLHeader header_;

private:
    Ui::MainWindow *ui;
};

#endif //ROBOSENSE_PLAY_MAINWINDOW_H
