//
// Created by mjj on 17-7-12.
//
#include <std_msgs/String.h>
#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(ros::NodeHandle node, QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    max_save_num_ = ui->spinBox_nFrame->value();
    pub_control_ = node.advertise<std_msgs::String>("control_signal",1);
    pub_continue_ = node.advertise<std_msgs::String>("continual_signal",1);
    pub_frame_ = node.advertise<std_msgs::String>("frame",1);
    pub_max_frame_num_ = node.advertise<std_msgs::String>("frame_num",1);
    cur_pcd_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    header_.frame_id = "rslidar";
    pub_pcd_ = node.advertise<pcl::PointCloud<pcl::PointXYZI> >("rslidar_points",1);

    connect(ui->spinBox_nFrame,SIGNAL(valueChanged(int)),this,SLOT(spinBox_nFrame_valueChanged(int)));
    connect(ui->comboBox_pcd, SIGNAL(activated(const QString&)), this, SLOT(comboBox_pcd_changed(const QString &)));
}

void MainWindow::comboBox_pcd_changed(const QString &cur_pcd){
    std::string tmp_pcd_file_name = cur_pcd.toStdString();
    int tmp_index = findIndex(pcd_filelists_,tmp_pcd_file_name);
    QString tmp_qstring;
    if (tmp_index == -1){
        ui->textEdit_msg->setText(tmp_qstring.fromStdString("error!! no" + tmp_pcd_file_name + " !"));
        return;
    }
    cur_pcd_index_ = tmp_index;
    publishPCD();
}

int MainWindow::findIndex(const std::vector<std::string>& file_lists,const std::string& cur_file_name){
    int tmp_index = -1;
    if (file_lists.empty())return tmp_index;
    for (int i = 0; i < file_lists.size(); ++i) {
        if (file_lists[i] == cur_file_name){
            tmp_index = i;
            return tmp_index;
        }
    }
    return tmp_index;
}

void MainWindow::on_pushButton_load_pcd_clicked(){
    QProPath_ = QFileDialog::getExistingDirectory(this, tr("Open Directory"), "",
                                                  QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    pcd_filelists_.clear();
    QString tmp_qstring;
    read_filelists(QProPath_.toStdString() + "/",pcd_filelists_,".pcd");
    if (pcd_filelists_.empty()){
        ui->textEdit_msg->setText(tmp_qstring.fromStdString("no pcd files founded!!"));
        return;
    }

    for (int i = 0; i < pcd_filelists_.size(); ++i) {
        ui->comboBox_pcd->addItem(tmp_qstring.fromStdString(pcd_filelists_[i]));
    }
    cur_pcd_index_ = 0;
    publishPCD();
}

void MainWindow::on_pushButton_show_pcd_clicked(){
    publishPCD();
}

void MainWindow::on_pushButton_pre_pcd_clicked(){
    if (pcd_filelists_.empty())return;
    if (cur_pcd_index_ > 0){
        cur_pcd_index_--;
    }
    publishPCD();
}

void MainWindow::on_pushButton_next_pcd_clicked(){
    if (pcd_filelists_.empty())return;
    if (cur_pcd_index_ < pcd_filelists_.size() - 1){
        cur_pcd_index_++;
    }
    publishPCD();
}

void MainWindow::publishPCD(){
    QString tmp_qstring;
    ui->comboBox_pcd->setCurrentText(tmp_qstring.fromStdString(pcd_filelists_[cur_pcd_index_]));
    std::string tmp_pcd_file_path = QProPath_.toStdString() + "/" + pcd_filelists_[cur_pcd_index_];
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (tmp_pcd_file_path.c_str(), *cur_pcd_ptr_) == -1) {
        ui->textEdit_msg->setText(tmp_qstring.fromStdString("failed load " + pcd_filelists_[cur_pcd_index_] + " !!"));
        return;
    }
    cur_pcd_ptr_->header = header_;
    pub_pcd_.publish(cur_pcd_ptr_);
}

void MainWindow::read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type){
    struct dirent *ptr;
    DIR *dir;
    dir = opendir(dir_path.c_str());
    out_filelsits.clear();
    while ((ptr = readdir(dir)) != NULL){
        std::string tmp_file = ptr->d_name;
        if (tmp_file[0] == '.')continue;
        if (type.size() <= 0){
            out_filelsits.push_back(ptr->d_name);
        }else{
            if (tmp_file.size() < type.size())continue;
            std::string tmp_cut_type = tmp_file.substr(tmp_file.size() - type.size(),type.size());
            if (tmp_cut_type == type){
                out_filelsits.push_back(ptr->d_name);
            }
        }
    }

    sort_filelists(out_filelsits,type);
}

bool computePairNum(std::pair<double,std::string> pair1,std::pair<double,std::string> pair2){
    return pair1.first < pair2.first;
}

void MainWindow::sort_filelists(std::vector<std::string>& filists,std::string type){
    if (filists.empty())return;
    std::vector<std::pair<double,std::string> > filelists_pair;
    for (int i = 0; i < filists.size(); ++i) {
        std::string tmp_string = filists[i];
        std::string tmp_num_string = tmp_string.substr(0,tmp_string.size() - type.size());
        double tmp_num = atof(tmp_num_string.c_str());
        std::pair<double,std::string> tmp_pair;
        tmp_pair.first = tmp_num;
        tmp_pair.second = tmp_string;
        filelists_pair.push_back(tmp_pair);
    }
    std::sort(filelists_pair.begin(),filelists_pair.end(),computePairNum);
    filists.clear();
    for (int i = 0; i < filelists_pair.size(); ++i) {
        filists.push_back(filelists_pair[i].second);
    }
}

void MainWindow::spinBox_nFrame_valueChanged(int){
    max_save_num_ = ui->spinBox_nFrame->value();
    publishString(pub_max_frame_num_,num2str(max_save_num_));
}

void MainWindow::on_pushButton_save_frame_clicked(){
    publishString(pub_frame_,"save_frame");
}

void MainWindow::on_pushButton_next_frame_clicked(){
    publishString(pub_frame_,"next_frame");
}

void MainWindow::on_pushButton_pre_frame_clicked(){
    publishString(pub_frame_,"pre_frame");
}

void MainWindow::on_pushButton_continue_clicked(){
    publishString(pub_continue_,"continue");
}

void MainWindow::on_pushButton_next_clicked(){
    publishString(pub_control_,"next");
}

void MainWindow::publishString(const ros::Publisher& pub,const std::string& str){
    std_msgs::String tmp_msg;
    tmp_msg.data = str.c_str();
    pub.publish(tmp_msg);
}

MainWindow::~MainWindow()
{
    delete ui;
}