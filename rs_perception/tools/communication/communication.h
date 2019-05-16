
/**
 *****************************************************************************
 * COPYRIGHT STATEMENT
 * Copyright (c) 2018, Robosense Co.,Ltd. - www.robosense.ai
 * All Rights Reserved.
 *
 * You can not use, copy or spread without official authorization.
 *****************************************************************************
 *
 * Author: Robosense Perception Group
 * Version: 2.0.0
 * Date: 2018.6
 *
 * DESCRIPTION
 *
 * Robosense communication module, for output perception result to user computer.
 *
 */

#ifndef ROBOSENSE_COMMUNICATION_H
#define ROBOSENSE_COMMUNICATION_H

#include <string>
#include <iostream>
#include <memory.h>

#ifdef __GNUC__
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
typedef   sockaddr_in    TSOCKADD_IN;
typedef   int            TSOCKET;
#endif


#ifdef _MSC_VER
#pragma comment(lib, "ws2_32.lib")
#include <WS2tcpip.h>
#include <WinSock.h>
typedef  SOCKADDR_IN    TSOCKADD_IN;
typedef  SOCKET         TSOCKET;
#endif


#include "communication/comm_util.hpp"

// 套接字无效值
#define TINVALID_SOCKET  (-1)

namespace robosense {
namespace perception {

struct alignas(16) rs_MsgHeader {
  unsigned int frmNumber;
  unsigned int totalMsgCnt;
  unsigned int msgID;
  unsigned int msgLen;
  Pose         poseInfos;
};


template<typename PointT>
class RobosenseConmunicater {

  public:

  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

  typedef std::shared_ptr<RobosenseConmunicater<PointT> > Ptr;
  typedef std::shared_ptr<const RobosenseConmunicater<PointT> > ConstPtr;


  RobosenseConmunicater();

  RobosenseConmunicater(const unsigned short port);

  RobosenseConmunicater(const std::string &server_ip);

  RobosenseConmunicater(const unsigned short port, const std::string &server_ip);

  ~RobosenseConmunicater();

  int sendPerceptResults(std::vector<typename Object<PointT>::Ptr>& percept_results, int frame_id, const Pose& pose);
  int receivePerceptResults(std::vector<typename Object<PointT>::Ptr>& recover_results, Pose& pose);
  int receivePerceptResults(std::vector<Perceptron>& perceptrons, Pose& pose);


  public:
  /// 参数：
  ///      pMsgData   : 发送的数据
  ///      msgLen     : 发送的数据长度
  ///      frmNumber  : 发送的Object数据所属的帧号
  ///      totalMsgCnt: 发送的帧包含的Object的个数
  ///      msgID      : 发送的Object的发送ID
  /// 返回值:
  ///     发送消息的长度，返回-1表示失败
  ///
  int m_sendMsg(const void *pMsgData, const rs_MsgHeader& header);

  /// 参数:
  ///      pMsgData:    接收的数据
  ///      msgMaxLen:   接收缓冲区的最大值
  ///      frmNumber:   接收的Object数据所属的帧号
  ///      totalMsgCnt: 帧中目标的个数
  ///      msgID:       接收的Object中目标编号
  ///      msgLen:      接收的Object目标的长度
  ///  返回值:
  ///     接收消息的长度,返回-1表示接收失败
  ///
  int m_receMsg(void *pMsgData, const int msgMaxLen, rs_MsgHeader& header);

  ///
  /// 参数: -
  /// 返回值： 0，表示成功；否则表示失败
  ///
  int m_initSender();

  int m_initReciver();

  private:
#ifdef _MSC_VER
  bool initWSAStartUp(unsigned int WSAMajor = 2, unsigned int WSAMinor = 2);
#endif
  private:
  bool m_isInitSender;
  bool m_isInitReciver;
  TSOCKET m_sender_sockfd_;
  TSOCKET m_receiver_sockfd_;
  TSOCKADD_IN m_sender_dest_addr_;
  TSOCKADD_IN m_receiver_dest_addr_;
  std::string m_server_ip_;
  unsigned short m_port_;

  static int s_max_loop;
  static int s_max_buff_size;
};

}
}

#endif //ROBOSENSE_COMMUNICATION_H
