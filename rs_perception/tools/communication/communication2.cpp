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

#include "communication/communication2.h"

namespace robosense {
namespace perception {

template<typename PointT>
int RobosenseConmunicater2<PointT>::s_max_all_buff_size = 50000;

template<typename PointT>
RobosenseConmunicater2<PointT>::RobosenseConmunicater2() {
	m_port_ = 60000;
	m_server_ip_ = "192.168.0.255";
	m_isInitReciver = false;
	m_isInitSender = false;
}

template<typename PointT>
RobosenseConmunicater2<PointT>::RobosenseConmunicater2(const std::string &server_ip) {
	m_port_ = 60000;
	m_server_ip_ = server_ip;
	m_isInitSender = false;
	m_isInitReciver = false;
}

template<typename PointT>
RobosenseConmunicater2<PointT>::RobosenseConmunicater2(const unsigned short port) {
	m_port_ = port;
	m_server_ip_ = "192.168.0.255";
	m_isInitReciver = false;
	m_isInitSender = false;
}

template<typename PointT>
RobosenseConmunicater2<PointT>::RobosenseConmunicater2(const unsigned short port, const std::string &server_ip) {
	m_port_ = port;
	m_server_ip_ = server_ip;
	m_isInitReciver = false;
	m_isInitSender = false;
}

template<typename PointT>
RobosenseConmunicater2<PointT>::~RobosenseConmunicater2() {

#ifdef __GNUC__
	if (m_sender_sockfd_ != TINVALID_SOCKET) {
		close(m_sender_sockfd_);
		m_sender_sockfd_ = TINVALID_SOCKET;
	}
#endif

#ifdef _MSC_VER
	if (m_sender_sockfd_ !=TINVALID_SOCKET) {
		closesocket(m_sender_sockfd_);
		m_sender_sockfd_ = TINVALID_SOCKET;
	}
#endif
}


template<typename PointT>
int RobosenseConmunicater2<PointT>::sendPerceptResults(const std::vector<typename Object<PointT>::Ptr> &percept_results,
                                                      int frame_id, const Pose& pose) {
	if (percept_results.empty()) {
		COUTR("Communication: input results is null!");
		return false;
	}

	rs_MsgHeader2 header;
	header.frmNumber = frame_id;
	header.totalMsgCnt = percept_results.size();
	header.poseInfos = pose; 

	PerceptronSet perceptron_set;
	for (int i = 0; i < percept_results.size(); ++i) {
		const typename Object<PointT>::Ptr &object = percept_results[i];
		Perceptron* percetron = perceptron_set.add_perceptron();
//		object->serialize(*percetron);
		serialize<PointT>(object, *percetron);
	}

	header.msgLen = perceptron_set.ByteSize();
	COUT("Communication: input message size is: "<<header.msgLen);

	void *buff = malloc(header.msgLen);
	perceptron_set.SerializeToArray(buff, header.msgLen);
	int ret = m_sendMsg(buff, header);
	free(buff);

	return ret;
}


template<typename PointT>
int RobosenseConmunicater2<PointT>::receivePerceptResults(std::vector<typename Object<PointT>::Ptr> &recover_results, Pose& pose) {

	recover_results.clear();

	void *pMsgData = malloc(s_max_all_buff_size);

	rs_MsgHeader2 header;

	int ret = m_receMsg(pMsgData, s_max_all_buff_size, header);

	pose = header.poseInfos; //解析定位信息

	//解析
	PerceptronSet perceptron_set;
	perceptron_set.ParseFromArray(pMsgData, header.msgLen);

	for (int i = 0; i < perceptron_set.perceptron_size(); ++i) {
		const auto& perceptron = perceptron_set.perceptron(i);
		typename Object<PointT>::Ptr object(new Object<PointT>());
//		object->deserialize(perceptron);
		deserialize<PointT>(perceptron, object);
		recover_results.emplace_back(object);
	}

	return ret;
}


template<typename PointT>
int RobosenseConmunicater2<PointT>::receivePerceptResults(PerceptronSet &perceptrons, Pose& pose) {

	void *pMsgData = malloc(s_max_all_buff_size);

	rs_MsgHeader2 header;

	int ret = m_receMsg(pMsgData, s_max_all_buff_size, header);

	pose = header.poseInfos; //解析定位信息

	//解析
	perceptrons.ParseFromArray(pMsgData, header.msgLen);

	return ret;
}


template<typename PointT>
int RobosenseConmunicater2<PointT>::m_receMsg(void *pMsgData, const int msgMaxLen, rs_MsgHeader2 &header) {
	int ret;
	if (m_isInitReciver == false) {
		COUTR("Communication: Try Initial Sender Socket Again (call m_initReceiver)!" );
		return -1;
	}

#ifdef __GNUC__
	socklen_t cliaddr_len = sizeof(m_receiver_dest_addr_);
#endif

#ifdef _MSC_VER
	int cliaddr_len = sizeof(m_receiver_dest_addr_);
#endif

	char *pRecvBuffer = new char[msgMaxLen + sizeof(rs_MsgHeader2)];

	// Step1: 接收rs_MsgHeader2
	ret = recvfrom(m_receiver_sockfd_, pRecvBuffer, msgMaxLen + sizeof(rs_MsgHeader2), 0,
	               (struct sockaddr *) &m_receiver_dest_addr_, &cliaddr_len);
#ifdef __GNUC__
	if (ret < 0) {
		if (ret == EINTR) {
			usleep(10000); // 10ms
			return 0;
		} else {
			// 关闭socket
			COUTR("Communication: Receive Message Data Failed, Error: " << errno << " !" );
			close(m_sender_sockfd_);
			m_sender_sockfd_ = TINVALID_SOCKET;
			usleep(10000); // 10ms
			return -1;
		}
	}
#endif

#ifdef _MSC_VER
	if(ret == SOCKET_ERROR)
	{
			COUTR("Receive Message Data Failed Error:" << WSAGetLastError() << " !" );
			closesocket(m_sender_sockfd_);
			m_sender_sockfd_ = TINVALID_SOCKET;
			Sleep(10);      // 10ms
			return -1;
	}
#endif
	rs_MsgHeader2 *pMsgHeader = (rs_MsgHeader2 *) (pRecvBuffer);

	header.frmNumber = pMsgHeader->frmNumber;
	header.totalMsgCnt = pMsgHeader->totalMsgCnt;
	header.poseInfos = pMsgHeader->poseInfos;
	header.msgLen = pMsgHeader->msgLen;

	if (ret < header.msgLen + sizeof(rs_MsgHeader2)) {
		COUTR("Communication: Reciver Data not Completely, message size is too big: "<<header.msgLen + sizeof(rs_MsgHeader2) );
		return -1;
	}

	memcpy(pMsgData, pRecvBuffer + sizeof(rs_MsgHeader2), header.msgLen);

	delete [] pRecvBuffer;

	return ret;
}


template<typename PointT>
int RobosenseConmunicater2<PointT>::m_sendMsg(const void *pMsgData, const rs_MsgHeader2 &header) {
	int ret;
	if (m_isInitSender == false) {
		COUTR("Communication: Try Initial Sender Socket Again (Call m_initSender())!" );
		return -1;
	}

	int sendLen = header.msgLen + sizeof(rs_MsgHeader2);
	char *sendBuffer = new char[sendLen];
	memset(sendBuffer, 0, sendLen);

	// 赋值
	rs_MsgHeader2 *pMsgHeader = (rs_MsgHeader2 *) (sendBuffer);
	pMsgHeader->frmNumber = header.frmNumber;
	pMsgHeader->totalMsgCnt = header.totalMsgCnt;
	pMsgHeader->poseInfos = header.poseInfos;
	pMsgHeader->msgLen = header.msgLen;
	memcpy(sendBuffer + sizeof(rs_MsgHeader2), pMsgData, header.msgLen);

	if (header.msgLen>s_max_all_buff_size) {
		COUTR("Communication: input message is too big, Please resize the buff size!");
		return -1;
	}

	ret = sendto(m_sender_sockfd_, sendBuffer, sendLen, 0, (struct sockaddr *) &m_sender_dest_addr_,
	             sizeof(m_sender_dest_addr_));

#ifdef __GNUC__
	if (ret < 0) {
		if (ret == EINTR) {
			usleep(10000); // 10ms
		} else {
			// 关闭socket
			COUTR("Communication: Send Message Data Failed, Error: " << errno << " !" );
			close(m_sender_sockfd_);
			m_sender_sockfd_ = TINVALID_SOCKET;
			usleep(10000); // 10ms
		}
	}
#endif

#ifdef _MSC_VER
	if(ret == SOCKET_ERROR)
	{
			COUTR("Communication: Send Message Data Failed Error:" << WSAGetLastError() << " !" );
			closesocket(m_sender_sockfd_);
			m_sender_sockfd_ = TINVALID_SOCKET;
			Sleep(10);      // 10ms
	}
#endif

	delete[] sendBuffer;

	return ret;
}


template<typename PointT>
int RobosenseConmunicater2<PointT>::m_initSender() {
	m_isInitSender = false;
#ifdef _MSC_VER
	if(!initWSAStartUp())
	{
			return -1;
	}
#endif
	m_sender_sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);

	if (m_sender_sockfd_ == TINVALID_SOCKET) {
		COUTR("Communication: Create Upd Sender Socket Failed !" );
		return -1;
	}

	memset((char *) &m_sender_dest_addr_, 0, sizeof(m_sender_dest_addr_));
	m_sender_dest_addr_.sin_family = AF_INET;
	m_sender_dest_addr_.sin_port = htons(m_port_);
	inet_pton(AF_INET, m_server_ip_.c_str(), &m_sender_dest_addr_.sin_addr);

	/*设置广播模式*/
	int opt = 1;
	setsockopt(m_sender_sockfd_, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt));

	m_isInitSender = true;

	return 0;
}


template<typename PointT>
int RobosenseConmunicater2<PointT>::m_initReciver() {
	m_isInitReciver = false;
#ifdef _MSC_VER
	if(!initWSAStartUp())
	{
			return -1;
	}
#endif
	m_receiver_sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);    // 创建套接字
	if (m_receiver_sockfd_ == TINVALID_SOCKET) {
		COUTR("Communication: Create Upd Reciver Socket Failed !" );
		return -1;
	}

	memset((char *) &m_receiver_dest_addr_, 0, sizeof(m_receiver_dest_addr_));

	m_receiver_dest_addr_.sin_family = AF_INET;                // ipv4
	m_receiver_dest_addr_.sin_port = htons(m_port_);         // 端口转换
	m_receiver_dest_addr_.sin_addr.s_addr = htonl(INADDR_ANY);      // 绑定网卡所有ip地址，INADDR_ANY为通配地址，值为0

	int err_log = ::bind(m_receiver_sockfd_, (struct sockaddr *) &m_receiver_dest_addr_, sizeof(m_receiver_dest_addr_)); // 绑定
#ifdef __GNUC__
	if (err_log != 0) {
		COUTR("Communication: Bind Network Failed, Error: " << errno << " !" );
		close(m_receiver_sockfd_);
		m_receiver_sockfd_ = TINVALID_SOCKET;
		return -1;
	}
#endif

#ifdef _MSC_VER
	if (err_log == SOCKET_ERROR) {
			COUTR("Communication: Bind Network Failed, Error: " << WSAGetLastError() << " !");
			closesocket(m_receiver_sockfd_);
			m_receiver_sockfd_ = TINVALID_SOCKET;
			return -1;
	}
#endif

	int bufferSize = 2 * 1024 * 1024;
	setsockopt(m_receiver_sockfd_, SOL_SOCKET, SO_RCVBUF, (const char*)(&bufferSize), sizeof(int));

	m_isInitReciver = true;

	return 0;
}


#ifdef _MSC_VER
template<typename PointT>
bool RobosenseConmunicater2<PointT>::initWSAStartUp(unsigned int major, unsigned int minor)
{
		WSADATA WSAData;
		if(WSAStartup(MAKEWORD(major, minor), &WSAData) != 0)
		{
				COUTR("Communication: Initial Window Socket Library Failed, Error: " << GetLastError() << " !");
				return false;
		}
		return true;
}
#endif


template
class RobosenseConmunicater2<pcl::PointXYZI>;

}
}