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


#include "communication/communication.h"

namespace robosense {
namespace perception {


template<typename PointT>
int RobosenseConmunicater<PointT>::s_max_loop = 1000;

template<typename PointT>
int RobosenseConmunicater<PointT>::s_max_buff_size = 2048;

template<typename PointT>
RobosenseConmunicater<PointT>::RobosenseConmunicater() {
	m_port_ = 60000;
	m_server_ip_ = "192.168.0.255";
	m_isInitReciver = false;
	m_isInitSender = false;
}

template<typename PointT>
RobosenseConmunicater<PointT>::RobosenseConmunicater(const std::string &server_ip) {
	m_port_ = 60000;
	m_server_ip_ = server_ip;
	m_isInitSender = false;
	m_isInitReciver = false;
}

template<typename PointT>
RobosenseConmunicater<PointT>::RobosenseConmunicater(const unsigned short port) {
	m_port_ = port;
	m_server_ip_ = "192.168.0.255";
	m_isInitReciver = false;
	m_isInitSender = false;
}

template<typename PointT>
RobosenseConmunicater<PointT>::RobosenseConmunicater(const unsigned short port, const std::string &server_ip) {
	m_port_ = port;
	m_server_ip_ = server_ip;
	m_isInitReciver = false;
	m_isInitSender = false;
}

template<typename PointT>
RobosenseConmunicater<PointT>::~RobosenseConmunicater() {

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
int RobosenseConmunicater<PointT>::sendPerceptResults(std::vector<typename Object<PointT>::Ptr> &percept_results,
                                                       int frame_id, const Pose& pose) {
	if (percept_results.empty()) {
		COUTR("Communication: input results is null!");
		return false;
	}

	rs_MsgHeader header;
	header.frmNumber = frame_id;
	header.totalMsgCnt = percept_results.size();
	header.poseInfos = pose;

	COUT("Total perceptions sent in frame "<<frame_id<<" is: "<<header.totalMsgCnt);

	for (int i = 0; i < percept_results.size(); ++i) {
		const typename Object<PointT>::Ptr &object = percept_results[i];
		header.msgID = i;
		Perceptron percetron;
//		object->serialize(percetron);
		serialize<PointT>(object, percetron);
		header.msgLen = percetron.ByteSize();
		void *buff = malloc(header.msgLen);
		percetron.SerializeToArray(buff, header.msgLen);
		int ret = m_sendMsg(buff, header);
		free(buff);
		if (ret < header.msgLen + sizeof(rs_MsgHeader)) {
			COUTY("warnning: Communication: mismatched msg is sent! ");
			COUTY("msg length is: "<<header.msgLen+ sizeof(rs_MsgHeader)<<", sent length is: "<<ret<<". ");
			return 1;
		}

		if (ret < 0) {
			COUTR("Communication: error occurs when sending!");
			return -1;
		}
	}

	return 0;
}

template<typename PointT>
int RobosenseConmunicater<PointT>::receivePerceptResults(std::vector<typename Object<PointT>::Ptr> &recover_results, Pose& pose) {

	void *pMsgData = malloc(s_max_buff_size);

	rs_MsgHeader header;

	bool receiv_pose_finished = false;	

	int rev_cnt = 0;
	bool receive_accomplished = false;
	while (!receive_accomplished && rev_cnt < s_max_loop) {
		m_receMsg(pMsgData, s_max_buff_size, header);
		if (!receiv_pose_finished && header.poseInfos.pos.norm()>0.f) { //首先解析定位信息
			pose = header.poseInfos;
			receiv_pose_finished = true;
		}		
		//解析
		Perceptron perceptron;
		perceptron.ParseFromArray(pMsgData, header.msgLen);
		typename Object<PointT>::Ptr object(new Object<PointT>());
//		object->deserialize(perceptron);
		deserialize<PointT>(perceptron, object);
		recover_results.emplace_back(object);
		rev_cnt++;

		if (header.msgID == header.totalMsgCnt-1) {
			receive_accomplished = true;
		}
	}

	COUT("Total perceptions received in frame "<<header.frmNumber<<" is: "<<header.totalMsgCnt);

	if (rev_cnt<header.totalMsgCnt) {
		COUTY("Communication: Warning!, missed "<<header.totalMsgCnt-rev_cnt<<" packages!");
		return 1;
	} else if (rev_cnt == s_max_loop) {
		COUTR("Communication: Error!, nothing received! ");
		return -1;
	}
	else if (rev_cnt>header.totalMsgCnt) {
		COUTY("Communication: Warning!, mismatching occurred when receiving! ");
		return 2;
	}

	return 0;
}


template<typename PointT>
int RobosenseConmunicater<PointT>::receivePerceptResults(std::vector<Perceptron> &perceptrons, Pose& pose) {

	void *pMsgData = malloc(s_max_buff_size);

	rs_MsgHeader header;

	bool receiv_pose_finished = false;

	int rev_cnt = 0;
	bool receive_accomplished = false;
	while (!receive_accomplished && rev_cnt < s_max_loop) {
		m_receMsg(pMsgData, s_max_buff_size, header);
		if (!receiv_pose_finished && header.poseInfos.pos.norm()>0.f) { //首先解析定位信息
			pose = header.poseInfos;
			receiv_pose_finished = true;
		}	
		//解析
		Perceptron perceptron;
		perceptron.ParseFromArray(pMsgData, header.msgLen);
		perceptrons.emplace_back(perceptron);
		rev_cnt++;

		if (header.msgID == header.totalMsgCnt-1) {
			receive_accomplished = true;
		}
	}

	COUT("Total perceptions received in frame "<<header.frmNumber<<" is: "<<header.totalMsgCnt);

	if (rev_cnt<header.totalMsgCnt) {
		COUTR("Communication: Warning!, missed "<<header.totalMsgCnt-rev_cnt<<" packages!");
		return 1;
	} else if (rev_cnt == s_max_loop) {
		COUTR("Communication: Error!, nothing received! ");
		return -1;
	}
	else if (rev_cnt>header.totalMsgCnt) {
		COUTR("Communication: Warning!, mismatching occurred when receiving! ");
		return 2;
	}

	return 0;
}


template<typename PointT>
int RobosenseConmunicater<PointT>::m_receMsg(void *pMsgData, const int msgMaxLen, rs_MsgHeader &header) {
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

	char *pRecvBuffer = new char[msgMaxLen + sizeof(rs_MsgHeader)];

	// Step1: 接收rs_MsgHeader
	ret = recvfrom(m_receiver_sockfd_, pRecvBuffer, msgMaxLen + sizeof(rs_MsgHeader), 0,
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
	rs_MsgHeader *pMsgHeader = (rs_MsgHeader *) (pRecvBuffer);

	header.msgLen = pMsgHeader->msgLen;
	header.frmNumber = pMsgHeader->frmNumber;
	header.totalMsgCnt = pMsgHeader->totalMsgCnt;
	header.poseInfos = pMsgHeader->poseInfos;
	header.msgID = pMsgHeader->msgID;

	if (ret < header.msgLen + sizeof(rs_MsgHeader)) {
		COUTR("Reciver Data not Completely !" );
		return -1;
	}

	memcpy(pMsgData, pRecvBuffer + sizeof(rs_MsgHeader), header.msgLen);

	delete [] pRecvBuffer;

	return ret;
}


template<typename PointT>
int RobosenseConmunicater<PointT>::m_sendMsg(const void *pMsgData, const rs_MsgHeader &header) {
	int ret;
	if (m_isInitSender == false) {
		COUTR("Communication: Try Initial Sender Socket Again (Call m_initSender())!" );
		return -1;
	}

	int sendLen = header.msgLen + sizeof(rs_MsgHeader);
	char *sendBuffer = new char[sendLen];
	memset(sendBuffer, 0, sendLen);

	// 赋值
	rs_MsgHeader *pMsgHeader = (rs_MsgHeader *) (sendBuffer);
	pMsgHeader->frmNumber = header.frmNumber;
	pMsgHeader->totalMsgCnt = header.totalMsgCnt;
	pMsgHeader->poseInfos = header.poseInfos;
	pMsgHeader->msgID = header.msgID;
	pMsgHeader->msgLen = header.msgLen;
	memcpy(sendBuffer + sizeof(rs_MsgHeader), pMsgData, header.msgLen);

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
int RobosenseConmunicater<PointT>::m_initSender() {
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
int RobosenseConmunicater<PointT>::m_initReciver() {
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
bool RobosenseConmunicater<PointT>::initWSAStartUp(unsigned int major, unsigned int minor)
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
class RobosenseConmunicater<pcl::PointXYZI>;

}
}
