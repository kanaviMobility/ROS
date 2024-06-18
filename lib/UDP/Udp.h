/**
 * @file kanavi_udp.h
 * @author twchong (twchong@kanavi-mobility.com)
 * @brief func define UDP
 * @version 0.1
 * @date 2022-07-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef UDP_H
#define UDP_H

#include "../include/header.h"
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <thread>		//for thread
#include <mutex>

typedef std::pair< bool, std::vector<u_char> > TEMP_UDP_BUF;

#define TEMP_BUF_MAX_SIZE	2000

#define HLK_GROUP_BUF_SIZE 60557
#define HLK_PACKE_SIZE 8651

#define USING_ARRAY

// #define CONNECTED_UDP

class kanaviUDP
{
public:
    kanaviUDP();

    //from LidAR sensor
    void setMulticast(const std::string &multicast_IP, bool checked=false);
    void InitUDP(const std::string &IP, const int &port);

    bool connect();
    int disconnect();
    std::vector<u_char> getData();

    void sendData(std::vector<u_char> send_data);
    void sendData(u_char data[], int size);

    void setLidarIP(const std::string &IP);
    std::string getLidarIP();

	void run();
	void end();

	std::vector<u_char> th_getBuf();
	void th_setSendBuf(u_char* byf_, const int &size_);

private:
	//func.
	bool isCompletePacket(const std::vector<u_char>& data);
	
	void th_recv_loop();
	void sendDatagram();

	void getData_fromUDP(std::vector<u_char> & data);
	void getData_fromUDP(u_char buf_[], int &size);

	void eraseVector(const std::vector<int> &er_num_,
					std::vector<TEMP_UDP_BUF> &vec_);
	void eraseElement(const int &num, std::vector<TEMP_UDP_BUF> &vec_);

	void setTempBuf(const TEMP_UDP_BUF &pair_);
	void setTempBuf(const std::vector<u_char> &buf_);
	void setTempBuf(u_char buf_[], const int &size_);

	// std::vector<TEMP_UDP_BUF> getTempBuf();
	void getTempBuf(std::vector<u_char> &buf_);

	//reset buf size
	void setNcheckUDPBufSize();

    //lidar -> pc
    struct sockaddr_in g_udpAddr;
    struct sockaddr_in g_SenderAddr;
    int g_udpSocket;

    struct ip_mreq multicastAddress;

    std::string g_localIP;
    std::string g_multicst_IP;
    std::string g_LiDAR_IP_01;
    int g_UDP_PORT;
    bool g_UDP_Multicast;

    u_char g_intput_buffer[BUFFER_SIZE];

    //lidar's IP
    std::string g_inputLidar_IP;
    bool g_checkLiDAR_IP;

    std::string g_getLidar_IP;

	//thread 
	bool g_th_loop_recv;
	bool g_th_loop_send;
	std::thread th_recv;
	std::thread th_send;

	// int th_buf_recv_size;
	int th_buf_send_size;
	// u_char th_buffer_recv[BUFFER_SIZE];
	u_char th_buffer_send[BUFFER_SIZE];

	bool g_th_recv_ready;

	std::vector<TEMP_UDP_BUF> g_th_temp_buf;
	std::vector<u_char> g_th_temp_group_buf;	// 전송 데이터를 하나의 버퍼에 max크기까지 적재
	std::vector<u_char> g_th_temp_recv_buf;
	// std::vector<TEMP_UDP_BUF> g_th_temp_recv_buf;
	int g_th_used_temp_buf_num;

#ifdef USING_ARRAY
	u_char g_th_temp_buf_arr[HLK_GROUP_BUF_SIZE];
	int g_checked_temp_buf_size;
#endif
};

#endif // UDP_H
