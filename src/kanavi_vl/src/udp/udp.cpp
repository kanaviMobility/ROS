#include "udp.h"

/**
 * @brief Construct a new kanavi udp::kanavi udp object
 * 
 * @param local_ip_ local IP address
 * @param port_ 	port number
 * @param multicast_ip_ multicast IP for UDP
 */
kanavi_udp::kanavi_udp(const std::string &local_ip_, const int &port_, const std::string &multicast_ip_)
{
	// init Unicast
	if(init(local_ip_, port_, multicast_ip_, true) == -1)
	{
		exit(1);
	}
}

/**
 * @brief Construct a new kanavi udp::kanavi udp object
 * 
 * @param local_ip_ local IP address
 * @param port_ 	port number
 */
kanavi_udp::kanavi_udp(const std::string &local_ip_, const int &port_)
{
	// init Unicast
	if(init(local_ip_, port_) == -1)
	{
		exit(1);
	}
}

/**
 * @brief Destroy the kanavi udp::kanavi udp object
 * 
 */
kanavi_udp::~kanavi_udp()
{
}

/**
 * @brief initialize UDP
 * 
 * @param ip_ 			local IP address set
 * @param port_ 		Port Number
 * @param multicast_ip_ Multicast IP address set
 * @param multi_checked_ checked Multicast Used
 * @return int 
 */
int kanavi_udp::init(const std::string &ip_, const int &port_, std::string multicast_ip_, bool multi_checked_)
{
	memset(g_udp_buf, 0, MAX_BUF_SIZE);

	g_udpSocket = socket(PF_INET, SOCK_DGRAM, 0);
	if(g_udpSocket == -1)
	{
		perror("UDP Socket Failed");
		return -1;
	}

	memset(&g_udpAddr, 0, sizeof(sockaddr_in));
	if(!multi_checked_) // unicast
	{
		printf("[UDP] Set Unicast Mode : %s %d\n", ip_.c_str(), port_);
		g_udpAddr.sin_family = AF_INET;
		g_udpAddr.sin_addr.s_addr = inet_addr(ip_.c_str());
		g_udpAddr.sin_port = htons(port_);
	}
	else				// multicast
	{
		printf("[UDP] Set Multicast Mode : %s %d %s\n", ip_.c_str(), port_, multicast_ip_.c_str());
		g_udpAddr.sin_family = AF_INET;
		g_udpAddr.sin_addr.s_addr = htonl(INADDR_ANY);
		g_udpAddr.sin_port = htons(port_);
		multi_Addr.imr_multiaddr.s_addr = inet_addr(multicast_ip_.c_str());
		multi_Addr.imr_interface.s_addr = inet_addr(ip_.c_str());
		setsockopt(g_udpSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (void*)&multi_Addr, sizeof(multi_Addr));
	}

	//time out
	struct timeval optVal = {1,500};
	int optlen = sizeof(optVal);
	setsockopt(g_udpSocket, SOL_SOCKET, SO_RCVTIMEO, &optVal, optlen);

	check_udp_buf_size();

	return 0;
}

/**
 * @brief check UDP send & recv buf Size
 * 
 */
void kanavi_udp::check_udp_buf_size()
{
	int send_size;
	socklen_t opt_size = sizeof(send_size);
	getsockopt(g_udpSocket, SOL_SOCKET, SO_SNDBUF, &send_size, &opt_size);
	printf("*[UDP] set send buffer size is %d\n", send_size);

	int recv_size;
	socklen_t opt_size2 = sizeof(recv_size);
	getsockopt(g_udpSocket, SOL_SOCKET, SO_RCVBUF, &recv_size, &opt_size2);
	printf("*[UDP] set recv buffer size is %d\n", recv_size);

	//reset udp buffer size
	int setSize = 0;

	if(send_size < MAX_BUF_SIZE || recv_size < MAX_BUF_SIZE)
	{
		if(send_size < recv_size)
		{
			setSize = recv_size;
		}
		else
			setSize = send_size;

		if(send_size < MAX_BUF_SIZE)
		{
			printf("send size resizing..\n");
			setsockopt(g_udpSocket, SOL_SOCKET, SO_SNDBUF, &setSize, sizeof(setSize));			
		}
		if(recv_size < MAX_BUF_SIZE)
		{
			printf("recv size resizing..\n");
			setsockopt(g_udpSocket, SOL_SOCKET, SO_RCVBUF, &setSize, sizeof(setSize));
		}
	}
}

/**
 * @brief receive data from UDP
 * 
 * @return std::vector<u_char> retrun Raw data from UDP
 */
std::vector<u_char> kanavi_udp::getData()
{
	memset(&g_senderAddr, 0, sizeof(struct sockaddr_in));
	socklen_t addr_len = sizeof(g_senderAddr);

	std::vector<u_char> output;

	int size = recvfrom(g_udpSocket, g_udp_buf, MAX_BUF_SIZE, 0, (struct sockaddr*)&g_senderAddr, &addr_len);

	printf("CHECK SIZE : %d\n", size);
	char ip_str[INET_ADDRSTRLEN];
    // IP 주소 변환 (network byte order → string)
    inet_ntop(AF_INET, &(g_senderAddr.sin_addr), ip_str, sizeof(ip_str));
	printf("IP Address: %s\n", ip_str);
    printf("Port: %d\n", ntohs(g_senderAddr.sin_port));

	if(size > 0)
	{
		output.resize(size);

		std::copy(g_udp_buf, g_udp_buf + size, output.begin());
	}

	return output;
}

/**
 * @brief send data using UDP
 * 
 * @param data_ data for send
 */
void kanavi_udp::sendData(std::vector<u_char> data_)
{
	printf("NOT ACTIVATED...\n");
}

/**
 * @brief UDP connect
 * 
 * @return int return bind code
 */
int kanavi_udp::connect()
{
	return bind(g_udpSocket, (struct sockaddr*)&g_udpAddr, sizeof(g_udpAddr));
}

/**
 * @brief UDP disconnect
 * 
 * @return int return close code
 */
int kanavi_udp::disconnect()
{
	return close(g_udpSocket);
}
