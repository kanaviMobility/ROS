#include "Udp.h"
#include <cassert>
#include <algorithm>
#include <iterator>

kanaviUDP::kanaviUDP()
{
    g_UDP_Multicast = true;
    g_localIP = "192.168.123.100";
    g_LiDAR_IP_01 = "192.168.123.200";
    g_UDP_PORT = 5000;

    memset(g_intput_buffer, 0, BUFFER_SIZE);

    g_checkLiDAR_IP = false;
    g_inputLidar_IP = "192.168.123.200";
}

/**
 * @brief set udp multicast
 * 
 * @param multicast_IP 	UDP multicast IP
 * @param checked 		multicast true/false
 */
void kanaviUDP::setMulticast(const std::string &multicast_IP, bool checked)
{
    g_UDP_Multicast = checked;
    if(checked)
    {
        g_multicst_IP = multicast_IP;
    }
}

/**
 * @brief Initialize UDP
 * 
 * @param IP 	UDP ip address
 * @param port 	UDP port number.
 */
void kanaviUDP::InitUDP(const std::string &IP, const int &port)
{
    g_localIP = IP;
    g_UDP_PORT = port;

    memset(&g_udpAddr, 0, sizeof(struct sockaddr_in));

    g_udpSocket = socket(PF_INET, SOCK_DGRAM, 0);

    if(g_udpSocket == -1)
    {
        perror("UDP Socket Failed");
        exit(1);
    }

    //unicast
    if(!g_UDP_Multicast)
    {
        g_udpAddr.sin_family = AF_INET;
        g_udpAddr.sin_addr.s_addr = inet_addr(IP.c_str());
        g_udpAddr.sin_port = htons(port);
    }
    else    //multicast
    {
	printf("[UDP] init Multicast...\n");
        g_udpAddr.sin_family = AF_INET;
        g_udpAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        g_udpAddr.sin_port = htons(port);

        multicastAddress.imr_multiaddr.s_addr = inet_addr(g_multicst_IP.c_str());
        multicastAddress.imr_interface.s_addr = inet_addr(IP.c_str());

        setsockopt(g_udpSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (void*)&multicastAddress, sizeof(multicastAddress));
    }

	//time out
	struct timeval optVal = {1,500};
	int optlen = sizeof(optVal);
	setsockopt(g_udpSocket, SOL_SOCKET, SO_RCVTIMEO, &optVal, optlen);

	setNcheckUDPBufSize();
}


/**
 * @brief connect UDP
 * 
 * @return true
 * @return false 
 */
bool kanaviUDP::connect()
{
// #ifndef CONNECTED_UDP
    if(bind(g_udpSocket, (struct sockaddr*)&g_udpAddr, sizeof(g_udpAddr)) == -1)
    {
        perror("[ERROR][UDP] Binding error\n");
		// printf("[ERROR][UDP] Binding error\n");
        return false;
    }
    else
    {
        printf("[UDP]CONNECTED\n");
        return true;
    }
// #else
	// if(connect(g_udpSocket, (struct sockaddr*)&g_udpAddr, sizeof(g_udpAddr)) == -1)
		// return false;
	// else
		// return true;
// #endif	//CONNECTED_UDP
}

/**
 * @brief disconnect UDP
 * 
 * @return int 
 */
int kanaviUDP::disconnect()
{
    printf("[UDP][DISCONNECT]\n");
    return close(g_udpSocket);
}

/**
 * @brief to get packet from UDP
 * 
 * @return std::vector<u_char> 
 */
std::vector<u_char> kanaviUDP::getData() {
    static std::vector<int> packetSizes;
    memset(&g_SenderAddr, 0, sizeof(struct sockaddr_in));
    socklen_t lidarAddress_length = sizeof(g_SenderAddr);

    std::vector<u_char> output;
    int packetCount = 0;  // Static counter to track the number of packets received
    
    int size = recvfrom(g_udpSocket, g_intput_buffer, BUFFER_SIZE, 0, (struct sockaddr*)&g_SenderAddr, &lidarAddress_length); //set blocking mode
    
    for(int i=0; i<size; i++)
    {
        output.push_back(g_intput_buffer[i]);
    }
        
    /*while(packetCount < 2)
    {
        int size = recvfrom(g_udpSocket, g_intput_buffer, BUFFER_SIZE, 0, (struct sockaddr*)&g_SenderAddr, &lidarAddress_length); //set blocking mode
        
        if (size == 0) continue;
        
	if(packetCount == 0)
	{
	    assert(size == 1472);
	    output.insert(output.end(), g_intput_buffer, g_intput_buffer + size);	  
	    // std::cout << "1st packet size: " << output.size() << std::endl;
	    packetCount++;
	}
	   
	else if(packetCount == 1)
	{
	    assert(size == 697);
	    output.insert(output.end(), g_intput_buffer, g_intput_buffer + size);	    
	    // std::cout << "2nd packet size: " << output.size() << std::endl;
	    packetCount++;
	}	   	
    }*/
    // std::cout << "Total packet size: " << output.size() << std::endl;
    // assert(packetCount == 2);
    g_getLidar_IP = inet_ntoa(g_SenderAddr.sin_addr);
    return output;
}       
   
/**
 * @brg_UDP_Multicastief to send data using UDPrecv_buf
 * 
 * @param send_data send data
 */
void kanaviUDP::sendData(std::vector<u_char> send_data)
{
    int res;
    if(send_data.size() <= BUFFER_SIZE)
    {
#ifndef CONNECTED_UDP
        if(sendto(g_udpSocket, send_data.data(), send_data.size(), 0, (struct sockaddr*)& g_udpAddr, sizeof(g_udpAddr)) != send_data.size())
        {
			setNcheckUDPBufSize();
            perror("UDP send Error1");
        }
#else
		write(g_udpSocket, send_data.data(), send_data.size());
#endif	//#ifndef CONNECTED_UDP
    }
    else
    {
        int n = 0;
        int remainSize = send_data.size();
        // u_char* dumBuff = (u_char*)malloc(BUFFER_SIZE);
        u_char sendbuffer[BUFFER_SIZE];
        memset(sendbuffer, 0, BUFFER_SIZE);

        for(int i=0; i<send_data.size(); i++)
        {
            sendbuffer[n] = send_data[i];
            n++;
            if(n == BUFFER_SIZE && remainSize > BUFFER_SIZE)
            {                
                if(sendto(g_udpSocket, sendbuffer, BUFFER_SIZE, 0, (struct sockaddr*)& g_udpAddr, sizeof(g_udpAddr)) == -1)
                {
                    perror("UDP send Error2");
                }
                remainSize -= n; 
                printf("[UDP][outer Server]remain data %d, %d\n", i, remainSize);
                n = 0;
            }
            else
            {
                if(remainSize == n)
                {
                    printf("all data send %d\n", remainSize);
                    if(sendto(g_udpSocket, sendbuffer, remainSize, 0, (struct sockaddr*)& g_udpAddr, sizeof(g_udpAddr)) == -1)
                    {
                        perror("UDP send Error");
                    }
                }                
            }
        }
    }

    // printf("[udp][send][end]");
}

/**
 * @brief to send data using UDP
 * 
 * @param data send data
 * @param size send data array size
 */
void kanaviUDP::sendData(u_char data[], int size)
{
    {
        if(sendto(g_udpSocket, data, size, 0, (struct sockaddr*)& g_udpAddr, sizeof(g_udpAddr)) == -1)
        {
            perror("UDP send Error");
        }
    }
}

/**
 * @brief set transmitted sensor IP
 * 
 * @param lidarIP 
 */
void kanaviUDP::setLidarIP(const std::string &IP)
{
    g_inputLidar_IP = IP;
    g_checkLiDAR_IP = true;
}

/**
 * @brief get transmitted sensor IP
 * 
 * @return std::string 
 */
std::string kanaviUDP::getLidarIP()
{
    return g_getLidar_IP;
}

/**
 * @brief run udp thread
 * 
 */
void kanaviUDP::run()
{
	g_th_loop_recv = true;
	g_th_loop_send = true;
	memset(th_buffer_send, 0, BUFFER_SIZE); // clear buffer before use

	g_th_temp_buf.clear(); // clear contents of the vector for preparing it for new data

	// runs two threads
	th_recv = std::thread(&kanaviUDP::th_recv_loop, this);
	th_send = std::thread(&kanaviUDP::sendDatagram, this);
}

/**
 * @brief end udp thread
 * 
 */
void kanaviUDP::end()
{
	g_th_loop_recv = false;
	g_th_loop_send = false;
}

std::vector<u_char> kanaviUDP::th_getBuf()
{
	std::vector<u_char> buf_;
	getTempBuf(buf_);

	return buf_;
}

/**
 * @brief public function for send data using udp thread
 * 
 * @param byf_ 
 * @param size_ 
 */
void kanaviUDP::th_setSendBuf(u_char* byf_, const int &size_)
{
	
}

/**
 * @brief thread function for udp recv thread
 * 
 */
void kanaviUDP::th_recv_loop()
{
	// printf("[udp]")

	// udp recv vector
	std::vector<u_char> buf_;
	u_char buffer[BUFFER_SIZE];
	int buf_size = 0;

	g_th_used_temp_buf_num = -1;
	g_th_recv_ready = false;
	g_checked_temp_buf_size = 0;

	while(g_th_loop_recv)
	{
#ifndef USING_ARRAY
		buf_.clear();

		if(g_th_used_temp_buf_num != -1)
		{
			printf("set erase element\n");
			g_th_temp_buf[g_th_used_temp_buf_num].first = true;
			g_th_used_temp_buf_num = -1;
		}
		
		//get data from udp socket.
		getData_fromUDP(buf_);

		// check data size
		if(buf_.size() > 0)
		{
			g_th_recv_ready = true;
			// setTempBuf(TEMP_UDP_BUF(false, buf_));
			setTempBuf( buf_ );
			// g_th_recv_ready = false;
		}
		g_th_recv_ready = false;
#else
		getData_fromUDP(buffer, buf_size);

		if(buf_size > 0)
		{
			setTempBuf(buffer, buf_size);
		}
#endif

	}
}

void kanaviUDP::sendDatagram()
{
	
}

void kanaviUDP::getData_fromUDP(std::vector<u_char> & data)
{
	u_char buf_[BUFFER_SIZE];
	memset(&buf_, 0, BUFFER_SIZE);
	memset(&g_SenderAddr, 0, sizeof(struct sockaddr_in));
	socklen_t lidarAddress_length = sizeof(g_SenderAddr);

	int size = recvfrom(g_udpSocket, buf_, BUFFER_SIZE, 0, (struct sockaddr*)&g_SenderAddr, &lidarAddress_length);
	if(size > 0)
	{
		printf("Received buffer size: %d bytes\n", size);
		printf("udp input2?:\t%d\n", buf_[size-1]);
		//test temporary buf
		data.assign(buf_, buf_+size);
	}
	g_getLidar_IP = inet_ntoa(g_SenderAddr.sin_addr);
}

void kanaviUDP::getData_fromUDP(u_char buf_[], int &size)
{
	//u_char buf_[BUFFER_SIZE];
	//memset(&buf_, 0, BUFFER_SIZE);
	memset(&g_SenderAddr, 0, sizeof(struct sockaddr_in));
	socklen_t lidarAddress_length = sizeof(g_SenderAddr);

	size = recvfrom(g_udpSocket, buf_, BUFFER_SIZE, 0, (struct sockaddr*)&g_SenderAddr, &lidarAddress_length);
	if(size > 0)
	{	
		printf("Received buffer size: %d bytes\n", size);
		printf("udp input2?:\t%d\n", buf_[size-1]);
	}
	g_getLidar_IP = inet_ntoa(g_SenderAddr.sin_addr);
}

void kanaviUDP::eraseVector(const std::vector<int> &er_num_,
								std::vector<TEMP_UDP_BUF> &vec_)
{
	if(er_num_.size() > 0 && 
		vec_.size() == 1)
	{
		vec_.clear();
	}
	else if(er_num_.size() > 0 && 
			vec_.size() > 1)
	{
		for(int i=er_num_.size()-1; i>=0; i--)
		{
			vec_.erase(vec_.begin() + er_num_[i]);
			eraseElement(er_num_[i], vec_);
		}
	}
}

void kanaviUDP::eraseElement(const int &num, std::vector<TEMP_UDP_BUF> &vec_)
{
	vec_.erase(vec_.begin() + num);
}

void kanaviUDP::setTempBuf(const TEMP_UDP_BUF &pair_)
{
	// erase elements vector
	std::vector<int> erase_num_vec;
	
	// temp buf check & erase
	if(g_th_temp_buf.size() > 0)
	{
		erase_num_vec.clear();

		// find erase elements
		for(int i=0; i<g_th_temp_buf.size(); i++)
		{
			// check temp vector is used..
			// first value is true == used
			if(g_th_temp_buf[i].first == true)
			{
				erase_num_vec.push_back(i);
			}
		}

		// erase
		// printf("erase ele\n");
		eraseVector(erase_num_vec, g_th_temp_buf);
	}

	// push back temp buf
	g_th_temp_buf.push_back(pair_);

	if(g_th_temp_buf.size() > TEMP_BUF_MAX_SIZE)
	{
		// printf("Temp vector 0th element ERASE because Overflow...\n");
		eraseElement(0, g_th_temp_buf);
	}

	if(g_th_temp_recv_buf.size() == 0)
	{
		g_th_temp_recv_buf = g_th_temp_buf[0].second;
		g_th_temp_buf[0].first = true;
	}
}

void kanaviUDP::setTempBuf(const std::vector<u_char> &buf_)
{	
	
	if(g_th_temp_group_buf.size() == 0)
	{
		g_th_temp_group_buf = buf_; // If the buffer is empty, store the group buffer into buf.
	}
	else if(g_th_temp_group_buf.size() + buf_.size() < BUFFER_SIZE)
	{
		for(int i=0; i<buf_.size(); i++)
		{
			g_th_temp_group_buf.push_back(buf_[i]); // add new data to group buffer until reaches the BUFFER_SIZE.
		}
	}
	else if(g_th_temp_group_buf.size() + buf_.size() >= BUFFER_SIZE)
	{
		g_th_temp_recv_buf = g_th_temp_group_buf; // if it exceeds the BUFFER_SIZE, store the remaining data to recv_buf.
		g_th_temp_group_buf.clear();
		g_th_temp_group_buf = buf_;
	}

	if(g_th_temp_group_buf.size() >= BUFFER_SIZE)
	{
		g_th_temp_recv_buf = g_th_temp_group_buf;
		g_th_temp_group_buf.clear();
	}
}

void kanaviUDP::setTempBuf(u_char buf_[], const int &size_)
{
	for(int i=g_checked_temp_buf_size; i<g_checked_temp_buf_size+size_; i++)
	{
		g_th_temp_buf_arr[i] = buf_[i-g_checked_temp_buf_size];
	}

	g_checked_temp_buf_size += size_;

	if((g_checked_temp_buf_size == HLK_GROUP_BUF_SIZE
		|| buf_[size_-1] == 191) && !g_th_recv_ready)
	{
		g_th_temp_recv_buf.clear();
		g_th_temp_recv_buf.assign(g_th_temp_buf_arr, g_th_temp_buf_arr+g_checked_temp_buf_size);

		//init.
		g_checked_temp_buf_size = 0;
		g_th_recv_ready = true;
	}	
}

void kanaviUDP::getTempBuf(std::vector<u_char> &buf_)
{
	// if(g_th_temp_recv_buf.size() != 0)
	if(g_th_recv_ready)
	{
		buf_ = g_th_temp_recv_buf;
		//g_th_temp_recv_buf.clear();
		g_th_recv_ready = false;
	}
	//return g_th_temp_buf;
}

void kanaviUDP::setNcheckUDPBufSize()
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
	sleep(1);
	if(send_size < BUFFER_SIZE || recv_size < BUFFER_SIZE)
	{
		if(send_size < recv_size)
		{
			setSize = recv_size;
		}
		else
			setSize = send_size;

		if(send_size < BUFFER_SIZE)
		{
			printf("send size resizing..\n");
			int n = setsockopt(g_udpSocket, SOL_SOCKET, SO_SNDBUF, &setSize, sizeof(setSize));			
		}
		if(recv_size < BUFFER_SIZE)
		{
			printf("recv size resizing..\n");
			int m = setsockopt(g_udpSocket, SOL_SOCKET, SO_RCVBUF, &setSize, sizeof(setSize));
		}
	}
}
