//
//  SocketServer.hpp
//  socket project
//
//  Created by Huang Chien-Fu on 2017/12/8.
//  Copyright © 2017年 Huang Chien-Fu. All rights reserved.
//

#ifndef SocketServer_hpp
#define SocketServer_hpp

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <queue>
#include <thread>
#include <ctime>
#ifdef __APPLE__
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/errno.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>
#include<netdb.h>
#elif defined __gnu_linux__
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/errno.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>
#include<netdb.h>
#elif defined _WIN32 || defined _WIN64
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <WinSock2.h>
#include <windows.h>
#pragma comment(lib, "ws2_32.lib")
#endif

#define MyGroupAddress "225.0.0.37"

using namespace std;

class SocketServer {
protected:
	void SocketServerCreator(int port);
	thread serverThread;
	bool serverIsOpen;
	queue<string> sendDataQueue;
public:
	SocketServer();
	SocketServer(int port);
	bool sentData(string inputString);
	bool sentData(int ROI_Left_Top_X, int ROI_Left_Top_Y, int ROI_Width, int ROI_Height, double Object_Distance, string Object_Type = "NULL", time_t TimeStamp = time(0));
	bool closeServer();
	~SocketServer();
};
#endif /* SocketServer_hpp */
