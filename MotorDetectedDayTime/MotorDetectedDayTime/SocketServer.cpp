//
//  SocketServer.cpp
//  socket project
//
//  Created by Huang Chien-Fu on 2017/12/8.
//  Copyright © 2017年 Huang Chien-Fu. All rights reserved.
//

#include "SocketServer.h"


SocketServer::SocketServer()
{
	serverIsOpen = true;
	serverThread = thread(&::SocketServer::SocketServerCreator, this, 15333);
}

SocketServer::SocketServer(int port)
{
	serverIsOpen = true;
	serverThread = thread(&::SocketServer::SocketServerCreator, this, port);
}

SocketServer::~SocketServer()
{
	closeServer();
}

bool SocketServer::closeServer()
{
	if (serverIsOpen)
	{
		serverIsOpen = false;
		serverThread.join();
		return true;
	}
	else
	{
		return false;
	}
}

bool SocketServer::sentData(int ROI_Left_Top_X, int ROI_Left_Top_Y, int ROI_Width, int ROI_Height, double Object_Distance, string Object_Type, time_t TimeStamp)
{
	stringstream ss;
	ss << "ROI_X:" << ROI_Left_Top_X << ",";
	ss << "ROI_Y:" << ROI_Left_Top_Y << ",";
	ss << "ROI_W:" << ROI_Width << ",";
	ss << "ROI_H:" << ROI_Height << ",";
	ss << "OBJ_D:" << Object_Distance << ",";
	ss << "OBJ_T:" << Object_Type << ",";
	ss << "TIME:" << TimeStamp;
	return sentData(ss.str());
}

bool SocketServer::sentData(string inputString)
{
	if (sendDataQueue.size() >= 10)
	{
		sendDataQueue.pop();
	}
	sendDataQueue.push(inputString);
	return true;
}

void SocketServer::SocketServerCreator(int port)
{
	int mySocketServer;
	struct sockaddr_in address;

	//啟動socket
#if defined _WIN32 || defined _WIN64
	WSADATA wsaData;
	int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (result != NO_ERROR)
	{
		printf("WSAStartup failed with error: %d\n", result);
		exit(-1);
	}
#endif

	mySocketServer = socket(AF_INET, SOCK_DGRAM, 0);
	if (mySocketServer <= 0)
	{
		cout << "API Server 啟動失敗！" << endl;
	}

	//socket參數設定
	memset(&address, 0, sizeof(struct sockaddr_in));
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = inet_addr(MyGroupAddress);
	address.sin_port = htons(port);

	cout << "Sent to " << MyGroupAddress << ":" << port << endl;

	//等待並接收連線
	int addrlen = sizeof(address);
	cout << "API Server was start ..." << endl;

	int send_nbytes = 1024;
	string send_buf = "";

	while (serverIsOpen) {
		while (sendDataQueue.size() != 0 && serverIsOpen) {
			send_buf = sendDataQueue.front();
			sendDataQueue.pop();
			int sendBytes;
			if ((sendBytes = sendto(mySocketServer, send_buf.c_str(), 1024, 0, (struct sockaddr *)&address, addrlen)) == -1) {
				perror("Sent Error!");
			}
#ifdef __APPLE__
			usleep(3000);
#elif defined __gnu_linux__
			usleep(3000);
#elif defined _WIN32 || defined _WIN64
			Sleep(5);
#endif
		}
	}

}

