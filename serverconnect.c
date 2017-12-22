/*
 * serverconnect.cpp
 *
 *  Created on: Jun 14, 2017
 *      Author: yenb
 */

#include<errno.h>
#include<stdio.h>
#include<string.h>
#include<sys/types.h>
#include<netinet/in.h>
#include<sys/socket.h>
#include<netdb.h>
#include<linux/tcp.h>

int serverconnect(char *hostname,int serverport)
{
	struct sockaddr_in server_addr; //服务器端地址信息结构体
	int sockfd;//定义socket连接返回值，发送和接收字符串
	struct hostent *host;//定义IP地址解析结构体

	host = gethostbyname(hostname);
	if( ( sockfd = socket(AF_INET,SOCK_STREAM,0) ) == -1 )//生成连接socket
	{
		printf("genarating socket failed!\n");
	}
	//填充服务器的地址信息
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(serverport);
	server_addr.sin_addr = *((struct in_addr *)host->h_addr);
	bzero(&(server_addr.sin_zero),8);

	//client_addr.sin_port=htons(clientport);

	//bind(sockfd,(struct sockaddr*)&client_addr,sizeof(struct sockaddr));//绑定客户端的端口号，只能绑定一次。

	if( ( connect(sockfd,(struct sockaddr *)&server_addr,sizeof(struct sockaddr)) ) == -1 ) //连接服务器
	{
		sockfd=-1;
	}
	return sockfd;
}

int IsConnect(int sockfd)
{
	struct tcp_info info;
	int len=sizeof(info);
	getsockopt(sockfd,IPPROTO_TCP,TCP_INFO,&info,(socklen_t *)&len);
	if((info.tcpi_state==1))
	{
		//connected
		return 1;
	}
	else
	{
		//disconnected
		return 0;
	}
}

