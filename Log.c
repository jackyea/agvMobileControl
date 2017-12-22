/*
 * Log.c
 *
 *  Created on: 2017-7-6
 *      Author: yenb
 */
#include<time.h>
#include<string.h>
#include<unistd.h>
#include<stdio.h>

//void Log(char* str,char * format, ...)
void Log(char* str)
{
	char buffer[1024]={0};
	//取得当前时间
	time_t now;
	time(&now);
	struct tm *local;
	local=localtime(&now);

	//把要写的字符串拼好并写到buffer中。
	sprintf(buffer,"%04d-%02d-%02d %02d:%02d:%02d %s\n",local->tm_year+1900,local->tm_mon+1,local->tm_mday,local->tm_hour,local->tm_min,local->tm_sec,str);

	//写入文件中
	FILE *file=fopen("agv.log","a+");//文件不存在就创建，存在就打开
	fwrite(buffer,1,strlen(buffer),file);
	fclose(file);
	/*清空str数组*/
	memset(buffer,0,sizeof(buffer));

	return ;
}
