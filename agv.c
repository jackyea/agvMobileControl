#include<signal.h>
#include<sys/time.h>
#include<sched.h>
#include<pthread.h>
//#include<sys/types.h>
//#include<sys/stat.h>
//#include<string.h>
#include<unistd.h>
#include<fcntl.h>
//#include<sys/socket.h>
#include <stddef.h>		/* for NULL */
#include"Log.h"
#include "InitAndSelfTest.h"
#include "ExchangingInfoStruct.h"
#include"serverconnect.h"
#include"CANOpenShellMasterOD.h"
#include"MotorControl.h"
#include "CanOpenShell.h"  //引用 extern CO_Data* CANOpenShellOD_Data;
#include "HandMotorControl.h"


pthread_t BbbExecuteThreadId,DisplayThreadId;//新线程的Id
//extern pthread_mutex_t mut;
char* hostIP="192.168.1.101";//上位机IP地址和端口
int port=3490;
int sockfd;
int savegetflag=0;//存车与取车标志位,存车是0,取车是1
int parkingflag=0;//存取车记数标志位
int timertimer=0;//定时器的计数
char sendbuf[100],recvbuf[100];
extern struct StructAgvReport agvreportstruct;
extern struct StructInner innerstruct;
void *BbbExecuteThreadFunc(void * arg);
void *DisplayThreadFunc(void * arg);
int RecvUpperFunc();

//main函数运行过程：1、连接上位机，2、连通后起rfid线程，3、初始化Can总线,检查电机、传感器是否正确，4、起执行命令线程，5、起定时器，相当于打开向上位机传输结构体线程，6、主程序中运行RecvUpperFunc()，接收上位机命令
int main()
{
	Log("\n\n\n******************程序启动******************"); //防止Agv.c中找不到开头
	if(InitRov()==-1)//初始化
	{
		printf("程序初始化失败！\n");
		Log("程序初始化失败！\n");
		return 0;
	}
	if(pthread_create(&BbbExecuteThreadId,NULL,BbbExecuteThreadFunc,NULL))//这个线程读取bbb接收到的命令，然后执行，最后把执行结果通知bbb。
	{
		printf("BbbExecuteThread error\n");
		Log("agv命令执行线程错误");
		return 0;
	}
	if(RecvUpperFunc()==-1)
	{
		canClose(CANOpenShellOD_Data);
		return 0;
	}
	return 0;
}
//执行命令子线程，开机先启动此线程，
void *BbbExecuteThreadFunc(void * arg)
{
	while(1)
	{
		usleep(1000);
		if(innerstruct.RWflag==1)
		{
			char * commandcopy=innerstruct.Command;
			innerstruct.RWflag=0;//将命令取出后，把标志位清空，使通信程序可以写新命令进去。

			//pthread_mutex_lock(&mut);
			agvreportstruct.ExecuteCommand=1;//取出命令后，将命令执行标志位置1,表示命令正在执行
			//pthread_mutex_unlock(&mut);
			//printf("agvreportstruct.ExecuteCommand==%d\n",agvreportstruct.ExecuteCommand);

			int ret=ExecuteCommand(commandcopy);//将命令传入执行程序

			if(ret<0)//如果命令执行未正常结束
			{
				agvreportstruct.ExecuteCommand=-1;
				Log("命令执行出现错误！");
			}
			else if(ret==0)//如果命令成功执行完成，
			{

				//printf("ret is %d \n",ret);
				agvreportstruct.ExecuteCommand=0;//将执行命令标志位清空，表示可以下发新命令。
				parkingflag++;//存车取车演示时做为当前命令执行完成的标志.
				//printf("agvreportstruct.ExecuteCommand is %d\n",agvreportstruct.ExecuteCommand);
			}
		}
	}
	return 0;
}

//bbb与上位机通信线程
int RecvUpperFunc()
{
	fd_set sockfdset;
	struct timeval tv;//select函数的超时参数
	//超时设定,将超时设置为0,是不阻塞模式，如果没有数据，立即返回。
	tv.tv_sec=0;
	tv.tv_usec=0;

	int keyboard=open("/dev/tty",O_RDONLY);//打开终端输入文件,可以读取键盘的输入
		/*遥控器*/
	int Remoter[13]={0};
	int Flag_Remoter=0;
	while(agvreportstruct.MotorError==0)  //遥控器模式行走无法判断EMCY，使用agvreportstruct.MotorError==0判断EMCY是否可行？？若EMCY 将切断接收上位机指令，此时还能否上报？？
	{
		usleep(1000);//线程暂停1ms,让其它线程有机会运行.
		/*用于检测是否要进入遥控模式，剩余的在遥控模式中进行，减小对上位机模式的影响*/
		Remoter[6] =(IN6_Remoter_bit9_16 & (1<<0)) ? 1 : 0;
		/*遥控器模式*/
		if(Remoter[6]==1) //遥控器1+2同时按下3S，准备进入遥控模式 Remoter[1] && Remoter[2]
		{
			int delaycount=0;
			while(delaycount<=100000000)
			{
				delaycount++;
			} //100000000大概1.2s左右；300000000=4.5s左右
			Remoter[6] =(IN6_Remoter_bit9_16 & (1<<0)) ? 1 : 0;
			if(Remoter[6]) //再次判断IO站电位，1&&2=1超过3S进入遥控模式 Remoter[1] && Remoter[2]
			{
				//绿灯取反=bit2--进入熄灭  退出点亮
				ReverseBit(2);
				//黄灯取反=bit3,--进入点亮，退出熄灭
				ReverseBit(3);
				//根据黄灯bit3给予Flag_Remoter赋值，决定是否触发遥控模式；黄灯亮=遥控模式；这样使通过灯来判断是否进入遥控模式更加准确。
				Flag_Remoter=(OUT8_Relay_bit1_8 & (1<<3)) ? 1 : 0;
				sendPDOevent(CANOpenShellOD_Data);
				//向上位机上报操作模式
				agvreportstruct.RemoteControl=Flag_Remoter; //向上位机上报操作模式 0=上位机模式，1=遥控器模式
				if (Flag_Remoter==0)
					{
						M1_TPDO_mode_of_operation=0x01; //上位机01位置模式
						sendPDOevent(CANOpenShellOD_Data);
						HandAgvStopVelocity();//重新进入上位机模式，去除使能
						Log("AGV退出遥控器模式,已去除使能");
						eprintf("AGV退出遥控器模式,已去除使能");
						fflush(stdout);
					}
				if (Flag_Remoter==1)
					{
						M1_TPDO_mode_of_operation=0x03; //遥控器03速度模式
						sendPDOevent(CANOpenShellOD_Data);
						Log("AGV进入遥控器模式");
						eprintf("AGV进入遥控器模式");
						fflush(stdout);
					}
			}
		}

		/* 进入遥控器模式
		 * 行走电机：按下遥控器触发电机，松开遥控器停止电机
		 * 举升+转向电机（位置模式）：按下遥控器，执行整个命令，即使松开遥控器也继续执行，并判断完成后才返回！*/
		if(Flag_Remoter==1)//遥控器标志位 =1 进入遥控器 Flag_Remoter
		{
			Remoter[1] =(IN6_Remoter_bit9_16 & (1<<5)) ? 1 : 0; //判断bit5=1，则Remoter1_Forward=1；若bit0=0，则Remoter1_Forward=0
			Remoter[2] =(IN6_Remoter_bit9_16 & (1<<4)) ? 1 : 0;
			Remoter[3] =(IN6_Remoter_bit9_16 & (1<<3)) ? 1 : 0;
			Remoter[4] =(IN6_Remoter_bit9_16 & (1<<2)) ? 1 : 0;
			Remoter[5] =(IN6_Remoter_bit9_16 & (1<<1)) ? 1 : 0;
//			Remoter[6] =(IN6_Remoter_bit9_16 & (1<<0)) ? 1 : 0;
			Remoter[7] =(IN6_Remoter_bit9_16 & (1<<6)) ? 1 : 0;
			Remoter[8] =(IN6_Remoter_bit9_16 & (1<<7)) ? 1 : 0;
			Remoter[10] =(IN5_Limit_Switch_bit9_10 & (1<<7)) ? 1 : 0;
			Remoter[11] =(IN5_Limit_Switch_bit9_10 & (1<<6)) ? 1 : 0;
			Remoter[12] =(IN5_Limit_Switch_bit9_10 & (1<<5)) ? 1 : 0;
			//遥控器主程序
			//1.首先检测遥控器所有直行电机(行进后退左移右移，左旋，右旋)，若全=0，则表示松开遥控器，向所有电机发送停止命令
			if(!(Remoter[1] || Remoter[2] || Remoter[3] || Remoter[4]||Remoter[7]||Remoter[8]))
			{
				//向所有行走电机发送停止命令
				M1_TPDO_control_word=WalkStop;
				sendPDOevent(CANOpenShellOD_Data);
			} //end 停止命令

			int remoter_count=0;
			int i=1;
			for (i=1;i<=12;i++) //判断几个按键同时被按下
			{
				remoter_count+=Remoter[i];
			}

			if (remoter_count==1) //判断 有 且只有 1个按键触发
			{
			/*2.然后开始判断各个IO站遥控器输出，执行具体命令，并启动电机（行走电机的停止命令在上一个if执行）*/
			if		(Remoter[1]) //前进
				{
					if (agvreportstruct.WheelDirection == 0)
					{
						if(HandAgvFoward())
						{ //出错！
							//HandAgvErrorStop();
							AgvEmcyStop();
							printf("遥控器-AGV直行出错");
							Log("遥控器-AGV直行出错");
							break; //跳出while循环
						}
					}
				}
			else if (Remoter[2]) //后退
				{
					if (agvreportstruct.WheelDirection == 0)
					{
						if(HandAgvBack())
						{ //出错！
							//HandAgvErrorStop();
							AgvEmcyStop();
							Log("遥控器-AGV后退出错");
							printf("遥控器-AGV后退出错\n");
							break;
						}
					}
				}
			else if (Remoter[3]) //左移
				{
					if (agvreportstruct.WheelDirection == 1)
					{
						if(HandAgvLeft())
						{ //出错！
							//HandAgvErrorStop();
							AgvEmcyStop();
							Log("遥控器-AGV左横移出错");
							printf("遥控器-AGV左横移出错\n");
							break;
						}
					}
				}
			else if (Remoter[4])//右移
				{
					if (agvreportstruct.WheelDirection == 1)
					{
						if(HandAgvRight())
						{ //出错！
							//HandAgvErrorStop();
							AgvEmcyStop();
							Log("遥控器-AGV右横移出错");
							printf("遥控器-AGV右横移出错\n");
							break;
						}
					}
				}
			else if (Remoter[5]) //举升+下降
				{
					if(HandAgvUpOrDown()==-1)
					{
						//HandAgvErrorStop();
						AgvEmcyStop();
						break;
					}
				}

			else if (Remoter[7])//左原地旋转
				{
					if (agvreportstruct.WheelDirection == 2)
					{
						if(HandAgvRotateLeft())
						{ //出错！
							//HandAgvErrorStop();
							AgvEmcyStop();
							Log("遥控器-AGV左原地旋转出错");
							printf("遥控器-AGV左原地旋转出错\n");
							break;
						}
					}
				}
			else if (Remoter[8])//右原地旋转
				{
					if (agvreportstruct.WheelDirection == 2)
					{
						if(HandAgvRotateRight())
						{ 	//出错！
							//HandAgvErrorStop();
							AgvEmcyStop();
							Log("遥控器-AGV右原地旋转出错");
							printf("遥控器-AGV右原地旋转出错\n");
							break;
						}
					}
				}
			else if (Remoter[10]) //轮子横移位置
				{
					if(HandRotatorWheelCrab()==-1)
					{
						//HandAgvErrorStop();
						AgvEmcyStop();
						Log("遥控器-至横移位置时出错！");
						printf("遥控器-至横移位置时出错！\n");
						break;
					}
					Log("遥控器-到达横移位置");
					printf("遥控器-到达横移位置\n");
				}
			else if (Remoter[11]) //轮子自旋位置
				{
					if(HandRotateWheelRotate()==-1)
					{
						//HandAgvErrorStop();
						AgvEmcyStop();
						Log("遥控器-至自旋位置时出错！");
						printf("遥控器-至自旋位置时出错！\n");
						break;
					}
					Log("遥控器-到达自旋位置");
				}
			else if (Remoter[12])//轮子零位置
				{
					if(HandRotateWheelZero()==-1)
					{
						//HandAgvErrorStop();
						AgvEmcyStop();
						Log("遥控器-至0位置时出错！");
						printf("遥控器-至0位置时出错！\n");
						break;
					}
					Log("遥控器-到达零位置");
					printf("遥控器-到达零位置\n");
				}
			}//end 只有一个按键触发
		} //end 遥控器具体指令

		/*进入等待命令输入模式*/
		else
		{
			FD_ZERO(&sockfdset);//清空描述符集
			FD_SET(0,&sockfdset);//添加连接到描述符集中
			if(select(keyboard,&sockfdset,NULL,NULL,&tv)>0)//select函数
			{
					memset(recvbuf,0,sizeof(recvbuf));//只当recvbuf有接收数据后才清空,避免每次while都清空一次.
					int rbytes=read(keyboard,recvbuf,sizeof(recvbuf));
					if((recvbuf[0]=='A'||recvbuf[0]=='a')&&(recvbuf[1]=='A'||recvbuf[1]=='a')&&(recvbuf[rbytes-2]=='E'||recvbuf[rbytes-2]=='e')&&(recvbuf[rbytes-1]=='E'||recvbuf[rbytes-1]=='e'))//如果命令以AA开头，以EE结尾，说明命令正确
					{
						innerstruct.Command=recvbuf;
						innerstruct.RWflag=1;//写入结构体后将标志位置1,让命令执行程序去读
					}
					else if(recvbuf[0]=='s'&&recvbuf[1]=='a'&&recvbuf[2]=='v'&&recvbuf[3]=='e'&&recvbuf[4]=='c'&&recvbuf[5]=='a'&&recvbuf[6]=='r')
					{
						savegetflag=0;
						if(pthread_create(&DisplayThreadId,NULL,DisplayThreadFunc,NULL))
						{
							printf("save create DisplayThread failed\n");
						}
					}
					else if(recvbuf[0]=='g'&&recvbuf[1]=='e'&&recvbuf[2]=='t'&&recvbuf[3]=='c'&&recvbuf[4]=='a'&&recvbuf[5]=='r')
					{
						savegetflag=1;
						if(pthread_create(&DisplayThreadId,NULL,DisplayThreadFunc,NULL))
						{
							printf("get create DisplayThread failed\n");
						}
					}
					else if(recvbuf[0]=='Q'||recvbuf[0]=='q')
					{
						innerstruct.StopCommand=1;
						innerstruct.RWflag=1;//写入结构体后将标志位置1,让命令执行程序去读
						return -1;
					}
					printf("%s \n",recvbuf);//将命令打印出来
			}
			pthread_join(DisplayThreadId,NULL);//如果存车线程或取车线程启动的话当其结束后将其回收,如果没有启动的话相当于是空语句.
		}
	} //end while
	printf("我是RecvUpperFunc函数，收到EMCY，程序退出\n");
	Log("我是RecvUpperFunc函数，收到EMCY，程序退出");
	AgvEmcyStop();
	return -1;
}

void *DisplayThreadFunc(void * arg)
{
	parkingflag=0;//线程启动前,将parkging置0
	if(savegetflag==0)//如果是存车
	{
		if(parkingflag==0)
		{
			innerstruct.Command="AABB31005.100003EE";//从充电位（车位1）出来，左移两个车位，到达车位3
			innerstruct.RWflag=1;//写入结构体后将标志位置1,让命令执行程序去读
			while(parkingflag==0)
			{
				if(agvreportstruct.ExecuteCommand==1)//如果BbbExecuteThreadFunc线程成功接收命令,将agvreportstruct.ExecuteCommand置1,则退出循环
				{
					parkingflag++;
				}
				usleep(1000);
			}
		}
		if(parkingflag==2)
		{
			innerstruct.Command="AABB11010.100009EE";//从车位3前进2个车位，到达车位9
			innerstruct.RWflag=1;//写入结构体后将标志位置1,让命令执行程序去读
			while(parkingflag==2)
			{
				if(agvreportstruct.ExecuteCommand==1)//如果BbbExecuteThreadFunc线程成功接收命令,将agvreportstruct.ExecuteCommand置1,则退出循环
				{
					parkingflag++;
				}
				usleep(1000);
			}
		}
		if(parkingflag==4)
		{
			innerstruct.Command="AAWR2EE";//调整到自旋位
			innerstruct.RWflag=1;//写入结构体后将标志位置1,让命令执行程序去读
			while(parkingflag==4)
			{
				if(agvreportstruct.ExecuteCommand==1)//如果BbbExecuteThreadFunc线程成功接收命令,将agvreportstruct.ExecuteCommand置1,则退出循环
				{
					parkingflag++;
				}
				usleep(1000);
			}
		}
		if(parkingflag==6)
		{
			innerstruct.Command="AACC1EE";//举升，把车顶起来
			innerstruct.RWflag=1;//写入结构体后将标志位置1,让命令执行程序去读
			while(parkingflag==6)
			{
				if(agvreportstruct.ExecuteCommand==1)//如果BbbExecuteThreadFunc线程成功接收命令,将agvreportstruct.ExecuteCommand置1,则退出循环
				{
					parkingflag++;
				}
				usleep(1000);
			}
		}
		if(parkingflag==8)
		{
			innerstruct.Command="AARR01EE";//在车位9自旋180°
			innerstruct.RWflag=1;//写入结构体后将标志位置1,让命令执行程序去读
			while(parkingflag==8)
			{
				if(agvreportstruct.ExecuteCommand==1)//如果BbbExecuteThreadFunc线程成功接收命令,将agvreportstruct.ExecuteCommand置1,则退出循环
				{
					parkingflag++;
				}
				usleep(1000);
			}
		}
		if(parkingflag==10)
		{
			innerstruct.Command="AABB31005.100007EE";//左移两个车位，到达车位7
			innerstruct.RWflag=1;//写入结构体后将标志位置1,让命令执行程序去读
			while(parkingflag==10)
			{
				if(agvreportstruct.ExecuteCommand==1)//如果BbbExecuteThreadFunc线程成功接收命令,将agvreportstruct.ExecuteCommand置1,则退出循环
				{
					parkingflag++;
				}
				usleep(1000);
			}
		}
		if(parkingflag==12)
		{
			innerstruct.Command="AACC0EE";//卸载，把车及车架放下
			innerstruct.RWflag=1;//写入结构体后将标志位置1,让命令执行程序去读
			while(parkingflag==12)
			{
				if(agvreportstruct.ExecuteCommand==1)//如果BbbExecuteThreadFunc线程成功接收命令,将agvreportstruct.ExecuteCommand置1,则退出循环
				{
					parkingflag++;
				}
				usleep(1000);
			}
		}
		if(parkingflag==14)
		{
			innerstruct.Command="AABB41002.100008EE";//右移一个车位，到达车位8
			innerstruct.RWflag=1;//写入结构体后将标志位置1,让命令执行程序去读
			while(parkingflag==14)
			{
				if(agvreportstruct.ExecuteCommand==1)//如果BbbExecuteThreadFunc线程成功接收命令,将agvreportstruct.ExecuteCommand置1,则退出循环
				{
					parkingflag++;
				}
				usleep(1000);
			}
		}
		if(parkingflag==16)
		{
			innerstruct.Command="AABB11010.100002EE";//前进两个车位，到达车位2
			innerstruct.RWflag=1;//写入结构体后将标志位置1,让命令执行程序去读
			while(parkingflag==16)
			{
				if(agvreportstruct.ExecuteCommand==1)//如果BbbExecuteThreadFunc线程成功接收命令,将agvreportstruct.ExecuteCommand置1,则退出循环
				{
					parkingflag++;
				}
				usleep(1000);
			}
		}
		if(parkingflag==18)
		{
			innerstruct.Command="AARR11EE";//在车位2自旋180°,车轮调整与自旋一起完成
			innerstruct.RWflag=1;//写入结构体后将标志位置1,让命令执行程序去读
			while(parkingflag==18)
			{
				if(agvreportstruct.ExecuteCommand==1)//如果BbbExecuteThreadFunc线程成功接收命令,将agvreportstruct.ExecuteCommand置1,则退出循环
				{
					parkingflag++;
				}
				usleep(1000);
			}
		}
		if(parkingflag==20)
		{
			innerstruct.Command="AABB41002.100001EE";//右移一个车位，到达充电位（车位1）
			innerstruct.RWflag=1;//写入结构体后将标志位置1,让命令执行程序去读
			while(parkingflag==20)
			{
				if(agvreportstruct.ExecuteCommand==1)//如果BbbExecuteThreadFunc线程成功接收命令,将agvreportstruct.ExecuteCommand置1,则退出循环
				{
					parkingflag++;
				}
				usleep(1000);
			}
		}
		printf("save car completed\n");

	}
	else if(savegetflag==1)//如果是取车
	{
		if(parkingflag==0)
		{
			innerstruct.Command="AABB31002.100002EE";//从充电位（车位1）出来，左移1个车位，到达车位2
			innerstruct.RWflag=1;//写入结构体后将标志位置1,让命令执行程序去读
			while(parkingflag==0)
			{
				if(agvreportstruct.ExecuteCommand==1)//如果BbbExecuteThreadFunc线程成功接收命令,将agvreportstruct.ExecuteCommand置1,则退出循环
				{
					parkingflag++;
				}
				usleep(1000);
			}
		}
		if(parkingflag==2)
		{
			innerstruct.Command="AABB11010.100008EE";//从车位2前进2个车位，到达车位8
			innerstruct.RWflag=1;//写入结构体后将标志位置1,让命令执行程序去读
			while(parkingflag==2)
			{
				if(agvreportstruct.ExecuteCommand==1)//如果BbbExecuteThreadFunc线程成功接收命令,将agvreportstruct.ExecuteCommand置1,则退出循环
				{
					parkingflag++;
				}
				usleep(1000);
			}
		}
		if(parkingflag==4)
		{
			innerstruct.Command="AABB41002.100007EE";//在车位8右移1个车位，到达车位7
			innerstruct.RWflag=1;//写入结构体后将标志位置1,让命令执行程序去读
			while(parkingflag==4)
			{
				if(agvreportstruct.ExecuteCommand==1)//如果BbbExecuteThreadFunc线程成功接收命令,将agvreportstruct.ExecuteCommand置1,则退出循环
				{
					parkingflag++;
				}
				usleep(1000);
			}
		}
		if(parkingflag==6)
		{
			innerstruct.Command="AACC1EE";//举升，把车顶起来
			innerstruct.RWflag=1;//写入结构体后将标志位置1,让命令执行程序去读
			while(parkingflag==6)
			{
				if(agvreportstruct.ExecuteCommand==1)//如果BbbExecuteThreadFunc线程成功接收命令,将agvreportstruct.ExecuteCommand置1,则退出循环
				{
					parkingflag++;
				}
				usleep(1000);
			}
		}
		if(parkingflag==8)
		{
			innerstruct.Command="AABB31005.100009EE";//左移2个车位，到达车位9
			innerstruct.RWflag=1;//写入结构体后将标志位置1,让命令执行程序去读
			while(parkingflag==8)
			{
				if(agvreportstruct.ExecuteCommand==1)//如果BbbExecuteThreadFunc线程成功接收命令,将agvreportstruct.ExecuteCommand置1,则退出循环
				{
					parkingflag++;
				}
				usleep(1000);
			}
		}
		if(parkingflag==10)
		{
			innerstruct.Command="AACC0EE";//卸载，把车及车架放下
			innerstruct.RWflag=1;//写入结构体后将标志位置1,让命令执行程序去读
			while(parkingflag==10)
			{
				if(agvreportstruct.ExecuteCommand==1)//如果BbbExecuteThreadFunc线程成功接收命令,将agvreportstruct.ExecuteCommand置1,则退出循环
				{
					parkingflag++;
				}
				usleep(1000);
			}
		}
		if(parkingflag==12)
		{
			innerstruct.Command="AABB21010.100003EE";//后退2个车位，到达车位3
			innerstruct.RWflag=1;//写入结构体后将标志位置1,让命令执行程序去读
			while(parkingflag==12)
			{
				if(agvreportstruct.ExecuteCommand==1)//如果BbbExecuteThreadFunc线程成功接收命令,将agvreportstruct.ExecuteCommand置1,则退出循环
				{
					parkingflag++;
				}
				usleep(1000);
			}
		}
		if(parkingflag==14)
		{
			innerstruct.Command="AABB41005.100001EE";//右移2个车位，到达充电位（车位1）
			innerstruct.RWflag=1;//写入结构体后将标志位置1,让命令执行程序去读
			while(parkingflag==14)
			{
				if(agvreportstruct.ExecuteCommand==1)//如果BbbExecuteThreadFunc线程成功接收命令,将agvreportstruct.ExecuteCommand置1,则退出循环
				{
					parkingflag++;
				}
				usleep(1000);
			}
		}
		printf("get car completed\n");
	}
	return 0;
}
