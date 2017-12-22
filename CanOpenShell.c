/*
 * CANOpenShell.c
 *
 *  Created on: Nov 25, 2016
 *      Author: guochence
 */
/*
 * CANOpenShell.c
 *
 *  Created on: Nov 22, 2016
 *      Author: guochence
 */

#if defined(WIN32) && !defined(__CYGWIN__)
#include <windows.h>
#define CLEARSCREEN "cls"
#define SLEEP(time) Sleep(time * 1000)
#else
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include <time.h>

#define CLEARSCREEN "clear"
#define SLEEP(time) sleep(time)

#endif

//****************************************************************************
// INCLUDES
#include "canfestival.h"
#include "CanOpenShell.h"
#include "CANOpenShellMasterOD.h"
#include "CANOpenShellSlaveOD.h"
#include "configureslave.h"
#include "callback.h"
#include "MotorOnline.h"
#include "Log.h"
#include "ExchangingInfoStruct.h"
#include "MotorControl.h"


//#include<time.h>
//#include<string.h>
//#include<unistd.h>
//#include<stdio.h>
//****************************************************************************
// DEFINES
#define MAX_NODES 127
#define cst_str4(c1, c2, c3, c4) ((((unsigned int)0 | \
                                    (char)c4 << 8) | \
                                   (char)c3) << 8 | \
                                  (char)c2) << 8 | \
                                 (char)c1


#define cst_str1(a1, a2, a3, a4, a5, a6, a7) ((((((unsigned int)0 | \
                                    (char)a7 << 8) | \
                                    (char)a6 << 8  | \
                                   (char)a5) << 8 | \
                                  (char)a4) << 8 | \
                                 (char)a3) << 8 | \
								(char)a2) << 8 | \
                               (char)a1

#define INIT_ERR 2
#define QUIT 1

//****************************************************************************
// 全局变量
char BoardBusName[31] = "0";
char BoardBaudRate[5] = "1M";
s_BOARD Board = { BoardBusName, BoardBaudRate };
CO_Data* CANOpenShellOD_Data;
//char LibraryPath[512] = "/root/libcanfestival_can_socket.so";
//char LibraryPath[512] = "/root/libcanfestival_can_socket_nodebug.so";
char LibraryPath[512] = "/root/Log_Nodebug.so";  //在.so中send recive出错写入agv.log中，有可能和其他写入造成冲突
//char LibraryPath[512] = "/root/Log_debug.so";


int sefltest_success = 0;
int boot_success_mark = 0;

/*声明结构体外部引用agvreportstruct*/
extern struct StructAgvReport agvreportstruct;
extern struct StructSenser senserstruct;
extern struct StructInner innerstruct;
/***************************************************************************/
/********************************配 置 从 站 指 令******************************/
/* 使从节点进入 operational mode */
void StartNode(UNS8 nodeid) {
	masterSendNMTstateChange(CANOpenShellOD_Data, nodeid, NMT_Start_Node);
	printf("从站 %2.2x operational\n",nodeid);
	//从站配置完成的标志以 主站向从站发送 startnode命令
	if (nodeid == 0x01){R1_Configuration = 1;Log("电机 0x01，初始化完成");}
	else if (nodeid == 0x02){R2_Configuration = 1;Log("电机 0x02，初始化完成");}
	else if (nodeid == 0x03){R3_Configuration = 1;Log("电机 0x03，初始化完成");}
	else if (nodeid == 0x04){R4_Configuration = 1;Log("电机 0x04，初始化完成");}
	else if (nodeid == 0x05){R5_Configuration = 1;Log("电机 0x05，初始化完成");}
	else if (nodeid == 0x06){R6_Configuration = 1;Log("电机 0x06，初始化完成");}
	else if (nodeid == 0x07){R7_Configuration = 1;Log("电机 0x07，初始化完成");}
	else if (nodeid == 0x08){R8_Configuration = 1;Log("电机 0x08，初始化完成");}
	else if (nodeid == 0x09){R9_Configuration = 1;Log("电机 0x09，初始化完成");}
	else if (nodeid == 0x0f){IO_Configuration = 1;Log("IO站 0x0F，初始化完成");}
	else if (nodeid == 0x0A){PGV_Configuration = 1;Log("PGV 0x0A，初始化完成");}
}

/* 使从节点进入 pre-operational mode */
void StopNode(UNS8 nodeid) {
	printf("从站被 Stop\n");
	masterSendNMTstateChange(CANOpenShellOD_Data, nodeid, NMT_Stop_Node);
}

/* 使从节点 reset */
void ResetNode(UNS8 nodeid) {
	printf("从站被 Reset\n");
	Log("从站被 Reset");
	masterSendNMTstateChange(CANOpenShellOD_Data, nodeid, NMT_Reset_Node);
}

/*************************** 主 站 回 调 函 数  *****************************************/
void CANOpenShellOD_heartbeatError(CO_Data* d, UNS8 heartbeatID)
{//主站上电 就开始检测心跳
	if (ChargeFlag==1) //充电开始屏蔽emcy
	{}

	else
	{
	if (HeartbeatStartFlag == 1)  //调试使用，仅IO站初始化完毕，屏蔽电机心跳
	{ /*只针对IO站检测，仅在调试时使用！！！*/
		char str[128]={0};
		//把MotorError（上位机结构体）=1 告诉上位机出错
		if (heartbeatID == 0x0f)
		{
			agvreportstruct.IOError = 1;

			sprintf(str,"Slave Disconnect %2.2x\n", heartbeatID);
			Log(str); //输出字符串str给LOG
			/*清空str数组*/
			memset(str,0,sizeof(str));

			eprintf("Slave Disconnect %2.2x\n", heartbeatID);
			fflush(stdout);

			/*发生错误后处理*/
			//停掉电机？？
			AgvEmcyStop();
		}
	}

	else if (HeartbeatStartFlag == 2) //当IO+电机都初始化完毕，开始检测两者的心跳
	{
		char str[128]={0};
		//把MotorError（上位机结构体）=1 告诉上位机出错
		if (heartbeatID == 0x0f||heartbeatID == 0x0A){agvreportstruct.IOError = 1;}
		else{agvreportstruct.MotorError = 1;}

		sprintf(str,"Slave Disconnect %2.2x\n", heartbeatID);
		Log(str); //输出字符串str给LOG
		/*清空str数组*/
		memset(str,0,sizeof(str));

		eprintf("Slave Disconnect %2.2x\n", heartbeatID);
		fflush(stdout);

		/*发生错误后处理*/
		//停掉电机？？
		AgvEmcyStop();
	}
	} //end else
}//end CANOpenShellOD_heartbeatError

void CANOpenShellOD_post_emcy(CO_Data* d, UNS8 nodeID, UNS16 errCode,
		UNS8 errReg,UNS16 Byte54)
{
	if (ChargeFlag==1) //充电开始屏蔽emcy
	{
		;
	}
	else
	{
	if (nodeID == 0x0f||nodeID == 0x0A){agvreportstruct.IOError = 1;}
	else{agvreportstruct.MotorError = 1;}

	char str[128]={0};
	sprintf(str,"Master received EMCY message. Node: %2.2x  ErrorCode: %4.4x  ErrorRegister: %2.2x Byte5&4: %4.4x\n",nodeID, errCode, errReg, Byte54);
	Log(str); //输出字符串str给LOG
	/*清空str数组*/
	memset(str,0,sizeof(str));

	eprintf("Master received EMCY message. Node: %2.2x  ErrorCode: %4.4x  ErrorRegister: %2.2x Byte5&4: %4.4x\n",
			nodeID, errCode, errReg, Byte54);
	fflush(stdout);

	/*发生错误后处理*/
	AgvEmcyStop();
	}
}

void CANOpenShellOD_post_SlaveBootup(CO_Data* d, UNS8 nodeid) {
	printf("从站 %2.2x 上电\n", nodeid);
	char str[128]={0};
	sprintf(str,"从站 %2.2x 上电",nodeid);
	Log(str); //输出字符串str给LOG
//	ConfigureSlaveNode(d, nodeid);
	if (nodeid == 0x01){R1_Online = 1;}
	else if (nodeid == 0x02){R2_Online = 1;}
	else if (nodeid == 0x03){R3_Online = 1;}
	else if (nodeid == 0x04){R4_Online = 1;}
	else if (nodeid == 0x05){R5_Online = 1;}
	else if (nodeid == 0x06){R6_Online = 1;}
	else if (nodeid == 0x07){R7_Online = 1;}
	else if (nodeid == 0x08){R8_Online = 1;}
	else if (nodeid == 0x09){R9_Online = 1;}
	else if (nodeid == 0x0f){IO_Online = 1;ConfigureSlaveNode(d, 0x0f);}
	else if (nodeid == 0x0A){PGV_Online = 1;}
}

void CANOpenShellOD_initialisation(CO_Data* d) {
	printf("主站 initialisation\n");
	Log("主站 initialisation");
	fflush(stdout);
}

void CANOpenShellOD_preOperational(CO_Data* d) {
	printf("主站 preOperational\n");
	fflush(stdout);
	Log("主站 preOperational");
	ResetNode(0x00);  //重置必须在这里，否则IO站有可能重置两次--没找到原因
}

void CANOpenShellOD_operational(CO_Data* d) {
	printf("主站 operational\n");
	Log("主站 operational");
	fflush(stdout);
}

void CANOpenShellOD_stopped(CO_Data* d) {
	printf("主站 stopped\n");
	Log("主站 stopped");
	fflush(stdout);
}

void CANOpenShellOD_post_sync(CO_Data* d) {
//	printf("Master_post_sync\n");
//	fflush(stdout);
}

void CANOpenShellOD_post_TPDO(CO_Data* d) {

}

void CANOpenShellOD_SDO_error(CO_Data* d, UNS8 noid_id) {

	printf("%2.2x SDO初始化配置出错\n",noid_id);
	fflush(stdout);

	char str[128]={0};
	sprintf(str,"%2.2x SDO初始化配置出错\n",noid_id);
	Log(str); //输出字符串str给LOG

	/*清空str数组*/
	memset(str,0,sizeof(str));

	AgvEmcyStop();
}

/*************************** 主 站 初 始 化 **********************************/
void Init(CO_Data* d, UNS32 id) {
	if (Board.baudrate) {
		/* 初始化主站状态 */
		setState(CANOpenShellOD_Data, Initialisation);
	}
}

/***************************  清 理 *****************************************/
void Exit(CO_Data* d, UNS32 nodeid) {
	if (strcmp(Board.baudrate, "none")) {
		/* 重置总线上的所有从站节点 */
		masterSendNMTstateChange(CANOpenShellOD_Data, 0, NMT_Reset_Node);

		/* 停止主站 */
		setState(CANOpenShellOD_Data, Stopped); //将会调用emcy.c中的emergencyStop函数
	}
}

/***************************主 站 节 点 初 始 化************************************/
int NodeInit(int NodeID, int NodeType) {

	CANOpenShellOD_Data = &CANOpenShellMasterOD_Data;

	/* 载入库文件 */
	LoadCanDriver(LibraryPath);

	/* 定义主站回调函数 */
	CANOpenShellOD_Data->initialisation = CANOpenShellOD_initialisation;
	CANOpenShellOD_Data->preOperational = CANOpenShellOD_preOperational;
	CANOpenShellOD_Data->operational = CANOpenShellOD_operational;
	CANOpenShellOD_Data->stopped = CANOpenShellOD_stopped;
	CANOpenShellOD_Data->post_sync = CANOpenShellOD_post_sync;
	CANOpenShellOD_Data->post_TPDO = CANOpenShellOD_post_TPDO;
	CANOpenShellOD_Data->post_SlaveBootup = CANOpenShellOD_post_SlaveBootup;

	CANOpenShellOD_Data->heartbeatError = CANOpenShellOD_heartbeatError;
	CANOpenShellOD_Data->post_emcy = CANOpenShellOD_post_emcy;
	CANOpenShellOD_Data->transfers[NodeID].Callback = CANOpenShellOD_SDO_error;

	/* 打开CANOPEN设备 */
	if (!canOpen(&Board, CANOpenShellOD_Data))
		return INIT_ERR;

	/*注册从站回调函数*/
	//RegisterODandCallback(CANOpenShellOD_Data);

	/* 定义主站ID */
	setNodeId(CANOpenShellOD_Data, NodeID);

	/* 开始时钟线程 */
	StartTimerLoop(&Init);
	return 0;
}

/**************************************************************/
void type(int commondflag) {
	//int commondflag;
	CO_Data* d=CANOpenShellOD_Data;
	printf("#############start : start the motor############################\n");
	//while (1)
	//{
//		if(!strcmp(commond,"forward"))
//		{
//			commondflag=0;
//		}
//		else if(!strcmp(commond,"turnleftright"))
//		{
//			commondflag=1;
//		}
//		else if(!strcmp(commond,"liftupdown"))
//		{
//			commondflag=2;
//		}
//		else if(!strcmp(commond,"stopforward"))
//		{
//			commondflag=3;
//		}
//		else if(!strcmp(commond,"stopturnleftright"))
//		{
//			commondflag=4;
//		}
//		else if(!strcmp(commond,"stopliftupdown"))
//		{
//			commondflag=5;
//		}
//		else if(!strcmp(commond,"speed1"))
//		{
//			commondflag=6;
//		}
//		else if(!strcmp(commond,"speed2"))
//		{
//			commondflag=7;
//		}
//		else if(!strcmp(commond,"speed3"))
//		{
//			commondflag=8;
//		}
//		else if(!strcmp(commond,"stop"))
//		{
//			commondflag=9;
//		}
//		else if(!strcmp(commond,"exit"))
//		{
//			commondflag=10;
//		}

		switch (commondflag)
		{
			case 0: /* start_motor 1*/
				M1_1_TPDO_target_speed = 0x00061A80;
				M1_1_TPDO_Profile_Acceleration = 0x0000FFA0;
				M1_1_TPDO_Profile_Deceleration = 0x0000FFA0;
				M1_TPDO_control_word = 0x000F;
				sendPDOevent(d);
			break;

			case 1: /* start_motor 2*/
				M2_6_TPDO_Target_Position = 0x000286A0;
				M2_6_TPDO_Profile_Acceleration = 0x0000FFA0;
				M2_6_TPDO_Profile_Deceleration = 0x0000FFA0;
				M2_6_TPDO_Profile_Velocity = 0x00061a80;
				sendPDOevent(d);
				M2_TPDO_control_word = 0x003F; //start move with absolute/change imedicate/bit 4 =0-1
				sendPDOevent(d);
				sleep(1); //in case bit4=0-1 & bit4=1-0 process in the same SYNC
				M2_TPDO_control_word = 0x002F; //set bit 4 =1-0, prepare for next move
				sendPDOevent(d);
				break;

			case 2: /* start_motor 3*/

				M3_TPDO_Target_Position = 0x000286A0;
				M3_TPDO_Profile_Acceleration = 0x0000FFA0;
				M3_TPDO_Profile_Deceleration = 0x0000FFA0;
				M3_TPDO_Profile_Velocity = 0x00061a80;
				sendPDOevent(d);
				M3_TPDO_control_word = 0x003F; //start move with absolute/change imedicate/bit 4 =0-1
				sendPDOevent(d);
				sleep(1); //in case bit4=0-1 & bit4=1-0 process in the same SYNC
				M3_TPDO_control_word = 0x002F; //set bit 4 =1-0, prepare for next move
				sendPDOevent(d);
				break;
	////////////////////////////////////////////////////////////////////////////
			case 3:/* stop motor*/
				M1_TPDO_control_word = 0x010F;
				sendPDOevent(d);
				break;

			case 4:/* stop motor*/
				M2_TPDO_control_word = 0x014F;
				sendPDOevent(d);
				break;

			case 5:/* stop motor*/
				M3_TPDO_control_word = 0x012F; //set bit 4 =1-0, prepare for next move
				sendPDOevent(d);
				break;
	////////////////////////////////////////////////////////////////////////////
			case 6:
				M1_1_TPDO_target_speed = 0x000F1A80;
				sendPDOevent(d);
				break;

			case 7:
				M2_6_TPDO_Profile_Velocity = 0x00061A80;
				M2_6_TPDO_Target_Position = 0x00000000; //negetive 286a0
				sendPDOevent(d);
				M2_TPDO_control_word = 0x003F; //start move with bit 4 =0-1
				sendPDOevent(d);
				sleep(1); //in case bit4=0-1 & bit4=1-0 process in the same SYNC
				M2_TPDO_control_word = 0x002F; //set bit 4 =1-0, prepare for next move
				sendPDOevent(d);
				break;

			case 8:
				M3_TPDO_Target_Position = 0x00000000;
				M3_TPDO_Profile_Acceleration = 0x000FFFA0;
				M3_TPDO_Profile_Deceleration = 0x000FFFA0;
				M3_TPDO_Profile_Velocity = 0x00161a80;
				sendPDOevent(d);
				M3_TPDO_control_word = 0x003F; //start move with absolute/change imedicate/bit 4 =0-1
				sendPDOevent(d);
				sleep(1); //in case bit4=0-1 & bit4=1-0 process in the same SYNC
				M3_TPDO_control_word = 0x002F; //set bit 4 =1-0, prepare for next move
				sendPDOevent(d);
				break;
	////////////////////////////////////////////////////////////////////////////
			case 9:
				printf("command is 'stop'\n");
				M1_TPDO_control_word = 0x010F;
				M2_TPDO_control_word = 0x016F;
				M3_TPDO_control_word = 0x010F;
				sleep(1);
				sendPDOevent(d);
				break;

			case 10:/* exit system and ask node to pre-op*/
				printf("command is 'exit'\n");
				break;

			case 11:
				OUT7_DLS_Control_Zone_bit1_8 = 0xff;
				OUT7_DLS_Control_Zone_bit9_16 = 0xff;
				OUT8_Relay_bit1_8 = 0xff;
					sendPDOevent(d);
					break;
		}
	//}
}

void boot_success(int a) {
	boot_success_mark = 1;
}

/*check_modo_change is used to
 * wait the mode change applied
 */
void check_modop_change(UNS8 mode_operation) {
	if (mode_operation == 0x03) {
		while (R1_Display_Mode_of_operation != mode_operation)
			;
		printf("Velocity Mode has been applied to %x\n", mode_operation);
	}
	if (mode_operation == 0x01) {
		while (R6_Display_Mode_of_operation != mode_operation)
			;
		printf("Position Mode has been applied to %x\n",
				R6_Display_Mode_of_operation);
	}

}

/*check statuswords bit 10
 * determin target reached bit10=1;
 * 用作selfcheck 位置模式和归零模式自检时 到达目标位置的检测
 */
void check_statuswords(int group) {
	sleep(1); //delay for renewing statuswords register
	if (group == 1) {
		while (!(R1_statusword & (1 << (10))))
			;
	}
	if (group == 2) {
		while (!(R6_statusword & (1 << (10))))
			;
	}
	if (group == 3) {
		while (!(R9_statusword & (1 << (10))))
			;
	}
}

void self_test(CO_Data* d) {
	//seft_test progress
	//error alarm
	int count_num = 0;
	int test_num = 0;
	while (count_num <= 2) {
		switch (test_num) {

		case 0:
			//firt group test
			//run in velocity mode

			M1_TPDO_mode_of_operation = 0x03; //set mode to velocity mode
			sendPDOevent(d);
			M1_1_TPDO_target_speed = 0x00061A80;
			M1_1_TPDO_Profile_Acceleration = 0x0000FFA0;
			M1_1_TPDO_Profile_Deceleration = 0x0000FFA0;
			M1_TPDO_control_word = 0x000F;
			sendPDOevent(d);
			sleep(3);
			M1_TPDO_control_word = 0x010F;
			sendPDOevent(d);
			sleep(1);
			M1_1_TPDO_target_speed = 0xFFF9E580;
			M1_TPDO_control_word = 0x000F;
			sendPDOevent(d);
			sleep(3);
			M1_TPDO_control_word = 0x010F;
			sendPDOevent(d);
			break;

		case 1:
//			//second group test
//			M2_TPDO_control_word = 0x001F; //start move with bit 4 =0-1
//			sendPDOevent(d);
////			while (R6_position != 0x00000000) {
////			}
//			check_statuswords(2);
			break;

		case 2:
			//thrid group test
//			M3_TPDO_Target_Position = 0x000286A0;
//			M3_TPDO_Profile_Acceleration = 0x0000FFA0;
//			M3_TPDO_Profile_Deceleration = 0x0000FFA0;
//			M3_TPDO_Profile_Velocity = 0x00061a80;
//			sendPDOevent(d);
//			M3_TPDO_control_word = 0x003F; //start move with absolute/change imedicate/bit 4 =0-1
//			sendPDOevent(d);
//			sleep(1); //in case bit4=0-1 & bit4=1-0 process in the same SYNC
//			M3_TPDO_control_word = 0x002F; //set bit 4 =1-0, prepare for next move
//			sendPDOevent(d);
////			while (R9_position < 0x000286A0) {
////			}
//			check_statuswords(3); //wait until target reached
//			M3_TPDO_Target_Position = 0x00000000;
//			M3_TPDO_Profile_Acceleration = 0x00000FA0;
//			M3_TPDO_Profile_Deceleration = 0x00000FA0;
//			M3_TPDO_Profile_Velocity = 0x000F1a80;
//			sendPDOevent(d);
//			M3_TPDO_control_word = 0x003F; //start move with bit 4 =0-1
//			sendPDOevent(d);
//			sleep(1); //in case bit4=0-1 & bit4=1-0 process in the same SYNC
//			M3_TPDO_control_word = 0x002F; //set bit 4 =1-0, prepare for next move
//			sendPDOevent(d);
////			while (R9_position != 0) {
////			} //wait until got 0 position
//			check_statuswords(3); //wait until target reached
			break;

		default:
			printf("error has occur");

		}
		test_num++;
		count_num++;
	}
	sefltest_success = 1;

}

void stand_by(CO_Data* d) {
	//set stand_by mode & parameter
	//set runing mode of operation
	//check whether goes in runing mode of operation

	/************1st group= position----velocity mode and reset all paremeters*****/
	M1_TPDO_mode_of_operation = 0x03; //change mode to velocity mode
	sendPDOevent(d);
	M1_1_TPDO_target_speed = 0x00000000;
	M1_1_TPDO_Profile_Acceleration = 0x00000000;
	M1_1_TPDO_Profile_Deceleration = 0x00000000;
	M1_TPDO_control_word = 0x010F;
	sendPDOevent(d);
	check_modop_change(0x03); //wait for velocity mode applied

	/************2st group= homing----position mode and reset all paremeters*****/
//	M2_TPDO_mode_of_operation = 0x01; //change to position mode
//	M2_TPDO_control_word = 0x012F;
//	sendPDOevent(d);
//	/*FOR NOW, only availabe for COBID 6*/
//	M2_6_TPDO_Target_Position = 0x00000000;
//	M2_6_TPDO_Profile_Acceleration = 0x00000000;
//	M2_6_TPDO_Profile_Deceleration = 0x00000000;
//	M2_6_TPDO_Profile_Velocity = 0x00000000;
//	sendPDOevent(d);
//	check_modop_change(0x01); //wait for position mode applied
}


void motor_control(char * command)
{
	//等待所有从站初始化完成
//	while (boot_success_mark != 1) {
//	}

	/*For now, self_test and stand_by
	 * only work for cobId 1,6,9
	 * self_test： 自检程序
	 * stand_by: 从自检状态切换到运行模式
	 */
//	self_test(CANOpenShellOD_Data);
//	stand_by(CANOpenShellOD_Data);
//	setState(CANOpenShellOD_Data, Operational);  ///////let master go operation JUST FOR TEST!!!!!
//	type(command,CANOpenShellOD_Data);



	type(command);

	printf("Finishing.\n");
	fflush(stdout);	//output printf immeditely, it's useful when muti-printf woking on one time.

	/* 停止时钟线程 */
	StopTimerLoop(&Exit);
	printf("Timer STOP DONE\n");
	fflush(stdout);

	/* 关闭CAN总线
	 * canClose函数不会返回，线程在函数中被申请中止
	 * 仅在realtime app中返回，TimerCleanup*/
	canClose(CANOpenShellOD_Data);

	init_fail: TimerCleanup();
	printf("TIMERCleanup\n");
	fflush(stdout);

}
