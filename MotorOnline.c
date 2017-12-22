/*
 * MotorOnline.c
 *
 *  Created on: 2017-7-24
 *      Author: guochence
 */

	/*电机上线标帜位，在slavebootup中置1*/
int R1_Online=0;
int R2_Online=0;
int R3_Online=0;
int R4_Online=0;
int R5_Online=0;
int R6_Online=0;
int R7_Online=0;
int R8_Online=0;
int R9_Online=0;
	/*IO在线*/
int IO_Online=0;

	/*PGV在线*/
int PGV_Online=0;

	/*电机初始化完成*/
int R1_Configuration=0;
int R2_Configuration=0;
int R3_Configuration=0;
int R4_Configuration=0;
int R5_Configuration=0;
int R6_Configuration=0;
int R7_Configuration=0;
int R8_Configuration=0;
int R9_Configuration=0;
	/*IO站初始化完成*/
int IO_Configuration=0;
/*IO站初始化完成*/
int PGV_Configuration=0;

	/*电机心跳正常*/
int R1_Connected=1;
int R2_Connected=1;
int R3_Connected=1;
int R4_Connected=1;
int R5_Connected=1;
int R6_Connected=1;
int R7_Connected=1;
int R8_Connected=1;
int R9_Connected=1;
	/*IO站心跳正常*/
int HeartbeatStartFlag=0; //心跳开始检测标志位，当初始化完成，绿灯亮起，则心跳报文检测开始工作。掉线时调用AGVStop()；1=IO站，2=IO+电机
