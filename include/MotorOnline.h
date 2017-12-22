/*
 * MotorOnline.h
 *
 *  Created on: 2017-7-24
 *      Author: guochence
 */

#ifndef MOTORONLINE_H_
#define MOTORONLINE_H_



#endif /* MOTORONLINE_H_ */

	/*电机在线：此变量由 R1_Configuration && R1_Disconnected 得到，当R1_Online=1时，代表R1心跳连接正常+配置完成*/
	extern int R1_Online;
	extern int R2_Online;
	extern int R3_Online;
	extern int R4_Online;
	extern int R5_Online;
	extern int R6_Online;
	extern int R7_Online;
	extern int R8_Online;
	extern int R9_Online;
	/*IO在线*/
	extern int IO_Online;
	/*PGV在线*/
	extern int PGV_Online;

	/*声明电机初始化完成变量*/
	extern int R1_Configuration;
	extern int R2_Configuration;
	extern int R3_Configuration;
	extern int R4_Configuration;
	extern int R5_Configuration;
	extern int R6_Configuration;
	extern int R7_Configuration;
	extern int R8_Configuration;
	extern int R9_Configuration;
	/*声明IO站初始化完成变量*/
	extern int IO_Configuration;
	/*IO站初始化完成*/
	extern int PGV_Configuration;
	/*电机心跳正常*/
	extern int R1_Connected;
	extern int R2_Connected;
	extern int R3_Connected;
	extern int R4_Connected;
	extern int R5_Connected;
	extern int R6_Connected;
	extern int R7_Connected;
	extern int R8_Connected;
	extern int R9_Connected;
	/*IO站心跳正常*/
	extern int HeartbeatStartFlag;
