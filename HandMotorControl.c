/*
 * HandMotorControl.c
 *手动模式下的电机控制函数，当程序接收到相应的开关量后，执行相应的程序
 *  Created on: Jun 21, 2017
 *      Author: ye
 */
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "MotorControl.h"
#include "ExchangingInfoStruct.h"
#include"CANOpenShellMasterOD.h"
#include "CanOpenShell.h"  //引用 extern CO_Data* CANOpenShellOD_Data;
#include"Log.h"
#include "InitAndSelfTest.h"//获取举升电机初始位置

extern struct StructSenser senserstruct;
extern struct StructInner innerstruct;
extern struct StructAgvReport agvreportstruct;

int SlefRotatePosition=0; //用于判断是否到达自旋位置 防止卡坏电机
int CrabPosition=0; //横移位置标志位
int ZeroPosition=0;//0位置标志位

/*小车四个轮子有三个位置，一是零位，即与小车长边平行，此时轮子可以向顶位转动，去和宽边平行，也可以向自旋位转动，到位后小车可以自旋，
 * 二是顶位，即与小车宽边平行，此时轮子接近90°限位开关，只能向零位或自旋位转动。三是自旋位，此时轮子接近另一个限位开关，只能向零位转动。
 * 轮子的三个位置要断电保存，所以每当轮子转换位置成功后要将其最新位置写入文件，设零位标志为0,顶位标志为1,自旋位为2，agv上电时轮子调到零位，
 * 上位机在发送直行、自旋等命令时，根据轮子的当前位置和行进方向确定是否需要调整轮子的位置.
 */
int HandRotatorWheelCrab()//横移位置Remoter10_Rotator_Crab
{
	/*将BIT4=0 避免纠偏未把bit4置0*/
//	M2_TPDO_control_word=Bit4Zero(M2_TPDO_control_word); //bit 4=0
	M2_TPDO_control_word=(M2_TPDO_control_word&(~0x10));//bit 4=0
	sendPDOevent(CANOpenShellOD_Data);

	//向四个电机发送转到左移右移位命令,此命令为位置命令，命令中包含initposition+motorcode
	/*1.转向电机：横移位置
	 *旋转电机1 3=逆时针=0XFF FF 9C 00 (noidID 5 7)
	 *旋转电机2 4=顺时针=0X64 00 (noidID 6 8)*/
	M2_5_TPDO_Target_Position=Anticlockwise_Rotator_Crab;
	M2_7_TPDO_Target_Position=Anticlockwise_Rotator_Crab;

	M2_6_TPDO_Target_Position=Clockwise_Rotator_Crab;
	M2_8_TPDO_Target_Position=Clockwise_Rotator_Crab;
	sendPDOevent(CANOpenShellOD_Data);

	//灯全灭，表示命令正在运行
	RelayOpen(K5);//灭黄灯K5，表示命令正在运行
	RelayOpen(K4);
	RelayOpen(K6);

	/*启动*/
	M2_TPDO_control_word = RotatorStart;
	sendPDOevent(CANOpenShellOD_Data);

	int delaycount=0;
	while(delaycount<=position_delay) //delay 0.5秒，等待电机开始运行再开始检测
	{
		delaycount++;
	} //100000000大概1.2s左右；300000000=4.5s左右

	while(agvreportstruct.MotorError==0)
	{
		//问题：直接使用IN5_Limit_Switch_bit1_8用于判断，判断时，是否可被写入？？是否冲突？？！
		//旋转电机I
		limitswitch_rotator5_up =(IN5_Limit_Switch_bit1_8 & (1<<0)) ? 1 : 0; //判断bit0=1，则limitswitch_rotator5_up=1；若bit0=0，则limitswitch_rotator5_up=0
		limitswitch_rotator5_down =(IN5_Limit_Switch_bit1_8 & (1<<1)) ? 1 : 0; //判断bit1=1，则limitswitch_rotator5_down=1；若bit1=0，则limitswitch_rotator5_down=0
		//旋转电机II
		limitswitch_rotator6_up =(IN5_Limit_Switch_bit1_8 & (1<<2)) ? 1 : 0; //判断bit2=1，则limitswitch_rotator6_up=1；若bit2=0，则limitswitch_rotator6_up=0
		limitswitch_rotator6_down =(IN5_Limit_Switch_bit1_8 & (1<<3)) ? 1 : 0; //判断bit3=1，则limitswitch_rotator6_down=1；若bit3=0，则limitswitch_rotator6_down=0
		//旋转电机III
		limitswitch_rotator7_up =(IN5_Limit_Switch_bit1_8 & (1<<6)) ? 1 : 0; //判断bit6=1，则limitswitch_rotator7_up=1；若bit6=0，则limitswitch_rotator7_up=0
		limitswitch_rotator7_down =(IN5_Limit_Switch_bit1_8 & (1<<7)) ? 1 : 0; //判断bit7=1，则limitswitch_rotator7_down=1；若bit7=0，则limitswitch_rotator7_down=0
		//旋转电机IV --IN5_Limit_Switch_bit9_10
		limitswitch_rotator8_up =(IN5_Limit_Switch_bit9_10 & (1<<0)) ? 1 : 0; //判断bit0=1，则limitswitch_rotator8_up=1；若bit0=0，则limitswitch_rotator8_up=0
		limitswitch_rotator8_down =(IN5_Limit_Switch_bit9_10 & (1<<1)) ? 1 : 0; //判断bit1=1，则limitswitch_rotator8_down=1；若bit1=0，则limitswitch_rotator8_down=0

		/*判断旋转电机是否到达目标位置*/
		if(((R5_statusword & (1<<(10)))!=0) &&
		   ((R6_statusword & (1<<(10)))!=0) &&
		   ((R7_statusword & (1<<(10)))!=0) &&
		   ((R8_statusword & (1<<(10)))!=0) )//如果四个旋转电机都已转到与宽边平行位且停止转动
		{
			agvreportstruct.WheelDirection=1;//告诉上位机当前位置=1
			RelayClose(K5);//点亮黄灯K5，表示命令完成
			RelayOpen(K4);
			RelayOpen(K6);
			M2_TPDO_control_word=RotatorStop;//修改bit4=0->1 !！！是否可以加到此处？！！！需测试！
			sendPDOevent(CANOpenShellOD_Data);
			return 0;//命令执行成功，返回正常值
		}

		/*判断限位开关*/
		if(limitswitch_rotator5_up==1||limitswitch_rotator5_down==1 ||
		   limitswitch_rotator6_up==1||limitswitch_rotator6_down==1 ||
		   limitswitch_rotator7_up==1||limitswitch_rotator7_down==1 ||
		   limitswitch_rotator8_up==1||limitswitch_rotator8_down==1)//旋转轮子 碰触限位开关，停止电机！发生错误！
		{
			Log("遥控器旋转电机的转至横移位置时限位开关被触发");
			printf("遥控器旋转电机的转至横移位置时限位开关被触发\n");
			return -1;
		}
	}
	Log("遥控器旋转电机的转至横移位置时收到EMCY");
	printf("遥控器旋转电机的转至横移位置时收到EMCY\n");
	return -1;

	}

int HandRotateWheelRotate()//轮子转向自旋方向
{
	/*将BIT4=0 避免纠偏未把bit4置0*/
//	M2_TPDO_control_word=Bit4Zero(M2_TPDO_control_word); //bit 4=0
	M2_TPDO_control_word=(M2_TPDO_control_word&(~0x10));//bit 4=0
	sendPDOevent(CANOpenShellOD_Data);

	//向四个电机发送转到自旋位命令，此命令为位置命令，命令中包含initposition+motorcode
	/*1.转向电机：自旋位置
	 *旋转电机1 3=逆时针=0xFFFFB787 (noidID 5 7)
	 *旋转电机2 4=顺时针=0x4879 (noidID 6 8)*/
	M2_5_TPDO_Target_Position=Anticlockwise_Rotator_Self;
	M2_7_TPDO_Target_Position=Anticlockwise_Rotator_Self;

	M2_6_TPDO_Target_Position=Clockwise_Rotator_Self;
	M2_8_TPDO_Target_Position=Clockwise_Rotator_Self;
	sendPDOevent(CANOpenShellOD_Data);

	//灯全灭，表示命令正在运行
	RelayOpen(K5);//灭黄灯K5，表示命令正在运行
	RelayOpen(K4);
	RelayOpen(K6);

	/*启动*/
	M2_TPDO_control_word = RotatorStart;
	sendPDOevent(CANOpenShellOD_Data);

	int delaycount=0;
	while(delaycount<=position_delay) //delay 0.5秒，等待电机开始运行再开始检测
	{
		delaycount++;
	} //100000000大概1.2s左右；300000000=4.5s左右

	while(agvreportstruct.MotorError==0)
	{
		//问题：直接使用IN5_Limit_Switch_bit1_8用于判断，判断时，是否可被写入？？是否冲突？？！
		//旋转电机I
		limitswitch_rotator5_up =(IN5_Limit_Switch_bit1_8 & (1<<0)) ? 1 : 0; //判断bit0=1，则limitswitch_rotator5_up=1；若bit0=0，则limitswitch_rotator5_up=0
		limitswitch_rotator5_down =(IN5_Limit_Switch_bit1_8 & (1<<1)) ? 1 : 0; //判断bit1=1，则limitswitch_rotator5_down=1；若bit1=0，则limitswitch_rotator5_down=0
		//旋转电机II
		limitswitch_rotator6_up =(IN5_Limit_Switch_bit1_8 & (1<<2)) ? 1 : 0; //判断bit2=1，则limitswitch_rotator6_up=1；若bit2=0，则limitswitch_rotator6_up=0
		limitswitch_rotator6_down =(IN5_Limit_Switch_bit1_8 & (1<<3)) ? 1 : 0; //判断bit3=1，则limitswitch_rotator6_down=1；若bit3=0，则limitswitch_rotator6_down=0
		//旋转电机III
		limitswitch_rotator7_up =(IN5_Limit_Switch_bit1_8 & (1<<6)) ? 1 : 0; //判断bit6=1，则limitswitch_rotator7_up=1；若bit6=0，则limitswitch_rotator7_up=0
		limitswitch_rotator7_down =(IN5_Limit_Switch_bit1_8 & (1<<7)) ? 1 : 0; //判断bit7=1，则limitswitch_rotator7_down=1；若bit7=0，则limitswitch_rotator7_down=0
		//旋转电机IV --IN5_Limit_Switch_bit9_10
		limitswitch_rotator8_up =(IN5_Limit_Switch_bit9_10 & (1<<0)) ? 1 : 0; //判断bit0=1，则limitswitch_rotator8_up=1；若bit0=0，则limitswitch_rotator8_up=0
		limitswitch_rotator8_down =(IN5_Limit_Switch_bit9_10 & (1<<1)) ? 1 : 0; //判断bit1=1，则limitswitch_rotator8_down=1；若bit1=0，则limitswitch_rotator8_down=0

		/*判断旋转电机是否到达目标位置*/
		if(((R5_statusword & (1<<(10)))!=0) &&
		   ((R6_statusword & (1<<(10)))!=0) &&
		   ((R7_statusword & (1<<(10)))!=0) &&
		   ((R8_statusword & (1<<(10)))!=0) )//如果四个旋转电机都已转到与宽边平行位且停止转动
		{
			agvreportstruct.WheelDirection=2;//告诉上位机当前位置=2
			RelayClose(K5);//点亮黄灯K5，表示命令完成
			RelayOpen(K4);
			RelayOpen(K6);
			M2_TPDO_control_word=RotatorStop;//修改bit4=0
			sendPDOevent(CANOpenShellOD_Data);
			return 0;//命令执行成功，返回正常值
		}

		/*判断限位开关*/
		if(limitswitch_rotator5_up==1||limitswitch_rotator5_down==1 ||
		   limitswitch_rotator6_up==1||limitswitch_rotator6_down==1 ||
		   limitswitch_rotator7_up==1||limitswitch_rotator7_down==1 ||
		   limitswitch_rotator8_up==1||limitswitch_rotator8_down==1)//旋转轮子 碰触限位开关，停止电机！发生错误！
		{
			Log("遥控器旋转电机至自旋位置时限位开关被触发");
			printf("遥控器旋转电机至自旋位置时限位开关被触发\n");
			return -1;
		}
	}
	Log("遥控器旋转电机至自旋位置时收到EMCY");
	printf("遥控器旋转电机至自旋位置时收到EMCY\n");
	return -1;
}

int HandRotateWheelZero()//轮子回到标准方向,注意回到标准方向时正常情况下没有限位开关，如果电机出现故障不停的话，会碰到另一侧的限位开关
{
	/*将BIT4=0 避免纠偏未把bit4置0*/
//	M2_TPDO_control_word=Bit4Zero(M2_TPDO_control_word); //bit 4=0
	M2_TPDO_control_word=(M2_TPDO_control_word&(~0x10));//bit 4=0
	sendPDOevent(CANOpenShellOD_Data);

	//第一次发送 现将position改为不是0，再发送0位置，避免0位置PDO与主站上线自动发送的0信息重复放弃发送
	M2_5_TPDO_Target_Position=0x00000001;
	M2_6_TPDO_Target_Position=0x00000001;
	M2_7_TPDO_Target_Position=0x00000001;
	M2_8_TPDO_Target_Position=0x00000001;
	sendPDOevent(CANOpenShellOD_Data);
	//向四个电机发送回零位命令
	M2_5_TPDO_Target_Position=Rotator_zero;
	M2_6_TPDO_Target_Position=Rotator_zero;
	M2_7_TPDO_Target_Position=Rotator_zero;
	M2_8_TPDO_Target_Position=Rotator_zero;
	sendPDOevent(CANOpenShellOD_Data);
	RelayOpen(K5);//灭黄灯K5，表示命令正在运行
	RelayOpen(K4);
	RelayOpen(K6);
	/*启动*/
	M2_TPDO_control_word = RotatorStart;
	sendPDOevent(CANOpenShellOD_Data);

	int delaycount=0;
	while(delaycount<=position_delay) //delay 0.5秒，等待电机开始运行再开始检测
	{
		delaycount++;
	} //100000000大概1.2s左右；300000000=4.5s左右

	while(agvreportstruct.MotorError==0)
	{
		//问题：直接使用IN5_Limit_Switch_bit1_8用于判断，判断时，是否可被写入？？是否冲突？？！
		//旋转电机I
		limitswitch_rotator5_up =(IN5_Limit_Switch_bit1_8 & (1<<0)) ? 1 : 0; //判断bit0=1，则limitswitch_rotator5_up=1；若bit0=0，则limitswitch_rotator5_up=0
		limitswitch_rotator5_down =(IN5_Limit_Switch_bit1_8 & (1<<1)) ? 1 : 0; //判断bit1=1，则limitswitch_rotator5_down=1；若bit1=0，则limitswitch_rotator5_down=0
		//旋转电机II
		limitswitch_rotator6_up =(IN5_Limit_Switch_bit1_8 & (1<<2)) ? 1 : 0; //判断bit2=1，则limitswitch_rotator6_up=1；若bit2=0，则limitswitch_rotator6_up=0
		limitswitch_rotator6_down =(IN5_Limit_Switch_bit1_8 & (1<<3)) ? 1 : 0; //判断bit3=1，则limitswitch_rotator6_down=1；若bit3=0，则limitswitch_rotator6_down=0
		//旋转电机III
		limitswitch_rotator7_up =(IN5_Limit_Switch_bit1_8 & (1<<6)) ? 1 : 0; //判断bit6=1，则limitswitch_rotator7_up=1；若bit6=0，则limitswitch_rotator7_up=0
		limitswitch_rotator7_down =(IN5_Limit_Switch_bit1_8 & (1<<7)) ? 1 : 0; //判断bit7=1，则limitswitch_rotator7_down=1；若bit7=0，则limitswitch_rotator7_down=0
		//旋转电机IV --IN5_Limit_Switch_bit9_10
		limitswitch_rotator8_up =(IN5_Limit_Switch_bit9_10 & (1<<0)) ? 1 : 0; //判断bit0=1，则limitswitch_rotator8_up=1；若bit0=0，则limitswitch_rotator8_up=0
		limitswitch_rotator8_down =(IN5_Limit_Switch_bit9_10 & (1<<1)) ? 1 : 0; //判断bit1=1，则limitswitch_rotator8_down=1；若bit1=0，则limitswitch_rotator8_down=0

		/*判断电机是否到达目标位置*/
		if(((R5_statusword & (1<<(10)))!=0) &&
		   ((R6_statusword & (1<<(10)))!=0) &&
		   ((R7_statusword & (1<<(10)))!=0) &&
		   ((R8_statusword & (1<<(10)))!=0) )//如果四个旋转电机都已转到零位且停止转动
		{
			agvreportstruct.WheelDirection=0;//告诉上位机当前位置=0
			RelayClose(K5);//点亮黄灯K5，表示命令完成
			RelayOpen(K4);
			RelayOpen(K6);
			M2_TPDO_control_word=RotatorStop;//修改bit4=0
			sendPDOevent(CANOpenShellOD_Data);
			return 0;//命令执行成功，返回正常值
		}

		/*判断限位开关*/
		if(limitswitch_rotator5_up==1||limitswitch_rotator5_down==1 ||
		   limitswitch_rotator6_up==1||limitswitch_rotator6_down==1 ||
		   limitswitch_rotator7_up==1||limitswitch_rotator7_down==1 ||
		   limitswitch_rotator8_up==1||limitswitch_rotator8_down==1 )//取出5-8位的LimitSwitch数据，判断是否大于0
		{
			Log("遥控器旋转电机至0位置时限位开关被触发");
			printf("遥控器旋转电机至0位置时限位开关被触发\n");
			return -1;
		}
	}
	Log("遥控器旋转电机至0位置时收到EMCY");
	printf("遥控器旋转电机至0位置时收到EMCY\n");
	return -1;
}


//agv左旋程序，手持终端左旋按钮按下后，此程序被调用。被调用之前，轮子必须已调整至自旋状态，并成功执行。左旋信号不消失，一直以固定速度左旋
int HandAgvRotateLeft()
{
	//AGV旋转时，以Slow慢速旋转
	/* AGV逆时针旋转
	 * 行走电机1 4 =前进  1=顺时针 4=逆时针；
	 * 行走电机2 3 =后退	2=顺时针 3=逆时针
	 * 左侧轮后退；右侧轮前进*/
	M1_1_TPDO_target_speed = Clockwise_WalkSlowSpeed;
	M1_2_TPDO_target_speed = Clockwise_WalkSlowSpeed;

	M1_3_TPDO_target_speed = Anticlockwise_WalkSlowSpeed;
	M1_4_TPDO_target_speed = Anticlockwise_WalkSlowSpeed;
	sendPDOevent(CANOpenShellOD_Data);

	/*启动*/
	M1_TPDO_control_word = WalkStart;
	sendPDOevent(CANOpenShellOD_Data);
	return 0;
}

//agv右旋程序，手持终端右旋按钮按下后，此程序被调用。被调用之前，轮子必须已调整至自旋状态，并成功执行。右旋信号不消失，一直以固定速度右旋
int HandAgvRotateRight()
{
	//AGV旋转时，以Slow慢速旋转
	/* AGV顺时针旋转：
	 * 行走电机1 4 =后退  1=逆时针 4=顺时针；
	 * 行走电机2 3=前进	2=逆时针 3=顺时针
	 * (左侧轮前进；右侧轮后退)*/
	M1_1_TPDO_target_speed = Anticlockwise_WalkSlowSpeed;
	M1_2_TPDO_target_speed = Anticlockwise_WalkSlowSpeed;

	M1_3_TPDO_target_speed = Clockwise_WalkSlowSpeed;
	M1_4_TPDO_target_speed = Clockwise_WalkSlowSpeed;
	sendPDOevent(CANOpenShellOD_Data);

	/*启动*/
	M1_TPDO_control_word = WalkStart;
	sendPDOevent(CANOpenShellOD_Data);
	return 0;
}


//下面是电机前行程序，手持终端前行按钮按下后，此程序被调用。被调用之前，轮子必须已调整至与前行方向平行状态，并成功执行。前行信号不消失，一直以固定速度前行
int HandAgvFoward()
{	//SLOW速度
	//给四个直行电机发送前进命令
	/*前进 行走电机：1 3是顺时针，2 4是逆时针*/
	M1_1_TPDO_target_speed = Clockwise_WalkSlowSpeed;
	M1_3_TPDO_target_speed = Clockwise_WalkSlowSpeed;

	M1_2_TPDO_target_speed = Anticlockwise_WalkSlowSpeed;
	M1_4_TPDO_target_speed = Anticlockwise_WalkSlowSpeed;
//	M1_1_TPDO_target_speed = Clockwise_WalkNomalSpeed;
//	M1_3_TPDO_target_speed = Clockwise_WalkNomalSpeed;
//
//	M1_2_TPDO_target_speed = Anticlockwise_WalkNomalSpeed;
//	M1_4_TPDO_target_speed = Anticlockwise_WalkNomalSpeed;
	sendPDOevent(CANOpenShellOD_Data);

	/*启动*/
	M1_TPDO_control_word = WalkStart;
	sendPDOevent(CANOpenShellOD_Data);
	return 0;
}


//下面是电机后退程序，手持终端后退按钮按下后，此程序被调用。被调用之前，轮子必须已调整至与后退方向平行状态，并成功执行。后退信号不消失，一直以固定速度后退
int HandAgvBack()
{	//给四个直行电机发送后退命令
	/*后退 行走电机：1 3是逆时针，2 4是顺时针*/
	M1_1_TPDO_target_speed = Anticlockwise_WalkSlowSpeed;
	M1_3_TPDO_target_speed = Anticlockwise_WalkSlowSpeed;

	M1_2_TPDO_target_speed = Clockwise_WalkSlowSpeed;
	M1_4_TPDO_target_speed = Clockwise_WalkSlowSpeed;

//	M1_1_TPDO_target_speed = Anticlockwise_WalkNomalSpeed;
//	M1_3_TPDO_target_speed = Anticlockwise_WalkNomalSpeed;
//
//	M1_2_TPDO_target_speed = Clockwise_WalkNomalSpeed;
//	M1_4_TPDO_target_speed = Clockwise_WalkNomalSpeed;
	sendPDOevent(CANOpenShellOD_Data);

	/*启动*/
	M1_TPDO_control_word = WalkStart;
	sendPDOevent(CANOpenShellOD_Data);

	return 0;
}


//下面是电机左移程序，手持终端左移按钮按下后，此程序被调用。被调用之前，轮子必须已调整至与左移方向平行状态，并成功执行。左移信号不消失，一直以固定速度左移
int HandAgvLeft()
{
	/* 行走电机 向左横移：
	 * 行走电机1 3=前进	1=顺时针 3=顺时针
	 * 行走电机2 4=后退	2=顺时针 4=顺时针*/
	M1_1_TPDO_target_speed=Clockwise_WalkSlowSpeed;
	M1_3_TPDO_target_speed=Clockwise_WalkSlowSpeed;

	M1_2_TPDO_target_speed=Clockwise_WalkSlowSpeed;
	M1_4_TPDO_target_speed=Clockwise_WalkSlowSpeed;
//	M1_1_TPDO_target_speed=Clockwise_WalkNomalSpeed;
//	M1_3_TPDO_target_speed=Clockwise_WalkNomalSpeed;
//
//	M1_2_TPDO_target_speed=Clockwise_WalkNomalSpeed;
//	M1_4_TPDO_target_speed=Clockwise_WalkNomalSpeed;

	/*启动*/
	M1_TPDO_control_word = WalkStart;
	sendPDOevent(CANOpenShellOD_Data);

	return 0;
}


//下面是电机右移程序，手持终端右移按钮按下后，此程序被调用。被调用之前，轮子必须已调整至与右移方向平行状态，并成功执行。右移信号不消失，一直以固定速度右移
int HandAgvRight()
{
	/* 行走电机 向右横移：
	 * 行走电机1 3=后退	1=逆时针 3=逆时针
	 * 行走电机2 4=前进	2=逆时针 4=逆时针*/
	M1_1_TPDO_target_speed=Anticlockwise_WalkSlowSpeed;
	M1_3_TPDO_target_speed=Anticlockwise_WalkSlowSpeed;

	M1_2_TPDO_target_speed=Anticlockwise_WalkSlowSpeed ;
	M1_4_TPDO_target_speed=Anticlockwise_WalkSlowSpeed ;
//	M1_1_TPDO_target_speed=Anticlockwise_WalkNomalSpeed;
//	M1_3_TPDO_target_speed=Anticlockwise_WalkNomalSpeed;
//
//	M1_2_TPDO_target_speed=Anticlockwise_WalkNomalSpeed ;
//	M1_4_TPDO_target_speed=Anticlockwise_WalkNomalSpeed ;

	/*启动*/
	M1_TPDO_control_word = WalkStart;
	sendPDOevent(CANOpenShellOD_Data);

	return 0;
}



//下面是电机举升下降程序，手持终端举升按钮按下后，此程序被调用。
int HandAgvUp()
{
	int limitswitch_up=0;//上限位开关
	int limitswitch_down=0;//下限位开关

	RelayOpen(K5);//灭黄灯K5，表示命令正在运行
	RelayOpen(K4);
	RelayOpen(K6);
	M3_TPDO_Target_Position=(LiftTargetPosition+UpMotorInitPosition);//将位置信息传给OD
	sendPDOevent(CANOpenShellOD_Data);//发送PDO
	M3_TPDO_control_word=LiftStart;//启动电机
	sendPDOevent(CANOpenShellOD_Data);

	int delaycount=0;
	while(delaycount<=position_delay) //delay 0.5秒，等待电机开始运行再开始检测
	{
		delaycount++;
	} //100000000大概1.2s左右；300000000=4.5s左右

	while(agvreportstruct.MotorError==0)
	{
		//问题：直接使用IN5_Limit_Switch_bit1_8用于判断，判断时，是否可被写入？？是否冲突？？！
		//取出举升上限位开关=bit4
		limitswitch_up =(IN5_Limit_Switch_bit1_8 & (1<<4)) ? 1 : 0; //判断bit4=1，则limitswitch_up=1；若bit4=0，则limitswitch_up=0

		//取出举升下限位开关=bit5
		limitswitch_down =(IN5_Limit_Switch_bit1_8 & (1<<5)) ? 1 : 0; //判断bit5=1，则limitswitch_up=1；若bit5=0，则limitswitch_up=0

		/*判断电机是否到达目标位置*/
		if((R9_statusword & (1<<(10)))!=0)//电机状态显示执行完毕,检测6041 bit10，到达位置时bit10=1；直到下次运动开始时，bit10自动清零
		{
			/*R9_statusword与bit10=1取“与”，得到R9的bit10位 其余位=0；结果只能判断是否==0
			 * 若R9到达位置，则bit10=1，则"1!=0",if成立，跳出while
			 * 若R9未到达位置，则bit10=0，，则“0!=0”不成立，继续while循环*/
			RelayClose(K5);//点亮黄灯K5，表示命令完成
			RelayOpen(K4);
			RelayOpen(K6);
			M3_TPDO_control_word=LiftStop;//修改bit4=0
			sendPDOevent(CANOpenShellOD_Data);
			return 0;
		}

		if(limitswitch_up==1||limitswitch_down==1)//上或下限位开关触发！
		{
			Log("遥控器-举升时限位开关触发");
			printf("遥控器-举升时限位开关触发\n");
			return -1;
		}
	}
	Log("遥控器-举升时收到EMCY");
	printf("遥控器-举升时收到EMCY\n");
	return -1;
}


//下面是电机下降程序，手持终端下降按钮按下后，此程序被调用。
int HandAgvDown()
{
	int limitswitch_up=0;//上限位开关
	int limitswitch_down=0;//下限位开关

	RelayOpen(K5);//灭黄灯K5，表示命令正在运行；
	RelayOpen(K4);
	RelayOpen(K6);

	//先发送一个不为0的值，以防第一次发送0与主站上线时遍历发送的0重叠导致放弃发送
	M3_TPDO_Target_Position=(0x00000001+UpMotorInitPosition);
	sendPDOevent(CANOpenShellOD_Data);//发送PDO

	M3_TPDO_Target_Position=(LiftZeroPosition+UpMotorInitPosition);//将位置信息传给OD
	sendPDOevent(CANOpenShellOD_Data);//发送PDO
	M3_TPDO_control_word=LiftStart;//启动电机
	sendPDOevent(CANOpenShellOD_Data);

	int delaycount=0;
	while(delaycount<=position_delay) //delay 0.5秒，等待电机开始运行再开始检测
	{
		delaycount++;
	} //100000000大概1.2s左右；300000000=4.5s左右

	while(agvreportstruct.MotorError==0)
	{
		//问题：直接使用IN5_Limit_Switch_bit1_8用于判断，判断时，是否可被写入？？是否冲突？？！
		//取出举升上限位开关=bit4
		limitswitch_up =(IN5_Limit_Switch_bit1_8 & (1<<4)) ? 1 : 0; //判断bit4=1，则limitswitch_up=1；若bit4=0，则limitswitch_up=0

		//取出举升下限位开关=bit5
		limitswitch_down =(IN5_Limit_Switch_bit1_8 & (1<<5)) ? 1 : 0; //判断bit5=1，则limitswitch_up=1；若bit5=0，则limitswitch_up=0

		/*判断电机是否到达目标位置*/
		if((R9_statusword & (1<<(10)))!=0)//电机状态显示执行完毕,检测6041 bit10，到达位置时bit10=1；直到下次运动开始时，bit10自动清零
		{
			/*R9_statusword与bit10=1取“与”，得到R9的bit10位 其余位=0；结果只能判断是否==0
			 * 若R9到达位置，则bit10=1，则"1!=0",if成立，跳出while
			 * 若R9未到达位置，则bit10=0，，则“0!=0”不成立，继续while循环*/
			RelayClose(K5);//点亮黄灯K5，表示命令完成
			RelayOpen(K4);
			RelayOpen(K6);
			M3_TPDO_control_word=LiftStop;//修改bit4=0
			sendPDOevent(CANOpenShellOD_Data);
			return 0;
		}

		if(limitswitch_up==1||limitswitch_down==1)//上或下限位开关触发！
		{
			Log("遥控器-下降时限位开关触发");
			printf("遥控器-下降时限位开关触发\n");
			return -1;
		}
	}
	Log("遥控器-下降时收到EMCY");
	printf("遥控器-下降时收到EMCY\n");
	return -1;
}

int HandAgvUpOrDown()
{
	if(R9_position<=((LiftTargetPosition+UpMotorInitPosition)/2)) //若举升杆在中间位置以下，则上升
	{	//上升
		if(HandAgvUp()==-1)
		{
			return -1; //举升出错
		}
		Log("遥控器-举升完成");
		printf("遥控器-举升完成\n");
	}
	else//若举升杆在中间位置以上，则下降
	{	//下降
		if(HandAgvDown()==-1)
		{
			return -1; //下降出错
		}
		Log("遥控器-下降完成");
		printf("遥控器-下降完成\n");
	}
	return 0;
}



int HandAgvErrorStop()
{	//AGV内部限位开关 电机反馈错误信息 断线 紧急停止！！！  最好能退出程序  需要追踪 reurn-1后到哪
	//AGV内部限位开关 电机反馈错误信息 断线 紧急停止！！！  最好能退出程序  需要追踪 reurn-1后到哪

	agvreportstruct.MotorError=1;//告诉上位机，电机发生故障
	innerstruct.StopCommand=1;

	//给九个电机发送停止命令//其实最主要的是四个直行电机可能在运行中被终止或中止，
	//四个旋转电机和一个升降电机在运行过程中一般不会遇到停止信号，但为了保险起见，向所有电机都发停止命令也不会造成不良影响。

	//红灯点亮 闭合K6=红灯+蜂鸣器； 绿灯熄灭 断开K4
//	RelayClose(K6);
//	RelayOpen(K4);
//	RelayOpen(K5);
	//	RelayClose(K6);
		RelayOpen(K6);

		RelayOpen(K4);

	//	RelayOpen(K5);
		RelayClose(K5); //yellow close

	//断开K2需要断开K9 K9为常闭，IO站高电平RelayClose(K9)断开K9，低电平闭合K9。触发EMCY后，给K9发高电平断开K9并自动闭合K2。 再次重置IO站后闭合K9。
	RelayClose(K9);
	sendPDOevent(CANOpenShellOD_Data);

	printf("AgvEmcyStop 被触发！！！！\n");
	Log("AgvEmcyStop 被触发！！！！");

	/*以防万一还是发吧！直接断电机供电，也不需要发送停止命令了*/
	/*停止4个行走电机*/
	M1_TPDO_control_word = WalkStop;
	/*停止4个旋转电机*/
	M2_TPDO_control_word = RotatorStop;
	/*停止举升电机*/
	M3_TPDO_control_word = LiftStop;
	sendPDOevent(CANOpenShellOD_Data);

	return -1;

}

int HandAgvStopVelocity() //用于遥控器模式速度模式下的断使能
{
	/*停止4个行走电机*/
	M1_TPDO_control_word = WalkStopDisable; //断行走始能

	/*停止4个旋转电机*/
	M2_TPDO_control_word = RotatorStopDisable; //断时能

	/*停止举升电机*/
	M3_TPDO_control_word = LiftStop; //断使能

	sendPDOevent(CANOpenShellOD_Data);

	return 0;
}
