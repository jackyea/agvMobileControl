/*
This file is part of CanFestival, a library implementing CanOpen Stack.


Copyright (C): Edouard TISSERANT and Francis DUPIN

See COPYING file for copyrights details.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifdef USE_XENO
#define eprintf(...)
#else
#define eprintf(...) printf (__VA_ARGS__)
#endif
#include "canfestival.h"
#include"CANOpenShellMasterOD.h"

int  ExecuteCommand(char* Command);
int RotateWheel2Zero();
int RotateWheel2LeftRight();
int RotateWheel2SelfRotating();
int RotateAgv(char direc,char angle);
int WalkingDistanceFowardAndAdjustRoute(int,int,char,char,char,char);
int WalkingDistanceBackAndAdjustRoute(int,int,char,char,char,char);
int WalkingDistanceLeftAndAdjustRoute(int,int,char,char,char,char);
int WalkingDistanceRightAndAdjustRoute(int,int,char,char,char,char);
int AgvFoward(char fastslow,float slowpoint,char rfidTarget);
int AgvBack(char fastslow,float slowpoint,char rfidTarget);
int AgvLeft(char fastslow,float slowpoint,char rfidTarget);
int AgvRight(char fastslow,float slowpoint,char rfidTarget);
int AgvUpDown(char direction);
int AgvStop();
int AgvHitAvoid3Stop();
int AgvEmcyStop();
float string2float(char * command);
int WalkingPreperationSpeedSet(char fastslow,int dir,float slowpoint);
int WalkingWhileAssist(int Direc,int rfidnum[4],int hitavoid[3],float PointDistance[2],int AdjustNavi[2],INTEGER32 M2_position[24]);
int WalkingDistanceAndAdjustRoute(int Direc,int a,int b,int rfidnum[4],float PointDistance[2],int AdjustNavi[2],INTEGER32 M2_position[24]);
int CalLookingRFIDSpeed();
int SetWalkingPosition(float slowpoint);
int CalLookingRFIDStartPosition();
int CalLookingRFIDStopPosition(float LookingRFIDStopPosition);
int FixedWalkingPosition(float FixedStopPoint);
int WalkingFixPositon(int dir);
int SetRotatePosition(int time);
int LookingRFID(int dir);
int RotateWheelReturn(int RotateWheelReturnDirec);
int AgvCharge(char State);

/******行走电机参数 速度模式******/
/* 速度
 * 面对电机轴，暂规定：顺时针=前进=正值；逆时针=后退=负值
 * 如果电机默认相反，只需调整 顺时针=前进=负值  逆时针=后退=正值
 * 直行正常速度=轮速1 m/s = 转速 521519 counts/s = 0007F52F ：FFF80AD1
 * 直行慢速=正常速度 *1/4= 轮速0.25 m/s = 转速 130380 counts/s= 0001FD4C ：FFFE02B4 */
extern INTEGER32 Clockwise_WalkNomalSpeed;
extern INTEGER32 Clockwise_WalkSlowSpeed;

extern INTEGER32 Anticlockwise_WalkNomalSpeed;
extern INTEGER32 Anticlockwise_WalkSlowSpeed;

/*启动/停止*/
extern INTEGER16 WalkStart;
extern INTEGER16 WalkStop;
extern INTEGER16 WalkStopDisable;

/******行走电机参数 位置模式******/
/*行走电机距离参数*/
extern INTEGER32 WalkingPositive;
extern INTEGER32 WalkingNegative;

extern INTEGER32 WalkingPositiveFixed;
extern INTEGER32 WalkingNegativeFixed;

/*修正停止点距离，寻找RFID*/
extern float FixedFordwardStopPoint;
extern float FixedCrabStopPoint;

/*LookingRFIDCount用于限制寻找RFID函数只进入一次，初始=0，进入后=1，agvstop中置0*/
extern int LookingRFIDCount;

/*自旋度位置*/
extern INTEGER32 RotatePositionPositive;
extern INTEGER32 RotatePositionNegative;

/*行走过程中蔽障标帜位，用于避免每个while都发送bit4=0->1，每次运动前，将标帜位置0; AvoidFlag=0:初始状态，AvoidFlag=1蔽障未触发，=2蔽障外层触发 =3蔽障内层触发*/
extern int AvoidFlag;

extern INTEGER16 WalkStartPosition;
extern INTEGER16 WalkStopPosition;
extern INTEGER16 WalkStopPositionDisable; //行走位置模式断始能
extern INTEGER16 WalkBitfourZero;
/******转向电机参数 位置模式******/
/* 方向控制--通过Target Position 607A 位置数值的正负HEX；
 * 面对电机轴俯视，暂规定：电机顺时针旋转=位置数值为正; 逆时针旋转=位置数值为负
 * 如果电机默认相反，只需调整 使电机顺时针旋转时，位置数值=正 即可
 */

/*轮子与长边平行时，作为0位置*/
extern INTEGER32 Rotator_zero;

/* 横移位置=90°
 * 轮子旋转90°电机需要旋转90*25=2250°=6.25圈 * 4096= 25600 counts=0x64 00
 * 顺时针= 0x6400
 * 逆时针= 0xFFFF9C00
 * */
extern INTEGER32 Clockwise_Rotator_Crab;
extern INTEGER32 Anticlockwise_Rotator_Crab;

/* 自旋位置=45°
 *轮子旋转45°电机需要旋转45*25=1125°=3.125圈*4096=12800 counts=0x32 00
 *顺时针=0x3200;
 *逆时针=0xFFFFCE00
 * */
extern INTEGER32 Clockwise_Rotator_Self;
extern INTEGER32 Anticlockwise_Rotator_Self;

/*启动停止*/
extern INTEGER16 RotatorStart;
extern INTEGER16 RotatorDelay; //需测试：此处延迟的1S 是否会对接受信息造成影响
extern INTEGER16 RotatorStop;
extern INTEGER16 RotatorStopDisable;

/*限位开关*/
//	UNS8 limitswitch=0x0; //把OD限位开关的变量复制过来，防止读取+写入同时进行!
extern int limitswitch_rotator5_up;//旋转电机I-上限位开关
extern int limitswitch_rotator5_down;//旋转电机I-下限位开关

extern int limitswitch_rotator6_up;//旋转电机II-上限位开关
extern int limitswitch_rotator6_down;//旋转电机II-下限位开关

extern int limitswitch_rotator7_up;//旋转电机III-上限位开关
extern int limitswitch_rotator7_down;//旋转电机III-下限位开关

extern int limitswitch_rotator8_up;//旋转电机VI-上限位开关
extern int limitswitch_rotator8_down;//旋转电机VI-下限位开关


/******举升电机参数 位置模式******/
/* 方向控制--通过Target Position 607A 位置数值的正负HEX；
 * 面对电机轴俯视，暂规定：电机顺时针旋转（位置数值为正）：螺杆下降；逆时针旋转（位置数值为负）：螺杆上升
 * 如果电机默认相反，只需调整 使电机顺时针旋转时，位置信息为正即可
 *
 * 举升电机位于底部时，位置=0
 * 需举升40mm距离，用时 4.2S（不包含加减速过程）*/

/*位置：电机需要转182圈 = 2981888 counts = 002D8000*/
extern INTEGER32 LiftZeroPosition;
extern INTEGER32 LiftTargetPosition;

/*启动停止*/
extern INTEGER16 LiftStart;
extern INTEGER16 LiftDelay;//需测试：此处延迟的1S 是否会对接受信息造成影响
extern INTEGER16 LiftStop;

/******继电器输出******/
/* K1充电 		第0位=0x01;
 * K2控制电机 	第1位=0x02;
 * K4=绿色灯	 	第2位=0x04;
 * K5=黄色灯	 	第3位=0x08;
 * K6=红色灯		第4位=0x10;
 * 将0位置1：OUT8_Relay_bit1_8 |= 0x01
 * 将0位清0：OUT8_Relay_bit1_8 &= ~0x01*/
extern UNS8 K1;
extern UNS8 K2;
extern UNS8 K4;
extern UNS8 K5;
extern UNS8 K6;
extern UNS8 K9;
extern int ChargeFlag;//用于K1 K2互斥，作为K1闭合时的标志位
//闭合继电器 输出1
#define RelayClose(a) OUT8_Relay_bit1_8 |=a

//断开继电器 输出0
#define RelayOpen(b) OUT8_Relay_bit1_8 &=~b

//取反 c=需要取反的位bit=0-n
#define ReverseBit(c) OUT8_Relay_bit1_8 ^=(1<<c)

//位置模式电机，启动后 与 开始while检测是否到位之间需要一个0,5S左右延迟
extern int position_delay;

/*位置模式下bit4=0  0x08=1000
 * 将4位清0：M2_TPDO_control_word &= ~0x10
 * */
#define Bit4Zero(A) A &= ~0x10;

/*转向纠偏角度*/
extern INTEGER32 Clockwis_Degree1;
extern INTEGER32 Clockwis_Degree2;
extern INTEGER32 Clockwis_Degree3;
extern INTEGER32 Clockwis_Degree4;
extern INTEGER32 Clockwis_Degree5;

extern INTEGER32 AntiClockwis_NegetiveDegree1;
extern INTEGER32 AntiClockwis_NegetiveDegree2;
extern INTEGER32 AntiClockwis_NegetiveDegree3;
extern INTEGER32 AntiClockwis_NegetiveDegree4;
extern INTEGER32 AntiClockwis_NegetiveDegree5;

/*纠偏标志位*/
extern int AdjustFlag;


