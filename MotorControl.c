/*
 * MotorControl.c
 *
 *  Created on: Jun 21, 2017
 *      Author: ye
 */
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include<math.h>
#include "MotorControl.h"
#include "ExchangingInfoStruct.h"
#include"CANOpenShellMasterOD.h"
#include "CanOpenShell.h"  //引用 extern CO_Data* CANOpenShellOD_Data;
#include"Log.h"
#include "InitAndSelfTest.h"//获取举升电机初始位置

/*声明结构体数组 agvreportstruct innerstruct， 定义在InitAndSelfTest.c中*/
extern struct StructInner innerstruct;
extern struct StructAgvReport agvreportstruct;

int delaytimecount=20000000;
/*转向纠偏角度*/
INTEGER32 Clockwis_Degree1=0x0000238E;
INTEGER32 Clockwis_Degree2=0x0000471C;
INTEGER32 Clockwis_Degree3=0x00006AAB;
INTEGER32 Clockwis_Degree4=0x00008E39;
INTEGER32 Clockwis_Degree5=0x0000B1C7;

INTEGER32 AntiClockwis_Degree1=0xFFFFDC72;
INTEGER32 AntiClockwis_Degree2=0xFFFFB8E4;
INTEGER32 AntiClockwis_Degree3=0xFFFF9555;
INTEGER32 AntiClockwis_Degree4=0xFFFF71C7;
INTEGER32 AntiClockwis_Degree5=0xFFFF4E39;

/*纠偏标志位*/
int AdjustFlag=0;
/*行走时用于计算距离的轮子周长*/
float CRight=0.6473389;
float CForward=0.6473389;
float CLeft=0.6473389;
float CBack=0.6473389;
/*前后轮交替纠偏计数器*/
int AdjustCount=0; //每次while循环++，并在while外面=0
int AdjustBackFlag=30; //除以AdjustBackFlag 不能整除前轮纠偏，能整除后轮纠偏,并在while外面=0

/*充电标志位*/
int ChargeFlag=0;//用于K1 K2互斥，作为K1闭合时的标志位

/*纠偏持续时间*/
int Adjust=100000000;//100000000大概1.2s左右；300000000=4.5s左右

/*位置模式电机，启动后 与 开始while检测是否到位之间需要一个0,5S左右延迟*/
int position_delay=50000000; //100000000大概1.2s左右；300000000=4.5s左右

/******行走电机参数 速度模式******/
/* 速度
 * 面对电机轴，暂规定：顺时针=前进=正值；逆时针=后退=负值
 * 如果电机默认相反，只需调整 顺时针=前进=负值  逆时针=后退=正值
 * 直行正常速度=轮速1 m/s = 转速 5215189 counts/s
 * 直行慢速=正常速度 *1/4= 轮速0.25 m/s = 转速 1303797 counts/s */
	/*1m/s正常速度*/
//INTEGER32 Clockwise_WalkNomalSpeed=0x004F93D5; //1910rpm
//INTEGER32 Clockwise_WalkSlowSpeed =0x0013E4F5; //477rpm
//
//INTEGER32 Anticlockwise_WalkNomalSpeed=0xFFB06C2B;//-1910rpm
//INTEGER32 Anticlockwise_WalkSlowSpeed =0XFFEC1B0B; //-477rpm


/*1.2 m/s正常速度*/
//INTEGER32 Clockwise_WalkNomalSpeed=0x005F7E33; //2291.83118rpm
//INTEGER32 Clockwise_WalkSlowSpeed =0x0017DF8D; //577.9577rpm
//
//INTEGER32 Anticlockwise_WalkNomalSpeed=0xFFA081CD;
//INTEGER32 Anticlockwise_WalkSlowSpeed =0xFFE82073;

INTEGER32 Clockwise_WalkNomalSpeed=0x005F7E33; //2291.83118rpm
INTEGER32 Clockwise_WalkSlowSpeed =0x005F7E33; //577.9577rpm

INTEGER32 Anticlockwise_WalkNomalSpeed=0xFFA081CD;
INTEGER32 Anticlockwise_WalkSlowSpeed =0xFFA081CD;

/*1.36 m/s==2600rpm正常速度*/
//INTEGER32 Clockwise_WalkNomalSpeed=0x006C5555; //2600rpm
//INTEGER32 Clockwise_WalkSlowSpeed =0x001B1555; //650rpm
//
//INTEGER32 Anticlockwise_WalkNomalSpeed=0xFF93AAAB;
//INTEGER32 Anticlockwise_WalkSlowSpeed =0XFFE4EAAB;

/*寻找RFID的速度，应小与0.1788m/s(以0.4减速度，4cm正好到0)*/
//float LookingRFIDSpeed=0.1788; //单位m/s，轮子的速度
float LookingRFIDSpeed=0.1; //单位m/s，轮子的速度
INTEGER32 LookingRFIDSpeedCount=0x1; //将LookingRFIDSpeed转换为电机指令值

/*行走速度模式，启动/停止*/
INTEGER16 WalkStart = 0x000F;
INTEGER16 WalkStop = 0x010F;
INTEGER16 WalkStopDisable = 0x0107;//行走速度模式断使能

/********************行走电机参数 位置模式********************/
/*行走电机距离参数*/
INTEGER32 WalkingPositive=0x009b4040;
INTEGER32 WalkingNegative=0xff64bfc0;

	/*自旋90度位置*/
//INTEGER32 RotatePositionPositive=0x00119952;
//INTEGER32 RotatePositionNegative=0xFFEE66AE;

/*自旋1度位置*/
INTEGER32 RotatePositionPositive=0x1;
INTEGER32 RotatePositionNegative=0x1;

/*距离停止点距离*/
INTEGER32 CommandPosition1=0x1;
INTEGER32 CommandPosition2=0x1;
INTEGER32 CommandPosition3=0x1;
INTEGER32 CommandPosition4=0x1;
int positivetime=1;

/*寻找RFID最小距离理论值 1m/s==1.25m   1.2m/s==1.8m  1.3m/s==2.1125m
 * LookingRFIDPoint应大于理论值，单位米m*/
float LookingRFIDPoint=1.4; //2m开始发送指令
/*寻找RFID最小距离 转换count*/
INTEGER32 LookingRFIDPointCount=0x1;


/*行走电机开始寻找RFID为了匀速运行，将停止点向前挪一个相对距离，十进制单位米m
 * 此距离用于计算 LookingRFIDWalkingPositiveCount 和 LookingRFIDWalkingNegativeCount*/
float LookingRFIDForwardStopPoint=2;
float LookingRFIDCrabStopPoint=2;
/* LookingRFIDForwardStopPoint或者LookingRFIDCrabStopPoint
 * 转换count*/
INTEGER32 LookingRFIDWalkingPositiveCount=0x1;
INTEGER32 LookingRFIDWalkingNegativeCount=0x1;


/*读到RFID，修正停止点为当前位置前FixedFordwardStopPoint距离，相对位置
 * 0.04=4厘米，此值固定，tag标签从边缘到中心=4cm*/
float FixedFordwardStopPoint=0.04;
float FixedCrabStopPoint=0.04;
/*FixedFordwardStopPoint或者FixedCrabStopPoint
 * 转换count*/
INTEGER32 WalkingPositiveFixed=0x009b4040;
INTEGER32 WalkingNegativeFixed=0xff64bfc0;

/*LookingRFIDCount用于限制寻找RFID函数只进入一次，初始=0，进入后=1，agvstop中置0*/
int LookingRFIDCount=0;

/*行走过程中蔽障标帜位，用于避免每个while都发送bit4=0->1，每次运动前，将标帜位置0; AvoidFlag=0:初始状态，AvoidFlag=1蔽障未触发，=2蔽障外层触发 =3蔽障内层触发*/
int AvoidFlag=0;

/*启动/停止*/
INTEGER16 WalkStartPosition = 0x003F; //绝对
INTEGER16 WalkStartPositionReletive = 0x007F; //相对
INTEGER16 WalkStopPosition = 0x016F;
INTEGER16 WalkStopPositionDisable=0x0167; //行走位置模式断始能
INTEGER16 WalkBitfourZero=0x006F;



/******转向电机参数 位置模式******/
/* 方向控制--通过Target Position 607A 位置数值的正负HEX；
 * 面对电机轴俯视，暂规定：电机顺时针旋转=位置数值为正; 逆时针旋转=位置数值为负
 * 如果电机默认相反，只需调整 使电机顺时针旋转时，位置数值=正 即可
 */

/*轮子与长边平行时，作为0位置*/
INTEGER32 Rotator_zero=0x00000000;

/* 横移位置=90°
 * 轮子旋转90°电机需要旋转90*25=2250°=6.25圈 * 4096= 25600 counts=0x64 00
 * 顺时针= 0x6400
 * 逆时针= 0xFFFF9C00
 * */
INTEGER32 Clockwise_Rotator_Crab= 0X000C8000;
INTEGER32 Anticlockwise_Rotator_Crab=0XFFF38000;

/* 自旋位置=65.22485°
 *轮子旋转arctan(1300/600)°电机需要旋转arctan(1300/600)*25=1630.621486°/360 =4.529504圈*4096=18552.8489=18553 counts=0X4879 (换算回角度=65.22539°)
 *顺时针=0x4879;
 *逆时针=0xFFFFB787
 * */
//INTEGER32 Clockwise_Rotator_Self=0X00090F1B;
//INTEGER32 Anticlockwise_Rotator_Self=0XFFF6F0E5;
INTEGER32 Clockwise_Rotator_Self=0X00090F1B ;
INTEGER32 Anticlockwise_Rotator_Self=0XFFF6F0E5;

/*启动停止*/
INTEGER16 RotatorStart = 0x003F;
INTEGER16 RotatorDelay = 0x002F; //需测试：此处延迟的1S 是否会对接受信息造成影响
INTEGER16 RotatorStop  = 0x012F; //不断使能
INTEGER16 RotatorStopDisable  = 0x012F; //断使能

/*限位开关*/
//	UNS8 limitswitch=0x0; //把OD限位开关的变量复制过来，防止读取+写入同时进行!
int limitswitch_rotator5_up=0;//旋转电机I-上限位开关
int limitswitch_rotator5_down=0;//旋转电机I-下限位开关

int limitswitch_rotator6_up=0;//旋转电机II-上限位开关
int limitswitch_rotator6_down=0;//旋转电机II-下限位开关

int limitswitch_rotator7_up=0;//旋转电机III-上限位开关
int limitswitch_rotator7_down=0;//旋转电机III-下限位开关

int limitswitch_rotator8_up=0;//旋转电机VI-上限位开关
int limitswitch_rotator8_down=0;//旋转电机VI-下限位开关


/******举升电机参数 位置模式******/
/* 方向控制--通过Target Position 607A 位置数值的正负HEX；
 * 面对电机轴俯视，暂规定：电机逆时针旋转（位置数值为正）：螺杆举升；顺时针旋转（位置数值为负）：螺杆下降
 * 如果电机默认相反，只需调整 使电机顺时针旋转时，位置信息为正即可
 *
 * 举升电机位于底部时，位置=0
 * 需举升40mm距离，用时 4.2S（不包含加减速过程）*/

/*位置：电机需要转182圈 = 2981888 counts = 002D8000*/
INTEGER32 LiftZeroPosition	=0x00000000;
INTEGER32 LiftTargetPosition=0x002D8000;

/*启动停止*/
INTEGER16 LiftStart = 0x003F;
INTEGER16 LiftDelay = 0x002F;//需测试：此处延迟的1S 是否会对接受信息造成影响
//INTEGER16 LiftStop  = 0x012F; //不断使能
INTEGER16 LiftStop  = 0x0127; //bit 3 enable=0 //断使能

/******继电器输出******/
/* K1充电 		第0位=0x01;
 * K2控制电机 	第1位=0x02;
 * K4=绿色灯	 	第2位=0x04;
 * K5=黄色灯	 	第3位=0x08;
 * K6=红色灯		第4位=0x10;
 * 将0位置1：OUT8_Relay_bit1_8 |= 0x01
 * 将0位清0：OUT8_Relay_bit1_8 &= ~0x01*/
UNS8 K1=0x01;
UNS8 K2=0x02;
UNS8 K4=0x04;
UNS8 K5=0x08;
UNS8 K6=0x10;
UNS8 K9=0x40;

////闭合继电器 输出1
//#define RelayClose(a) OUT8_Relay_bit1_8 |=a
//
////断开继电器 输出0
//#define RelayOpen(b) OUT8_Relay_bit1_8 &=~b


INTEGER64 encodeMAX=2147483647;//编码器是INTEGER32格式，理论上可以表示-2147483648----2147483647之间的所有整数，现在假定当转数达到正最大时，再加1会变成负最大，同样，
//当转数为负最大时，再减小会变成正最大。

INTEGER32 encodeOldR1,encodeOldR2,encodeOldR3,encodeOldR4;//用来计算行走电机行走距离时，存储编码器旧值

float meterDistance=0;//转化成米数时的行走距离，
INTEGER64 distance=0;//存储编码器数值的累计行走距离，此距离值用整数表示，设成64位整数值，可以表示2的63次方的整数值

int correctNaviError=0;//当导航读到超过0xfc00但又小于0xffff的数值时，无法判断究竟是导航错误还是本来应该读0xffff的地方少读了
//一位数据，因此用这个标志位来辅助，如果这个标志位很多次被触发，说明导航出了错误，如果只是被触发几次，说明是该读全1的
//地方漏读了某几位，暂时以一次运行中被触发50次来做标准，实际值要根据测试实例更改。

//记录直行电机的初始速度值，因为直行时有两种速度，一种是正常速度，一种是爬行速度
INTEGER32 motor1=0;
INTEGER32 motor2=0;
INTEGER32 motor3=0;
INTEGER32 motor4=0;

//使用行走方向的后面两个rfid读卡器值的变化作为判断小车到达指定位置并停止的依据，因为rfid不清空，当小车启动时，默认四个rfid读卡器都能读到标签，
//当行进方向的两个前轮读到新的rfid标签时，其实它们两个读到的是应该后两轮的停止标签，所以这个变化不能用，此时后两轮的读卡器中存储的依然是起始位
//的标签，等到后两轮读到的标签更新后，就认为小车到达了下一个车位，应该停止运动，如果要严格判断的话，此时还可以判断前两轮读到的标签的最后1位加上
//后两轮读到标签的最后1位的和是不是10,
char rfid[4];

int norfidstop=0;//如果检测不到rfid,需要有一个停止条件让循环终止,这就是那个停止条件的flag,因为程序刚刚启动时,那个停止条件可能也满足,所以用这个
//变量来延迟一下停止条件的运行时间.

/*执行命令函数，客户端收到的上位机命令Command[]作为参数传进来，在此处按位解析，并分别调用相应的程序执行
 * Command[]有固定格式，以AA开头，以EE结束，
 * AA后面是控制字，分别是："BB"(控制agv行走)，"CC"(控制agv举升和下降)，"DD"(控制agv充电)，"WR"(控制agv轮子的旋转)，"RR"(控制小车的自旋）
 * BB(1,2,3,4)(0,1):12346分别代表向前、向后、左移、右移,0,1代表爬行速度、正常速度
 * CC(0,1)：0,1分别代表下降、举升
 * DD(0,1):0,1分别代表去充电、停止充电
 * WR(0,1,2):012分别代表回零位，横移位，自旋位
 * RR(0,1)(x):01分别代表顺时针转、逆时针转，x为'0','1','2','3',分别代表转动90、180、270、360
 */
int  ExecuteCommand(char* Command)
{
	if((Command[2]=='B'||Command[2]=='b')&&(Command[3]=='B'||Command[3]=='b'))  //控制agv的运动，1：向前行走;  2：向后行走;   3：左移;   4:右移;
	{
		if(Command[4]=='1')//电机向前运动
		{
			float slowpoint=string2float(Command);
			char rfidTarget=Command[15];
			int ret=AgvFoward(Command[5],slowpoint,rfidTarget);
			printf("前进返回的值是%d\n",ret);
			return ret;
		}
		else if(Command[4]=='2')//向后运动
		{
			float slowpoint=string2float(Command);
			char rfidTarget=Command[15];
			int ret=AgvBack(Command[5],slowpoint,rfidTarget);
			printf("后退返回的值是%d\n",ret);
			return ret;
		}
		else if(Command[4]=='3')//左移
		{
			float slowpoint=string2float(Command);
			char rfidTarget=Command[15];
			return AgvLeft(Command[5],slowpoint,rfidTarget);
		}
		else if(Command[4]=='4')//右移
		{
			float slowpoint=string2float(Command);
			char rfidTarget=Command[15];
			return AgvRight(Command[5],slowpoint,rfidTarget);
		}
		else//命令出现错误
		{
			Log("agv运动命令出错，下发命令不是前、后、左、右、停");
			return -1;
		}
		return 0;//如果agv执行此条命令成功，返回0;实际上这条命令永远也不会执行，因为前面所有情况都包含return.
	}


	else if((Command[2]=='C'||Command[2]=='c')&&(Command[3]=='C'||Command[3]=='c'))  //控制agv举升和下降
	{
		return AgvUpDown(Command[4]);
	}


	else if((Command[2]=='D'||Command[2]=='d')&&(Command[3]=='D'||Command[3]=='d')) //控制agv充电
	{
		return AgvCharge('c');
	}


	else if((Command[2]=='W'||Command[2]=='w')&&(Command[3]=='R'||Command[3]=='r')) //控制agv轮子的旋转,0:轮子回零位;  1:横移位;  2:自旋位;
	{
		if(Command[4]=='0')//轮子回零位
		{
			return RotateWheel2Zero();
		}
		else if(Command[4]=='1')//横移位
		{
			return RotateWheel2LeftRight();
		}
		else if(Command[4]=='2')//自旋位
		{
			return RotateWheel2SelfRotating();
		}
		else//命令出现错误
		{
			Log("agv轮子旋转命令出错，不是规定的3个位置");
			return -1;
		}
	}


	else if((Command[2]=='R'||Command[2]=='r')&&(Command[3]=='R'||Command[3]=='r'))//agv小车旋转
	{
		return RotateAgv(Command[4],Command[5]);
	}


	else//命令出现错误
	{
		Log("上位机下发命令出错，不是运动、举升卸载、充电、轮子旋转、小车旋转");
		return -1;
	}
	return 0;//命令执行完成，返回0;此return不会执行。
}

/*！！！！！！！！！！！！！！！！错了！！！！！！！！！！！！！*/
//零位时，接近限位开关！0位不能在转盘的中间位置！ 因为转盘最大旋转100度，若在中间位置则两边转角只有50度，无法满足90度要求！！！！
/*小车四个轮子有三个位置，一是零位，即与小车长边平行，此时轮子可以向顶位转动，去和宽边平行，也可以向自旋位转动，到位后小车可以自旋，
 * 二是顶位，即与小车宽边平行，此时轮子接近90°限位开关，只能向零位或自旋位转动。三是自旋位，此时轮子接近另一个限位开关，只能向零位转动。
 * 轮子的三个位置要断电保存，所以每当轮子转换位置成功后要将其最新位置写入文件，设零位标志为0,顶位标志为1,自旋位为2，agv上电时轮子调到零位，
 * 上位机在发送直行、自旋等命令时，根据轮子的当前位置和行进方向确定是否需要调整轮子的位置。
 */
int RotateWheel2Zero()//轮子回零位
{
	//innerstruct.StopCommand=0;//把停止命令清空

	/*将BIT4=0 避免纠偏未把bit4置0*/
//	M2_TPDO_control_word=Bit4Zero(M2_TPDO_control_word); //bit 4=0  A &= ~0x10
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

	/*启动*/
	M2_TPDO_control_word = RotatorStart;
	sendPDOevent(CANOpenShellOD_Data);

	int delaycount=0;
	while(delaycount<=position_delay) //delay 0.5秒，等待电机开始运行再开始检测
	{
		delaycount++;
	} //100000000大概1.2s左右；300000000=4.5s左右

//	sleep(1);//此处等待1s，旋转电机旋转1s后程序才会继续走，而紧急停止和限位开关停止命令在while循环中;若旋转电机旋转方向反了，则无法立刻停下!!!!
//	M2_TPDO_control_word=RotatorDelay;//修改bit4=0
//	sendPDOevent(CANOpenShellOD_Data);

	while(agvreportstruct.MotorError==0) //使用agvreportstruct.MotorError 判断EMCY是否触发，若触发agvreportstruct.MotorError=1，停止while并返回-1
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
			Log("旋转电机到达0位置");
			printf("旋转电机到达0位置\n");
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
			AgvEmcyStop();//停止所有电机，红灯亮
			Log("轮子转向零位过程中限位开关被触发");
			printf("轮子转向零位过程中限位开关被触发\n");
			return -1; //遇到故障，返回-1，表明命令并位正常执行
		}

		if(innerstruct.StopCommand==1)//如果收到停止命令，正常情况下轮子转动过程中不能停止，但遇到特殊情况强行停止也可以，只是要记录到日志里
		{
			Log("轮子转向零位过程中收到了停止命令");
			printf("轮子转向零位过程中收到了停止命令\n");
			return AgvStop();//上位机下达的停止命令正常执行，所以返回AgvStop=0。
		}
	}
	printf("轮子转向零位过程中收到EMCY\n");
	Log("轮子转向零位过程中收到EMCY");
	return -1;
}

int RotateWheel2LeftRight()//轮子转到横移位
{
	//int motorcode[4]={1,2,3,4};//根据经验值确定四个旋转电机转动90°时编码器要变化的数值
	//int initposition[4]={R5_position,R6_position,R7_position,R8_position};//获得四个旋转电机当前的编码器的值，
	//如果电机足够精确，向宽边转动时，四个轮子肯定在零位，此时四个电机的编码器其实是固定值，要变化的值也是固定值，那么上面两个变量都不需要，只要将motorcode的内容写到命令中去就可以。

//	innerstruct.StopCommand=0;//把停止命令清空

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
			Log("旋转电机到达横移位置=1");
			printf("旋转电机到达横移位置=1\n");
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
			AgvEmcyStop();//停止所有电机，红灯亮
			Log("轮子转向横移位过程中限位开关被触发");
			printf("轮子转向横移位过程中限位开关被触发\n");
			return -1;
		}
		else if(innerstruct.StopCommand==1)//如果收到停止命令，正常情况下轮子转动过程中不能停止，但遇到特殊情况强行停止也可以，只是要记录到日志里
		{
			Log("轮子转向横移位过程中收到了停止命令");
			printf("轮子转向横移位过程中收到了停止命令\n");
			return AgvStop();
		}
	}
	Log("轮子转向横移位过程中收到了EMCY");
	printf("轮子转向横移位过程中收到了EMCY\n");
	return -1;
}

int RotateWheel2SelfRotating()//轮子转向自旋位
{
	//int motorcode[4]={1,2,3,4};//根据经验值确定四个旋转电机转到自旋方向时编码器要变化的数值
	//int initposition[4]={R5_position,R6_position,R7_position,R8_position};//获得四个旋转电机当前的编码器的值，
	//如果电机足够精确，向宽边转动时，四个轮子肯定在零位，此时四个电机的编码器其实是固定值，要变化的值也是固定值，那么上面两个变量都不需要，只要将motorcode的内容写到命令中去就可以。
//	innerstruct.StopCommand=0;

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
			Log("旋转电机到达自旋位置=2");
			printf("旋转电机到达自旋位置=2\n");
			M2_TPDO_control_word=RotatorStop;//修改bit4=0
			sendPDOevent(CANOpenShellOD_Data);
			return 0;//命令执行成功，返回正常值
		}

		/*判断限位开关*/
		else if(limitswitch_rotator5_up==1||limitswitch_rotator5_down==1 ||
		   limitswitch_rotator6_up==1||limitswitch_rotator6_down==1 ||
		   limitswitch_rotator7_up==1||limitswitch_rotator7_down==1 ||
		   limitswitch_rotator8_up==1||limitswitch_rotator8_down==1)//旋转轮子 碰触限位开关，停止电机！发生错误！
		{
			AgvEmcyStop();//停止所有电机，红灯亮
			Log("轮子转向自旋过程中限位开关被触发");
			printf("轮子转向自旋过程中限位开关被触发\n");
			return -1;
		}

		else if(innerstruct.StopCommand==1)//如果收到停止命令，正常情况下轮子转动过程中不能停止，但遇到特殊情况强行停止也可以，只是要记录到日志里
		{
			Log("轮子转向自旋过程中收到了停止命令");
			printf("轮子转向自旋过程中收到了停止命令\n");
			return AgvStop();
		}
	}
	Log("轮子转向自旋过程中收到了EMCY");
	printf("轮子转向自旋过程中收到了EMCY\n");
	return -1;
}

/*agv自旋程序，上位机向小车发送旋转方向和旋转角度，此程序被调用之前，上位机必须已经向rotatewheel发送调整至自旋状态的命令，并成功执行。
 * 此程序运行时需要实时考虑防撞传感器是否触发，将导航传感器信号和rfid信号作为旋转是否到位的判定条件
 */
int RotateAgv(char direc,char angle)//direction '1'代表逆时针，'0'代表顺时针;  angle '0' 代表90°，'1'代表180°，‘2’代表270°,'3'代表360°。
{
	/*将bit4置0*/
	M1_TPDO_control_word=(M1_TPDO_control_word&(~0x10));//bit 4=0
	sendPDOevent(CANOpenShellOD_Data);

	int time=(int)angle-47;//判断要旋转几个90°，因为int '0'=48,('0'的ascii码是48),所以减去47后就代表转一个90°，
	//printf("time is %d\n",time);

	innerstruct.StopCommand=0;//把电机停止命令清空

	int log_count=0; //碰到障碍物等待时，向LOG中写入信息的时间间隔
	int wait_timeout=0; //等待碰到障碍物等待时，超时时间
	/*轮子到自旋位置*/
	if(RotateWheel2SelfRotating()!=0)
	{
		printf("自旋前转向电机到自旋位置失败\n");
		Log("自旋前转向电机到自旋位置失败");
		return -1;
	}
	else
	{
		printf("自旋前转向电机到自旋位置完成\n");
		Log("自旋前转向电机到自旋位置完成");
	}
	//计算旋转距离，减少while时间
	if(SetRotatePosition(time)!=0)
	{
		printf("计算旋转距离失败\n");
		Log("计算旋转距离失败");
		return -1;
	}
	else
	{
		printf("计算旋转距离完成\n");
		Log("计算旋转距离完成");
	}
//
//	if(agvreportstruct.RFIDSenser[0][3]=='0'||agvreportstruct.RFIDSenser[1][3]=='0'||agvreportstruct.RFIDSenser[2][3]=='0'||agvreportstruct.RFIDSenser[3][3]=='0')//如果自旋前所有rfid读卡器都收不到信号，说明agv此时没在车位上
//	{
//		Log("agv检测不到rfid初始位，无法自旋");
//		printf("agv检测不到rfid初始位，无法自旋\n");
//		//return  -1; return EmcyStop();
//	}
	//INTEGER32 r1position=R1_position;
	//printf("r1position is %d\n",r1position);

	/*臂章没有触发*/
	if(agvreportstruct.HitAvoidSenser[0]==0 && agvreportstruct.HitAvoidSenser[1]==0 &&agvreportstruct.HitAvoidSenser[2]==0 && agvreportstruct.HitAvoidSenser[3]==0)//防撞信号没有触发
	{
		//黄灯K5灭 绿灯K4亮，K5=open K4=close
		RelayClose(K4);
		RelayOpen(K5);
		RelayOpen(K6);

	    /**************************************************************************/
		/*重设速度为正常值*/
		M1_TPDO_Profile_Velocity=Clockwise_WalkNomalSpeed;
		printf("重设行走电机位置模式速度==Clockwise_WalkNomalSpeed\n");
	    /**************************************************************************/

		 //向四个直行电机发送按照爬行速度，以速度*direction的方式旋转的命令，此命令在while循环内部，如果防撞信号始终没有触发，则此
		//命令由于pdo特性不会发出，如果触发了防撞信号，则电机会停下，防撞信号消失后，这个电机运行命令会执行，保证电机能重新启动。
		if(direc=='1')//1=逆时针旋转
			{
				//AGV旋转时，以Slow慢速旋转
				/* AGV逆时针旋转
				 * 行走电机1 4 =前进  1=正数 4=负数；
				 * 行走电机2 3 =后退	2=正数 3=负数
				 * 左侧轮后退；右侧轮前进*/
				M1_1_TPDO_Target_Position =RotatePositionPositive;
				M1_2_TPDO_Target_Position =RotatePositionPositive ;

				M1_3_TPDO_Target_Position =RotatePositionNegative;
				M1_4_TPDO_Target_Position =RotatePositionNegative;
				sendPDOevent(CANOpenShellOD_Data);

				/*启动*/
				M1_TPDO_control_word = WalkStartPositionReletive;
				sendPDOevent(CANOpenShellOD_Data);

			}
			else if(direc=='0')//0=顺时针旋转
			{
				//AGV旋转时，以Slow慢速旋转
				/* AGV顺时针旋转：
				 * 行走电机1 4 =后退  1=负数 4=正数；
				 * 行走电机2 3=前进	2=负数 3=正数
				 * (左侧轮前进；右侧轮后退)*/
				M1_1_TPDO_Target_Position =RotatePositionNegative;
				M1_2_TPDO_Target_Position =RotatePositionNegative ;

				M1_3_TPDO_Target_Position =RotatePositionPositive;
				M1_4_TPDO_Target_Position =RotatePositionPositive;
				sendPDOevent(CANOpenShellOD_Data);

				/*启动*/
				M1_TPDO_control_word = WalkStartPositionReletive;
				sendPDOevent(CANOpenShellOD_Data);
			}

	}//end 无防撞信号判断
	else//开始转动前，蔽障被触发，报错并结束自旋
	{
		Log("自旋动作有障碍物");
		printf("自旋动作有障碍物\n");
		return -1;
	}

	int delaycount=0;
	while(delaycount<=100000000) //delay 0.5秒，等待电机开始运行再开始检测
	{
		delaycount++;
	} //100000000大概1.2s左右；300000000=4.5s左右

	while(agvreportstruct.MotorError==0)
	{
//		/*将bit4置0*/
//		M1_TPDO_control_word=(M1_TPDO_control_word&(~0x10));//bit 4=0
//		sendPDOevent(CANOpenShellOD_Data);
		usleep(1000);
		if(innerstruct.StopCommand==1)//如果收到停止命令，正常情况下轮子转动过程中不能停止，但遇到特殊情况强行停止也可以，只是要记录到日志里
		{
			Log("agv转向过程中收到了停止命令");
			printf("agv转向过程中收到了停止命令\n");
			return AgvStop();
		}

		/****判断行走电机是否运行到位！！！*****/
		if(((R1_statusword & (1<<(10)))!=0) ||
		   ((R2_statusword & (1<<(10)))!=0) ||
		   ((R3_statusword & (1<<(10)))!=0) ||
		   ((R4_statusword & (1<<(10)))!=0) )//如果任意一个行走电机到位，则立刻停止所有行走电机
		{
			Log("自旋动作完成");
			printf("自旋动作完成\n");
			AgvStop();
			return 0;
		}

		//蔽障没有触发
		if(agvreportstruct.HitAvoidSenser[0]==0 && agvreportstruct.HitAvoidSenser[1]==0 &&agvreportstruct.HitAvoidSenser[2]==0 && agvreportstruct.HitAvoidSenser[3]==0)//防撞信号没有触发
		{
			/*将bit4置0*/
			M1_TPDO_control_word=(M1_TPDO_control_word&(~0x10));//bit 4=0
			sendPDOevent(CANOpenShellOD_Data);

			//黄灯K5灭 绿灯K4亮，K5=open K4=close
			RelayClose(K4);
			RelayOpen(K5);
			RelayOpen(K6);

			//此处应该重设正常速度，防止蔽障将速度设置为0，需要加入标帜位，防止每次都发送启动命令

		}//end 无防撞信号判断

		//蔽障触发
		else if(agvreportstruct.HitAvoidSenser[0]==0x02 || agvreportstruct.HitAvoidSenser[1]==0x02 ||
				agvreportstruct.HitAvoidSenser[2]==0x02 || agvreportstruct.HitAvoidSenser[3]==0x02 ||
				agvreportstruct.HitAvoidSenser[0]==0x03 || agvreportstruct.HitAvoidSenser[1]==0x03 ||
				agvreportstruct.HitAvoidSenser[2]==0x03 || agvreportstruct.HitAvoidSenser[3]==0x03)//防撞信号1区被触发(旋转时不再考虑1区2区，有障碍就停下)，while程序并不停止，当防撞信号消失后，进入前一个判断条件
		{
			/*将bit4置0*/
			M1_TPDO_control_word=(M1_TPDO_control_word&(~0x10));//bit 4=0
			sendPDOevent(CANOpenShellOD_Data);

			//黄灯K5亮 绿灯K4灭，K5=close K4=open
			RelayClose(K5);
			RelayOpen(K4);
			RelayOpen(K6);

			//向电机发送停止命令,while不退出，等待障碍物消失，继续运行
			//此处应该将速度设置为0！让停下，需要加入标帜位，防止每次都发送启动命令

			log_count++;
			if (log_count==1) //首次触发以及每间隔几秒后，打印信息
			{
				Log("自旋时遇到障碍物，等待障碍物消失");
				printf("自旋时遇到障碍物，等待障碍物消失\n");
				wait_timeout++;//等待时间++，用于超时退出
			}
			if (log_count==10000000){log_count=0;}//大概几秒清零
			if (wait_timeout == 100) //大概几分钟
			{
				AgvEmcyStop();//停止所有电机，红灯亮
				Log("自旋时遇到障碍物，等待超时");
				printf("自旋时遇到障碍物，等待超时\n");
				return -1;
			}
		}
	} //end while
	Log("agv转向过程中收到了EMCY");
	printf("agv转向过程中收到了EMCY\n");
	return -1;
} //end AGV自旋

//电机举升下降程序：在举升程序运行前需要加入一个归零指令！！！！！
//问题：举升电机0位置使用startcode=R9_position，则0位置=初始位置，若举升电机初始位置不在0，则可能出错
int AgvUpDown(char direction)//direction为1或0,分别代表举升和下降
{
	int limitswitch_up=0;//上限位开关
	int limitswitch_down=0;//下限位开关

//	innerstruct.StopCommand=0;

	if(direction=='1')//举升
	{
		M3_TPDO_Target_Position=(LiftTargetPosition+UpMotorInitPosition);//将位置信息传给OD
		sendPDOevent(CANOpenShellOD_Data);//发送PDO
		M3_TPDO_control_word=LiftStart;//启动电机
		sendPDOevent(CANOpenShellOD_Data);

		int delaycount=0;
		while(delaycount<=position_delay) //delay 0.5秒，等待电机开始运行再开始检测
		{
			delaycount++;
		} //100000000大概1.2s左右；300000000=4.5s左右
	}
	if(direction=='0')//下降
	{
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
	}
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
			Log("举升或下降完成");
			printf("举升或下降完成\n");
			M3_TPDO_control_word=LiftStop;//修改bit4=0
			sendPDOevent(CANOpenShellOD_Data);
			return 0;
		}
		else if(innerstruct.StopCommand==1)//如果收到停止命令，正常情况下轮子转动过程中不能停止，但遇到特殊情况强行停止也可以，只是要记录到日志里
		{
			//发送停止命令
			Log("agv举升或下降过程中收到了停止命令");
			printf("agv举升或下降过程中收到了停止命令\n");
			return AgvStop();
		}

		if(limitswitch_up==1||limitswitch_down==1)//上或下限位开关触发！
		{
			AgvEmcyStop();//停止所有电机，红灯亮
			Log("agv举升或下降过程中限位开关触发");
			printf("agv举升或下降过程中限位开关触发\n");
			return -1;
		}
	}
	Log("agv举升或下降过程中收到了EMCY");
	printf("agv举升或下降过程中收到了EMCY\n");
	return -1;
}

//下面是电机前行程序
int AgvFoward(char fastslow,float slowpoint,char rfidTarget)
{
	if(WalkingPreperationSpeedSet(fastslow,1,slowpoint)!=0)
	{
		printf("电机速度设置出错，无法运动\n");
		Log("电机速度设置出错，无法运动\n");
		return -1;
	}

	int rfidnum[4];//分别代表rfid1,rfid2,rfid3,rfid4
	int hitavoid[3];//存储行进方向前左右三个导航的名称
	rfidnum[0]=0;rfidnum[1]=1;rfidnum[2]=2;rfidnum[3]=3;
	hitavoid[0]=0;hitavoid[1]=1;hitavoid[2]=2;


	int direc=1;//行进方向前
	float PointDistance[2];//存储减速点和停止点距离
	PointDistance[0]=slowpoint;
	PointDistance[1]=slowpoint+2;
	int AdjustNavi[2];//纠偏使用的导航,第一个是前导航，第二个是后导航
	AdjustNavi[0]=1;AdjustNavi[1]=3;
	INTEGER32 M2_position[24];//纠偏所需的旋转电机的位置值，从0-23依次是前轮、后轮所需的24个赋值。
	//前轮偏左
	M2_position[0]=Clockwis_Degree3;
	M2_position[1]=Clockwis_Degree3;
	M2_position[2]=Rotator_zero;
	M2_position[3]=Rotator_zero;
	//前轮偏右
	M2_position[4]=AntiClockwis_Degree3;
	M2_position[5]=AntiClockwis_Degree3;
	M2_position[6]=Rotator_zero;
	M2_position[7]=Rotator_zero;
	//前轮归零*/
	M2_position[8]=Rotator_zero;
	M2_position[9]=Rotator_zero;
	M2_position[10]=Rotator_zero;
	M2_position[11]=Rotator_zero;
	//后轮偏右
	M2_position[12]=Rotator_zero;
	M2_position[13]=Rotator_zero;
	M2_position[14]=Clockwis_Degree4;
	M2_position[15]=Clockwis_Degree4;
	//后轮偏左
	M2_position[16]=Rotator_zero;
	M2_position[17]=Rotator_zero;
	M2_position[18]=AntiClockwis_Degree4;
	M2_position[19]=AntiClockwis_Degree4;
	//后轮归零
	M2_position[20]=Rotator_zero;
	M2_position[21]=Rotator_zero;
	M2_position[22]=Rotator_zero;
	M2_position[23]=Rotator_zero;

	rfid[0]=rfidTarget;
	rfid[1]=rfid[0];
	rfid[2]=rfid[0];
	rfid[3]=rfid[0];

	//printf("rfid[0]==%d, rfid[1]==%d,rfid[2]==%d,rfid[3]==%d\n",rfid[0]-48,rfid[1]-48,rfid[2],rfid[3]);
	//printf("rfidnum[1]=1==%d\n",rfidnum[1]=1);

	while(agvreportstruct.MotorError==0)
	{
		/*通过第一个组RFID后，开始检测行走电机是否到位，并修改纠偏宽度*/
			if(innerstruct.SlowCommand==2)
			{
				/*此处扩大纠偏范围，减少进入纠偏次数*/
//				while(((R1_statusword & (1<<(10)))==0) ||
//				   ((R2_statusword & (1<<(10)))==0) ||
//				   ((R3_statusword & (1<<(10)))==0) ||
//				   ((R4_statusword & (1<<(10)))==0) )//如果四个都到,再停
					while(((R1_statusword & (1<<(10)))==0) &&
					   ((R2_statusword & (1<<(10)))==0) &&
					   ((R3_statusword & (1<<(10)))==0) &&
					   ((R4_statusword & (1<<(10)))==0) )//如果任意一个行走到位，则停止
					{
						;
					}
					printf("运动触发RFID后到达停止点\n");
					AgvStop();
					return 0;//返回0推出函数
			}

		if(innerstruct.StopCommand==0)
		{
			int ret=WalkingWhileAssist(direc,rfidnum,hitavoid,PointDistance,AdjustNavi,M2_position);
			if(ret==4)//如果在WalkingWhileAssist中收到停止标志位置1，返回特殊值4，退出while循环。
			{
				return 0;
			}
			else if(ret==-1)//如果在WalkingWhileAssist中触发了紧急停止，返回特殊值-1，退出while循环，同时返回-1。
			{
				return -1;
			}
		}
		else if(innerstruct.StopCommand==1)//如果收到停止标志位置1,退出while循环
		{
			innerstruct.SlowCommand=0;
			innerstruct.StopCommand=0;
			return AgvStop();
		}


	}//while循环结束，如果程序走到这里，说明上面所有的return都没有执行，那么agvreportstruct.MotorError肯定被置1,执行下面内容：
	printf("行走时，触发EMCY，行走方向是前行方向");
	Log("行走时，触发EMCY，行走方向是前行方向");
	return -1;
}

//下面是电机后退程序
int AgvBack(char fastslow,float slowpoint,char rfidTarget)
{
	if(WalkingPreperationSpeedSet(fastslow,2,slowpoint)!=0)
	{
		printf("电机速度设置出错，无法运动\n");
		Log("电机速度设置出错，无法运动\n");
		return -1;
	}
	int rfidnum[4],hitavoid[3];
	rfidnum[0]=2;rfidnum[1]=3;rfidnum[2]=0;rfidnum[3]=1;
	hitavoid[0]=0;hitavoid[1]=2;hitavoid[2]=3;


	int direc=2;//行进方向
	float PointDistance[2];
	PointDistance[0]=slowpoint;
	PointDistance[1]=slowpoint+2;
	int AdjustNavi[2];//纠偏使用的导航,第一个是前导航，第二个是后导航
	AdjustNavi[0]=3;
	AdjustNavi[1]=1;
	INTEGER32 M2_position[24];//纠偏所需的旋转电机的位置值，从0-23依次是前轮、后轮所需的24个赋值。
	//前轮偏左
	M2_position[0]=Rotator_zero;
	M2_position[1]=Rotator_zero;
	M2_position[2]=Clockwis_Degree3;
	M2_position[3]=Clockwis_Degree3;
	//前轮偏右
	M2_position[4]=Rotator_zero;
	M2_position[5]=Rotator_zero;
	M2_position[6]=AntiClockwis_Degree3;
	M2_position[7]=AntiClockwis_Degree3;
	//前轮归零*/
	M2_position[8]=Rotator_zero;
	M2_position[9]=Rotator_zero;
	M2_position[10]=Rotator_zero;
	M2_position[11]=Rotator_zero;
	//后轮偏右
	M2_position[12]=Clockwis_Degree4;
	M2_position[13]=Clockwis_Degree4;
	M2_position[14]=Rotator_zero;
	M2_position[15]=Rotator_zero;
	//后轮偏左
	M2_position[16]=AntiClockwis_Degree4;
	M2_position[17]=AntiClockwis_Degree4;
	M2_position[18]=Rotator_zero;
	M2_position[19]=Rotator_zero;
	//后轮归零
	M2_position[20]=Rotator_zero;
	M2_position[21]=Rotator_zero;
	M2_position[22]=Rotator_zero;
	M2_position[23]=Rotator_zero;

	rfid[0]=rfidTarget;
	rfid[1]=rfid[0];
	rfid[2]=rfid[0];
	rfid[3]=rfid[0];
//int abc=0;
	while(agvreportstruct.MotorError==0)
	{
		/*通过第一个组RFID后，开始检测行走电机是否到位，并修改纠偏宽度*/
			if(innerstruct.SlowCommand==2)
			{
				/*此处扩大纠偏范围，减少进入纠偏次数*/
					while(((R1_statusword & (1<<(10)))==0) &&
					   ((R2_statusword & (1<<(10)))==0) &&
					   ((R3_statusword & (1<<(10)))==0) &&
					   ((R4_statusword & (1<<(10)))==0) )//如果任意一个行走电机未到位置，则等待 屏蔽纠偏+蔽障，防止被改为绝对距离
					{
						;
					}
					printf("运动触发RFID后到达停止点\n");
					AgvStop();
					return 0;//返回0推出函数
			}
		if(innerstruct.StopCommand==0)
		{
			//WalkingWhileAssist(direc,rfidnum,hitavoid,PointDistance,AdjustNavi,M2_position);
			int ret=WalkingWhileAssist(direc,rfidnum,hitavoid,PointDistance,AdjustNavi,M2_position);
			if(ret==4)//如果在WalkingWhileAssist中收到停止标志位置1，返回特殊值4，退出while循环。
			{
				return 0;
			}
			else if(ret==-1)//如果在WalkingWhileAssist中触发了紧急停止，返回特殊值-1，退出while循环，同时返回-1。
			{
				return -1;
			}
		}
		else if(innerstruct.StopCommand==1)
		{
			return WalkingWhileAssist(direc,rfidnum,hitavoid,PointDistance,AdjustNavi,M2_position);
		}
	}//while循环结束，如果程序走到这里，说明上面所有的return都没有执行，那么agvreportstruct.MotorError肯定被置1,执行下面内容：
	printf("行走时，触发EMCY，行走方向为后退\n");
	Log("行走时，触发EMCY，行走方向为后退\n");
	return -1;
}

//下面是电机左移程序
int AgvLeft(char fastslow,float slowpoint,char rfidTarget)
{
	if(WalkingPreperationSpeedSet(fastslow,3,slowpoint)!=0)
	{
		printf("电机速度设置出错，无法运动\n");
		Log("电机速度设置出错，无法运动\n");
		return -1;
	}
	int rfidnum[4],hitavoid[3];
	rfidnum[0]=1;rfidnum[1]=2;rfidnum[2]=0;rfidnum[3]=3;
	hitavoid[0]=1;hitavoid[1]=2;hitavoid[2]=3;


	int direc=3;//行进方向左
	float PointDistance[2];
	PointDistance[0]=slowpoint;
	PointDistance[1]=slowpoint+2;
	int AdjustNavi[2];//纠偏使用的导航,第一个是前导航，第二个是后导航
	AdjustNavi[0]=2;AdjustNavi[1]=0;
	INTEGER32 M2_position[24];//纠偏所需的旋转电机的位置值，从0-23依次是前轮、后轮所需的24个赋值。
	//前轮偏左
	M2_position[0]=Anticlockwise_Rotator_Crab;
	M2_position[1]=Clockwise_Rotator_Crab+Clockwis_Degree3;
	M2_position[2]=Anticlockwise_Rotator_Crab+Clockwis_Degree3;
	M2_position[3]=Clockwise_Rotator_Crab;
	//前轮偏右
	M2_position[4]=Anticlockwise_Rotator_Crab;
	M2_position[5]=Clockwise_Rotator_Crab+AntiClockwis_Degree3;
	M2_position[6]=Anticlockwise_Rotator_Crab+AntiClockwis_Degree3;
	M2_position[7]=Clockwise_Rotator_Crab;
	//前轮归零*/
	M2_position[8]=Anticlockwise_Rotator_Crab;
	M2_position[9]=Clockwise_Rotator_Crab;
	M2_position[10]=Anticlockwise_Rotator_Crab;
	M2_position[11]=Clockwise_Rotator_Crab;
	//后轮偏右
	M2_position[12]=Anticlockwise_Rotator_Crab+Clockwis_Degree4;
	M2_position[13]=Clockwise_Rotator_Crab;
	M2_position[14]=Anticlockwise_Rotator_Crab;
	M2_position[15]=Clockwise_Rotator_Crab+Clockwis_Degree4;
	//后轮偏左
	M2_position[16]=Anticlockwise_Rotator_Crab+AntiClockwis_Degree4;
	M2_position[17]=Clockwise_Rotator_Crab;
	M2_position[18]=Anticlockwise_Rotator_Crab;
	M2_position[19]=Clockwise_Rotator_Crab+AntiClockwis_Degree4;
	//后轮归零
	M2_position[20]=Anticlockwise_Rotator_Crab;
	M2_position[21]=Clockwise_Rotator_Crab;
	M2_position[22]=Anticlockwise_Rotator_Crab;
	M2_position[23]=Clockwise_Rotator_Crab;

	rfid[0]=rfidTarget;
	rfid[1]=rfid[0];
	rfid[2]=rfid[0];
	rfid[3]=rfid[0];

	while(agvreportstruct.MotorError==0)
	{
		/*通过第一个组RFID后，开始检测行走电机是否到位，并修改纠偏宽度*/
			if(innerstruct.SlowCommand==2)
			{
				/*此处扩大纠偏范围，减少进入纠偏次数*/
//					while(((R1_statusword & (1<<(10)))==0) ||
//					   ((R2_statusword & (1<<(10)))==0) ||
//					   ((R3_statusword & (1<<(10)))==0) ||
//					   ((R4_statusword & (1<<(10)))==0) )//如果四个都到
				while(((R1_statusword & (1<<(10)))==0) &&
				   ((R2_statusword & (1<<(10)))==0) &&
				   ((R3_statusword & (1<<(10)))==0) &&
				   ((R4_statusword & (1<<(10)))==0) )//如果任意一个行走电机未到位置，则等待 屏蔽纠偏+蔽障，防止被改为绝对距离
					{
						;
					}
					printf("运动触发RFID后到达停止点\n");
					AgvStop();
					return 0;//返回0推出函数
			}
		if(innerstruct.StopCommand==0)
		{
			//WalkingWhileAssist(direc,rfidnum,hitavoid,PointDistance,AdjustNavi,M2_position);
			int ret=WalkingWhileAssist(direc,rfidnum,hitavoid,PointDistance,AdjustNavi,M2_position);
			if(ret==4)//如果在WalkingWhileAssist中收到停止标志位置1，返回特殊值4，退出while循环。
			{
				return 0;
			}
			else if(ret==-1)//如果在WalkingWhileAssist中触发了紧急停止，返回特殊值-1，退出while循环，同时返回-1。
			{
				return -1;
			}
		}
		else if(innerstruct.StopCommand==1)
		{
			return WalkingWhileAssist(direc,rfidnum,hitavoid,PointDistance,AdjustNavi,M2_position);
		}
	}//while循环结束，如果程序走到这里，说明上面所有的return都没有执行，那么agvreportstruct.MotorError肯定被置1,执行下面内容：

	printf("行走时，触发EMCY，行走方向左移\n");
	Log("行走时，触发EMCY，行走方向左移\n"); //输出字符串str给LOG
	return -1;
}

//下面是电机右移程序
int AgvRight(char fastslow,float slowpoint,char rfidTarget)
{
	if(WalkingPreperationSpeedSet(fastslow,4,slowpoint)!=0)
	{
		printf("电机速度设置出错，无法运动\n");
		Log("电机速度设置出错，无法运动\n");
		return -1;
	}
	int rfidnum[4],hitavoid[3];
	rfidnum[0]=0;rfidnum[1]=3;rfidnum[2]=1;rfidnum[3]=2;
	hitavoid[0]=0;hitavoid[1]=1;hitavoid[2]=3;


	int direc=4;//行进方向右
	float PointDistance[2];
	PointDistance[0]=slowpoint;
	PointDistance[1]=slowpoint+2;
	int AdjustNavi[2];//纠偏使用的导航,第一个是前导航，第二个是后导航
	AdjustNavi[0]=0;AdjustNavi[1]=2;
	INTEGER32 M2_position[24];//纠偏所需的旋转电机的位置值，从0-23依次是前轮、后轮所需的24个赋值。
	//前轮偏左
	M2_position[0]=Anticlockwise_Rotator_Crab+Clockwis_Degree3;
	M2_position[1]=Clockwise_Rotator_Crab;
	M2_position[2]=Anticlockwise_Rotator_Crab;
	M2_position[3]=Clockwise_Rotator_Crab+Clockwis_Degree3;
	//前轮偏右
	M2_position[4]=Anticlockwise_Rotator_Crab+AntiClockwis_Degree3;
	M2_position[5]=Clockwise_Rotator_Crab;
	M2_position[6]=Anticlockwise_Rotator_Crab;
	M2_position[7]=Clockwise_Rotator_Crab+AntiClockwis_Degree3;
	//前轮归零*/
	M2_position[8]=Anticlockwise_Rotator_Crab;
	M2_position[9]=Clockwise_Rotator_Crab;
	M2_position[10]=Anticlockwise_Rotator_Crab;
	M2_position[11]=Clockwise_Rotator_Crab;
	//后轮偏右
	M2_position[12]=Anticlockwise_Rotator_Crab;
	M2_position[13]=Clockwise_Rotator_Crab+Clockwis_Degree4;
	M2_position[14]=Anticlockwise_Rotator_Crab+Clockwis_Degree4;
	M2_position[15]=Clockwise_Rotator_Crab;
	//后轮偏左
	M2_position[16]=Anticlockwise_Rotator_Crab;
	M2_position[17]=Clockwise_Rotator_Crab+AntiClockwis_Degree4;
	M2_position[18]=Anticlockwise_Rotator_Crab+AntiClockwis_Degree4;
	M2_position[19]=Clockwise_Rotator_Crab;
	//后轮归零
	M2_position[20]=Anticlockwise_Rotator_Crab;
	M2_position[21]=Clockwise_Rotator_Crab;
	M2_position[22]=Anticlockwise_Rotator_Crab;
	M2_position[23]=Clockwise_Rotator_Crab;

	rfid[0]=rfidTarget;
	rfid[1]=rfid[0];
	rfid[2]=rfid[0];
	rfid[3]=rfid[0];

	while(agvreportstruct.MotorError==0)
	{
		/*通过第一个组RFID后，开始检测行走电机是否到位，并修改纠偏宽度*/
			if(innerstruct.SlowCommand==2)
			{
				/*此处扩大纠偏范围，减少进入纠偏次数*/
//					while(((R1_statusword & (1<<(10)))==0) ||
//					   ((R2_statusword & (1<<(10)))==0) ||
//					   ((R3_statusword & (1<<(10)))==0) ||
//					   ((R4_statusword & (1<<(10)))==0) )//如果四个都到
				while(((R1_statusword & (1<<(10)))==0) &&
				   ((R2_statusword & (1<<(10)))==0) &&
				   ((R3_statusword & (1<<(10)))==0) &&
				   ((R4_statusword & (1<<(10)))==0) )//如果任意一个行走电机未到位置，则等待 屏蔽纠偏+蔽障，防止被改为绝对距离
					{
						;
					}
					printf("运动触发RFID后到达停止点\n");
					AgvStop();
					return 0;//返回0推出函数
			}
		if(innerstruct.StopCommand==0)
		{
			//WalkingWhileAssist(direc,rfidnum,hitavoid,PointDistance,AdjustNavi,M2_position);
			int ret=WalkingWhileAssist(direc,rfidnum,hitavoid,PointDistance,AdjustNavi,M2_position);
			if(ret==4)//如果在WalkingWhileAssist中收到停止标志位置1，返回特殊值4，退出while循环。
			{
				return 0;
			}
			else if(ret==-1)//如果在WalkingWhileAssist中触发了紧急停止，返回特殊值-1，退出while循环，同时返回-1。
			{
				return -1;
			}
		}
		else if(innerstruct.StopCommand==1)
		{
			return WalkingWhileAssist(direc,rfidnum,hitavoid,PointDistance,AdjustNavi,M2_position);
		}
	}//while循环结束，如果程序走到这里，说明上面所有的return都没有执行，那么agvreportstruct.MotorError肯定被置1,执行下面内容：

	printf("行走时，触发EMCY，行走方向右移\n");
	Log("行走时，触发EMCY，行走方向右移\n");
	return -1;
}

int AgvStop()
{	//上位机让AGV停止命令！！ 并不是AGV内部错误紧急停止！！！！！
	/*停止4个行走电机*/
	M1_TPDO_control_word = WalkStopPositionDisable; //断行走始能
	//M1_TPDO_control_word = WalkStopPosition;
	/*停止4个旋转电机*/
	//M2_TPDO_control_word = RotatorStop;//不断使能
	M2_TPDO_control_word = RotatorStopDisable; //断时能
	/*停止举升电机*/
	M3_TPDO_control_word = LiftStop; //断使能
//	norfidstop=0;
	agvreportstruct.ExecuteCommand=0;
	sendPDOevent(CANOpenShellOD_Data);
	LookingRFIDCount=0;
	printf("使能断开\n");
	return 0;
}

int AgvEmcyStop()
{	//AGV内部限位开关 电机反馈错误信息 断线 紧急停止！！！  最好能退出程序  需要追踪 reurn-1后到哪

	agvreportstruct.MotorError=1;//告诉上位机，电机发生故障
	innerstruct.StopCommand=1;

	//给九个电机发送停止命令//其实最主要的是四个直行电机可能在运行中被终止或中止，
	//四个旋转电机和一个升降电机在运行过程中一般不会遇到停止信号，但为了保险起见，向所有电机都发停止命令也不会造成不良影响。

	//红灯点亮 闭合K6=红灯+蜂鸣器； 绿灯熄灭 断开K4
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
	M1_TPDO_control_word = WalkStopPosition;
	/*停止4个旋转电机*/
	M2_TPDO_control_word = RotatorStop;
	/*停止举升电机*/
	M3_TPDO_control_word = LiftStop;
	sendPDOevent(CANOpenShellOD_Data);

	return -1;
}

//充电命令
int AgvCharge(char State)
{
	//闭合继电器K1=close K1闭合时，K2一定=Open断开状态；设置flag标志位，K2闭合=close时，要判断K1的标志位
	ChargeFlag=1; //开始充电，K2不能闭合
	RelayOpen(K2);//断开电机电源K2
	RelayClose(K9); //断开驱动器
	sendPDOevent(CANOpenShellOD_Data);
	printf("断开驱动器电源,延迟1S后断开K2,闭合K9\n");
	//延迟1s
	//加入一个延迟，0.5S后断开K2
	int delaycount=0;
	while(delaycount<=100000000) //0.5S
		{
			delaycount++;
		} //100000000大概1.2s左右；300000000=4.5s左右
	printf("延迟结束,黄灯点亮,闭合K1,开始充电\n");
	RelayClose(K1);//闭合充电继电器K1
	RelayClose(K5);//黄灯亮=K5=close
	RelayOpen(K4);
	RelayOpen(K6);
	sendPDOevent(CANOpenShellOD_Data);
	return 0;
}

int WalkingPreperationSpeedSet(char fastslow,int dir,float slowpoint)
{
	//将LookingRFIDPoint转化为LookingRFIDPointCount，并用于开始降速进入寻找RFID判断
	if(CalLookingRFIDStartPosition()!=0)
	{
		printf("寻找RFID距离计算失败\n");
		Log("寻找RFID距离计算失败");
		return -1;
	}
	else
	{
		printf("电机寻找RFID距离计算成功\n");
		Log("电机寻找RFID距离计算成功");
	}
    /**************************************************************************/
	//计算LookingRFID的速度
	if(CalLookingRFIDSpeed()!=0)
	{
		printf("计算CalLookingRFIDSpeed失败\n");
		Log("计算CalLookingRFIDSpeed失败");
		return -1;
	}
	else
	{
		printf("计算CalLookingRFIDSpeed成功\n");
		Log("计算CalLookingRFIDSpeed成功");
	}
    /**************************************************************************/
	/*重设速度为正常值*/
	M1_TPDO_Profile_Velocity=Clockwise_WalkNomalSpeed;
	sendPDOevent(CANOpenShellOD_Data);
	printf("重设行走电机位置模式速度==Clockwise_WalkNomalSpeed\n");
    /**************************************************************************/

	if(dir==1)//如果是前行
	{
		//提前计算fix修正行走距离，减少while时间
		if(FixedWalkingPosition(FixedFordwardStopPoint)!=0)
		{
			printf("提前计算fix修正行走距离失败\n");
			Log("提前计算fix修正行走距离失败\n");
			return -1;
		}
		else
		{
			printf("提前计算fix修正行走距离完成\n");
			Log("提前计算fix修正行走距离完成\n");
		}

			/*行走前转向电机回0位置*/
		if(RotateWheel2Zero()!=0)
		{
			printf("直行前转向电机回0失败\n");
			Log("直行前转向电机回0失败");
			return -1;
		}
		else
		{
			printf("直行前转向电机回0完成\n");
			Log("直行前转向电机回0完成\n");
		}

		//根据停止点，计算行走电机距离参数
		if(SetWalkingPosition(slowpoint)!=0)
		{
			printf("前进时电机初始距离计算失败\n");
			Log("前进时电机初始距离计算失败");
			return -1;
		}
		else
		{
			printf("前进时电机初始距离计算成功\n");
			Log("前进时电机初始距离计算成功");
		}

		//计算前进时电机寻找RFID后第二次的停止点
		if(CalLookingRFIDStopPosition(LookingRFIDForwardStopPoint)!=0)
		{
			printf("前进时电机寻找RFID预估停止点计算失败\n");
			Log("前进时电机寻找RFID预估停止点计算失败");
			return -1;
		}
		else
		{
			printf("前进时电机寻找RFID预估停止点计算成功\n");
			Log("前进时电机寻找RFID预估停止点计算成功");
		}

		/*计算目标点counts值*/
		CommandPosition1=R1_position+WalkingPositive;
		CommandPosition2=R2_position+WalkingNegative;
		CommandPosition3=R3_position+WalkingPositive;
		CommandPosition4=R4_position+WalkingNegative;



		if(fastslow=='0')//爬行速度
		{	//给四个直行电机发送前进命令
			/*前进 行走电机：1 3是正数，2 4是负数*/
			M1_1_TPDO_Target_Position = CommandPosition1;
			M1_3_TPDO_Target_Position = CommandPosition3;

			M1_2_TPDO_Target_Position = CommandPosition2;
			M1_4_TPDO_Target_Position = CommandPosition4;
			//可以再设置一下速度
			sendPDOevent(CANOpenShellOD_Data);

			/*启动*/
			M1_TPDO_control_word = WalkStartPosition;
			sendPDOevent(CANOpenShellOD_Data);
			Log("agv爬行速度前进");
			printf("agv爬行速度前进\n");
		}
		if(fastslow=='1')//正常速度
		{	//给四个直行电机发送前进命令
			/*前进 行走电机：1 3是正数，2 4是负数*/
			M1_1_TPDO_Target_Position = CommandPosition1;
			M1_3_TPDO_Target_Position = CommandPosition3;

			M1_2_TPDO_Target_Position = CommandPosition2;
			M1_4_TPDO_Target_Position = CommandPosition4;
			//可以再设置一下速度
			sendPDOevent(CANOpenShellOD_Data);

			/*启动*/
			M1_TPDO_control_word = WalkStartPosition;
			sendPDOevent(CANOpenShellOD_Data);
			Log("agv正常速度前进");
			printf("agv正常速度前进\n");
		}
	}
	else if(dir==2)//后退
	{
		//提前计算fix修正行走距离，减少while时间
		if(FixedWalkingPosition(FixedFordwardStopPoint)!=0)
		{
			printf("提前计算fix修正行走距离失败\n");
			Log("提前计算fix修正行走距离失败\n");
			return -1;
		}
		else
		{
			printf("提前计算fix修正行走距离完成\n");
			Log("提前计算fix修正行走距离完成\n");
		}

			/*行走前转向电机回0位置*/
		if(RotateWheel2Zero()!=0)
		{
			printf("直行前转向电机回0失败\n");
			Log("直行前转向电机回0失败");
			return -1;
		}
		else
		{
			printf("直行前转向电机回0完成\n");
			Log("直行前转向电机回0完成\n");
		}

		//根据停止点，计算行走电机距离参数
		if(SetWalkingPosition(slowpoint)!=0)
		{
			printf("后退时电机初始距离计算失败\n");
			Log("后退时电机初始距离计算失败");
			return -1;
		}
		else
		{
			printf("后退时电机初始距离计算成功\n");
			Log("后退时电机初始距离计算成功");
		}

		//计算前进时电机寻找RFID后第二次的停止点
		if(CalLookingRFIDStopPosition(LookingRFIDForwardStopPoint)!=0)
		{
			printf("后退时电机寻找RFID预估停止点计算失败\n");
			Log("后退时电机寻找RFID预估停止点计算失败");
			return -1;
		}
		else
		{
			printf("后退时电机寻找RFID预估停止点距离计算成功\n");
			Log("后退时电机寻找RFID预估停止点计算成功");
		}

		/*计算目标点counts值*/
		CommandPosition1=R1_position+WalkingNegative;
		CommandPosition2=R2_position+WalkingPositive;
		CommandPosition3=R3_position+WalkingNegative;
		CommandPosition4=R4_position+WalkingPositive;



		if(fastslow=='0')//爬行速度
		{	//给四个直行电机发送后退命令
			/*后退 行走电机：1 3负数，2 4是正数*/
			M1_1_TPDO_Target_Position = CommandPosition1;
			M1_3_TPDO_Target_Position = CommandPosition3;

			M1_2_TPDO_Target_Position = CommandPosition2;
			M1_4_TPDO_Target_Position = CommandPosition4;
			//可以设置速度
			sendPDOevent(CANOpenShellOD_Data);

			/*启动*/
			M1_TPDO_control_word = WalkStartPosition;
			sendPDOevent(CANOpenShellOD_Data);
			Log("agv爬行速度后退");
			printf("agv爬行速度后退\n");
		}
		if(fastslow=='1')//正常速度
		{	//给四个直行电机发送后退命令
			/*后退 行走电机：1 3负数，2 4是正数*/
			M1_1_TPDO_Target_Position = CommandPosition1;
			M1_3_TPDO_Target_Position = CommandPosition3;

			M1_2_TPDO_Target_Position = CommandPosition2;
			M1_4_TPDO_Target_Position = CommandPosition4;
			//可以设置速度
			sendPDOevent(CANOpenShellOD_Data);

			/*启动*/
			M1_TPDO_control_word = WalkStartPosition;
			sendPDOevent(CANOpenShellOD_Data);
			Log("agv正常速度后退");
			printf("agv正常速度后退\n");
		}
	}
	else if(dir==3)//左移
	{
		//提前计算fix修正行走距离，减少while时间
		if(FixedWalkingPosition(FixedCrabStopPoint)!=0)
		{
			printf("提前计算fix修正行走距离失败\n");
			Log("提前计算fix修正行走距离失败\n");
			return -1;
		}
		else
		{
			printf("提前计算fix修正行走距离完成\n");
			Log("提前计算fix修正行走距离完成\n");
		}

			/*行走前转向电机回横移位置*/
			if(RotateWheel2LeftRight()!=0)
			{
				printf("左移前转向电机回90失败\n");
				Log("左移前转向电机回90失败");
				return -1;
			}
			else
			{
				printf("左移前转向电机回90完成\n");
				Log("左移前转向电机回90完成\n");
			}

			//根据停止点，计算行走电机距离参数
			if(SetWalkingPosition(slowpoint)!=0)
			{
				printf("左移时电机初始距离计算失败\n");
				Log("左移时电机初始距离计算失败");
				return -1;
			}
			else
			{
				printf("左移时电机初始距离计算成功\n");
				Log("左移时电机初始距离计算成功");
			}

			//前进时电机寻找RFID距离计算
			if(CalLookingRFIDStopPosition(LookingRFIDCrabStopPoint)!=0)
			{
				printf("左移时电机寻找RFID预估停止点计算失败\n");
				Log("左移时电机寻找RFID预估停止点计算失败");
				return -1;
			}
			else
			{
				printf("左移时电机寻找RFID预估停止点计算成功\n");
				Log("左移时电机寻找RFID预估停止点计算成功");
			}

			/*计算目标点counts值*/
			CommandPosition1=R1_position+WalkingPositive;
			CommandPosition2=R2_position+WalkingPositive;
			CommandPosition3=R3_position+WalkingPositive;
			CommandPosition4=R4_position+WalkingPositive;



			if(fastslow=='0')//爬行速度
			{
				/* 行走电机 向左横移：
				 * 行走电机1 3=前进	1=正数 3=正数
				 * 行走电机2 4=后退	2=正数 4=正数*/
				M1_1_TPDO_Target_Position = CommandPosition1;
				M1_3_TPDO_Target_Position = CommandPosition3;

				M1_2_TPDO_Target_Position = CommandPosition2;
				M1_4_TPDO_Target_Position = CommandPosition4;
				//可以设置速度
				sendPDOevent(CANOpenShellOD_Data);

				/*启动*/
				M1_TPDO_control_word = WalkStartPosition;
				sendPDOevent(CANOpenShellOD_Data);
				Log("agv爬行速度向左横移");
				printf("agv爬行速度向左横移\n");
			}
			if(fastslow=='1')//正常速度
			{
				/* 行走电机 向左横移：
				 * 行走电机1 3=前进	1=正数 3=正数
				 * 行走电机2 4=后退	2=正数 4=正数*/
				M1_1_TPDO_Target_Position = CommandPosition1;
				M1_3_TPDO_Target_Position = CommandPosition3;

				M1_2_TPDO_Target_Position = CommandPosition2;
				M1_4_TPDO_Target_Position = CommandPosition4;
				//可以设置速度
				sendPDOevent(CANOpenShellOD_Data);

				/*启动*/
				M1_TPDO_control_word = WalkStartPosition;
				sendPDOevent(CANOpenShellOD_Data);
				Log("agv正常速度向左横移");
				printf("agv正常速度向左横移\n");
			}
		}
	else if(dir==4)//右移
	{
		//提前计算fix修正行走距离，减少while时间
		if(FixedWalkingPosition(FixedCrabStopPoint)!=0)
		{
			printf("提前计算fix修正行走距离失败\n");
			Log("提前计算fix修正行走距离失败\n");
			return -1;
		}
		else
		{
			printf("提前计算fix修正行走距离完成\n");
			Log("提前计算fix修正行走距离完成\n");
		}

		/*行走前转向电机回横移位置*/
		if(RotateWheel2LeftRight()!=0)
		{
			printf("右移前转向电机回90失败\n");
			Log("右移前转向电机回90失败");
			return -1;
		}
		else
		{
			printf("右移前转向电机回90完成\n");
			Log("右移前转向电机回90完成\n");
		}

		//根据停止点，计算行走电机距离参数
		if(SetWalkingPosition(slowpoint)!=0)
		{
			printf("右移时电机初始距离计算失败\n");
			Log("右移时电机初始距离计算失败");
			return -1;
		}
		else
		{
			printf("右移时电机初始距离计算成功\n");
			Log("右移时电机初始距离计算成功");
		}

		//前进时电机寻找RFID距离计算
		if(CalLookingRFIDStopPosition(LookingRFIDCrabStopPoint)!=0)
		{
			printf("右移时电机寻找RFID预估停止点计算失败\n");
			Log("右移时电机寻找RFID预估停止点计算失败");
			return -1;
		}
		else
		{
			printf("右移时电机寻找RFID预估停止点计算成功\n");
			Log("右移时电机寻找RFID预估停止点计算成功");
		}

		/*计算目标点counts值*/
		CommandPosition1=R1_position+WalkingNegative;
		CommandPosition2=R2_position+WalkingNegative;
		CommandPosition3=R3_position+WalkingNegative;
		CommandPosition4=R4_position+WalkingNegative;



		if(fastslow=='0')//爬行速度
		{
			/* 行走电机 向右横移：
			 * 行走电机1 3=后退	1=负数 3=负数
			 * 行走电机2 4=前进	2=负数 4=负数*/
			M1_1_TPDO_Target_Position = CommandPosition1;
			M1_3_TPDO_Target_Position = CommandPosition3;

			M1_2_TPDO_Target_Position = CommandPosition2;
			M1_4_TPDO_Target_Position = CommandPosition4;
			//可以设置速度
			sendPDOevent(CANOpenShellOD_Data);

			/*启动*/
			M1_TPDO_control_word = WalkStartPosition;
			sendPDOevent(CANOpenShellOD_Data);
			Log("agv爬行速度向右横移");
			printf("agv爬行速度向右横移\n");
		}
		if(fastslow=='1')//正常速度
		{
			/* 行走电机 向右横移：
			 * 行走电机1 3=后退	1=负数 3=负数
			 * 行走电机2 4=前进	2=负数 4=负数*/
			M1_1_TPDO_Target_Position = CommandPosition1;
			M1_3_TPDO_Target_Position = CommandPosition3;

			M1_2_TPDO_Target_Position = CommandPosition2;
			M1_4_TPDO_Target_Position = CommandPosition4;
			//可以设置速度
			sendPDOevent(CANOpenShellOD_Data);

			/*启动*/
			M1_TPDO_control_word = WalkStartPosition;
			sendPDOevent(CANOpenShellOD_Data);
			Log("agv正常速度向右横移");
			printf("agv正常速度向右横移\n");
		}
	}

	//向电机发出行走命令后，马上记录直行电机的编码器初始值，这里暂时是以行走方向的两个后轮的编码器值的平均值来计算行走距离的。
	//取四个直行电机的编码器的初始位置，即直行电机开始运动前记录其编码器的初始值
	encodeOldR1=R1_position;
	encodeOldR2=R2_position;
	encodeOldR3=R3_position;
	encodeOldR4=R4_position;
	meterDistance=0;//调用前、后、左、右运动的程序运行前，把距离值归零。以米计算
	distance=0;//调用前、后、左、右运动的程序运行前，把距离值归零。以转数计算
	correctNaviError=0;//把导航错误标志位归零，每次行走时调用重新累计。

	//标志位清零。
	innerstruct.StopCommand=0;//把电机停止命令标志置0
	innerstruct.SlowCommand=0;//减速标志位

	/*重置蔽障标志位=0*/
	AvoidFlag=0;

	//记录直行电机的初始速度值，因为直行时有两种速度，一种是正常速度，一种是爬行速度
	motor1=M1_1_TPDO_target_speed;
	motor2=M1_2_TPDO_target_speed;
	motor3=M1_3_TPDO_target_speed;
	motor4=M1_4_TPDO_target_speed;

	//使用行走方向的后面两个rfid读卡器值的变化作为判断小车到达指定位置并停止的依据，因为rfid不清空，当小车启动时，默认四个rfid读卡器都能读到标签，
	//当行进方向的两个前轮读到新的rfid标签时，其实它们两个读到的是应该后两轮的停止标签，所以这个变化不能用，此时后两轮的读卡器中存储的依然是起始位
	//的标签，等到后两轮读到的标签更新后，就认为小车到达了下一个车位，应该停止运动，如果要严格判断的话，此时还可以判断前两轮读到的标签的最后1位加上
	//后两轮读到标签的最后1位的和是不是10,

//	rfid[0]=agvreportstruct.RFIDSenser[0][2];
//	rfid[1]=agvreportstruct.RFIDSenser[1][2];
//	rfid[2]=agvreportstruct.RFIDSenser[2][2];
//	rfid[3]=agvreportstruct.RFIDSenser[3][2];

	AdjustFlag=0;
	AdjustCount=0;
	agvreportstruct.RFIDSenser[0][2]=0;
	agvreportstruct.RFIDSenser[1][2]=0;
	agvreportstruct.RFIDSenser[2][2]=0;
	agvreportstruct.RFIDSenser[3][2]=0;
	return 0;
}

/*读到RFID，使小车继续前进FixedFordwardStopPoint*/
int WalkingFixPositon(int dir)
{
	//轮子回0
	RotateWheelReturn(dir);

	if(dir==1)//如果是前行
	{
			//给四个直行电机发送前进命令
			/*前进 行走电机：1 3是正数，2 4是负数*/
			M1_1_TPDO_Target_Position = WalkingPositiveFixed;
			M1_3_TPDO_Target_Position = WalkingPositiveFixed;

			M1_2_TPDO_Target_Position = WalkingNegativeFixed;
			M1_4_TPDO_Target_Position = WalkingNegativeFixed;
			//可以再设置一下速度
			sendPDOevent(CANOpenShellOD_Data);

			/*启动*/
			M1_TPDO_control_word = WalkStartPositionReletive;
			sendPDOevent(CANOpenShellOD_Data);
	}
	else if(dir==2)//后退
	{
			//给四个直行电机发送后退命令
			/*后退 行走电机：1 3负数，2 4是正数*/
			M1_1_TPDO_Target_Position = WalkingNegativeFixed;
			M1_3_TPDO_Target_Position = WalkingNegativeFixed;

			M1_2_TPDO_Target_Position = WalkingPositiveFixed;
			M1_4_TPDO_Target_Position = WalkingPositiveFixed;

			//可以设置速度
			sendPDOevent(CANOpenShellOD_Data);

			/*启动*/
			M1_TPDO_control_word = WalkStartPositionReletive;
			sendPDOevent(CANOpenShellOD_Data);
	}
	else if(dir==3)//左移
		{
				/* 行走电机 向左横移：
				 * 行走电机1 3=前进	1=正数 3=正数
				 * 行走电机2 4=后退	2=正数 4=正数*/
				M1_1_TPDO_Target_Position = WalkingPositiveFixed;
				M1_3_TPDO_Target_Position = WalkingPositiveFixed;

				M1_2_TPDO_Target_Position = WalkingPositiveFixed;
				M1_4_TPDO_Target_Position = WalkingPositiveFixed;

				//可以设置速度
				sendPDOevent(CANOpenShellOD_Data);

				/*启动*/
				M1_TPDO_control_word = WalkStartPositionReletive;
				sendPDOevent(CANOpenShellOD_Data);
		}
	else if(dir==4)//右移
	{

			/* 行走电机 向右横移：
			 * 行走电机1 3=后退	1=负数 3=负数
			 * 行走电机2 4=前进	2=负数 4=负数*/
			M1_1_TPDO_Target_Position = WalkingNegativeFixed;
			M1_3_TPDO_Target_Position = WalkingNegativeFixed;

			M1_2_TPDO_Target_Position = WalkingNegativeFixed;
			M1_4_TPDO_Target_Position = WalkingNegativeFixed;
			//可以设置速度
			sendPDOevent(CANOpenShellOD_Data);

			/*启动*/
			M1_TPDO_control_word = WalkStartPositionReletive;
			sendPDOevent(CANOpenShellOD_Data);
	}
//	printf("agv读到RFID后修该距离完成\n");
	return 0;
}

/*将速度降为小于0.1788m/s的速度(0.4加速度4cm正好停下)，将绝对停止点向前移2m
 * 此函数值进入1此！！！*/
int LookingRFID(int dir)
{
	/*设置小于0.1788m/s的速度*/
	M1_TPDO_Profile_Velocity=LookingRFIDSpeedCount;
	sendPDOevent(CANOpenShellOD_Data);
	/*此时，速度并没有被执行，需要bit4 0-1的转换，发送完距离后，统一发送0-1转换*/
	printf("开始减速，速度设置<0.1788m/s\n");

	if(dir==1)//如果是前行
	{
			/*计算目标点counts值*/
			CommandPosition1=CommandPosition1+LookingRFIDWalkingPositiveCount;
			CommandPosition2=CommandPosition2+LookingRFIDWalkingNegativeCount;
			CommandPosition3=CommandPosition3+LookingRFIDWalkingPositiveCount;
			CommandPosition4=CommandPosition4+LookingRFIDWalkingNegativeCount;

			//给四个直行电机发送前进命令
			/*前进 行走电机：1 3是正数，2 4是负数*/
			M1_1_TPDO_Target_Position = CommandPosition1;
			M1_3_TPDO_Target_Position = CommandPosition3;

			M1_2_TPDO_Target_Position = CommandPosition2;
			M1_4_TPDO_Target_Position = CommandPosition4;

			sendPDOevent(CANOpenShellOD_Data);

			/*启动*/
			M1_TPDO_control_word = WalkStartPosition;
			sendPDOevent(CANOpenShellOD_Data);

	}
	else if(dir==2)//后退
	{
			/*计算目标点counts值*/
			CommandPosition1=CommandPosition1+LookingRFIDWalkingNegativeCount;
			CommandPosition2=CommandPosition2+LookingRFIDWalkingPositiveCount;
			CommandPosition3=CommandPosition3+LookingRFIDWalkingNegativeCount;
			CommandPosition4=CommandPosition4+LookingRFIDWalkingPositiveCount;

			//给四个直行电机发送后退命令
			/*后退 行走电机：1 3负数，2 4是正数*/
			M1_1_TPDO_Target_Position = CommandPosition1;
			M1_3_TPDO_Target_Position = CommandPosition3;

			M1_2_TPDO_Target_Position = CommandPosition2;
			M1_4_TPDO_Target_Position = CommandPosition4;

			//可以设置速度
			sendPDOevent(CANOpenShellOD_Data);

			/*启动*/
			M1_TPDO_control_word = WalkStartPosition;
			sendPDOevent(CANOpenShellOD_Data);
//			Log("agv修正后距离后退");
//			printf("agv修正后距离后退\n");
	}
	else if(dir==3)//左移
		{
				/*计算目标点counts值*/
				CommandPosition1=CommandPosition1+LookingRFIDWalkingPositiveCount;
				CommandPosition2=CommandPosition2+LookingRFIDWalkingPositiveCount;
				CommandPosition3=CommandPosition3+LookingRFIDWalkingPositiveCount;
				CommandPosition4=CommandPosition4+LookingRFIDWalkingPositiveCount;

				/* 行走电机 向左横移：
				 * 行走电机1 3=前进	1=正数 3=正数
				 * 行走电机2 4=后退	2=正数 4=正数*/
				M1_1_TPDO_Target_Position = CommandPosition1;
				M1_3_TPDO_Target_Position = CommandPosition3;

				M1_2_TPDO_Target_Position = CommandPosition2;
				M1_4_TPDO_Target_Position = CommandPosition4;

				//可以设置速度
				sendPDOevent(CANOpenShellOD_Data);

				/*启动*/
				M1_TPDO_control_word = WalkStartPosition;
				sendPDOevent(CANOpenShellOD_Data);
//				Log("agv修正后距离向左横移");
//				printf("agv修正后距离向左横移\n");
		}
	else if(dir==4)//右移
	{
			/*计算目标点counts值*/
			CommandPosition1=CommandPosition1+LookingRFIDWalkingNegativeCount;
			CommandPosition2=CommandPosition2+LookingRFIDWalkingNegativeCount;
			CommandPosition3=CommandPosition3+LookingRFIDWalkingNegativeCount;
			CommandPosition4=CommandPosition4+LookingRFIDWalkingNegativeCount;

			/* 行走电机 向右横移：
			 * 行走电机1 3=后退	1=负数 3=负数
			 * 行走电机2 4=前进	2=负数 4=负数*/
			M1_1_TPDO_Target_Position = CommandPosition1;
			M1_3_TPDO_Target_Position = CommandPosition3;

			M1_2_TPDO_Target_Position = CommandPosition2;
			M1_4_TPDO_Target_Position = CommandPosition4;

			//可以设置速度
			sendPDOevent(CANOpenShellOD_Data);

			/*启动*/
			M1_TPDO_control_word = WalkStartPosition;
			sendPDOevent(CANOpenShellOD_Data);
//			Log("agv修正后距离向右横移");
//			printf("agv修正后距离向右横移\n");
	}
//	printf("LookingRFID__M1_1_TPDO_Target_Position==%d \n",M1_1_TPDO_Target_Position);
	Log("agv开始寻找RFID");
	printf("agv开始寻找RFID\n");
	return 0;
}

int WalkingWhileAssist(int direc,int rfidnum[4],int hitavoid[3],float PointDistance[2],int AdjustNavi[2],INTEGER32 M2_position[24])
{
	int A,B;
	AdjustCount++;
//	norfidstop++;

	/*将bit4置0*/
	M1_TPDO_control_word=(M1_TPDO_control_word&(~0x10));//bit 4=0
	sendPDOevent(CANOpenShellOD_Data);

	//如果运行方向前面两个rfid中至少一个读到的第三位（车位号）发生了变化，说明小车已经接近走到了下一个车位，应该让小车减速。
//	if(rfid[rfidnum[2]]==agvreportstruct.RFIDSenser[rfidnum[2]][2]||rfid[rfidnum[3]]==agvreportstruct.RFIDSenser[rfidnum[3]][2]) //后RFID
	//如果读到TAG且是需要的TAG，则再往前走几厘米停车
	if(PGV_Data_Matrixtag_TAG00_TAG07==(rfid[0]-48))
	{
		if(innerstruct.SlowCommand!=2)//防止对停止点重复赋值，第一次进入这个if，innerstruct.SlowCommand没有置2,对停止点赋值，第二次再进入这个if,innerstruct.SlowCommand已经被置2,不会再赋值。
		{
			/*将bit4置0*/
			M1_TPDO_control_word=(M1_TPDO_control_word&(~0x10));//bit 4=0
			sendPDOevent(CANOpenShellOD_Data);

			//根据停止点，计算行走电机距离参数
			int ret=WalkingFixPositon(direc);
			if(ret!=0)
			{
				printf("RFID修正停止距离出错\n");
				Log("RFID修正停止距离出错");
				return -1;
			}
			else
			{
				printf("RFID修正停止距离成功\n");
				Log("RFID修正停止距离成功");
				printf("PGV_Data_Matrixtag_TAG00_TAG07==%x\n",PGV_Data_Matrixtag_TAG00_TAG07);
			}
		}
		innerstruct.SlowCommand=2;
		printf("M1_1_TPDO_Target_Position==%d \n",M1_1_TPDO_Target_Position);
		return 0;//不再进行其他判断，屏蔽传感器
	}

	/*没有到达第一组RFID，正常宽度纠偏*/
	if(innerstruct.SlowCommand==0)
	{
		/*正常纠偏宽度*/ //上下1个
		A=0x01E0;
		B=0x0780;
	}

	/* 如果距离停止点距离小于LookingRFIDPoint，则发送LookingRFIDSpeed=0.1788m/s，并设置新相对目标点距离=3m
	 * 为避免重复赋值，进在LookingRFIDCount=0第一次进入，找RFID赋值函数只进入一次，在AGVSTOP中将LookingRFIDCount==0*/
	if(LookingRFIDCount==0)
	{
//	if((
//		(abs(abs(CommandPosition3)-abs(R3_position))<LookingRFIDPointCount) ||
//		(abs(abs(CommandPosition1)-abs(R1_position))<LookingRFIDPointCount) ||
//		(abs(abs(CommandPosition2)-abs(R2_position))<LookingRFIDPointCount) ||
//		(abs(abs(CommandPosition4)-abs(R4_position))<LookingRFIDPointCount) )
//		&& innerstruct.SlowCommand!=2  )
		if((
			(abs(CommandPosition3-R3_position)<LookingRFIDPointCount) ||
			(abs(CommandPosition1-R1_position)<LookingRFIDPointCount) ||
			(abs(CommandPosition2-R2_position)<LookingRFIDPointCount) ||
			(abs(CommandPosition4-R4_position)<LookingRFIDPointCount) )
			&& innerstruct.SlowCommand!=2  )
	{
		/*将bit4置0*/
		M1_TPDO_control_word=(M1_TPDO_control_word&(~0x10));//bit 4=0
		sendPDOevent(CANOpenShellOD_Data);

		LookingRFIDCount=1;
		int ret=LookingRFID(direc);
		if(ret!=0)
		{
			printf("修正停止距离出错\n");
			Log("修正停止距离出错");
			return -1;
		}
		else
		{
			//			printf("修正停止距离成功\n");
			//			Log("修正停止距离成功");
		}
	}//END if(((CommandPosition1-R1_position)<WalkingPositiveFixed) && innerstruct.SlowCommand!=2  )
	} //end if

//	if(direc ==1 ||direc ==3)
//	{
//		if((
//			(abs(abs(CommandPosition3)-abs(R3_position))<LookingRFIDPointCount) ||
//			(abs(abs(CommandPosition1)-abs(R1_position))<LookingRFIDPointCount) ||
//			(abs(abs(CommandPosition2)-abs(R2_position))<LookingRFIDPointCount) ||
//			(abs(abs(CommandPosition4)-abs(R4_position))<LookingRFIDPointCount) )
//			&& innerstruct.SlowCommand!=2  )
//		{
//			int ret=LookingRFID(direc);
//			if(ret!=0)
//			{
//				printf("修正停止距离出错\n");
//				Log("修正停止距离出错");
//				return -1;
//			}
//			else
//			{
//				//			printf("修正停止距离成功\n");
//				//			Log("修正停止距离成功");
//			}
//		}//END if(((CommandPosition1-R1_position)<WalkingPositiveFixed) && innerstruct.SlowCommand!=2  )
//	}//end if(direc ==1 ||direc ==3)
//
//	/*没有读到RFID，多走1CM*/
//	if(direc ==2 ||direc ==4)
//	{
//		if((
//			(abs(CommandPosition3-R3_position)<LookingRFIDPointCount) ||
//			(abs(CommandPosition1-R1_position)<LookingRFIDPointCount) ||
//			(abs(CommandPosition2-R2_position)<LookingRFIDPointCount) ||
//			(abs(CommandPosition4-R4_position)<LookingRFIDPointCount) )
//			&& innerstruct.SlowCommand!=2  )
//		{
//			int ret=LookingRFID(direc);
//			if(ret!=0)
//			{
//				printf("修正停止距离出错\n");
//				Log("修正停止距离出错");
//				return -1;
//			}
//			else
//			{
//				//			printf("修正停止距离成功\n");
//				//			Log("修正停止距离成功");
//			}
//		}//END if(((CommandPosition1-R1_position)<WalkingPositiveFixed) && innerstruct.SlowCommand!=2  )
//	}//end if(direc ==2 ||direc ==4)

	/****判断行走电机是否运行到位！！！return 4*****/
//	if(((R1_statusword & (1<<(10)))!=0) &&
//	   ((R2_statusword & (1<<(10)))!=0) &&
//	   ((R3_statusword & (1<<(10)))!=0) &&
//	   ((R4_statusword & (1<<(10)))!=0) &&norfidstop>50000)//如果任意一个行走电机到位，则立刻停止所有行走电机
////		if(
////			(((R1_statusword & (1<<(10)))!=0) ||
////		   ((R2_statusword & (1<<(10)))!=0) ||
////		   ((R3_statusword & (1<<(10)))!=0) ||
////		   ((R4_statusword & (1<<(10)))!=0) )&&norfidstop>50000)//如果任意一个行走电机到位，则立刻停止所有行走电机
//	{
//		printf("r1 is %d,r2 is %d,r3 is %d,r4 is %d,norfidstop is %d\n",R1_statusword,R2_statusword,R3_statusword,R4_statusword,norfidstop);
//		printf("运动到达停止点\n");
//		AgvStop();
//		return 4;//上级函数收到返回值4时，会退出while循环，结束运动程序。
//	}

	/*蔽障未触发*/
	if(agvreportstruct.HitAvoidSenser[hitavoid[0]]==0 && agvreportstruct.HitAvoidSenser[hitavoid[1]]==0 && agvreportstruct.HitAvoidSenser[hitavoid[2]]==0)//如果前，左、右防撞信号没有触发,不考虑后防撞
	{
		if(innerstruct.StopCommand==0)//如果没有收到停止命令,调用距离和纠偏函数
		{
			RelayOpen(K6);//红灯灭=K6=open
			RelayOpen(K5);//黄灯灭=K5=open
			RelayClose(K4);//绿灯亮

			/*将bit4置0*/
			M1_TPDO_control_word=(M1_TPDO_control_word&(~0x10));//bit 4=0
			sendPDOevent(CANOpenShellOD_Data);

			//标帜位判断，上次是否蔽障触发，若蔽障被触发则速度被修改，需要重新bit4==1；
			if(AvoidFlag!=1)
			{
				//在这里可以再次设置一下正常速度，防止在壁障触发后，速度被修改。
				//这里不能再重设速度了，因为在寻找RFID时，已经修改了速度，若这里重设速度将出问题
				//但不重设速度，行进过程撞到臂章将无法再次启动
				if(LookingRFIDCount==0) //未进入寻找rfid，则速度可重设最高速
				{
				M1_TPDO_Profile_Velocity=Clockwise_WalkNomalSpeed;
				sendPDOevent(CANOpenShellOD_Data);
				M1_TPDO_control_word = WalkStartPosition;
				sendPDOevent(CANOpenShellOD_Data);
				AvoidFlag=1;
				}
				if(LookingRFIDCount==1) //已经进入LookingRfid，则速度要设寻找RFID速度
				{
				M1_TPDO_Profile_Velocity=LookingRFIDSpeedCount;
				sendPDOevent(CANOpenShellOD_Data);
				M1_TPDO_control_word = WalkStartPosition;
				sendPDOevent(CANOpenShellOD_Data);
				AvoidFlag=1;
				}
			}

			int ret=WalkingDistanceAndAdjustRoute(direc,A,B,rfidnum,PointDistance,AdjustNavi,M2_position);//如果WalkingDistanceAndAdjustRoute返回值是4,说明WalkingDistanceAndAdjustRoute调用了AgvStop,此程序应该返回4,让上级程序停止while循环。
			if(ret==4)
			{
				return 4;
			}
			else
			{
				return ret;
			}
		}
		else if(innerstruct.StopCommand==1)//如果行走过程中收到停止命令
		{
			printf("行进过程中收到非正常停止命令\n");
			Log("行进过程中收到非正常停止命令\n");
			AgvStop();
			return 4;//上级函数收到返回值4时，会退出while循环，结束运动程序。
		}
		else
		{
			printf("运动时避障未触发，StopCommand参数错误\n");
			Log("运动时避障未触发，StopCommand参数错误");
			return AgvEmcyStop();//紧急停止，上级函数收到返回值-1时，会退出while循环，结束运动程序。
		}
	}
	else if(agvreportstruct.HitAvoidSenser[hitavoid[0]]==0x02 || agvreportstruct.HitAvoidSenser[hitavoid[1]]==0x02 ||agvreportstruct.HitAvoidSenser[hitavoid[2]]==0x02)//防撞信号2区被触发,电机减速,后面那个导航不考虑
	{
		if(innerstruct.StopCommand==0)//如果没有收到停止命令，进入防撞2区，电机减速，但依然在行走，所以这里还是要继续计算距离和纠偏
		{
			RelayClose(K5);//黄灯亮=K5=close
			RelayOpen(K4);//绿灯灭
			RelayOpen(K6);//红灯灭=K6=close
			/*将bit4置0*/
			M1_TPDO_control_word=(M1_TPDO_control_word&(~0x10));//bit 4=0
			sendPDOevent(CANOpenShellOD_Data);

			//标帜位判断，上次是否蔽障触发，若蔽障被触发则速度被修改，需要重新bit4==1 AvoidFlag=0:初始状态，AvoidFlag=1蔽障未触发，=2蔽障外层触发 =3蔽障内层触发；
			if(AvoidFlag!=2)
			{
				//在这里可以再次设置一下正常速度，防止在壁障触发后，速度被修改。
				M1_TPDO_Profile_Velocity=(LookingRFIDSpeedCount/2);
				sendPDOevent(CANOpenShellOD_Data);
				M1_TPDO_control_word = WalkStartPosition;
				sendPDOevent(CANOpenShellOD_Data);
				AvoidFlag=2;
			}

			int ret=WalkingDistanceAndAdjustRoute(direc,A,B,rfidnum,PointDistance,AdjustNavi,M2_position);//如果WalkingDistanceAndAdjustRoute返回值是4,说明WalkingDistanceAndAdjustRoute调用了AgvStop,此程序应该返回4,让上级程序停止while循环。
			if(ret==4)
			{
					return 4;
			}
			else
			{
				return ret;
			}
		}
		else if(innerstruct.StopCommand==1)//如果行走过程中收到停止命令
		{
			AgvStop();
			printf("防撞2区触发过程中收到非正常停止命令\n");
			Log("防撞2区触发过程中收到非正常停止命令\n");
			return 4;//上级函数收到返回值4时，会退出while循环，结束运动程序。
		}
		else
		{
			printf("慢速行进避障2区触发时 StopCommand参数错误\n");
			Log("慢速行进避障2区触发时 StopCommand参数错误");
			return AgvEmcyStop();//紧急停止，上级函数收到返回值-1时，会退出while循环，结束运动程序。
		}
	}
	else if(agvreportstruct.HitAvoidSenser[hitavoid[0]]==0x03 || agvreportstruct.HitAvoidSenser[hitavoid[1]]==0x03 ||agvreportstruct.HitAvoidSenser[hitavoid[2]]==0x03 )//防撞信号1区2区被同时触发，电机停止
	{
		if(innerstruct.StopCommand==0)//如果没有收到停止命令
		{
			RelayClose(K6);//红灯亮=K6=close
			RelayOpen(K5);//黄灯灭
			RelayOpen(K4);//绿灯灭

			/*将bit4置0*/
			M1_TPDO_control_word=(M1_TPDO_control_word&(~0x10));//bit 4=0
			sendPDOevent(CANOpenShellOD_Data);

			//标帜位判断，上次是否蔽障触发，若蔽障被触发则速度被修改，需要重新bit4==1 AvoidFlag=0:初始状态，AvoidFlag=1蔽障未触发，=2蔽障外层触发 =3蔽障内层触发；
			if(AvoidFlag!=3)
			{
//				//转向电机回0,避免减速过程中轮子偏的导致滑行偏离路线
//				RotateWheelReturn(direc);
				//在这里可以再次设置一下正常速度，防止在壁障触发后，速度被修改。
				M1_TPDO_Profile_Velocity=0x1;  //若最终发送数据=0,则发送前现发送一个不=0的数据，防止信息发不出去
				sendPDOevent(CANOpenShellOD_Data);
				M1_TPDO_Profile_Velocity=0x0;
				sendPDOevent(CANOpenShellOD_Data);
				M1_TPDO_control_word = WalkStartPosition;
				sendPDOevent(CANOpenShellOD_Data);
				AvoidFlag=3;
			}
			int ret=WalkingDistanceAndAdjustRoute(direc,A,B,rfidnum,PointDistance,AdjustNavi,M2_position);//如果WalkingDistanceAndAdjustRoute返回值是4,说明WalkingDistanceAndAdjustRoute调用了AgvStop,此程序应该返回4,让上级程序停止while循环。
			if(ret==4)
			{
					return 4;
			}
			else
			{
				return ret;
			}
			return 0;//返回值为0,上一级函数收到此返回值时，while循环不停止，继续调用这个函数，
		}
		else if (innerstruct.StopCommand==1)
		{
			AgvStop();  //防撞3区时，收到stopcommand=1命令，红灯将一直亮着
			return 4;//上级函数收到返回值4时，会退出while循环，结束运动程序。
		}
		else
		{
			printf("行走避障1+2区同时触发时 StopCommand参数错误\n");
			Log("行走避障1+2区同时触发时 StopCommand参数错误");
			return AgvEmcyStop(); //紧急停止，上级函数收到返回值-1时，会退出while循环，结束运动程序。
		}
	}
	else//正常条件下，上面三个判断条件肯定会进入，一旦进入的话就会在里面返回，不该执行到此else条件中，
	{
		printf("WalkingWhileAssist程序出错,执行到了不该到达的位置\n");
		Log("WalkingWhileAssist程序出错,执行到了不该到达的位置\n");
		AgvStop();
		return -1;
	}

	return -1;//正常情况下，这个return永远不会执行，如果执行，说明程序有问题，应该返回错误信号
}

/*WalkingDistanceAndAdjustRoute函数是计算agv小车走了多远同时纠偏的程序，它是通过记录直行电机编码器的反馈值并换算成米数来实现的
 * 方式是：用一个while循环来不停更新实际距离distance,首先用encodeOld记录电机的上一个编码器值，然后在下一个while
 * 里面把编码器值读到encodeNew里面，并用new值减去old值，这就是一个while里面电机走过的距离，把得到的值累加到
 * distance里面，即可算出距离值，此函数取前进方向的两个后轮来做为衡量标准，因为这两个后轮在纠偏过程中误差会小一些。
 */
int WalkingDistanceAndAdjustRoute(int direc,int a,int b,int rfidnum[4],float PointDistance[2],int AdjustNavi[2],INTEGER32 M2_position[24])
{
	//如果运行方向前面两个rfid中至少一个读到的第三位（车位号）发生了变化，说明小车已经接近走到了下一个车位，应该让小车减速。
//	if(rfid[rfidnum[2]]==agvreportstruct.RFIDSenser[rfidnum[2]][2]||rfid[rfidnum[3]]==agvreportstruct.RFIDSenser[rfidnum[3]][2]) //后RFID
	if(PGV_Data_Matrixtag_TAG00_TAG07==(rfid[0]-48))
	{

		if(innerstruct.SlowCommand!=2)//防止对停止点重复赋值，第一次进入这个if，innerstruct.SlowCommand没有置2,对停止点赋值，第二次再进入这个if,innerstruct.SlowCommand已经被置2,不会再赋值。
		{
			/*将bit4置0*/
			M1_TPDO_control_word=(M1_TPDO_control_word&(~0x10));//bit 4=0
			sendPDOevent(CANOpenShellOD_Data);

			int ret=WalkingFixPositon(direc);
			if(ret!=0)
			{
				printf("RFID修正停止距离出错\n");
				Log("RFID修正停止距离出错");
				return -1;
			}
			else
			{
				printf("RFID修正停止距离成功\n");
				Log("RFID修正停止距离成功");
				printf("PGV_Data_Matrixtag_TAG00_TAG07==%x\n",PGV_Data_Matrixtag_TAG00_TAG07);
			}
		}
		innerstruct.SlowCommand=2;
		printf("M1_1_TPDO_Target_Position==%d \n",M1_1_TPDO_Target_Position);
		return 0;//不再进行其他判断
	}

	/* 如果距离停止点距离小于LookingRFIDPoint，则发送LookingRFIDSpeed=0.1788m/s，并设置新相对目标点距离=3m
	 * 为避免重复赋值，进在LookingRFIDCount=0第一次进入，找RFID赋值函数只进入一次，在AGVSTOP中将LookingRFIDCount==0*/
	if(LookingRFIDCount==0)
	{
//	if((
//		(abs(abs(CommandPosition3)-abs(R3_position))<LookingRFIDPointCount) ||
//		(abs(abs(CommandPosition1)-abs(R1_position))<LookingRFIDPointCount) ||
//		(abs(abs(CommandPosition2)-abs(R2_position))<LookingRFIDPointCount) ||
//		(abs(abs(CommandPosition4)-abs(R4_position))<LookingRFIDPointCount) )
//		&& innerstruct.SlowCommand!=2  )
		if((
			(abs(CommandPosition3-R3_position)<LookingRFIDPointCount) ||
			(abs(CommandPosition1-R1_position)<LookingRFIDPointCount) ||
			(abs(CommandPosition2-R2_position)<LookingRFIDPointCount) ||
			(abs(CommandPosition4-R4_position)<LookingRFIDPointCount) )
			&& innerstruct.SlowCommand!=2  )
	{
		/*将bit4置0*/
		M1_TPDO_control_word=(M1_TPDO_control_word&(~0x10));//bit 4=0
		sendPDOevent(CANOpenShellOD_Data);

		LookingRFIDCount=1;
		int ret=LookingRFID(direc);
		if(ret!=0)
		{
			printf("修正停止距离出错\n");
			Log("修正停止距离出错");
			return -1;
		}
		else
		{
			//			printf("修正停止距离成功\n");
			//			Log("修正停止距离成功");
		}
	}//END if(((CommandPosition1-R1_position)<WalkingPositiveFixed) && innerstruct.SlowCommand!=2  )
	} //end if


//	if(direc ==1 ||direc ==3)
//	{
//		if((((CommandPosition3-R3_position)<LookingRFIDPointCount*positivetime) ||
//			((CommandPosition1-R1_position)<LookingRFIDPointCount*positivetime) ||
//			((CommandPosition2-R2_position)>LookingRFIDPointCount*positivetime) ||
//			((CommandPosition4-R4_position)>LookingRFIDPointCount*positivetime) )
//			&& innerstruct.SlowCommand!=2  )
//		{
//			int ret=LookingRFID(direc);
//			if(ret!=0)
//			{
//				printf("修正停止距离出错\n");
//				Log("修正停止距离出错");
//				return -1;
//			}
//			else
//			{
//				//			printf("修正停止距离成功\n");
//				//			Log("修正停止距离成功");
//			}
//		}//END if(((CommandPosition1-R1_position)<WalkingPositiveFixed) && innerstruct.SlowCommand!=2  )
//	}//end if(direc ==1 ||direc ==3)
//
//	/*没有读到RFID，多走1CM*/
//	if(direc ==2 ||direc ==4)
//	{
////		if(((CommandPosition3-R3_position)>LookingRFIDPointCount*positivetime) && innerstruct.SlowCommand!=2  )
//		if((((CommandPosition3-R3_position)>LookingRFIDPointCount*positivetime) ||
//			((CommandPosition1-R1_position)>LookingRFIDPointCount*positivetime) ||
//			((CommandPosition2-R2_position)<LookingRFIDPointCount*positivetime) ||
//			((CommandPosition4-R4_position)<LookingRFIDPointCount*positivetime) )
//			&& innerstruct.SlowCommand!=2  )
//		{
//			int ret=LookingRFID(direc);
//			if(ret!=0)
//			{
//				printf("修正停止距离出错\n");
//				Log("修正停止距离出错");
//				return -1;
//			}
//			else
//			{
//				//			printf("修正停止距离成功\n");
//				//			Log("修正停止距离成功");
//			}
//		}//END if(((CommandPosition1-R1_position)<WalkingPositiveFixed) && innerstruct.SlowCommand!=2  )
//	}//end if(direc ==2 ||direc ==4)



	/****判断行走电机是否运行到位！！！return 4*****/
//	if(((R1_statusword & (1<<(10)))!=0) &&
//	   ((R2_statusword & (1<<(10)))!=0) &&
//	   ((R3_statusword & (1<<(10)))!=0) &&
//	   ((R4_statusword & (1<<(10)))!=0) )//如果任意一个行走电机到位，则立刻停止所有行走电机
//	if(((R1_statusword & (1<<(10)))!=0) ||
//	   ((R2_statusword & (1<<(10)))!=0) ||
//	   ((R3_statusword & (1<<(10)))!=0) ||
//	   ((R4_statusword & (1<<(10)))!=0) )//如果任意一个行走电机到位，则立刻停止所有行走电机
//	{
//		printf("运动到达停止点\n");
//		AgvStop();
//		return 4;//上级函数收到返回值4时，会退出while循环，结束运动程序。
//	}

	if(direc==1)//前进
	{
		//更新直行电机3的行走距离，这个电机的编码器值在增加
		if((R3_position<encodeOldR3)&&(R3_position<0&&encodeOldR3>0))//如果直行电机3编码器反馈的新值小于旧值，
			//且新值为负，旧值为正，说明编码器值经历了由正变负的过程,INTEGER32溢出了
		{
			printf("Old3 is %d,New3 is %d\n",encodeOldR3,R3_position);
			distance=encodeMAX*2+R3_position-encodeOldR3+distance;//更新距离值
			encodeOldR3=R3_position;
		}
		else//如果不是上面编码器溢出，新值肯定是大于旧值，因为前行时R3编码器值在增加
		{
			distance=R3_position-encodeOldR3+distance;//更新距离值
			encodeOldR3=R3_position;
		}
		//更新直行电机4的行走距离，这个电机的编码器值在减小
		if((R4_position>encodeOldR4)&&(R4_position>0&&encodeOldR4<0))//如果直行电机4编码器反馈的新值大于旧值，
			//且新值为正，旧值为负，说明编码器值经历了由负变正的过程,INTEGER32溢出了
		{
			printf("Old4 is %d,New4 is %d\n",encodeOldR4,R4_position);
			distance=encodeMAX*2-R4_position+encodeOldR4+distance;//更新距离值
			encodeOldR4=R4_position;
		}
		else//如果不是上面编码器溢出，新值肯定是小于旧值，因为前行时R4的编码器值是递减的
		{
			distance=encodeOldR4-R4_position+distance;//更新距离值
			encodeOldR4=R4_position;
		}
		//meterDistance=((((distance[0]+distance[1]+distance[2]+distance[3])/4)/16384)/20)*0.6283;//把编码器值转化成实际运行的米数
		meterDistance=(distance*CForward)/655360;//把电机3和电机4编码器值除以2,转化成实际运行的米数
	}
	else if(direc==2)//后退
	{
		//更新直行电机1的行走距离，这个电机的编码器值在减小
		if((R1_position>encodeOldR1)&&(R1_position>0&&encodeOldR1<0))//如果直行电机1编码器反馈的新值大于旧值，
			//且新值为正，旧值为负，说明编码器值经历了由负变正的过程,INTEGER32溢出了
		{
			printf("Old1 is %d,New1 is %d\n",encodeOldR1,R1_position);
			distance=encodeMAX*2-R1_position+encodeOldR1+distance;//更新距离值
			encodeOldR1=R1_position;
		}
		else//如果不是上面编码器溢出，新值肯定是小于旧值，因为后退时R1的编码器值是递减的
		{
			distance=encodeOldR1-R1_position+distance;//更新距离值
			encodeOldR1=R1_position;
		}
		//更新直行电机2的行走距离，这个电机的编码器值在增加
		if((R2_position<encodeOldR2)&&(R2_position<0&&encodeOldR2>0))//如果直行电机2编码器反馈的新值小于旧值，
			//且新值为负，旧值为正，说明编码器值经历了由正变负的过程,INTEGER32溢出了
		{
			printf("Old2 is %d,New2 is %d\n",encodeOldR2,R2_position);
			distance=encodeMAX*2+R2_position-encodeOldR2+distance;//更新距离值
			encodeOldR2=R2_position;
		}
		else//如果不是上面编码器溢出，新值肯定是大于旧值，因为后退时R2的编码器值是递增的
		{
			distance=R2_position-encodeOldR2+distance;//更新距离值
			encodeOldR2=R2_position;
		}

		//meterDistance=((((distance[0]+distance[1]+distance[2]+distance[3])/4)/16384)/20)*0.6283;//把编码器值转化成实际运行的米数
		meterDistance=(distance*CBack)/655360;//把电机1和电机2编码器值除以2,转化成实际运行的米数
	}
	else if(direc==3)//左移
	{
		//更新直行电机1的行走距离，这个电机的编码器值在增加
		if((R1_position<encodeOldR1)&&(R1_position<0&&encodeOldR1>0))//如果直行电机1编码器反馈的新值小于旧值，
			//且新值为负，旧值为正，说明编码器值经历了由正变负的过程,INTEGER32溢出了
		{
			distance=encodeMAX*2+R1_position-encodeOldR1+distance;//更新距离值
			encodeOldR1=R1_position;
		}
		else//如果不是上面编码器溢出，新值肯定是大于旧值，因为左移时R1编码器值在增加
		{
			distance=R1_position-encodeOldR1+distance;//更新距离值
			encodeOldR1=R1_position;
		}
		//更新直行电机4的行走距离，这个电机的编码器值在增加
		if((R4_position<encodeOldR4)&&(R4_position<0&&encodeOldR4>0))//如果直行电机4编码器反馈的新值小于旧值，
			//且新值为负，旧值为正，说明编码器值经历了由正变负的过程,INTEGER32溢出了
		{
			distance=encodeMAX*2+R4_position-encodeOldR4+distance;//更新距离值
			encodeOldR4=R4_position;
		}
		else//如果不是上面编码器溢出，新值肯定是大于旧值，因为左移时R4编码器值在增加
		{
			distance=R4_position-encodeOldR4+distance;//更新距离值
			encodeOldR4=R4_position;
		}

		//meterDistance=((((distance[0]+distance[1]+distance[2]+distance[3])/4)/16384)/20)*0.6283;//把编码器值转化成实际运行的米数
		meterDistance=(distance*CLeft)/655360;//把电机1和电机4编码器值除以2,转化成实际运行的米数
	}
	else if(direc==4)//右移
	{
		//更新直行电机2的行走距离，这个电机的编码器值在减小
		if((R2_position>encodeOldR2)&&(R2_position>0&&encodeOldR2<0))//如果直行电机2编码器反馈的新值大于旧值，
			//且新值为正，旧值为负，说明编码器值经历了由负变正的过程,INTEGER32溢出了
		{
			distance=encodeMAX*2-R2_position+encodeOldR2+distance;//更新距离值
			encodeOldR2=R2_position;
		}
		else//如果不是上面编码器溢出，新值肯定是小于旧值，因为右移时R2的编码器值是递减的
		{
			distance=encodeOldR2-R2_position+distance;//更新距离值
			encodeOldR2=R2_position;
		}
		//更新直行电机3的行走距离，这个电机的编码器值在减小
		if((R3_position>encodeOldR3)&&(R3_position>0&&encodeOldR3<0))//如果直行电机3编码器反馈的新值大于旧值，
			//且新值为正，旧值为负，说明编码器值经历了由负变正的过程,INTEGER32溢出了
		{
			distance=encodeMAX*2-R3_position+encodeOldR3+distance;//更新距离值
			encodeOldR3=R3_position;
		}
		else//如果不是上面编码器溢出，新值肯定是小于旧值，因为右移时R3的编码器值是递减的
		{
			distance=encodeOldR3-R3_position+distance;//更新距离值
			encodeOldR3=R3_position;
		}
		meterDistance=(distance*CRight)/655360;//把电机2和电机3编码器值除以2,转化成实际运行的米数
	}
	if(meterDistance-PointDistance[1]>0.001)//如果实际运行的米数超过停止点0.01米，则小车停止
	{
		printf("行进距离超过停止点\n");
		Log("行进距离超过停止点\n");
		AgvStop();  //rfid定的停止点和距离测算的停止点核对不一致，要么距离测算不对，要么没有读到rfid信息，也下发停止命令
		return 4;//上级函数收到返回值4时，会根据此值向更上级返回4,以退出while循环。
	}

//	//下面是纠偏程序
	if((AdjustCount%AdjustBackFlag)!=0) //不能整除，则前轮纠偏
	{
		//printf("前轮纠偏进入，使用导航：%d\n",AdjustNavi[0]);
			//下面是直行纠偏部分
		if(agvreportstruct.NavigationSenser[AdjustNavi[0]]>b && agvreportstruct.NavigationSenser[AdjustNavi[0]]<=0xF000)//如果偏向左边，磁导航高位触发
		{
			/*将bit4置0*/
			M2_TPDO_control_word=(M2_TPDO_control_word&(~0x10));//bit 4=0
			sendPDOevent(CANOpenShellOD_Data);

			//把转向电机1和2向右调整。 转向电机5：顺时针 转向电机6：顺时针
			M2_5_TPDO_Target_Position=M2_position[0];
			M2_6_TPDO_Target_Position=M2_position[1];
			//后轮归零
			M2_7_TPDO_Target_Position=M2_position[2];
			M2_8_TPDO_Target_Position=M2_position[3];
			sendPDOevent(CANOpenShellOD_Data);

			if(AdjustFlag!=21)
			{
			/*启动*/
			M2_TPDO_control_word = RotatorStart;
			sendPDOevent(CANOpenShellOD_Data);
			}

			AdjustFlag=21;
			//return 0;//return 0是必须的，因为这些if和else if 是互斥的，所以进入其中一个后肯定不会进入其它所有判断，所以直接retrun，不再去if其它条件。
		}
		else if(agvreportstruct.NavigationSenser[AdjustNavi[0]]<a && agvreportstruct.NavigationSenser[AdjustNavi[0]]>0)	//如果偏向右边，磁导航低位触发 应为>0
		{
			/*将BIT4=0 避免纠偏未把bit4置0*/
			M2_TPDO_control_word=(M2_TPDO_control_word&(~0x10));//bit 4=0
			sendPDOevent(CANOpenShellOD_Data);

			//把转向电机1和2向左调整。转向电机5：逆时针  转向电机6：逆时针
			M2_5_TPDO_Target_Position=M2_position[4];
			M2_6_TPDO_Target_Position=M2_position[5];
			//后轮归零
			M2_7_TPDO_Target_Position=M2_position[6];
			M2_8_TPDO_Target_Position=M2_position[7];
			sendPDOevent(CANOpenShellOD_Data);

			if(AdjustFlag!=22)
			{
			/*启动*/
			M2_TPDO_control_word = RotatorStart;
			sendPDOevent(CANOpenShellOD_Data);
			}

			AdjustFlag=22;
			//return 0;
		}
		else if(agvreportstruct.NavigationSenser[AdjustNavi[0]]>=a && agvreportstruct.NavigationSenser[AdjustNavi[0]]<=b)//如果在可容忍范围内，不纠偏，把轮子摆正。
		{
			/*将bit4置0*/
			M2_TPDO_control_word=(M2_TPDO_control_word&(~0x10));//bit 4=0
			sendPDOevent(CANOpenShellOD_Data);

			//把转向电机1和2调整回前行状态. 转向电机5 6 回0位置
			M2_5_TPDO_Target_Position=M2_position[8];
			M2_6_TPDO_Target_Position=M2_position[9];
			//后轮归零
			M2_7_TPDO_Target_Position=M2_position[10];
			M2_8_TPDO_Target_Position=M2_position[11];

			if(AdjustFlag!=20)
			{
			/*启动*/
			M2_TPDO_control_word = RotatorStart;
			sendPDOevent(CANOpenShellOD_Data);
			}

			AdjustFlag=20;
			//return 0;
		}
		else if(agvreportstruct.NavigationSenser[AdjustNavi[0]]==0xffff)//如果导航16位全读到1,说明导航遇到了交叉导航条
		{
			//把转向电机1和2调整回前行状态，或者什么都不做？
			//printf("全是11111111111111111111111111111111111111111111111\n");
			//return 0;
		}
		//上面四种是导航的正常状态，下面是导航的两种不正常状态，一种是全零，一种是大于0xf000,小于0xffff,两种情况一起考虑。

//		else if(agvreportstruct.NavigationSenser[AdjustNavi[0]]==0)//未检测到导航信号 未检测信号都是0！ 应为=0
//		{
//			//此时应该禁止小车行走，同时上报上位机未检测到导航信号。
//			printf("前行方向无导航信号，小车停止\n");
//			Log("前行方向无导航信号，小车停止\n");
//			AgvStop();//暂时关闭，调试方便，正式运行时打开
//			return 4;
//		}
		else
	{
//			correctNaviError++;//每次进入这个if给correctNaviError加1,如果连续进入此if，说明导航读到的信息错误，应该紧急停止小车。
//			if(correctNaviError>10000)
//			{
//				printf("前行纠偏信号错误！agvreportstruct.NavigationSenser[%d]==%x！！\n",AdjustNavi[0],agvreportstruct.NavigationSenser[AdjustNavi[0]]);
//				Log("前行纠偏信号错误\n");
//				AgvStop();
//				return 4;//上级函数收到返回值4时，会根据此值向更上级返回4,以退出while循环。
//			}
	}
	}//end不能整除

	else  //如果能整除，则进入后轮纠偏
	{
		/*开始判断后轮纠偏*/
		//printf("后轮纠偏进入，使用导航：%d\n",AdjustNavi[1]);
			if(agvreportstruct.NavigationSenser[AdjustNavi[1]]<a && agvreportstruct.NavigationSenser[AdjustNavi[1]]>0)//如果偏向右边，磁导航低位触发
			{
				/*将bit4置0*/
				M2_TPDO_control_word=(M2_TPDO_control_word&(~0x10));//bit 4=0
				sendPDOevent(CANOpenShellOD_Data);

				/*前轮归零*/
				M2_5_TPDO_Target_Position=M2_position[12];
				M2_6_TPDO_Target_Position=M2_position[13];
				//把转向电机7和8向右调整 转向电机7：顺时针  转向电机8：顺时针
				M2_7_TPDO_Target_Position=M2_position[14];
				M2_8_TPDO_Target_Position=M2_position[15];
				sendPDOevent(CANOpenShellOD_Data);

				if(AdjustFlag!=23)
				{
					/*启动*/
					M2_TPDO_control_word = RotatorStart;
					sendPDOevent(CANOpenShellOD_Data);
				}
				AdjustFlag=23;
				//return 0;
			}
			else if(agvreportstruct.NavigationSenser[AdjustNavi[1]]>b && agvreportstruct.NavigationSenser[AdjustNavi[1]]<=0xF000)//如果偏向左边，磁导航高位触发
			{
				/*将bit4置0*/
				M2_TPDO_control_word=(M2_TPDO_control_word&(~0x10));//bit 4=0
				sendPDOevent(CANOpenShellOD_Data);

				/*前轮归零*/
				M2_5_TPDO_Target_Position=M2_position[16];
				M2_6_TPDO_Target_Position=M2_position[17];
				//把转向电机7和8向左调整。 转向电机7：逆时针  转向电机8：逆时针
				M2_7_TPDO_Target_Position=M2_position[18];
				M2_8_TPDO_Target_Position=M2_position[19];
				sendPDOevent(CANOpenShellOD_Data);

				if(AdjustFlag!=24)
				{
					/*启动*/
					M2_TPDO_control_word = RotatorStart;
					sendPDOevent(CANOpenShellOD_Data);
				}
				AdjustFlag=24;
				//return 0;
			}
			else if(agvreportstruct.NavigationSenser[AdjustNavi[1]]>=a && agvreportstruct.NavigationSenser[AdjustNavi[1]]<=b)//如果在可容忍范围内，不纠偏，把轮子摆正。
			{
				/*将bit4置0*/
				M2_TPDO_control_word=(M2_TPDO_control_word&(~0x10));//bit 4=0
				sendPDOevent(CANOpenShellOD_Data);

				/*前轮归零*/
				M2_5_TPDO_Target_Position=M2_position[20];
				M2_6_TPDO_Target_Position=M2_position[21];
				//把转向电机7和8调整回前行状态
				M2_7_TPDO_Target_Position=M2_position[22];
				M2_8_TPDO_Target_Position=M2_position[23];
				sendPDOevent(CANOpenShellOD_Data);

				if(AdjustFlag!=25)
				{
					/*启动*/
					M2_TPDO_control_word = RotatorStart;
					sendPDOevent(CANOpenShellOD_Data);
				}
				AdjustFlag=25;
				//return 0;
			}
			else if(agvreportstruct.NavigationSenser[AdjustNavi[1]]==0xffff)//如果导航16位全读到1,说明导航遇到了交叉导航条
			{
				//把转向电机1和2调整回前行状态，或者什么都不做？
				//printf("全是11111111111111111111111111111111111111111111111\n");
				//return 0;
			}

			//上面四种是导航的正常状态，下面是导航的两种不正常状态，一种是全零，一种是大于0xf000,小于0xffff,两种情况一起考虑。

//			else if(agvreportstruct.NavigationSenser[AdjustNavi[1]]==0)//未检测到导航信号 未检测信号都是0！ 应为=0
//			{
//				//此时应该禁止小车行走，同时上报上位机未检测到导航信号。
//				printf("前行时后轮纠偏无导航信号，小车停止\n");
//				Log("前行时后轮纠偏无导航信号，小车停止\n");
//				AgvStop();//暂时关闭，调试方便，正式运行时打开
//				return 4;
//			}
			else
			{
//				correctNaviError++;//每次进入这个if给correctNaviError加1,如果连续进入此if，说明导航读到的信息错误，应该紧急停止小车。
//				if(correctNaviError>20000)
//				{
//					printf("后轮纠偏信号错误！agvreportstruct.NavigationSenser[%d]==%x！！\n",AdjustNavi[1],agvreportstruct.NavigationSenser[AdjustNavi[1]]);
//					Log("后轮纠偏信号错误\n");
//					AgvStop();
//					return 4;//上级函数收到返回值4时，会根据此值向更上级返回4,以退出while循环。
//				}
			}
	} //end能整除

	//如果运行方向前面两个rfid中至少一个读到的第三位（车位号）发生了变化，说明小车已经接近走到了下一个车位，应该让小车减速。
//	if(rfid[rfidnum[2]]==agvreportstruct.RFIDSenser[rfidnum[2]][2]||rfid[rfidnum[3]]==agvreportstruct.RFIDSenser[rfidnum[3]][2]) //后RFID
	if(PGV_Data_Matrixtag_TAG00_TAG07==(rfid[0]-48))
	{
		if(innerstruct.SlowCommand!=2)//防止对停止点重复赋值，第一次进入这个if，innerstruct.SlowCommand没有置2,对停止点赋值，第二次再进入这个if,innerstruct.SlowCommand已经被置2,不会再赋值。
		{
			/*将bit4置0*/
			M1_TPDO_control_word=(M1_TPDO_control_word&(~0x10));//bit 4=0
			sendPDOevent(CANOpenShellOD_Data);

			int ret=WalkingFixPositon(direc);
			if(ret!=0)
			{
				printf("RFID修正停止距离出错\n");
				Log("RFID修正停止距离出错");
				return -1;
			}
			else
			{
				printf("RFID修正停止距离成功\n");
				Log("RFID修正停止距离成功");
				printf("PGV_Data_Matrixtag_TAG00_TAG07==%x\n",PGV_Data_Matrixtag_TAG00_TAG07);
			}
		}
		innerstruct.SlowCommand=2;
		printf("M1_1_TPDO_Target_Position==%d \n",M1_1_TPDO_Target_Position);
		return 0;//不再进行其他判断
	}

//	}

	/* 如果距离停止点距离小于LookingRFIDPoint，则发送LookingRFIDSpeed=0.1788m/s，并设置新相对目标点距离=3m
	 * 为避免重复赋值，进在LookingRFIDCount=0第一次进入，找RFID赋值函数只进入一次，在AGVSTOP中将LookingRFIDCount==0*/
	if(LookingRFIDCount==0)
	{
//	if((
//		(abs(abs(CommandPosition3)-abs(R3_position))<LookingRFIDPointCount) ||
//		(abs(abs(CommandPosition1)-abs(R1_position))<LookingRFIDPointCount) ||
//		(abs(abs(CommandPosition2)-abs(R2_position))<LookingRFIDPointCount) ||
//		(abs(abs(CommandPosition4)-abs(R4_position))<LookingRFIDPointCount) )
//		&& innerstruct.SlowCommand!=2  )
		if((
			(abs(CommandPosition3-R3_position)<LookingRFIDPointCount) ||
			(abs(CommandPosition1-R1_position)<LookingRFIDPointCount) ||
			(abs(CommandPosition2-R2_position)<LookingRFIDPointCount) ||
			(abs(CommandPosition4-R4_position)<LookingRFIDPointCount) )
			&& innerstruct.SlowCommand!=2  )
	{
		/*将bit4置0*/
		M1_TPDO_control_word=(M1_TPDO_control_word&(~0x10));//bit 4=0
		sendPDOevent(CANOpenShellOD_Data);

		LookingRFIDCount=1;
		int ret=LookingRFID(direc);
		if(ret!=0)
		{
			printf("修正停止距离出错\n");
			Log("修正停止距离出错");
			return -1;
		}
		else
		{
			//			printf("修正停止距离成功\n");
			//			Log("修正停止距离成功");
		}
	}//END if(((CommandPosition1-R1_position)<WalkingPositiveFixed) && innerstruct.SlowCommand!=2  )
	} //end if


//	if(direc ==1 ||direc ==3)
//	{
////		if(((CommandPosition3-R3_position)<LookingRFIDPointCount*positivetime) && innerstruct.SlowCommand!=2  )
//		if((((CommandPosition3-R3_position)<LookingRFIDPointCount*positivetime) ||
//			((CommandPosition1-R1_position)<LookingRFIDPointCount*positivetime) ||
//			((CommandPosition2-R2_position)>LookingRFIDPointCount*positivetime) ||
//			((CommandPosition4-R4_position)>LookingRFIDPointCount*positivetime) )
//			&& innerstruct.SlowCommand!=2  )
//		{
//			int ret=LookingRFID(direc);
//			if(ret!=0)
//			{
//				printf("修正停止距离出错\n");
//				Log("修正停止距离出错");
//				return -1;
//			}
//			else
//			{
//				//			printf("修正停止距离成功\n");
//				//			Log("修正停止距离成功");
//			}
//		}//END if(((CommandPosition1-R1_position)<WalkingPositiveFixed) && innerstruct.SlowCommand!=2  )
//	}//end if(direc ==1 ||direc ==3)
//
//	/*没有读到RFID，多走1CM*/
//	if(direc ==2 ||direc ==4)
//	{
////		if(((CommandPosition3-R3_position)>LookingRFIDPointCount*positivetime) && innerstruct.SlowCommand!=2  )
//		if((((CommandPosition3-R3_position)>LookingRFIDPointCount*positivetime) ||
//			((CommandPosition1-R1_position)>LookingRFIDPointCount*positivetime) ||
//			((CommandPosition2-R2_position)<LookingRFIDPointCount*positivetime) ||
//			((CommandPosition4-R4_position)<LookingRFIDPointCount*positivetime) )
//			&& innerstruct.SlowCommand!=2  )
//		{
//			int ret=LookingRFID(direc);
//			if(ret!=0)
//			{
//				printf("修正停止距离出错\n");
//				Log("修正停止距离出错");
//				return -1;
//			}
//			else
//			{
//				//			printf("修正停止距离成功\n");
//				//			Log("修正停止距离成功");
//			}
//		}//END if(((CommandPosition1-R1_position)<WalkingPositiveFixed) && innerstruct.SlowCommand!=2  )
//	}//end if(direc ==2 ||direc ==4)

	return 0;
}


//string 转成float的小函数
float string2float(char * command)
{
	char floatnum[7];
	float f;

	int i=0;
	for(i=0;i<7;i++)
	{
		floatnum[i]=command[i+6];
	}
	f=atof(floatnum);
	return f;
}



/*将LookingRFIDSpeed轮速m/s转换为电机的指令counts值。*/
int CalLookingRFIDSpeed()
{
	//使用CForward=0.6473389大于理论的值，导致最终结果小于C=0.6283计算出来的速度
	LookingRFIDSpeedCount=(INTEGER32)(((LookingRFIDSpeed*20)/CForward)*16384*10);
	float motorspeed=((LookingRFIDSpeed*20)/CForward)*60; //无实际意义，只是为了显示转换为电机速度rpm数值，方便检测;
	printf("LookingRFIDSpeed==%f, LookingRFIDSpeedCount=%d=%x,电机转速rpm==%f\n",LookingRFIDSpeed,LookingRFIDSpeedCount,LookingRFIDSpeedCount,motorspeed);
	return 0;
}


/*上位机赋值停止点传进来，把距离转换成电机位置信息WalkingPositive  WalkingNegative 16进制*/
int SetWalkingPosition(float slowpoint)
{
	//将距离米转换为电机位置参数
	WalkingPositive=(INTEGER32)((slowpoint/CForward)*20*16384);
	WalkingNegative=(INTEGER32)((((slowpoint/CForward)*20*16384))*(-1));
	//printf("WalkingPositive==%x, WalkingNegative==%x\n",WalkingPositive,WalkingNegative);
	return 0;
}

/*计算LookingRFIDPoint，开始寻找RFID降速的最小距离*/
int CalLookingRFIDStartPosition()
{
	//将距离米转换为电机位置参数
	LookingRFIDPointCount=(INTEGER32)((LookingRFIDPoint/CForward)*20*16384);
	printf("LookingRFIDPointCount==%x, LookingRFIDPointCount==%d\n",LookingRFIDPointCount,LookingRFIDPointCount);
	return 0;
}

/*计算相对于第一次发送的绝对停止点，再多走的距离count值，用于匀速寻找RFID*/
int CalLookingRFIDStopPosition(float LookingRFIDStopPoint)
{
	//将距离米转换为电机位置参数
	LookingRFIDWalkingPositiveCount=(INTEGER32)((LookingRFIDStopPoint/CForward)*20*16384);
	LookingRFIDWalkingNegativeCount=(INTEGER32)((((LookingRFIDStopPoint/CForward)*20*16384))*(-1));
	//printf("WalkingPositive==%x, WalkingNegative==%x\n",WalkingPositive,WalkingNegative);
	return 0;
}

/* 计算读到RFID后修正准确停止点距离
 * 传进来的数据定死的，只是用于提前计算，修正距离参数，后期可以去掉此函数把赋值写死WalkingPositiveFixed WalkingNegativeFixed*/
int FixedWalkingPosition(float FixedStopPoint)
{
	//将距离米转换为电机位置参数
	WalkingPositiveFixed=(INTEGER32)((FixedStopPoint/CForward)*20*16384);
	WalkingNegativeFixed=(INTEGER32)((((FixedStopPoint/CForward)*20*16384))*(-1));
	return 0;
}

/*旋转距离计算*/
int SetRotatePosition(int time)
{
	double onedegreemeter=((0.02498931193*366.617987288)/360);
	//将距离米转换为电机位置参数
	RotatePositionPositive=(INTEGER32)((onedegreemeter/CForward)*20*16384*90*time);
	RotatePositionNegative=(INTEGER32)((((onedegreemeter/CForward)*20*16384)*90*time)*(-1));
	//printf("RotatePositionPositive==%x %d, RotatePositionNegative==%x %d\n",RotatePositionPositive,RotatePositionPositive,RotatePositionNegative,RotatePositionNegative);
	return 0;
}

/*转向轮回0*/
int RotateWheelReturn(int RotateWheelReturnDirec)
{
	/*将BIT4=0 避免纠偏未把bit4置0*/
	M2_TPDO_control_word=(M2_TPDO_control_word&(~0x10));//bit 4=0
	sendPDOevent(CANOpenShellOD_Data);

	if(RotateWheelReturnDirec==1||RotateWheelReturnDirec==2) //前进+后退
	{
		//摆正4个转向轮
		M2_5_TPDO_Target_Position=Rotator_zero;
		M2_6_TPDO_Target_Position=Rotator_zero;
		//后轮归零
		M2_7_TPDO_Target_Position=Rotator_zero;
		M2_8_TPDO_Target_Position=Rotator_zero;
		sendPDOevent(CANOpenShellOD_Data);

		M2_TPDO_control_word=RotatorStart;
		sendPDOevent(CANOpenShellOD_Data);
		AdjustFlag=0;
		printf("转向电机前后回0,AdjustFlag==%d\n",AdjustFlag);
		return 0;
	}

	else if(RotateWheelReturnDirec==3 ||RotateWheelReturnDirec==4 )//左移+右移
	{
		//轮子归零
		M2_5_TPDO_Target_Position=Anticlockwise_Rotator_Crab;
		M2_6_TPDO_Target_Position=Clockwise_Rotator_Crab;
		M2_7_TPDO_Target_Position=Anticlockwise_Rotator_Crab;
		M2_8_TPDO_Target_Position=Clockwise_Rotator_Crab;
		sendPDOevent(CANOpenShellOD_Data);

		M2_TPDO_control_word=RotatorStart;
		sendPDOevent(CANOpenShellOD_Data);
		AdjustFlag=0;
		printf("转向电机横移回0,AdjustFlag==%d\n",AdjustFlag);
		return 0;
	}
	return 0;
}
