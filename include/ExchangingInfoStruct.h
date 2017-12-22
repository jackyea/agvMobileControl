/*
 * aaa.h
 *
 *  Created on: May 18, 2017
 *      Author: ye
 */


struct StructAgvReport{
	int MotorVel[4];//4个直行电机的实时速度///////当电机运行时，运行程序向此处写入此电机的实时速度

	int WheelDirection;//四个车轮的当前位置，位置分为0=零位置与长边平行,1=横移位置与宽边平行,2=自旋位置，

	int MotorError;//任何时候电机反馈信号表明此电机故障时，向此处相应位处写入1。

	int IOError;//IO栈故障时写入1。

	int ExecuteCommand;//执行命令,0代表空闲，可以下发命令，1命令正在执行，不要发新命令，-1命令执行中发生错误，执行中止

	int RfidError;//rfid掉线

	int RWFlag;//读写标志位，当此标志位为1时，通信程序可以读这个结构体，并将其发送到上位机，当标志位为0时，通信程序不去读它，命令处理程序可以去向这个结构体写数据。;

	int RemoteControl;//当前操作模式标志位 =0：上位机模式  =1：遥控器模式

	int HitAvoidZone; //0: 1: 2: 3: 行走过程中，根据不同场地切换避障区域
	int R1position;
	int R2position;
	int R3position;
	int R4position;
	int R5position;
	int R6position;
	int R7position;
	int R8position;
	char RFIDSenser[4][5]; //rfid数组，四个数据分别存储rfid读卡器1-4所读出的数据：每个数据为4位，前3位是组编码，最后一位是自身编码，如rfid[0]=0101,说明这是第一个rfid读卡器读到的第010组第1个卡,rfid[3]=0114,说明这是第四个rfid读卡器读到的第011组第4个卡

	int  HitAvoidSenser[4];//避障传感器数组，四个数据分别存储避障传感器1-4输出的数据,其中每个数据2位，4种组合代表避障1区（内层）和2区（中间层）的障碍物情况，/////上位机根据小车的朝向决定应该采用哪个传感器的数据

	int  NavigationSenser[4];//导航传感器数组，四个数据分别存储导航传感器1-4读到的16进制的导航数据，这个数据有4位，从0x0001-0x8000,并不连续。/////上位机根据小车的朝向决定应该采用哪个导航条的数据

	int LimitSwitch;//一个16位的数据，其中低10位表示限位开关的开关量，0-3依次为旋转电机1、2、3、4的下限位，4-7依次为旋转电机1、2、3、4上限位，8-9升降电机的上升限位，下降限位。

	int BattaryCharge; //电池信息：子程序写，主程序读。 注：这个信息是通过485协议发送到bbb上的，不是通过I/O栈。
};

//内部交流结构体，包括通信程序接收到的上位机命令，接收到就存到此处，命令处理程序再从此处读取命令，进行分析和执行，
struct StructInner{
	int RWflag;//读写标志位，当RWflag为1时，命令处理程序从此处读取命令，并将其置位为0,当RWflag为0时，通信程序将接收到的命令写到此处。
	char * Command;//通信程序接收到的上位机命令
	int StopCommand;//1：收到了停止命令，0：没有收到停止命令
	int SlowCommand;//1：收到减速命令，0：没有减速命令
	int AdjustRouteFlag;//开始纠偏标志位
};
