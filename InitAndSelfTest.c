#include <string.h>
#include "InitAndSelfTest.h"
#include "ExchangingInfoStruct.h"
#include "CanOpenShell.h"
#include "Log.h"
#include "MotorOnline.h"
#include"CANOpenShellMasterOD.h"
#include "MotorControl.h"
#include "HandMotorControl.h"
#include "configureslave.h"
/* "ExchangingInfoStruct.h"中定义StructAgvReport结构体类型
 * InitAndSelfTets.c中(包含"ExchangingInfoStruct.h")定义StructAgvReport类型的结构体变量 agvreportstruct={}
 * 需要外部引用时：
 * 1.包含#include "ExchangingInfoStruct.h"
 * 2.声明：extern struct StructAgvReport agvreportstruct;*/
struct StructAgvReport agvreportstruct={};
struct StructInner innerstruct={};

//举升电机初始位置
INTEGER32 UpMotorInitPosition = 0x0;
int UpMotorCount=0;//用于区分第一次第二次获取0位打印信息

int InitRov()
{
	InitStruct();//初始化结构体

	if(CanInit()==-1)//Can总线初始化
	{
		printf("Can初始化失败！,退出程序\n");
		Log("Can初始化失败！,退出程序");
		return -1; //can线如果失败，不需要AgvEmcyStop
	}

	/*判断IO站初始化完成*/
	if(LimitSwitchInit()==-1)//IO站初始化在电机+sensor前！限位开关初始化，其实是等待限位开关上电后给IO栈的信号
	{
		printf("IO站初始化失败！，检查log日志,退出程序\n");
		Log("IO站初始化失败！，检查log日志,退出程序");
		return -1; //IO如果初始化失败，AgvEmcyStop也无意义
	}

	/*IO初始化完成后闭合K2 0.5S后断开K2*/
	if(CloseAndOpenK2()==-1)
	{
		printf("闭合K2失败！检查log日志,退出程序\n");
		Log("闭合K2失败！检查log日志,退出程序");
		//return -1;
		return AgvEmcyStop();
	}

	/*检测到PGV上线后，开始配置*/
	if(ConfigPGV()==-1)
	{
		printf("PGV，配置失败\n");
		Log("PGV，配置失败");
		return AgvEmcyStop();
	}

	/*检测到所有电机上线后，依次配置，防止偶尔某台没有配置成功*/
	if(ConfigMotor()==-1)
	{
		printf("电机全部上线后，配置失败\n");
		Log("电机全部上线后，配置失败");
		return AgvEmcyStop();
	}


	/*传感器初始化，包括rfid，避障、导航共12个传感器的初始化*/
	if(SenserInit()==-1)//传感器初始化，包括rfid，避障、导航共12个传感器的初始化
	{
		printf("传感器工作不正常！退出程序\n");
		Log("传感器工作不正常！退出程序");
		//return -1;
		return AgvEmcyStop();
	}

	/*电源管理初始化*/
	if(PowerManageInit()==-1)//电源管理初始化
	{
		printf("电源管理工作不正常！退出程序\n");
		Log("电源管理工作不正常！退出程序");
		//return -1;
		return AgvEmcyStop();
	}

	printf("检测IO Motor error标帜位前延迟1s，防止电机始能后报警\n");
	Log("检测IO Motor error标帜位前延迟1s，防止电机始能后报警");
	//加入一个延迟，为电机回零预留一些时间，防止接受失败
	int delaycount=0;
	while(delaycount<=100000000)
	{
		delaycount++;
	} //100000000大概1.2s左右；300000000=4.5s左右

	/*再次检测电机和IO是否初始化完成*/
	if(!(agvreportstruct.MotorError==0 && agvreportstruct.IOError==0))
	{
		Log("错误！电机或IO站初始化未完成,退出程序");
		printf("错误！电机或IO站初始化未完成,退出程序\n");
		return AgvEmcyStop();
	}
	printf("IO Motor error标帜位正常，开始寻0\n");
	Log("IO Motor error标帜位正常，开始寻0");

	//4个转向电机初始化回0位置
//	if(InitZeroPosition()==-1)
//	{
//		Log("转向电机初始化归0时出错,退出程序");
//		printf("转向电机初始化归0时出错，退出程序\n");
//		//return -1;
//		return AgvEmcyStop();
//	}

	//举升电机获取初始位置作为ZERO位
	if(GetUpMotorInitPosition())
	{
		printf("获取举升电机初始位置失败,退出程序！\n");
		Log("获取举升电机初始位置失败,退出程序！！！");
		//return -1;
		return AgvEmcyStop();
	}
	printf("举升第一次获取0位置完成\n");
	Log("举升第一次获取0位置完成");

	//举升点击找0点
	if(UpMotorGoZero()==-1)
	{
		printf("举升电机找0点失败,退出程序！！！\n");
		Log("举升电机找0点失败,退出程序！！！");
		//return -1;

		return AgvEmcyStop();
	}
	printf("举升找0位置完成\n");
	Log("举升找0位置完成");

	//举升电机找到0位置后以此位置作为初始位置
	if(GetUpMotorInitPosition()==-1)
	{
		printf("归零后获取举升电机初始位置失败,退出程序！！！\n");
		Log("归零获取举升电机初始位置失败,退出程序！！！");
		//return -1;
		return AgvEmcyStop();
	}
	printf("举升第二次获取0位置完成\n");
	Log("举升第二次获取0位置完成");

	if(UpMotorZero(CANOpenShellOD_Data, 0x09))
	{ //无论如何都返回0 所以以下语句不会执行
		printf("举升电机重设正常速度失败,退出程序！！！！\n");
		Log("举升电机重设正常速度失败,退出程序！！！！");
		//return -1;
		return AgvEmcyStop();
	}
	printf("举升重设速度完成\n");

	//断所有电机使能
	AgvStop();
	printf("所有电机断开使能\n");

	if(CLoseGreen())//点亮AGV绿色灯，AGV所有设备初始化正常，可以开始运行
	{
		printf("绿灯未正常点亮，程序返回出错,退出程序！\n");
		Log("绿灯未正常点亮，程序返回出错！");
		return AgvEmcyStop();
	}
	printf("AGV开始运行\n");
	return 0;
} //end init

/**********************************以下是具体函数****************************************************************************/
int ConfigPGV()
{
	printf("等待PGV并开始配置\n");
	while((PGV_Online)!=1)
	{;}

	ConfigureSlaveNode(CANOpenShellOD_Data, 0x0A);
	while(!PGV_Configuration){}

	 if((PGV_Configuration) ==1)
	 {
		 Log("PGV初始化完成！");
		 printf("PGV初始化完成！\n");
		 return 0;
	 }
	return -1;
}


int ConfigMotor()
{
	while((R1_Online && R2_Online && R3_Online && R4_Online && R5_Online && R6_Online &&
				R7_Online && R8_Online && R9_Online)!=1)
	{;}

	//加入一个延迟，检测到电机全部上线后，等待1s再开始配置
	int delaycount=0;
	while(delaycount<=100000000) //0.5S
		{
			delaycount++;
		} //100000000大概1.2s左右；300000000=4.5s左右
	printf("所有电机上线正常，开始配置\n");
	Log("所有电机上线正常，开始配置！");

	ConfigureSlaveNode(CANOpenShellOD_Data, 0x01);
	while(!R1_Configuration){}
	printf("R1_Configuration==%d配置完成\n",R1_Configuration);
	Log("R1_Configuration配置完成\n");

	ConfigureSlaveNode(CANOpenShellOD_Data, 0x02);
	while(!R2_Configuration){}
	printf("R2_Configuration==%d配置完成\n",R2_Configuration);
	Log("R2_Configuration配置完成\n");

	ConfigureSlaveNode(CANOpenShellOD_Data, 0x03);
	while(!R3_Configuration){}
	printf("R3_Configuration==%d配置完成\n",R3_Configuration);
	Log("R3_Configuration配置完成\n");

	ConfigureSlaveNode(CANOpenShellOD_Data, 0x04);
	while(!R4_Configuration){}
	printf("R4_Configuration==%d配置完成\n",R4_Configuration);
	Log("R4_Configuration配置完成\n");

	ConfigureSlaveNode(CANOpenShellOD_Data, 0x05);
	while(!R5_Configuration){}
	printf("R5_Configuration==%d配置完成\n",R5_Configuration);
	Log("R5_Configuration配置完成\n");

	ConfigureSlaveNode(CANOpenShellOD_Data, 0x06);
	while(!R6_Configuration){}
	printf("R6_Configuration==%d配置完成\n",R6_Configuration);
	Log("R6_Configuration配置完成\n");

	ConfigureSlaveNode(CANOpenShellOD_Data, 0x07);
	while(!R7_Configuration){}
	printf("R7_Configuration==%d配置完成\n",R7_Configuration);
	Log("R7_Configuration配置完成\n");

	ConfigureSlaveNode(CANOpenShellOD_Data, 0x08);
	while(!R8_Configuration){}
	printf("R8_Configuration==%d配置完成\n",R8_Configuration);
	Log("R8_Configuration配置完成\n");

	ConfigureSlaveNode(CANOpenShellOD_Data, 0x09);
	while(!R9_Configuration){}
	printf("R9_Configuration==%d配置完成\n",R9_Configuration);
	Log("R9_Configuration配置完成\n");

	printf("延迟0.5S后再次判断电机初始化状态并开始检测电机+IO心跳\n");
	Log("延迟0.5S后再次判断电机初始化状态并开始检测电机+IO心跳\n");
	//加入一个延迟0.5S后再次判断电机初始化状态并开始检测电机+IO心跳
	int delaycount1=0;
	while(delaycount1<=100000000) //0.5S
		{
			delaycount1++;
		} //100000000大概1.2s左右；300000000=4.5s左右

	 if((R1_Configuration &&
		 R2_Configuration &&
		 R3_Configuration &&
		 R4_Configuration &&
		 R5_Configuration &&
		 R6_Configuration &&
		 R7_Configuration &&
		 R8_Configuration &&
		 R9_Configuration) ==1)
	 {
		 agvreportstruct.MotorError = 0;//上报电机运行正常
		 HeartbeatStartFlag=2;//心跳报文开始检测标志位 1=IO站，2=IO+电机
		 Log("电机初始化完成！电机心跳开始检测");
		 printf("电机初始化完成！电机心跳开始检测\n");
		 return 0;
	 }
	return -1;
}


int CloseAndOpenK2()
{
	/*向IO站发送信号，打开K2接通电机电源，打开K2时，应判断K1标志位
		 * K1-闭合  K2-断开
		 * K1-断开  K2-任何
		 * */
		//			UNS8 K1=0x01;
		UNS8 K2 = 0x02;
		if (ChargeFlag ==0) //判断K1是否闭合，若k1闭合则断开K2 0=未在充电，1=在充电
			{
			RelayClose(K2); //闭合K2;只在第一次配置IO站时，可以触发，若IO站重新上电切BBB程序未重新启动，则两次发送结果一样 放弃发送
			sendPDOevent(CANOpenShellOD_Data);
			printf("打开K2,接通电机电源\n");
			Log("打开K2,接通电机电源");
			//加入一个延迟，0.5S后断开K2
			int delaycount=0;
			while(delaycount<=50000000) //0.5S
				{
					delaycount++;
				} //100000000大概1.2s左右；300000000=4.5s左右
			RelayOpen(K2); //1s后断开K2
			sendPDOevent(CANOpenShellOD_Data);
			printf("延迟0.5s断开K2\n");
			Log("延迟0.5s断开K2");
			return 0;
			}
		 else if(ChargeFlag == 1)
			{
			RelayOpen(K2); //加入自锁后，就不能通过openK2来断开电机供电，应使用open K2的常闭
			RelayClose(K9);
			sendPDOevent(CANOpenShellOD_Data);
			printf("错误！K1闭合时，曾尝试闭合K2，已断开K2 \n");
			Log("错误！K1闭合时，曾尝试闭合K2，已断开K2");
			return -1;
			}
	return -1;
}


int UpMotorGoZero()
{
	printf("举升电机开始找0\n");

	int limitswitch_up = 0; //上限位开关
	int limitswitch_down = 0; //下限位开关
	int LiftZeroSwitch=0;//举升找零接近开关

	INTEGER32 LiftMotorFIndZero=0xFFD28000;

//	innerstruct.StopCommand = 0;

	M3_TPDO_Target_Position = (LiftMotorFIndZero + UpMotorInitPosition); //将位置信息传给OD
	sendPDOevent(CANOpenShellOD_Data); //发送PDO
	M3_TPDO_control_word = LiftStart; //启动电机
	sendPDOevent(CANOpenShellOD_Data);

	while (1) {
		//问题：直接使用IN5_Limit_Switch_bit1_8用于判断，判断时，是否可被写入？？是否冲突？？！
		//取出举升上限位开关=bit4
		limitswitch_up = (IN5_Limit_Switch_bit1_8 & (1 << 4)) ? 1 : 0; //判断bit4=1，则limitswitch_up=1；若bit4=0，则limitswitch_up=0

		//取出举升下限位开关=bit5
		limitswitch_down = (IN5_Limit_Switch_bit1_8 & (1 << 5)) ? 1 : 0; //判断bit5=1，则limitswitch_up=1；若bit5=0，则limitswitch_up=0

		//取出举升找零接近开关IN5_Limit_Switch_bit9_10
		LiftZeroSwitch =(IN5_Limit_Switch_bit9_10 & (1<<4)) ? 1 : 0;

		/*判断电机是否到达目标位置*/
		if (LiftZeroSwitch) //检测LiftZeroSwitch举升0位置传感器触发=1
		{
			printf("举升找到0点\n");
			Log("举升找到0点");
			M3_TPDO_control_word = LiftStop; //修改bit4=0
			sendPDOevent(CANOpenShellOD_Data);
			return 0;
		}
		else if (innerstruct.StopCommand == 1) //如果收到停止命令，正常情况下轮子转动过程中不能停止，但遇到特殊情况强行停止也可以，只是要记录到日志里
			{
			//发送停止命令
			printf("举升电机找零过程收到停止命令\n");
			Log("举升电机找零过程收到停止命令");
			//return AgvStop();
			//return AgvEmcyStop(); //初始化过程中，innerstruct.StopCommand == 1说明总线收到EMCY命令，设备出错
			return -1;
			}

		if (limitswitch_up == 1 || limitswitch_down == 1) //上或下限位开关触发！
				{
			Log("agv举升或下降过程中限位开关触发");
			printf("agv举升或下降过程中限位开关触发\n");
			//return AgvEmcyStop(); //停止所有电机，红灯亮;
			return -1;
				}
	}

	return 0;
}

int GetUpMotorInitPosition()
{
	UpMotorCount++;
	while(R9_position==0){};
	UpMotorInitPosition=R9_position;

	if(UpMotorCount==1){
	printf("第一次获取举升电机初始位置成功 UpMotorInitPosition==%d\n",UpMotorInitPosition);
	char str[128]={0};
	sprintf(str,"第一次获取举升电机初始位置成功 UpMotorInitPosition==%d",UpMotorInitPosition);
	Log(str); //输出字符串str给LOG
	/*清空str数组*/
	memset(str,0,sizeof(str));
	}

	if(UpMotorCount==2){
	printf("第二次获取举升电机初始位置成功 UpMotorInitPosition==%d\n",UpMotorInitPosition);
	char str[128]={0};
	sprintf(str,"第二次获取举升电机初始位置成功 UpMotorInitPosition==%d",UpMotorInitPosition);
	Log(str); //输出字符串str给LOG
	/*清空str数组*/
	memset(str,0,sizeof(str));
	}

	return 0;
}

int CLoseGreen()
{
	//闭合绿灯
	RelayClose(K4);//所有设备初始化完成，可以正常运行
	RelayOpen(K5);
	RelayOpen(K6);
	sendPDOevent(CANOpenShellOD_Data);
	Log("所有设备初始化完毕，绿灯点亮，AGV可以开始运行");
	printf("所有设备初始化完毕，绿灯点亮，AGV可以开始运行\n");
	return 0;
}

int InitZeroPosition()//轮子回到标准方向,注意回到标准方向时正常情况下没有限位开关，如果电机出现故障不停的话，会碰到另一侧的限位开关
{

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
	while(delaycount<=position_delay) //delay 1秒，等待电机开始运行再开始检测
	{
		delaycount++;
	} //100000000大概1.2s左右；300000000=4.5s左右

	while(1)
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
			//M2_TPDO_control_word=RotatorStop;//修改bit4=0
			//sendPDOevent(CANOpenShellOD_Data);
			AgvStop();//断转向使能
			Log("轮子初始化归0成功");
			printf("轮子初始化归0成功\n");
			return 0;//命令执行成功，返回正常值
		}

		else if (innerstruct.StopCommand == 1) //如果收到停止命令，正常情况下轮子转动过程中不能停止，但遇到特殊情况强行停止也可以，只是要记录到日志里
			{
			//发送停止命令
			printf("轮子初始化归0过程收到停止命令\n");
			Log("轮子初始化归0过程收到停止命令");
		//	return AgvStop();
		//	return AgvEmcyStop(); //初始化过程中，innerstruct.StopCommand == 1说明总线收到EMCY命令，设备出错
			return -1;
			}

		/*判断限位开关*/
		if(limitswitch_rotator5_up==1||limitswitch_rotator5_down==1 ||
		   limitswitch_rotator6_up==1||limitswitch_rotator6_down==1 ||
		   limitswitch_rotator7_up==1||limitswitch_rotator7_down==1 ||
		   limitswitch_rotator8_up==1||limitswitch_rotator8_down==1 )//取出5-8位的LimitSwitch数据，判断是否大于0
		{
			Log("转向电机初始化归0过程中触发限位开关");
			printf("转向电机初始化归0过程中触发限位开关\n");
		//	return AgvEmcyStop(); //停止所有电机，红灯亮;
			return -1;
		}
	}
	return 0;
}

int CanInit()
{
	TimerInit(); //初始化Can总线所需的时间环
	int aa=NodeInit(0, 0); //初始化Can主节点及其从节点
	if(aa==0)
	{
		printf("Can总线初始化完毕！\n");
		Log("Can总线初始化完毕！");
		return 0;
	}
	if(aa==2)
	{
		printf("Can总线出现中断错误！！\n");
		Log("Can总线出现中断错误！");
	}
	return -1;
}

int SenserInit()//传感器初始化，以能读到各传感器信号为准，也就是agvreportstruct里面的内容全都由初始化值变成了正常值
{
	int flag=0,overtime=0;//定义一个标志位，每while10次打印一次等待数据。避免输出太频繁,定义一个超时标志，时间过长退出初始化。
	while(1)
	{
		//如果12个传感器有任何一个没收到信号，就继续等待。
		if(!strcmp(agvreportstruct.RFIDSenser[0],"-1")||!strcmp(agvreportstruct.RFIDSenser[1],"-1")||!strcmp(agvreportstruct.RFIDSenser[2],"-1")||!strcmp(agvreportstruct.RFIDSenser[3],"-1")||agvreportstruct.NavigationSenser[0]==-1||agvreportstruct.NavigationSenser[1]==-1||
				agvreportstruct.NavigationSenser[2]==-1||agvreportstruct.NavigationSenser[3]==-1||agvreportstruct.HitAvoidSenser[0]==-1||agvreportstruct.HitAvoidSenser[1]==-1||agvreportstruct.HitAvoidSenser[2]==-1||agvreportstruct.HitAvoidSenser[3]==-1)
		{
			if(flag==10000)
			{
				Log("正在等待传感器信号！");
				printf("正在等待传感器信号！\n");
				overtime++;
				flag=0;
			}
			else
			{
				flag++;
			}
		}
		else
		{
			Log("收到传感器正常信号，传感器初始化完成！");
			printf("收到传感器正常信号，传感器初始化完成!\n");
			return 0;
		}

		if(overtime==10)
		{
			Log("传感器初始化超时！");
			printf("传感器初始化超时！\n");
			return -1;
		}

	}
	return -1;
}

int PowerManageInit()//电源管理初始化
{
	printf("电源管理初始化完毕！\n");
	Log("电源管理初始化完毕");
	return 0;
}


/*IO站*/
int LimitSwitchInit()
{
	int flag=0,overtime=0;//定义一个标志位，每while10次打印一次等待数据。避免输出太频繁,定义一个超时标志，时间过长退出初始化
	while(1)
	{
		/*检测IO_Configuration，直到初始化完成=1*/
		if(IO_Configuration !=1)
		{
			if(flag==100000000)
			{
				Log("正在等待IO站初始化!");
				printf("正在等待IO站初始化！\n");
				overtime++;
				flag=0;
			}
			else
			{
				flag++;
			}
		}
		else
		{
			agvreportstruct.IOError=0;
			HeartbeatStartFlag=1;//心跳报文开始检测标志位 1=IO站，2=IO+电机
			printf("IO站初始化完成！IO心跳报文开始检测\n");
			Log("IO站初始化完成！IO心跳报文开始检测");
			return 0;
		}
		if(overtime==10)
		{
			agvreportstruct.IOError=1;//上报IO运行正常
			Log("IO站初始化超时！\n");
			printf("IO站初始化超时！\n");
			return -1;
		}
	}
	return -1;
}

void InitStruct()
{
	agvreportstruct.MotorVel[0]=R1_speed;
	agvreportstruct.MotorVel[1]=R2_speed;
	agvreportstruct.MotorVel[2]=R3_speed;
	agvreportstruct.MotorVel[3]=R4_speed;

	agvreportstruct.WheelDirection=8;//初始位置=8=未知未知，上线后首先要回0，每到达位置后，都向此变量写入位置数值;0=零位置与长边平行,1=横移位置与宽边平行,2=自旋位置，
	agvreportstruct.MotorError=1; //初始=1，完成初始化后 =0
	agvreportstruct.IOError=1;//初始=1，完成初始化后 =0
	agvreportstruct.ExecuteCommand=0;
	agvreportstruct.RfidError=0;
	agvreportstruct.RWFlag=1;
	agvreportstruct.RemoteControl=0; //0=上位机模式，1=遥控器模式

	strcpy(agvreportstruct.RFIDSenser[0],"0011");
	strcpy(agvreportstruct.RFIDSenser[1],"0012");
	strcpy(agvreportstruct.RFIDSenser[2],"0013");
	strcpy(agvreportstruct.RFIDSenser[3],"0014");

	agvreportstruct.LimitSwitch=0;
	agvreportstruct.BattaryCharge=0;
	agvreportstruct.RWFlag=1;

	innerstruct.RWflag=0;
	innerstruct.StopCommand=0;
	innerstruct.SlowCommand=0;
	innerstruct.Command="";
}
