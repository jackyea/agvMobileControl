#include "MotorControl.h"
#include "CanOpenShell.h" //引用extern CO_Data* CANOpenShellOD_Data;
#include"CANOpenShellMasterOD.h" //用于发送PDO时修改IO站变量打开K2
#include"Log.h"
#include "configureslave.h"

//声明函数ConfigureSlaveNode 因在CheckSDOAndContinue中使用且此函数在VOID ConfigureSlaveNode之前
//void ConfigureSlaveNode(CO_Data* d, UNS8 nodeId);

//闭合继电器 输出1
#define RelayClose(a) OUT8_Relay_bit1_8 |=a
//断开继电器 输出0
#define RelayOpen(b) OUT8_Relay_bit1_8 &=~b

static int init_step1 = 0; //init_step用于配置从站时case的数值
static int init_step2 = 0; //init_step用于配置从站时case的数值
static int init_step3 = 0; //init_step用于配置从站时case的数值
static int init_step4 = 0; //init_step用于配置从站时case的数值
static int init_step5 = 0; //init_step用于配置从站时case的数值
static int init_step6 = 0; //init_step用于配置从站时case的数值
static int init_step7 = 0; //init_step用于配置从站时case的数值
static int init_step8 = 0; //init_step用于配置从站时case的数值
static int init_step9 = 0; //init_step用于配置从站时case的数值
static int init_stepF = 0; //init_step用于配置从站时case的数值
static int init_stepA = 0; //init_step用于配置从站时case的数值
static int inti_stepUpMotor=0;

int num_slavebootup = 0; //用于监测何时配置完成所有主站，配置完成之后主程序继续

int configcout = 0;

static void CheckSDOAndContinue(CO_Data* d, UNS8 nodeId)
{
	UNS32 abortCode;

	if (getWriteResultNetworkDict(d, nodeId, &abortCode) != SDO_FINISHED)
		{

			eprintf("从站 %2.2x 初始化SDO配置失败 AbortCode :%4.4x \n",nodeId, abortCode);

			char str[128]={0};
			sprintf(str,"从站 %2.2x 初始化SDO配置失败, AbortCode :%4.4x \n",nodeId,  abortCode);
			Log(str); //输出字符串str给LOG
		/*清空str数组*/
			memset(str,0,sizeof(str));

			AgvEmcyStop();

			return;
			}

	/* Finalise last SDO transfer with this node */
	closeSDOtransfer(CANOpenShellOD_Data, nodeId, SDO_CLIENT);

	ConfigureSlaveNode(CANOpenShellOD_Data, nodeId);
}

static void CheckUpMotorNomalSpeed(CO_Data* d, UNS8 nodeId) {
	UNS32 abortCode;

	if (getWriteResultNetworkDict(d, nodeId, &abortCode) != SDO_FINISHED) {

			eprintf("从站 %2.2x 初始化SDO配置失败, AbortCode :%4.4x \n",
					nodeId,abortCode);
		eprintf("从站 %2.2x 初始化SDO配置失败,\n", nodeId);
		fflush(stdout);

			char str[128]={0};
			sprintf(str,"从站 %2.2x 初始化SDO配置失败,AbortCode :%4.4x \n",
					nodeId,abortCode);
			Log(str); //输出字符串str给LOG

		/*清空str数组*/
			memset(str,0,sizeof(str));

			AgvEmcyStop();

			return;
		}

	printf("进入举升重设速度\n");

/* Finalise last SDO transfer with this node */
closeSDOtransfer(CANOpenShellOD_Data, nodeId, SDO_CLIENT);

UpMotorZero(CANOpenShellOD_Data, nodeId);
}

/**************************从 站 初 始 化*********************************/
void ConfigureSlaveNode(CO_Data* d, UNS8 nodeId) {

	UNS8 Transmission_TPDO = 0x01;  //01同步：用于从站TPDO 状态PDO
	UNS8 Transmission_RPDO = 0x00; //For Slave RPDO type  0-240: Hold the data until the following SYNC received. 254-255:immediately upon reception
	UNS8 Position_Mode_RPDO = 0xFF; //异步FF：用于位置模式，防止数据更新不及时
	UNS16 heartdata = 0x03E8;		//心跳周期ms：1000ms
	UNS32 RotateVelocity = 0x005A0000; //270  转向速度
//	UNS32 RotateVelocity = 0x008A0000; //410 转向速度
	UNS32 RotateAcc = 0x001C0000; //140 转向加速度
	INTEGER16 HaltOption = 0x0001; //bit8HALT停止模式，02=using quick stop deceleration，01=正常减速度停止
	UNS32 QuickStopDeceleration=0x000145F4; //1.6;
//	UNS32 straight_deceleration = 0x000065DC;//0.5rps
//	UNS32 straight_deceleration = 0x00007A3B;//0.6rps
	UNS32 straight_deceleration2 =0x0000517D;//0.4rps 行走减速度
//	UNS32 straight_deceleration2 =0x000065DC;//0.4*1.25rps 行走减速度
//	UNS32 straight_deceleration2 =0x00006ADC;//0.4*1.25rps 行走减速度
	UNS32 straight_deceleration =straight_deceleration2;//0.4rps 行走减速度
//	UNS32 straight_deceleration = 0x00013193;//1.44+rpsrps
	INTEGER32 WalkingSpeed=Clockwise_WalkNomalSpeed;// 行走位置模式速度
	UNS32 straight_acceleration2 = 0x0000517D;//0.4 行走加速度
	UNS32 straight_acceleration = straight_acceleration2;//0.4 行走加速度

	UNS8 Velocity_mode = 0x01;  //03=速度 01=位置


	/*第一组：行走电机 nodeID:01-04 速度模式
	 *
	 *TPDO:状态PDO 同步：01-240  异步：254-255  （十进制）
	 *		状态字  (status words 6041)				== 180 + nodeID 异步FF
	 *		运行模式(Mode of Operation Display 6061)	== 280 + nodeID 异步FF
	 *		当前速度+位置(Actual Velocity 606C,Position Acture value 6064) == 380 + nodeID 同步01
	 *      禁用默认TPDO4（Actural Velocity 606C）480+nodeID 1803-1A03；避免发送无畏信息
	 *
	 *RPDO:控制PDO 同步：00-240  异步：254-255  （十进制）
	 *		控制字  (control words 6040)		== 210	同步00
	 *		运行模式(Mode of Operation 6060)	== 310	同步00
	 *
	 *		**速度模式:目标速度(target velocity 60FF)	==	410 同步00
	 *		速度模式:目标速度(target velocity 60FF)	==	410 + nodeID 同步00
	 *
	 //	 *		**加速度，减速度(Profile Acceleraion 6083,Profile Deceleration 6084)	==	510 同步00
	 //	 *		加速度，减速度(Profile Acceleraion 6083,Profile Deceleration 6084)	==	510 + nodeID 同步00
	 //	 *
	 //	 *		**位置模式:目标位置(target position 607A),速度(Profile Velocity 6081)	== 610 异步FF
	 //	 *		位置模式:目标位置(target position 607A),速度(Profile Velocity 6081)	== 610 + nodeID 异步FF
	 *		 */

	if (nodeId == 0x01) {
		switch (++init_step1) {

		/************************** 配置TPDO **************************************/
		/* 运行模式(Mode of Operation Display 6061)	== 280 + nodeID 异步FF */
		case 1: { /*disable Slave's TPDO2 */
			UNS32 TPDO2_COBId = 0x80000280 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x01, 4, 0,
					&TPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 2: { /*Set Slave's TPDO 2 to be transmitted on FF*/
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 3: { /*Reset Slave's RPDO 2 Mapping number */
			UNS8 TPDO2_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x00, 1, 0,
					&TPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 4: { /*Maping Slave's TPDO 2 with mode_of_operation_Display in 6061 */
			UNS32 TPDO2_mode_of_operation_Display = 0x60610008;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x01, 4, 0,
					&TPDO2_mode_of_operation_Display, CheckSDOAndContinue, 0);
		}
			break;

		case 5: { /*Set Slave's RPDO 3 Mapping number */
			UNS8 TPDO2_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x00, 1, 0,
					&TPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 6: { /*Re-Ensable Slave's TPDO*/
			UNS32 TPDO2_COBId = 0x00000280 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x01, 4, 0,
					&TPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 当前速度+位置(Actual Velocity 606C,Position Acture value 6064) == 380 + nodeID 同步01 */
		case 7: { /*disable Slave's TPDO3 */
			UNS32 TPDO3_COBId = 0x80000380 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x01, 4, 0,
					&TPDO3_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 8: { /*Set Slave's TPDO 3 to be transmitted on 01*/
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x02, 1, 0,
					&Transmission_TPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 9: { /*Reset Slave's RPDO 3 Mapping number */
			UNS8 TPDO3_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x00, 1, 0,
					&TPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 10: { /*Maping Slave's TPDO 3 with speed in 606C */
			UNS32 TPDO3_MapSpeed = 0x606C0020;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x01, 4, 0,
					&TPDO3_MapSpeed, CheckSDOAndContinue, 0);
		}
			break;

		case 11: { /*Maping Slave's TPDO 3 with Position in 6064 */
			UNS32 TPDO3_Mapposition = 0x60640020;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x02, 4, 0,
					&TPDO3_Mapposition, CheckSDOAndContinue, 0);
		}
			break;

		case 12: { /*Set Slave's RPDO 3 Mapping number */
			UNS8 TPDO3_MapNum = 0x02;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x00, 1, 0,
					&TPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 13: { /*Re-Ensable Slave's TPDO*/
			UNS32 TPDO3_COBId = 0x00000380 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x01, 4, 0,
					&TPDO3_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 禁用默认TPDO4（Actural Velocity 606C）480+nodeID 1803-1A03；避免发送无畏信息 */
		case 14: { /*disable Slave's TPDO4 */
			UNS32 TPDO4_COBId = 0x80000480 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1803, 0x01, 4, 0,
					&TPDO4_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/************************** 配置RPDO **************************************/

			/* 控制字  (control words 6040)		== 210	异步FF */
		case 15: { /*set Slave's RPDO 1 cobid to 210*/
			UNS32 RPDO1_COBId = 0x00000210;
			writeNetworkDictCallBack(d, nodeId, 0x1400, 0x01, 4, 0,
					&RPDO1_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 16: { /*Set Slave's RPDO 1 to be transmitted on FF!!! */
			writeNetworkDictCallBack(d, nodeId, 0x1400, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

			/* 运行模式(Mode of Operation 6060)	== 310	同步00 */
		case 17: { /*disable Slave's RPDO3 */
			UNS32 RPDO2_COBId = 0x80000310;
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x01, 4, 0,
					&RPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 18: { /*Set Slave's RPDO 2 to be transmitted on 00*/
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x02, 1, 0,
					&Transmission_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 19: { /*Reset Slave's RPDO 2 Mapping number */
			UNS8 RPDO2_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x00, 1, 0,
					&RPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 20: { /*Maping Slave's RPDO 2 with mode_of_operation in 6060 */
			UNS32 RPDO3_Map_mode_of_operation = 0x60600008;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x01, 4, 0,
					&RPDO3_Map_mode_of_operation, CheckSDOAndContinue, 0);
		}
			break;

		case 21: { /*Set Slave's RPDO 2 Mapping number */
			UNS8 RPDO2_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x00, 1, 0,
					&RPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 22: { /*Re-Ensable Slave's RPDO2*/
			UNS32 RPDO2_COBId = 0x00000310;
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x01, 4, 0,
					&RPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 速度模式:目标速度(target velocity 60FF)==	410 + nodeID 同步00 */
		case 23: { /*disable Slave's RPDO3 */
			UNS32 RPDO3_COBId = 0x80000410 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x01, 4, 0,
					&RPDO3_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 24: { /*Set Slave's RPDO 3 to be transmitted on 00*/
			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x02, 1, 0,
					&Transmission_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 25: { /*Reset Slave's RPDO 3 Mapping number */
			UNS8 RPDO3_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x00, 1, 0,
					&RPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 26: { /*Maping Slave's RPDO 3 with Target_Velocity in 60FF */
			UNS32 RPDO3_MapTarget_Velocity = 0x60FF0020;
			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x01, 4, 0,
					&RPDO3_MapTarget_Velocity, CheckSDOAndContinue, 0);
		}
			break;

		case 27: { /*Set Slave's RPDO 3 Mapping number */
			UNS8 RPDO3_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x00, 1, 0,
					&RPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 28: { /*Re-Ensable Slave's RPDO*/
			UNS32 RPDO3_COBId = 0x00000410 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x01, 4, 0,
					&RPDO3_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 加速度，减速度(Profile Acceleraion 6083,Profile Deceleration 6084)	==	510  同步00 */
		case 29: { /*disable Slave's RPDO4 */
			UNS32 RPDO4_COBId = 0x80000510;
			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x01, 4, 0,
					&RPDO4_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 30: { /*Set Slave's RPDO 4 to be transmitted on ff*/
			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 31: { /*Reset Slave's RPDO 4 Mapping number */
			UNS8 RPDO4_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x00, 1, 0,
					&RPDO4_MapNum, CheckSDOAndContinue, 0);
		}
			break;

//		case 31: { /*Maping Slave's RPDO 4 with profile acceleration */
//			UNS32 RPDO4_MapProfile_acceleration = 0x60830020;
//			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x01, 4, 0,
//					&RPDO4_MapProfile_acceleration, CheckSDOAndContinue, 0);
//		}
//			break;

		case 32: { /*Maping Slave's RPDO 4 with profile deceleration */
			UNS32 RPDO4_MapProfile_deceleration = 0x60840020;
			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x01, 4, 0,
					&RPDO4_MapProfile_deceleration, CheckSDOAndContinue, 0);
		}
			break;
		case 33: { /*Set Slave's RPDO 4 Mapping number */
			UNS8 RPDO4_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x00, 1, 0,
					&RPDO4_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 34: { /*Re-Ensable Slave's RPDO*/
			UNS32 RPDO4_COBId = 0x00000510;
			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x01, 4, 0,
					&RPDO4_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 位置模式:目标位置(target position 607A),速度(Profile Velocity 6081) == 610 + nodeID 异步FF */
		case 35: { /*disable Slave's RPDO5*/
			UNS32 RPDO5_COBId = 0x80000610 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x01, 4, 0,
					&RPDO5_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 36: { /*Set Slave's RPDO 5 to be transmitted on FF*/
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 37: { /*Reset Slave's RPDO 5 Mapping number */
			UNS8 RPDO5_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x00, 1, 0,
					&RPDO5_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 38: { /*Maping Slave's RPDO 5 with Target_position in 607A */
			UNS32 RPDO5_MapTarget_position = 0x607A0020;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x01, 4, 0,
					&RPDO5_MapTarget_position, CheckSDOAndContinue, 0);
		}
			break;

//		case 38: { /*Maping Slave's RPDO 5 with profile_velocity */
//			UNS32 RPDO5_MapProfile_velocity = 0x60810020;
//			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x02, 4, 0,
//					&RPDO5_MapProfile_velocity, CheckSDOAndContinue, 0);
//		}
//			break;
//
		case 39: { /*Set Slave's RPDO 5 Mapping number */
			UNS8 RPDO4_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x00, 1, 0,
					&RPDO4_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 40: { /*Re-Ensable Slave's RPDO5*/
			UNS32 RPDO5_COBId = 0x00000610 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x01, 4, 0,
					&RPDO5_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/*** 位置模式:速度(Profile Velocity 6081) == 710  异步FF ***/
		case 41: { /*disable Slave's RPDO 6*/
			UNS32 RPDO6_COBId = 0x80000710;
			writeNetworkDictCallBack(d, nodeId, 0x1405, 0x01, 4, 0,
					&RPDO6_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 42: { /*Set Slave's RPDO 6 to be transmitted on FF*/
			writeNetworkDictCallBack(d, nodeId, 0x1405, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 43: { /*Reset Slave's RPDO 6 Mapping number */
			UNS8 RPDO6_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1605, 0x00, 1, 0,
					&RPDO6_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 44: { /*Maping Slave's RPDO 6 with profile_velocity6081 */
			UNS32 RPDO6_MapProfile_velocity = 0x60810020;
			writeNetworkDictCallBack(d, nodeId, 0x1605, 0x01, 4, 0,
					&RPDO6_MapProfile_velocity, CheckSDOAndContinue, 0);
		}
			break;

		case 45: { /*Set Slave's RPDO 5 Mapping number */
			UNS8 RPDO4_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1605, 0x00, 1, 0,
					&RPDO4_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 46: { /*Re-Ensable Slave's RPDO6*/
			UNS32 RPDO6_COBId = 0x00000710;
			writeNetworkDictCallBack(d, nodeId, 0x1405, 0x01, 4, 0,
					&RPDO6_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/*************************设置电机运行参数**************************/

			//1.运行模式 ---速度模式
			//2.加速度减速度
			//3.control word=010F；
			//4.homeing =当前位置
		case 47: { /*Set Slave into Velocity Mode = 01*/
			writeNetworkDictCallBack(d, nodeId, 0x6060, 0x00, 1, 0,
					&Velocity_mode, CheckSDOAndContinue, 0);
		}
			break;

		case 48: { /*Set Slave Motion profile type--梯形*/
			UNS16 Motion_Profile_type = 0x0000;
			writeNetworkDictCallBack(d, nodeId, 0x6086, 0x00, 2, 0,
					&Motion_Profile_type, CheckSDOAndContinue, 0);
		}
			break;

		case 49: { /* Set Slave profile Velocity 6081
		 旋转电机转速=0.5转/s =655360 counts/s =0x000A0000*/
			writeNetworkDictCallBack(d, nodeId, 0x6081, 0x00, 4, 0,
					&WalkingSpeed, CheckSDOAndContinue, 0);
		}
			break;

		case 50: { /*Set Slave Acceleration
		 行走轮=0.4 m/s^2;
		 电机=20861 counts/s^2 = 0x0000517D*/
			writeNetworkDictCallBack(d, nodeId, 0x6083, 0x00, 4, 0,
					&straight_acceleration, CheckSDOAndContinue, 0);
		}
			break;

		case 51: { /*Set Slave Deceleration;与加速度数值相同*/
			writeNetworkDictCallBack(d, nodeId, 0x6084, 0x00, 4, 0,
					&straight_deceleration, CheckSDOAndContinue, 0);
		}
			break;

		case 52: { /*Set homing position 00：Home is Current Position*/
			UNS8 home_current_position = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x6098, 0x00, 1, 0,
					&home_current_position, CheckSDOAndContinue, 0);
		}
			break;

		case 53: { /*0x605D Halt option 02=using quick stop ramp,*/
			writeNetworkDictCallBack(d, nodeId, 0x605D, 0x00, 2, 0, &HaltOption,
					CheckSDOAndContinue, 0);
		}
			break;


		case 54: { /*0x6085 quick stop deceleration,*/
			writeNetworkDictCallBack(d, nodeId, 0x6085, 0x00, 4, 0, &QuickStopDeceleration,
					CheckSDOAndContinue, 0);
		}
			break;

		case 55: { /*不使能电机 0107*/
			UNS16 enable = 0x016F;
//			UNS16 enable = 0x0167; //断使能
			writeNetworkDictCallBack(d, nodeId, 0x6040, 0x00, 2, 0, &enable,
					CheckSDOAndContinue, 0);
		}
			break;

			/**************************设置从站心跳报文**************************/

		case 56: /*Third step: configure Slave Heart-beat interval*/
			writeNetworkDictCallBack(d, /*CO_Data* d*/
			nodeId, /*UNS8 nodeId*/
			0x1017, /*UNS16 index*/
			0x00, /*UNS16 index*/
			2, /*UNS8 count*/
			0, /*UNS8 dataType*/
			&heartdata,/*data must be a index*/
			CheckSDOAndContinue, /*SDOCallback_t Callback*/
			0); /* use block mode */
			break;


			/****************************** 启动从节点 *******************************/
		case 57: {
			/* Put the master in operational mode */
			setState(CANOpenShellOD_Data, Operational);

			/* Ask slave node to go in operational mode */
			StartNode(nodeId);

			/* reset counter in case the slave boot-up again */
//			init_step1++; //不需要置0，避免电机重新上电从新配置。当电机重新上电此时init_step1==35，进入配置后++init_step1 ==36，并没有case36则跳过。
			num_slavebootup++;
			break;
				}
		}
	}

	if (nodeId == 0x02) {
		switch (++init_step2) {

		/************************** 配置TPDO **************************************/
		/* 运行模式(Mode of Operation Display 6061)	== 280 + nodeID 异步FF */
		case 1: { /*disable Slave's TPDO2 */
			UNS32 TPDO2_COBId = 0x80000280 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x01, 4, 0,
					&TPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 2: { /*Set Slave's TPDO 2 to be transmitted on FF*/
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 3: { /*Reset Slave's RPDO 2 Mapping number */
			UNS8 TPDO2_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x00, 1, 0,
					&TPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 4: { /*Maping Slave's TPDO 2 with mode_of_operation_Display in 6061 */
			UNS32 TPDO2_mode_of_operation_Display = 0x60610008;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x01, 4, 0,
					&TPDO2_mode_of_operation_Display, CheckSDOAndContinue, 0);
		}
			break;

		case 5: { /*Set Slave's RPDO 3 Mapping number */
			UNS8 TPDO2_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x00, 1, 0,
					&TPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 6: { /*Re-Ensable Slave's TPDO*/
			UNS32 TPDO2_COBId = 0x00000280 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x01, 4, 0,
					&TPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 当前速度+位置(Actual Velocity 606C,Position Acture value 6064) == 380 + nodeID 同步01 */
		case 7: { /*disable Slave's TPDO3 */
			UNS32 TPDO3_COBId = 0x80000380 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x01, 4, 0,
					&TPDO3_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 8: { /*Set Slave's TPDO 3 to be transmitted on 01*/
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x02, 1, 0,
					&Transmission_TPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 9: { /*Reset Slave's RPDO 3 Mapping number */
			UNS8 TPDO3_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x00, 1, 0,
					&TPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 10: { /*Maping Slave's TPDO 3 with speed in 606C */
			UNS32 TPDO3_MapSpeed = 0x606C0020;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x01, 4, 0,
					&TPDO3_MapSpeed, CheckSDOAndContinue, 0);
		}
			break;

		case 11: { /*Maping Slave's TPDO 3 with Position in 6064 */
			UNS32 TPDO3_Mapposition = 0x60640020;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x02, 4, 0,
					&TPDO3_Mapposition, CheckSDOAndContinue, 0);
		}
			break;

		case 12: { /*Set Slave's RPDO 3 Mapping number */
			UNS8 TPDO3_MapNum = 0x02;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x00, 1, 0,
					&TPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 13: { /*Re-Ensable Slave's TPDO*/
			UNS32 TPDO3_COBId = 0x00000380 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x01, 4, 0,
					&TPDO3_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 禁用默认TPDO4（Actural Velocity 606C）480+nodeID 1803-1A03；避免发送无畏信息 */
		case 14: { /*disable Slave's TPDO4 */
			UNS32 TPDO4_COBId = 0x80000480 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1803, 0x01, 4, 0,
					&TPDO4_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/************************** 配置RPDO **************************************/

			/* 控制字  (control words 6040)		== 210	FF */
		case 15: { /*set Slave's RPDO 1 cobid to 210*/
			UNS32 RPDO1_COBId = 0x00000210;
			writeNetworkDictCallBack(d, nodeId, 0x1400, 0x01, 4, 0,
					&RPDO1_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 16: { /*Set Slave's RPDO 1 to be transmitted on FF!!! */
			writeNetworkDictCallBack(d, nodeId, 0x1400, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

			/* 运行模式(Mode of Operation 6060)	== 310	同步00 */
		case 17: { /*disable Slave's RPDO3 */
			UNS32 RPDO2_COBId = 0x80000310;
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x01, 4, 0,
					&RPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 18: { /*Set Slave's RPDO 2 to be transmitted on 00*/
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x02, 1, 0,
					&Transmission_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 19: { /*Reset Slave's RPDO 2 Mapping number */
			UNS8 RPDO2_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x00, 1, 0,
					&RPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 20: { /*Maping Slave's RPDO 2 with mode_of_operation in 6060 */
			UNS32 RPDO3_Map_mode_of_operation = 0x60600008;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x01, 4, 0,
					&RPDO3_Map_mode_of_operation, CheckSDOAndContinue, 0);
		}
			break;

		case 21: { /*Set Slave's RPDO 2 Mapping number */
			UNS8 RPDO2_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x00, 1, 0,
					&RPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 22: { /*Re-Ensable Slave's RPDO2*/
			UNS32 RPDO2_COBId = 0x00000310;
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x01, 4, 0,
					&RPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 速度模式:目标速度(target velocity 60FF)==	410 + nodeID 同步00 */
		case 23: { /*disable Slave's RPDO3 */
			UNS32 RPDO3_COBId = 0x80000410 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x01, 4, 0,
					&RPDO3_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 24: { /*Set Slave's RPDO 3 to be transmitted on 00*/
			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x02, 1, 0,
					&Transmission_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 25: { /*Reset Slave's RPDO 3 Mapping number */
			UNS8 RPDO3_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x00, 1, 0,
					&RPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 26: { /*Maping Slave's RPDO 3 with Target_Velocity in 60FF */
			UNS32 RPDO3_MapTarget_Velocity = 0x60FF0020;
			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x01, 4, 0,
					&RPDO3_MapTarget_Velocity, CheckSDOAndContinue, 0);
		}
			break;

		case 27: { /*Set Slave's RPDO 3 Mapping number */
			UNS8 RPDO3_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x00, 1, 0,
					&RPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 28: { /*Re-Ensable Slave's RPDO*/
			UNS32 RPDO3_COBId = 0x00000410 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x01, 4, 0,
					&RPDO3_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 加速度，减速度(Profile Acceleraion 6083,Profile Deceleration 6084)	==	510 + nodeID 同步00 */
		case 29: { /*disable Slave's RPDO4 */
			UNS32 RPDO4_COBId = 0x80000510;
			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x01, 4, 0,
					&RPDO4_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 30: { /*Set Slave's RPDO 4 to be transmitted on ff*/
			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 31: { /*Reset Slave's RPDO 4 Mapping number */
			UNS8 RPDO4_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x00, 1, 0,
					&RPDO4_MapNum, CheckSDOAndContinue, 0);
		}
			break;

//		case 31: { /*Maping Slave's RPDO 4 with profile acceleration */
//			UNS32 RPDO4_MapProfile_acceleration = 0x60830020;
//			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x01, 4, 0,
//					&RPDO4_MapProfile_acceleration, CheckSDOAndContinue, 0);
//		}
//			break;

		case 32: { /*Maping Slave's RPDO 4 with profile deceleration */
			UNS32 RPDO4_MapProfile_deceleration = 0x60840020;
			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x01, 4, 0,
					&RPDO4_MapProfile_deceleration, CheckSDOAndContinue, 0);
		}
			break;
		case 33: { /*Set Slave's RPDO 4 Mapping number */
			UNS8 RPDO4_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x00, 1, 0,
					&RPDO4_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 34: { /*Re-Ensable Slave's RPDO*/
			UNS32 RPDO4_COBId = 0x00000510;
			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x01, 4, 0,
					&RPDO4_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 位置模式:目标位置(target position 607A),速度(Profile Velocity 6081) == 610 + nodeID 异步FF */
		case 35: { /*disable Slave's RPDO5*/
			UNS32 RPDO5_COBId = 0x80000610 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x01, 4, 0,
					&RPDO5_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 36: { /*Set Slave's RPDO 5 to be transmitted on FF*/
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 37: { /*Reset Slave's RPDO 5 Mapping number */
			UNS8 RPDO5_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x00, 1, 0,
					&RPDO5_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 38: { /*Maping Slave's RPDO 5 with Target_position in 607A */
			UNS32 RPDO5_MapTarget_position = 0x607A0020;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x01, 4, 0,
					&RPDO5_MapTarget_position, CheckSDOAndContinue, 0);
		}
			break;

		case 39: { /*Set Slave's RPDO 5 Mapping number */
			UNS8 RPDO4_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x00, 1, 0,
					&RPDO4_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 40: { /*Re-Ensable Slave's RPDO5*/
			UNS32 RPDO5_COBId = 0x00000610 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x01, 4, 0,
					&RPDO5_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/*** 位置模式:速度(Profile Velocity 6081) == 710  异步FF ***/
		case 41: { /*disable Slave's RPDO 6*/
			UNS32 RPDO6_COBId = 0x80000710;
			writeNetworkDictCallBack(d, nodeId, 0x1405, 0x01, 4, 0,
					&RPDO6_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 42: { /*Set Slave's RPDO 6 to be transmitted on FF*/
			writeNetworkDictCallBack(d, nodeId, 0x1405, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 43: { /*Reset Slave's RPDO 6 Mapping number */
			UNS8 RPDO6_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1605, 0x00, 1, 0,
					&RPDO6_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 44: { /*Maping Slave's RPDO 6 with profile_velocity6081 */
			UNS32 RPDO6_MapProfile_velocity = 0x60810020;
			writeNetworkDictCallBack(d, nodeId, 0x1605, 0x01, 4, 0,
					&RPDO6_MapProfile_velocity, CheckSDOAndContinue, 0);
		}
			break;

		case 45: { /*Set Slave's RPDO 5 Mapping number */
			UNS8 RPDO4_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1605, 0x00, 1, 0,
					&RPDO4_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 46: { /*Re-Ensable Slave's RPDO6*/
			UNS32 RPDO6_COBId = 0x00000710;
			writeNetworkDictCallBack(d, nodeId, 0x1405, 0x01, 4, 0,
					&RPDO6_COBId, CheckSDOAndContinue, 0);
		}
			break;
			/*************************设置电机运行参数**************************/

			//1.运行模式 ---速度模式
			//2.加速度减速度
			//3.control word=010F；
			//4.homeing =当前位置
		case 47: { /*Set Slave into Velocity Mode = 01*/
			writeNetworkDictCallBack(d, nodeId, 0x6060, 0x00, 1, 0,
					&Velocity_mode, CheckSDOAndContinue, 0);
		}
			break;

		case 48: { /*Set Slave Motion profile type--梯形*/
			UNS16 Motion_Profile_type = 0x0000;
			writeNetworkDictCallBack(d, nodeId, 0x6086, 0x00, 2, 0,
					&Motion_Profile_type, CheckSDOAndContinue, 0);
		}
			break;

		case 49: { /* Set Slave profile Velocity 6081
		 旋转电机转速=0.5转/s =655360 counts/s =0x000A0000*/
			writeNetworkDictCallBack(d, nodeId, 0x6081, 0x00, 4, 0,
					&WalkingSpeed, CheckSDOAndContinue, 0);
		}
			break;

		case 50: { /*Set Slave Acceleration
		 行走轮=0.4 m/s^2;
		 电机=20861 counts/s^2 = 0x0000517D*/
			writeNetworkDictCallBack(d, nodeId, 0x6083, 0x00, 4, 0,
					&straight_acceleration, CheckSDOAndContinue, 0);
		}
			break;

		case 51: { /*Set Slave Deceleration;与加速度数值相同*/
			writeNetworkDictCallBack(d, nodeId, 0x6084, 0x00, 4, 0,
					&straight_deceleration, CheckSDOAndContinue, 0);
		}
			break;

		case 52: { /*Set homing position 00：Home is Current Position*/
			UNS8 home_current_position = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x6098, 0x00, 1, 0,
					&home_current_position, CheckSDOAndContinue, 0);
		}
			break;

		case 53: { /*0x605D Halt option 02=using quick stop ramp,*/
			writeNetworkDictCallBack(d, nodeId, 0x605D, 0x00, 2, 0, &HaltOption,
					CheckSDOAndContinue, 0);
		}
			break;


		case 54: { /*0x6085 quick stop deceleration,*/
			writeNetworkDictCallBack(d, nodeId, 0x6085, 0x00, 4, 0, &QuickStopDeceleration,
					CheckSDOAndContinue, 0);
		}
			break;

		case 55: { /*使能电机 010F*/
			UNS16 enable = 0x016F;
//			UNS16 enable = 0x0167;//断使能
			writeNetworkDictCallBack(d, nodeId, 0x6040, 0x00, 2, 0, &enable,
					CheckSDOAndContinue, 0);
		}
			break;

			/**************************设置从站心跳报文**************************/

		case 56: /*Third step: configure Slave Heart-beat interval*/
			writeNetworkDictCallBack(d, /*CO_Data* d*/
			nodeId, /*UNS8 nodeId*/
			0x1017, /*UNS16 index*/
			0x00, /*UNS16 index*/
			2, /*UNS8 count*/
			0, /*UNS8 dataType*/
			&heartdata,/*data must be a index*/
			CheckSDOAndContinue, /*SDOCallback_t Callback*/
			0); /* use block mode */
			break;


			/*****作为主节点发送SYNC  5ms 0x1006 00, 0x00001388==5000ns****/

		case 57: { //设置SYNC=5ms
			UNS32 SYNC1 = 0x00001388;
			writeNetworkDictCallBack(d, nodeId, 0x1006, 0x00, 4, 0,
					&SYNC1, CheckSDOAndContinue, 0);
		}
			break;

		case 58: { //激活SYNC 0x1005 00, 0x40000080
			UNS32 enableSYNC = 0x40000080;
			writeNetworkDictCallBack(d, nodeId, 0x1005, 0x00, 4, 0,
					&enableSYNC, CheckSDOAndContinue, 0);
		}
			break;
			/*************************设置电机运行参数**************************/

			/****************************** 启动从节点 *******************************/
		case 59: {
			/* Put the master in operational mode */
			setState(CANOpenShellOD_Data, Operational);

			/* Ask slave node to go in operational mode */
			StartNode(nodeId);

			/* reset counter in case the slave boot-up again */
//			init_step2 = 0;//不需要置0，避免电机重新上电从新配置。当电机重新上电此时init_step1==35，进入配置后++init_step1 ==36，并没有case36则跳过。
			num_slavebootup++;
			break;

		}

		}
	}

	if (nodeId == 0x03) {
		switch (++init_step3) {

		/************************** 配置TPDO **************************************/
		/* 运行模式(Mode of Operation Display 6061)	== 280 + nodeID 异步FF */
		case 1: { /*disable Slave's TPDO2 */
			UNS32 TPDO2_COBId = 0x80000280 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x01, 4, 0,
					&TPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 2: { /*Set Slave's TPDO 2 to be transmitted on FF*/
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 3: { /*Reset Slave's RPDO 2 Mapping number */
			UNS8 TPDO2_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x00, 1, 0,
					&TPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 4: { /*Maping Slave's TPDO 2 with mode_of_operation_Display in 6061 */
			UNS32 TPDO2_mode_of_operation_Display = 0x60610008;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x01, 4, 0,
					&TPDO2_mode_of_operation_Display, CheckSDOAndContinue, 0);
		}
			break;

		case 5: { /*Set Slave's RPDO 3 Mapping number */
			UNS8 TPDO2_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x00, 1, 0,
					&TPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 6: { /*Re-Ensable Slave's TPDO*/
			UNS32 TPDO2_COBId = 0x00000280 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x01, 4, 0,
					&TPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 当前速度+位置(Actual Velocity 606C,Position Acture value 6064) == 380 + nodeID 同步01 */
		case 7: { /*disable Slave's TPDO3 */
			UNS32 TPDO3_COBId = 0x80000380 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x01, 4, 0,
					&TPDO3_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 8: { /*Set Slave's TPDO 3 to be transmitted on 01*/
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x02, 1, 0,
					&Transmission_TPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 9: { /*Reset Slave's RPDO 3 Mapping number */
			UNS8 TPDO3_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x00, 1, 0,
					&TPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 10: { /*Maping Slave's TPDO 3 with speed in 606C */
			UNS32 TPDO3_MapSpeed = 0x606C0020;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x01, 4, 0,
					&TPDO3_MapSpeed, CheckSDOAndContinue, 0);
		}
			break;

		case 11: { /*Maping Slave's TPDO 3 with Position in 6064 */
			UNS32 TPDO3_Mapposition = 0x60640020;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x02, 4, 0,
					&TPDO3_Mapposition, CheckSDOAndContinue, 0);
		}
			break;

		case 12: { /*Set Slave's RPDO 3 Mapping number */
			UNS8 TPDO3_MapNum = 0x02;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x00, 1, 0,
					&TPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 13: { /*Re-Ensable Slave's TPDO*/
			UNS32 TPDO3_COBId = 0x00000380 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x01, 4, 0,
					&TPDO3_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 禁用默认TPDO4（Actural Velocity 606C）480+nodeID 1803-1A03；避免发送无畏信息 */
		case 14: { /*disable Slave's TPDO4 */
			UNS32 TPDO4_COBId = 0x80000480 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1803, 0x01, 4, 0,
					&TPDO4_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/************************** 配置RPDO **************************************/

			/* 控制字  (control words 6040)		== 210	ff */
		case 15: { /*set Slave's RPDO 1 cobid to 210*/
			UNS32 RPDO1_COBId = 0x00000210;
			writeNetworkDictCallBack(d, nodeId, 0x1400, 0x01, 4, 0,
					&RPDO1_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 16: { /*Set Slave's RPDO 1 to be transmitted on ff!!! */
			writeNetworkDictCallBack(d, nodeId, 0x1400, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

			/* 运行模式(Mode of Operation 6060)	== 310	同步00 */
		case 17: { /*disable Slave's RPDO3 */
			UNS32 RPDO2_COBId = 0x80000310;
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x01, 4, 0,
					&RPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 18: { /*Set Slave's RPDO 2 to be transmitted on 00*/
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x02, 1, 0,
					&Transmission_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 19: { /*Reset Slave's RPDO 2 Mapping number */
			UNS8 RPDO2_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x00, 1, 0,
					&RPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 20: { /*Maping Slave's RPDO 2 with mode_of_operation in 6060 */
			UNS32 RPDO3_Map_mode_of_operation = 0x60600008;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x01, 4, 0,
					&RPDO3_Map_mode_of_operation, CheckSDOAndContinue, 0);
		}
			break;

		case 21: { /*Set Slave's RPDO 2 Mapping number */
			UNS8 RPDO2_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x00, 1, 0,
					&RPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 22: { /*Re-Ensable Slave's RPDO2*/
			UNS32 RPDO2_COBId = 0x00000310;
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x01, 4, 0,
					&RPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 速度模式:目标速度(target velocity 60FF)==	410 + nodeID 同步00 */
		case 23: { /*disable Slave's RPDO3 */
			UNS32 RPDO3_COBId = 0x80000410 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x01, 4, 0,
					&RPDO3_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 24: { /*Set Slave's RPDO 3 to be transmitted on 00*/
			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x02, 1, 0,
					&Transmission_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 25: { /*Reset Slave's RPDO 3 Mapping number */
			UNS8 RPDO3_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x00, 1, 0,
					&RPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 26: { /*Maping Slave's RPDO 3 with Target_Velocity in 60FF */
			UNS32 RPDO3_MapTarget_Velocity = 0x60FF0020;
			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x01, 4, 0,
					&RPDO3_MapTarget_Velocity, CheckSDOAndContinue, 0);
		}
			break;

		case 27: { /*Set Slave's RPDO 3 Mapping number */
			UNS8 RPDO3_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x00, 1, 0,
					&RPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 28: { /*Re-Ensable Slave's RPDO*/
			UNS32 RPDO3_COBId = 0x00000410 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x01, 4, 0,
					&RPDO3_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 加速度，减速度(Profile Acceleraion 6083,Profile Deceleration 6084)	==	510 + nodeID 同步00 */
		case 29: { /*disable Slave's RPDO4 */
			UNS32 RPDO4_COBId = 0x80000510;
			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x01, 4, 0,
					&RPDO4_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 30: { /*Set Slave's RPDO 4 to be transmitted on ff*/
			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 31: { /*Reset Slave's RPDO 4 Mapping number */
			UNS8 RPDO4_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x00, 1, 0,
					&RPDO4_MapNum, CheckSDOAndContinue, 0);
		}
			break;

//		case 31: { /*Maping Slave's RPDO 4 with profile acceleration */
//			UNS32 RPDO4_MapProfile_acceleration = 0x60830020;
//			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x01, 4, 0,
//					&RPDO4_MapProfile_acceleration, CheckSDOAndContinue, 0);
//		}
//			break;

		case 32: { /*Maping Slave's RPDO 4 with profile deceleration */
			UNS32 RPDO4_MapProfile_deceleration = 0x60840020;
			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x01, 4, 0,
					&RPDO4_MapProfile_deceleration, CheckSDOAndContinue, 0);
		}
			break;
		case 33: { /*Set Slave's RPDO 4 Mapping number */
			UNS8 RPDO4_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x00, 1, 0,
					&RPDO4_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 34: { /*Re-Ensable Slave's RPDO*/
			UNS32 RPDO4_COBId = 0x00000510;
			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x01, 4, 0,
					&RPDO4_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 位置模式:目标位置(target position 607A),速度(Profile Velocity 6081) == 610 + nodeID 异步FF */
		case 35: { /*disable Slave's RPDO5*/
			UNS32 RPDO5_COBId = 0x80000610 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x01, 4, 0,
					&RPDO5_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 36: { /*Set Slave's RPDO 5 to be transmitted on FF*/
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 37: { /*Reset Slave's RPDO 5 Mapping number */
			UNS8 RPDO5_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x00, 1, 0,
					&RPDO5_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 38: { /*Maping Slave's RPDO 5 with Target_position in 607A */
			UNS32 RPDO5_MapTarget_position = 0x607A0020;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x01, 4, 0,
					&RPDO5_MapTarget_position, CheckSDOAndContinue, 0);
		}
			break;

		case 39: { /*Set Slave's RPDO 5 Mapping number */
			UNS8 RPDO4_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x00, 1, 0,
					&RPDO4_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 40: { /*Re-Ensable Slave's RPDO5*/
			UNS32 RPDO5_COBId = 0x00000610 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x01, 4, 0,
					&RPDO5_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/*** 位置模式:速度(Profile Velocity 6081) == 710  异步FF ***/
		case 41: { /*disable Slave's RPDO 6*/
			UNS32 RPDO6_COBId = 0x80000710;
			writeNetworkDictCallBack(d, nodeId, 0x1405, 0x01, 4, 0,
					&RPDO6_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 42: { /*Set Slave's RPDO 6 to be transmitted on FF*/
			writeNetworkDictCallBack(d, nodeId, 0x1405, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 43: { /*Reset Slave's RPDO 6 Mapping number */
			UNS8 RPDO6_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1605, 0x00, 1, 0,
					&RPDO6_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 44: { /*Maping Slave's RPDO 6 with profile_velocity6081 */
			UNS32 RPDO6_MapProfile_velocity = 0x60810020;
			writeNetworkDictCallBack(d, nodeId, 0x1605, 0x01, 4, 0,
					&RPDO6_MapProfile_velocity, CheckSDOAndContinue, 0);
		}
			break;

		case 45: { /*Set Slave's RPDO 5 Mapping number */
			UNS8 RPDO4_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1605, 0x00, 1, 0,
					&RPDO4_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 46: { /*Re-Ensable Slave's RPDO6*/
			UNS32 RPDO6_COBId = 0x00000710;
			writeNetworkDictCallBack(d, nodeId, 0x1405, 0x01, 4, 0,
					&RPDO6_COBId, CheckSDOAndContinue, 0);
		}
			break;
			/*************************设置电机运行参数**************************/

			//1.运行模式 ---速度模式
			//2.加速度减速度
			//3.control word=010F；
			//4.homeing =当前位置
		case 47: { /*Set Slave into Velocity Mode = 01*/
			writeNetworkDictCallBack(d, nodeId, 0x6060, 0x00, 1, 0,
					&Velocity_mode, CheckSDOAndContinue, 0);
		}
			break;

		case 48: { /*Set Slave Motion profile type--梯形*/
			UNS16 Motion_Profile_type = 0x0000;
			writeNetworkDictCallBack(d, nodeId, 0x6086, 0x00, 2, 0,
					&Motion_Profile_type, CheckSDOAndContinue, 0);
		}
			break;

		case 49: { /* Set Slave profile Velocity 6081
		 旋转电机转速=0.5转/s =655360 counts/s =0x000A0000*/
			writeNetworkDictCallBack(d, nodeId, 0x6081, 0x00, 4, 0,
					&WalkingSpeed, CheckSDOAndContinue, 0);
		}
			break;

		case 50: { /*Set Slave Acceleration
		 行走轮=0.4 m/s^2;
		 电机=20861 counts/s^2 = 0x0000517D*/
	//		UNS32 straight_acceleration = 0x0000517D;
			writeNetworkDictCallBack(d, nodeId, 0x6083, 0x00, 4, 0,
					&straight_acceleration, CheckSDOAndContinue, 0);
		}
			break;

		case 51: { /*Set Slave Deceleration;与加速度数值相同*/
			writeNetworkDictCallBack(d, nodeId, 0x6084, 0x00, 4, 0,
					&straight_deceleration, CheckSDOAndContinue, 0);
		}
			break;

		case 52: { /*Set homing position 00：Home is Current Position*/
			UNS8 home_current_position = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x6098, 0x00, 1, 0,
					&home_current_position, CheckSDOAndContinue, 0);
		}
			break;

		case 53: { /*0x605D Halt option 02=using quick stop ramp,*/
			writeNetworkDictCallBack(d, nodeId, 0x605D, 0x00, 2, 0, &HaltOption,
					CheckSDOAndContinue, 0);
		}
			break;


		case 54: { /*0x6085 quick stop deceleration,*/
			writeNetworkDictCallBack(d, nodeId, 0x6085, 0x00, 4, 0, &QuickStopDeceleration,
					CheckSDOAndContinue, 0);
		}
			break;

		case 55: { /*使能电机 010F*/
			UNS16 enable = 0x016F;
//			UNS16 enable = 0x0167;//断使能
			writeNetworkDictCallBack(d, nodeId, 0x6040, 0x00, 2, 0, &enable,
					CheckSDOAndContinue, 0);
		}
			break;

			/**************************设置从站心跳报文**************************/

		case 56: /*Third step: configure Slave Heart-beat interval*/
			writeNetworkDictCallBack(d, /*CO_Data* d*/
			nodeId, /*UNS8 nodeId*/
			0x1017, /*UNS16 index*/
			0x00, /*UNS16 index*/
			2, /*UNS8 count*/
			0, /*UNS8 dataType*/
			&heartdata,/*data must be a index*/
			CheckSDOAndContinue, /*SDOCallback_t Callback*/
			0); /* use block mode */
			break;

			/****************************** 启动从节点 *******************************/
		case 57: {
			/* Put the master in operational mode */
			setState(CANOpenShellOD_Data, Operational);

			/* Ask slave node to go in operational mode */
			StartNode(nodeId);

			/* reset counter in case the slave boot-up again */
//			init_step3 = 0;//不需要置0，避免电机重新上电从新配置。当电机重新上电此时init_step1==35，进入配置后++init_step1 ==36，并没有case36则跳过。
			num_slavebootup++;
			break;

		}

		}
	}

	if (nodeId == 0x04) {
		switch (++init_step4) {

		/************************** 配置TPDO **************************************/
		/* 运行模式(Mode of Operation Display 6061)	== 280 + nodeID 异步FF */
		case 1: { /*disable Slave's TPDO2 */
			UNS32 TPDO2_COBId = 0x80000280 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x01, 4, 0,
					&TPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 2: { /*Set Slave's TPDO 2 to be transmitted on FF*/
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 3: { /*Reset Slave's RPDO 2 Mapping number */
			UNS8 TPDO2_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x00, 1, 0,
					&TPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 4: { /*Maping Slave's TPDO 2 with mode_of_operation_Display in 6061 */
			UNS32 TPDO2_mode_of_operation_Display = 0x60610008;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x01, 4, 0,
					&TPDO2_mode_of_operation_Display, CheckSDOAndContinue, 0);
		}
			break;

		case 5: { /*Set Slave's RPDO 3 Mapping number */
			UNS8 TPDO2_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x00, 1, 0,
					&TPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 6: { /*Re-Ensable Slave's TPDO*/
			UNS32 TPDO2_COBId = 0x00000280 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x01, 4, 0,
					&TPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 当前速度+位置(Actual Velocity 606C,Position Acture value 6064) == 380 + nodeID 同步01 */
		case 7: { /*disable Slave's TPDO3 */
			UNS32 TPDO3_COBId = 0x80000380 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x01, 4, 0,
					&TPDO3_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 8: { /*Set Slave's TPDO 3 to be transmitted on 01*/
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x02, 1, 0,
					&Transmission_TPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 9: { /*Reset Slave's RPDO 3 Mapping number */
			UNS8 TPDO3_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x00, 1, 0,
					&TPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 10: { /*Maping Slave's TPDO 3 with speed in 606C */
			UNS32 TPDO3_MapSpeed = 0x606C0020;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x01, 4, 0,
					&TPDO3_MapSpeed, CheckSDOAndContinue, 0);
		}
			break;

		case 11: { /*Maping Slave's TPDO 3 with Position in 6064 */
			UNS32 TPDO3_Mapposition = 0x60640020;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x02, 4, 0,
					&TPDO3_Mapposition, CheckSDOAndContinue, 0);
		}
			break;

		case 12: { /*Set Slave's RPDO 3 Mapping number */
			UNS8 TPDO3_MapNum = 0x02;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x00, 1, 0,
					&TPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 13: { /*Re-Ensable Slave's TPDO*/
			UNS32 TPDO3_COBId = 0x00000380 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x01, 4, 0,
					&TPDO3_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 禁用默认TPDO4（Actural Velocity 606C）480+nodeID 1803-1A03；避免发送无畏信息 */
		case 14: { /*disable Slave's TPDO4 */
			UNS32 TPDO4_COBId = 0x80000480 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1803, 0x01, 4, 0,
					&TPDO4_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/************************** 配置RPDO **************************************/

			/* 控制字  (control words 6040)		== 210	FF */
		case 15: { /*set Slave's RPDO 1 cobid to 210*/
			UNS32 RPDO1_COBId = 0x00000210;
			writeNetworkDictCallBack(d, nodeId, 0x1400, 0x01, 4, 0,
					&RPDO1_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 16: { /*Set Slave's RPDO 1 to be transmitted onFF!!! */
			writeNetworkDictCallBack(d, nodeId, 0x1400, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

			/* 运行模式(Mode of Operation 6060)	== 310	同步00 */
		case 17: { /*disable Slave's RPDO3 */
			UNS32 RPDO2_COBId = 0x80000310;
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x01, 4, 0,
					&RPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 18: { /*Set Slave's RPDO 2 to be transmitted on 00*/
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x02, 1, 0,
					&Transmission_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 19: { /*Reset Slave's RPDO 2 Mapping number */
			UNS8 RPDO2_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x00, 1, 0,
					&RPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 20: { /*Maping Slave's RPDO 2 with mode_of_operation in 6060 */
			UNS32 RPDO3_Map_mode_of_operation = 0x60600008;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x01, 4, 0,
					&RPDO3_Map_mode_of_operation, CheckSDOAndContinue, 0);
		}
			break;

		case 21: { /*Set Slave's RPDO 2 Mapping number */
			UNS8 RPDO2_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x00, 1, 0,
					&RPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 22: { /*Re-Ensable Slave's RPDO2*/
			UNS32 RPDO2_COBId = 0x00000310;
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x01, 4, 0,
					&RPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 速度模式:目标速度(target velocity 60FF)==	410 + nodeID 同步00 */
		case 23: { /*disable Slave's RPDO3 */
			UNS32 RPDO3_COBId = 0x80000410 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x01, 4, 0,
					&RPDO3_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 24: { /*Set Slave's RPDO 3 to be transmitted on 00*/
			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x02, 1, 0,
					&Transmission_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 25: { /*Reset Slave's RPDO 3 Mapping number */
			UNS8 RPDO3_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x00, 1, 0,
					&RPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 26: { /*Maping Slave's RPDO 3 with Target_Velocity in 60FF */
			UNS32 RPDO3_MapTarget_Velocity = 0x60FF0020;
			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x01, 4, 0,
					&RPDO3_MapTarget_Velocity, CheckSDOAndContinue, 0);
		}
			break;

		case 27: { /*Set Slave's RPDO 3 Mapping number */
			UNS8 RPDO3_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x00, 1, 0,
					&RPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 28: { /*Re-Ensable Slave's RPDO*/
			UNS32 RPDO3_COBId = 0x00000410 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x01, 4, 0,
					&RPDO3_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 加速度，减速度(Profile Acceleraion 6083,Profile Deceleration 6084)	==	510 + nodeID 同步00 */
		case 29: { /*disable Slave's RPDO4 */
			UNS32 RPDO4_COBId = 0x80000510;
			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x01, 4, 0,
					&RPDO4_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 30: { /*Set Slave's RPDO 4 to be transmitted on ff*/
			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 31: { /*Reset Slave's RPDO 4 Mapping number */
			UNS8 RPDO4_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x00, 1, 0,
					&RPDO4_MapNum, CheckSDOAndContinue, 0);
		}
			break;

//		case 31: { /*Maping Slave's RPDO 4 with profile acceleration */
//			UNS32 RPDO4_MapProfile_acceleration = 0x60830020;
//			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x01, 4, 0,
//					&RPDO4_MapProfile_acceleration, CheckSDOAndContinue, 0);
//		}
//			break;

		case 32: { /*Maping Slave's RPDO 4 with profile deceleration */
			UNS32 RPDO4_MapProfile_deceleration = 0x60840020;
			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x01, 4, 0,
					&RPDO4_MapProfile_deceleration, CheckSDOAndContinue, 0);
		}
			break;

		case 33: { /*Set Slave's RPDO 4 Mapping number */
			UNS8 RPDO4_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x00, 1, 0,
					&RPDO4_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 34: { /*Re-Ensable Slave's RPDO*/
			UNS32 RPDO4_COBId = 0x00000510;
			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x01, 4, 0,
					&RPDO4_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 位置模式:目标位置(target position 607A),速度(Profile Velocity 6081) == 610 + nodeID 异步FF */
		case 35: { /*disable Slave's RPDO5*/
			UNS32 RPDO5_COBId = 0x80000610 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x01, 4, 0,
					&RPDO5_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 36: { /*Set Slave's RPDO 5 to be transmitted on FF*/
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 37: { /*Reset Slave's RPDO 5 Mapping number */
			UNS8 RPDO5_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x00, 1, 0,
					&RPDO5_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 38: { /*Maping Slave's RPDO 5 with Target_position in 607A */
			UNS32 RPDO5_MapTarget_position = 0x607A0020;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x01, 4, 0,
					&RPDO5_MapTarget_position, CheckSDOAndContinue, 0);
		}
			break;

		case 39: { /*Set Slave's RPDO 5 Mapping number */
			UNS8 RPDO4_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x00, 1, 0,
					&RPDO4_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 40: { /*Re-Ensable Slave's RPDO5*/
			UNS32 RPDO5_COBId = 0x00000610 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x01, 4, 0,
					&RPDO5_COBId, CheckSDOAndContinue, 0);
		}
			break;
			/*** 位置模式:速度(Profile Velocity 6081) == 710  异步FF ***/
		case 41: { /*disable Slave's RPDO 6*/
			UNS32 RPDO6_COBId = 0x80000710;
			writeNetworkDictCallBack(d, nodeId, 0x1405, 0x01, 4, 0,
					&RPDO6_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 42: { /*Set Slave's RPDO 6 to be transmitted on FF*/
			writeNetworkDictCallBack(d, nodeId, 0x1405, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 43: { /*Reset Slave's RPDO 6 Mapping number */
			UNS8 RPDO6_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1605, 0x00, 1, 0,
					&RPDO6_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 44: { /*Maping Slave's RPDO 6 with profile_velocity6081 */
			UNS32 RPDO6_MapProfile_velocity = 0x60810020;
			writeNetworkDictCallBack(d, nodeId, 0x1605, 0x01, 4, 0,
					&RPDO6_MapProfile_velocity, CheckSDOAndContinue, 0);
		}
			break;

		case 45: { /*Set Slave's RPDO 5 Mapping number */
			UNS8 RPDO4_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1605, 0x00, 1, 0,
					&RPDO4_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 46: { /*Re-Ensable Slave's RPDO6*/
			UNS32 RPDO6_COBId = 0x00000710;
			writeNetworkDictCallBack(d, nodeId, 0x1405, 0x01, 4, 0,
					&RPDO6_COBId, CheckSDOAndContinue, 0);
		}
			break;
			/*************************设置电机运行参数**************************/

			//1.运行模式 ---速度模式
			//2.加速度减速度
			//3.control word=010F；
			//4.homeing =当前位置
		case 47: { /*Set Slave into Velocity Mode = 01*/
			writeNetworkDictCallBack(d, nodeId, 0x6060, 0x00, 1, 0,
					&Velocity_mode, CheckSDOAndContinue, 0);
		}
			break;

		case 48: { /*Set Slave Motion profile type--梯形*/
			UNS16 Motion_Profile_type = 0x0000;
			writeNetworkDictCallBack(d, nodeId, 0x6086, 0x00, 2, 0,
					&Motion_Profile_type, CheckSDOAndContinue, 0);
		}
			break;

		case 49: { /* Set Slave profile Velocity 6081
		 旋转电机转速=0.5转/s =655360 counts/s =0x000A0000*/
			writeNetworkDictCallBack(d, nodeId, 0x6081, 0x00, 4, 0,
					&WalkingSpeed, CheckSDOAndContinue, 0);
		}
			break;

		case 50: { /*Set Slave Acceleration
		 行走轮=0.4 m/s^2;
		 电机=20861 counts/s^2 = 0x0000517D*/
//			UNS32 straight_acceleration = 0x0000517D;
			writeNetworkDictCallBack(d, nodeId, 0x6083, 0x00, 4, 0,
					&straight_acceleration, CheckSDOAndContinue, 0);
		}
			break;

		case 51: { /*Set Slave Deceleration;与加速度数值相同*/
			writeNetworkDictCallBack(d, nodeId, 0x6084, 0x00, 4, 0,
					&straight_deceleration, CheckSDOAndContinue, 0);
		}
			break;

		case 52: { /*Set homing position 00：Home is Current Position*/
			UNS8 home_current_position = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x6098, 0x00, 1, 0,
					&home_current_position, CheckSDOAndContinue, 0);
		}
			break;

		case 53: { /*0x605D Halt option 02=using quick stop ramp,*/
			writeNetworkDictCallBack(d, nodeId, 0x605D, 0x00, 2, 0, &HaltOption,
					CheckSDOAndContinue, 0);
		}
			break;


		case 54: { /*0x6085 quick stop deceleration,*/
			writeNetworkDictCallBack(d, nodeId, 0x6085, 0x00, 4, 0, &QuickStopDeceleration,
					CheckSDOAndContinue, 0);
		}
			break;

		case 55: { /*使能电机 010F*/
			UNS16 enable = 0x016F;
//			UNS16 enable = 0x0167;//断使能
			writeNetworkDictCallBack(d, nodeId, 0x6040, 0x00, 2, 0, &enable,
					CheckSDOAndContinue, 0);
		}
			break;

			/**************************设置从站心跳报文**************************/

		case 56: /*Third step: configure Slave Heart-beat interval*/
			writeNetworkDictCallBack(d, /*CO_Data* d*/
			nodeId, /*UNS8 nodeId*/
			0x1017, /*UNS16 index*/
			0x00, /*UNS16 index*/
			2, /*UNS8 count*/
			0, /*UNS8 dataType*/
			&heartdata,/*data must be a index*/
			CheckSDOAndContinue, /*SDOCallback_t Callback*/
			0); /* use block mode */
			break;

			/*************************设置电机运行参数**************************/

			/****************************** 启动从节点 *******************************/
		case 57: {
			/* Put the master in operational mode */
			setState(CANOpenShellOD_Data, Operational);

			/* Ask slave node to go in operational mode */
			StartNode(nodeId);

			/* reset counter in case the slave boot-up again */
//			init_step4 = 0;//不需要置0，避免电机重新上电从新配置。当电机重新上电此时init_step1==35，进入配置后++init_step1 ==36，并没有case36则跳过。
			num_slavebootup++;
			break;

		}

		}
	}
	/*第二组：转向电机 nodeID:05-08 位置模式
	 *
	 *TPDO:状态PDO 同步：01-240  异步：254-255  （十进制）
	 *		状态字  (status words 6041)				== 180 + nodeID 异步FF
	 *		运行模式(Mode of Operation Display 6061)	== 280 + nodeID 异步FF
	 *		当前速度+位置(Actual Velocity 606C,Position Acture value 6064) == 380 + nodeID 同步01
	 *		禁用默认TPDO4（Actural Velocity 606C）480+nodeID 1803-1A03；避免发送无畏信息
	 *
	 *RPDO:控制PDO 同步：00-240  异步：254-255  （十进制）
	 *		控制字  (control words 6040)		== 220	同步00
	 *		运行模式(Mode of Operation 6060)	== 320	同步00
	 *
	 //	 *		**速度模式:目标速度(target velocity 60FF)	==	420 同步00
	 //	 *		速度模式:目标速度(target velocity 60FF)	==	420 + nodeID 同步00
	 //	 *
	 //	 *		**加速度，减速度(Profile Acceleraion 6083,Profile Deceleration 6084)	==	520 异步FF
	 //	 *		加速度，减速度(Profile Acceleraion 6083,Profile Deceleration 6084)	==	520 + nodeID 异步FF
	 *
	 *		**位置模式:目标位置(target position 607A)	== 620 异步FF
	 *		位置模式:目标位置(target position 607A)	== 620 + nodeID 异步FF
	 *
	 *
	 *		 */
	if (nodeId == 0x05) {
		switch (++init_step5) {
		/************************** 配置TPDO **************************************/

		/* 运行模式(Mode of Operation Display 6061)	== 280 + nodeID 异步FF */
		case 1: { /*disable Slave's TPDO2 */
			UNS32 TPDO2_COBId = 0x80000280 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x01, 4, 0,
					&TPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 2: { /*Set Slave's TPDO 2 to be transmitted on FF*/
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 3: { /*Reset Slave's RPDO 2 Mapping number */
			UNS8 TPDO2_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x00, 1, 0,
					&TPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 4: { /*Maping Slave's TPDO 2 with mode_of_operation_Display in 6061 */
			UNS32 TPDO2_mode_of_operation_Display = 0x60610008;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x01, 4, 0,
					&TPDO2_mode_of_operation_Display, CheckSDOAndContinue, 0);
		}
			break;

		case 5: { /*Set Slave's RPDO 3 Mapping number */
			UNS8 TPDO2_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x00, 1, 0,
					&TPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 6: { /*Re-Ensable Slave's TPDO*/
			UNS32 TPDO2_COBId = 0x00000280 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x01, 4, 0,
					&TPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 当前速度+位置(Actual Velocity 606C,Position Acture value 6064) == 380 + nodeID 同步01 */
		case 7: { /*disable Slave's TPDO3 */
			UNS32 TPDO3_COBId = 0x80000380 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x01, 4, 0,
					&TPDO3_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 8: { /*Set Slave's TPDO 3 to be transmitted on 01*/
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x02, 1, 0,
					&Transmission_TPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 9: { /*Reset Slave's RPDO 3 Mapping number */
			UNS8 TPDO3_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x00, 1, 0,
					&TPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 10: { /*Maping Slave's TPDO 3 with speed in 606C */
			UNS32 TPDO3_MapSpeed = 0x606C0020;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x01, 4, 0,
					&TPDO3_MapSpeed, CheckSDOAndContinue, 0);
		}
			break;

		case 11: { /*Maping Slave's TPDO 3 with Position in 6064 */
			UNS32 TPDO3_Mapposition = 0x60640020;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x02, 4, 0,
					&TPDO3_Mapposition, CheckSDOAndContinue, 0);
		}
			break;

		case 12: { /*Set Slave's RPDO 3 Mapping number */
			UNS8 TPDO3_MapNum = 0x02;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x00, 1, 0,
					&TPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 13: { /*Re-Ensable Slave's TPDO*/
			UNS32 RPDO3_COBId = 0x00000380 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x01, 4, 0,
					&RPDO3_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 禁用默认TPDO4（Actural Velocity 606C）480+nodeID 1803-1A03；避免发送无畏信息 */
		case 14: { /*disable Slave's TPDO4 */
			UNS32 TPDO4_COBId = 0x80000480 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1803, 0x01, 4, 0,
					&TPDO4_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/************************** 配置RPDO **************************************/

			/* 控制字  (control words 6040)	== 220	同步00--改为FF*/
		case 15: { /*set Slave's RPDO 1 cobid to 220*/
			UNS32 RPDO2_control_word = 0x00000220;
			writeNetworkDictCallBack(d, nodeId, 0x1400, 0x01, 4, 0,
					&RPDO2_control_word, CheckSDOAndContinue, 0);
		}
			break;

		case 16: { /*Set Slave's RPDO 1 to be transmitted on 00--改为FF!!! */
			writeNetworkDictCallBack(d, nodeId, 0x1400, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

			/* 运行模式(Mode of Operation 6060)	== 320	同步00 */
		case 17: { /*disable Slave's RPDO2 */
			UNS32 RPDO2_COBId = 0x80000320;
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x01, 4, 0,
					&RPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 18: { /*Set Slave's RPDO 2 to be transmitted on 00*/
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x02, 1, 0,
					&Transmission_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 19: { /*Reset Slave's RPDO 2 Mapping number */
			UNS8 RPDO2_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x00, 1, 0,
					&RPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 20: { /*Maping Slave's RPDO 2 with mode_of_operation in 6060 */
			UNS32 RPDO3_Map_mode_of_operation = 0x60600008;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x01, 4, 0,
					&RPDO3_Map_mode_of_operation, CheckSDOAndContinue, 0);
		}
			break;

		case 21: { /*Set Slave's RPDO 2 Mapping number */
			UNS8 RPDO2_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x00, 1, 0,
					&RPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 22: { /*Re-Ensable Slave's RPDO2*/
			UNS32 RPDO2_COBId = 0x00000320;
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x01, 4, 0,
					&RPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

//		/* 速度模式:目标速度(target velocity 60FF)	==	420 + nodeID 同步00 */
//		case 22: { /*disable Slave's RPDO3 */
//			UNS32 RPDO3_COBId = 0x80000420 + nodeId;
//			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x01, 4, 0,
//					&RPDO3_COBId, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 23: { /*Set Slave's RPDO 3 to be transmitted on 00*/
//			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x02, 1, 0,
//					&Transmission_RPDO, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 24: { /*Reset Slave's RPDO 3 Mapping number */
//			UNS8 RPDO3_MapNum = 0x00;
//			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x00, 1, 0,
//					&RPDO3_MapNum, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 25: { /*Maping Slave's RPDO 3 with Target_Velocity in 60FF */
//			UNS32 RPDO3_MapTarget_Velocity = 0x60FF0020;
//			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x01, 4, 0,
//					&RPDO3_MapTarget_Velocity, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 26: { /*Set Slave's RPDO 3 Mapping number */
//			UNS8 RPDO3_MapNum = 0x01;
//			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x00, 1, 0,
//					&RPDO3_MapNum, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 27: { /*Re-Ensable Slave's RPDO*/
//			UNS32 RPDO3_COBId = 0x00000420 + nodeId;
//			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x01, 4, 0,
//					&RPDO3_COBId, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		/* 加速度，减速度(Profile Acceleraion 6083,Profile Deceleration 6084)	==	520 + nodeID 异步FF */
//		case 28: { /*disable Slave's RPDO4 */
//			UNS32 RPDO4_COBId = 0x80000520 + nodeId;
//			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x01, 4, 0,
//					&RPDO4_COBId, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 29: { /*Set Slave's RPDO 4 to be transmitted on FF*/
//			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x02, 1, 0,
//					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 30: { /*Reset Slave's RPDO 4 Mapping number */
//			UNS8 RPDO4_MapNum = 0x00;
//			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x00, 1, 0,
//					&RPDO4_MapNum, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 31: { /*Maping Slave's RPDO 4 with profile acceleration */
//			UNS32 RPDO4_MapProfile_acceleration = 0x60830020;
//			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x01, 4, 0,
//					&RPDO4_MapProfile_acceleration, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 32: { /*Maping Slave's RPDO 4 with profile deceleration */
//			UNS32 RPDO4_MapProfile_deceleration = 0x60840020;
//			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x02, 4, 0,
//					&RPDO4_MapProfile_deceleration, CheckSDOAndContinue, 0);
//		}
//			break;
//		case 33: { /*Set Slave's RPDO 4 Mapping number */
//			UNS8 RPDO4_MapNum = 0x02;
//			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x00, 1, 0,
//					&RPDO4_MapNum, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 34: { /*Re-Ensable Slave's RPDO*/
//			UNS32 RPDO4_COBId = 0x00000520 + nodeId;
//			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x01, 4, 0,
//					&RPDO4_COBId, CheckSDOAndContinue, 0);
//		}
//			break;

			/* 位置模式:目标位置(target position 607A)== 620 + nodeID 异步FF */
		case 23: { /*disable Slave's RPDO5*/
			UNS32 RPDO5_COBId = 0x80000620 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x01, 4, 0,
					&RPDO5_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 24: { /*Set Slave's RPDO 5 to be transmitted on FF*/
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 25: { /*Reset Slave's RPDO 5 Mapping number */
			UNS8 RPDO5_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x00, 1, 0,
					&RPDO5_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 26: { /*Maping Slave's RPDO 5 with Target_position in 607A */
			UNS32 RPDO5_MapTarget_position = 0x607A0020;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x01, 4, 0,
					&RPDO5_MapTarget_position, CheckSDOAndContinue, 0);
		}
			break;

//		case 39: { /*Maping Slave's RPDO 5 with profile_velocity in 6081 */
//			UNS32 RPDO5_MapProfile_velocity = 0x60810020;
//			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x02, 4, 0,
//					&RPDO5_MapProfile_velocity, CheckSDOAndContinue, 0);
//		}
//			break;

		case 27: { /*Set Slave's RPDO 5 Mapping number */
			UNS8 RPDO4_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x00, 1, 0,
					&RPDO4_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 28: { /*Re-Ensable Slave's RPDO5*/
			UNS32 RPDO5_COBId = 0x00000620 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x01, 4, 0,
					&RPDO5_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/*************************设置电机运行参数**************************/

//			/*Homing mode parameter*/
//		case 42: { /*Set Slave into Homing Mode*/
//			eprintf("Master : set slave %2.2x into Velocity Mode\n", nodeId);
//			fflush(stdout);
//			UNS8 Homing_mode = 0x06;
//			writeNetworkDictCallBack(d, nodeId, 0x6060, 0x00, 1, 0,
//					&Homing_mode, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 43: { /*Set Homing_Speed_Fast 6099 01*/
//			/***Use if to allocate
//			 * specific move parameters for each rotation_motor
//			 */
//			UNS32 Homing_Speed_Fast = 0x00161a80;
//			writeNetworkDictCallBack(d, nodeId, 0x6099, 0x01, 4, 0,
//					&Homing_Speed_Fast, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 44: { /*Set Homing_Speed_Slow 6099 02*/
//			UNS32 Homing_Speed_Slow = 0x00061a80;
//			writeNetworkDictCallBack(d, nodeId, 0x6099, 0x02, 4, 0,
//					&Homing_Speed_Slow, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 45: { /*Set Homing_Acceleration 609A*/
//			UNS32 Homing_Acceleration = 0x0000FFA0;
//			writeNetworkDictCallBack(d, nodeId, 0x609A, 0x00, 4, 0,
//					&Homing_Acceleration, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 46: { /*Set homing_position_method H23:Home is Current Position; Move to New Zero 35*/
//			UNS8 homing_position_method = 0x23;
//			writeNetworkDictCallBack(d, nodeId, 0x6098, 0x00, 1, 0,
//					&homing_position_method, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 47: { /*Set homing_offset 607C*/
//			UNS32 Homing_offset = 0x000F86A0;
//			writeNetworkDictCallBack(d, nodeId, 0x607C, 0x00, 4, 0,
//					&Homing_offset, CheckSDOAndContinue, 0);
//		}
//			break;
			//设置位置模式
			//梯形
			//速度
			//加速度+减速度
			//是否需要设置HOMING?????
			//enable
			/*Position mode parameter*/
		case 29: { /*Set Slave into position Mode*/
			UNS8 position_mode = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x6060, 0x00, 1, 0,
					&position_mode, CheckSDOAndContinue, 0);
		}
			break;

		case 30: { /*Set Slave Motion profile type--梯形*/
			UNS16 Motion_Profile_type = 0x0000;
			writeNetworkDictCallBack(d, nodeId, 0x6086, 0x00, 2, 0,
					&Motion_Profile_type, CheckSDOAndContinue, 0);
		}
			break;

		case 31: { /* Set Slave profile Velocity 6081
		 旋转电机转速=0.5转/s =655360 counts/s =0x000A0000*/
			writeNetworkDictCallBack(d, nodeId, 0x6081, 0x00, 4, 0,
					&RotateVelocity, CheckSDOAndContinue, 0);
		}
			break;

		case 32: { /*Set Slave Acceleration 6083 = 6×旋转速度=0.5*6=3rps=0x0000999A*/
			writeNetworkDictCallBack(d, nodeId, 0x6083, 0x00, 4, 0, &RotateAcc,
					CheckSDOAndContinue, 0);
		}
			break;

		case 33: { /*Set Slave Deceleration 6084与加速度相同=6×旋转速度=0.5*6=3rps=0x0000999A*/
			writeNetworkDictCallBack(d, nodeId, 0x6084, 0x00, 4, 0, &RotateAcc,
					CheckSDOAndContinue, 0);
		}
			break;

			/*位置模式使用：绝对值（bit6=0）+ change immediate（bit5=1）
			 * 使能	:012F  halt/absolute/change immediate/bit4=0/
			 *启动 	:003F  	  absolute/change immediate/bit4=1
			 *启动后bit4 1-0	:002F 	  absolute/change immediate/bit4=0
			 *停止  :012F  halt/absolute/change immediate/bit4=0/
			 */
		case 34: {
			/* enable motor halt/absolute/change imeadiate/bit4=0/ */
			UNS16 enable = 0x012F;
//			UNS16 enable = 0x0127;//断使能
			writeNetworkDictCallBack(d, nodeId, 0x6040, 0x00, 2, 0, &enable,
					CheckSDOAndContinue, 0);
		}
			break;

			/**************************设置从站心跳报文**************************/

		case 35: /*Third step: configure Slave Heart-beat interval*/
			writeNetworkDictCallBack(d, /*CO_Data* d*/
			nodeId, /*UNS8 nodeId*/
			0x1017, /*UNS16 index*/
			0x00, /*UNS16 index*/
			2, /*UNS8 count*/
			0, /*UNS8 dataType*/
			&heartdata,/*data must be a index*/
			CheckSDOAndContinue, /*SDOCallback_t Callback*/
			0); /* use block mode */
			break;

			/****************************** 启动从节点 *******************************/
		case 36: {
			/* Put the master in operational mode */
			setState(CANOpenShellOD_Data, Operational);

			/* Ask slave node to go in operational mode */
			StartNode(nodeId);

			/* reset counter in case the slave boot-up again */
//			init_step5 = 0;//不需要置0，避免电机重新上电从新配置。当电机重新上电此时init_step1==36，进入配置后++init_step1 ==37，并没有case36则跳过。
			num_slavebootup++;
			break;

		}
		}

	}

	if (nodeId == 0x06) {
		switch (++init_step6) {
		/************************** 配置TPDO **************************************/

		/* 运行模式(Mode of Operation Display 6061)	== 280 + nodeID 异步FF */
		case 1: { /*disable Slave's TPDO2 */
			UNS32 TPDO2_COBId = 0x80000280 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x01, 4, 0,
					&TPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 2: { /*Set Slave's TPDO 2 to be transmitted on FF*/
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 3: { /*Reset Slave's RPDO 2 Mapping number */
			UNS8 TPDO2_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x00, 1, 0,
					&TPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 4: { /*Maping Slave's TPDO 2 with mode_of_operation_Display in 6061 */
			UNS32 TPDO2_mode_of_operation_Display = 0x60610008;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x01, 4, 0,
					&TPDO2_mode_of_operation_Display, CheckSDOAndContinue, 0);
		}
			break;

		case 5: { /*Set Slave's RPDO 3 Mapping number */
			UNS8 TPDO2_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x00, 1, 0,
					&TPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 6: { /*Re-Ensable Slave's TPDO*/
			UNS32 TPDO2_COBId = 0x00000280 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x01, 4, 0,
					&TPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 当前速度+位置(Actual Velocity 606C,Position Acture value 6064) == 380 + nodeID 同步01 */
		case 7: { /*disable Slave's TPDO3 */
			UNS32 TPDO3_COBId = 0x80000380 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x01, 4, 0,
					&TPDO3_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 8: { /*Set Slave's TPDO 3 to be transmitted on 01*/
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x02, 1, 0,
					&Transmission_TPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 9: { /*Reset Slave's RPDO 3 Mapping number */
			UNS8 TPDO3_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x00, 1, 0,
					&TPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 10: { /*Maping Slave's TPDO 3 with speed in 606C */
			UNS32 TPDO3_MapSpeed = 0x606C0020;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x01, 4, 0,
					&TPDO3_MapSpeed, CheckSDOAndContinue, 0);
		}
			break;

		case 11: { /*Maping Slave's TPDO 3 with Position in 6064 */
			UNS32 TPDO3_Mapposition = 0x60640020;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x02, 4, 0,
					&TPDO3_Mapposition, CheckSDOAndContinue, 0);
		}
			break;

		case 12: { /*Set Slave's RPDO 3 Mapping number */
			UNS8 TPDO3_MapNum = 0x02;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x00, 1, 0,
					&TPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 13: { /*Re-Ensable Slave's TPDO*/
			UNS32 RPDO3_COBId = 0x00000380 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x01, 4, 0,
					&RPDO3_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 禁用默认TPDO4（Actural Velocity 606C）480+nodeID 1803-1A03；避免发送无畏信息 */
		case 14: { /*disable Slave's TPDO4 */
			UNS32 TPDO4_COBId = 0x80000480 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1803, 0x01, 4, 0,
					&TPDO4_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/************************** 配置RPDO **************************************/

			/* 控制字  (control words 6040)	== 220	同步00--改为FF*/
		case 15: { /*set Slave's RPDO 1 cobid to 220*/
			UNS32 RPDO2_control_word = 0x00000220;
			writeNetworkDictCallBack(d, nodeId, 0x1400, 0x01, 4, 0,
					&RPDO2_control_word, CheckSDOAndContinue, 0);
		}
			break;

		case 16: { /*Set Slave's RPDO 1 to be transmitted on 00!!!改为FF */
			writeNetworkDictCallBack(d, nodeId, 0x1400, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

			/* 运行模式(Mode of Operation 6060)	== 320	同步00 */
		case 17: { /*disable Slave's RPDO2 */
			UNS32 RPDO2_COBId = 0x80000320;
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x01, 4, 0,
					&RPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 18: { /*Set Slave's RPDO 2 to be transmitted on 00*/
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x02, 1, 0,
					&Transmission_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 19: { /*Reset Slave's RPDO 2 Mapping number */
			UNS8 RPDO2_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x00, 1, 0,
					&RPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 20: { /*Maping Slave's RPDO 2 with mode_of_operation in 6060 */
			UNS32 RPDO3_Map_mode_of_operation = 0x60600008;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x01, 4, 0,
					&RPDO3_Map_mode_of_operation, CheckSDOAndContinue, 0);
		}
			break;

		case 21: { /*Set Slave's RPDO 2 Mapping number */
			UNS8 RPDO2_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x00, 1, 0,
					&RPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 22: { /*Re-Ensable Slave's RPDO2*/
			UNS32 RPDO2_COBId = 0x00000320;
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x01, 4, 0,
					&RPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

//		/* 速度模式:目标速度(target velocity 60FF)	==	420 + nodeID 同步00 */
//		case 22: { /*disable Slave's RPDO3 */
//			UNS32 RPDO3_COBId = 0x80000420 + nodeId;
//			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x01, 4, 0,
//					&RPDO3_COBId, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 23: { /*Set Slave's RPDO 3 to be transmitted on 00*/
//			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x02, 1, 0,
//					&Transmission_RPDO, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 24: { /*Reset Slave's RPDO 3 Mapping number */
//			UNS8 RPDO3_MapNum = 0x00;
//			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x00, 1, 0,
//					&RPDO3_MapNum, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 25: { /*Maping Slave's RPDO 3 with Target_Velocity in 60FF */
//			UNS32 RPDO3_MapTarget_Velocity = 0x60FF0020;
//			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x01, 4, 0,
//					&RPDO3_MapTarget_Velocity, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 26: { /*Set Slave's RPDO 3 Mapping number */
//			UNS8 RPDO3_MapNum = 0x01;
//			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x00, 1, 0,
//					&RPDO3_MapNum, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 27: { /*Re-Ensable Slave's RPDO*/
//			UNS32 RPDO3_COBId = 0x00000420 + nodeId;
//			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x01, 4, 0,
//					&RPDO3_COBId, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		/* 加速度，减速度(Profile Acceleraion 6083,Profile Deceleration 6084)	==	520 + nodeID 异步FF */
//		case 28: { /*disable Slave's RPDO4 */
//			UNS32 RPDO4_COBId = 0x80000520 + nodeId;
//			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x01, 4, 0,
//					&RPDO4_COBId, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 29: { /*Set Slave's RPDO 4 to be transmitted on FF*/
//			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x02, 1, 0,
//					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 30: { /*Reset Slave's RPDO 4 Mapping number */
//			UNS8 RPDO4_MapNum = 0x00;
//			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x00, 1, 0,
//					&RPDO4_MapNum, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 31: { /*Maping Slave's RPDO 4 with profile acceleration */
//			UNS32 RPDO4_MapProfile_acceleration = 0x60830020;
//			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x01, 4, 0,
//					&RPDO4_MapProfile_acceleration, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 32: { /*Maping Slave's RPDO 4 with profile deceleration */
//			UNS32 RPDO4_MapProfile_deceleration = 0x60840020;
//			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x02, 4, 0,
//					&RPDO4_MapProfile_deceleration, CheckSDOAndContinue, 0);
//		}
//			break;
//		case 33: { /*Set Slave's RPDO 4 Mapping number */
//			UNS8 RPDO4_MapNum = 0x02;
//			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x00, 1, 0,
//					&RPDO4_MapNum, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 34: { /*Re-Ensable Slave's RPDO*/
//			UNS32 RPDO4_COBId = 0x00000520 + nodeId;
//			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x01, 4, 0,
//					&RPDO4_COBId, CheckSDOAndContinue, 0);
//		}
//			break;

			/* 位置模式:目标位置(target position 607A)== 620 + nodeID 异步FF */
		case 23: { /*disable Slave's RPDO5*/
			UNS32 RPDO5_COBId = 0x80000620 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x01, 4, 0,
					&RPDO5_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 24: { /*Set Slave's RPDO 5 to be transmitted on FF*/
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 25: { /*Reset Slave's RPDO 5 Mapping number */
			UNS8 RPDO5_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x00, 1, 0,
					&RPDO5_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 26: { /*Maping Slave's RPDO 5 with Target_position in 607A */
			UNS32 RPDO5_MapTarget_position = 0x607A0020;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x01, 4, 0,
					&RPDO5_MapTarget_position, CheckSDOAndContinue, 0);
		}
			break;

//		case 39: { /*Maping Slave's RPDO 5 with profile_velocity in 6081 */
//			UNS32 RPDO5_MapProfile_velocity = 0x60810020;
//			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x02, 4, 0,
//					&RPDO5_MapProfile_velocity, CheckSDOAndContinue, 0);
//		}
//			break;

		case 27: { /*Set Slave's RPDO 5 Mapping number */
			UNS8 RPDO4_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x00, 1, 0,
					&RPDO4_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 28: { /*Re-Ensable Slave's RPDO5*/
			UNS32 RPDO5_COBId = 0x00000620 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x01, 4, 0,
					&RPDO5_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/*************************设置电机运行参数**************************/

//			/*Homing mode parameter*/
//		case 42: { /*Set Slave into Homing Mode*/
//			eprintf("Master : set slave %2.2x into Velocity Mode\n", nodeId);
//			fflush(stdout);
//			UNS8 Homing_mode = 0x06;
//			writeNetworkDictCallBack(d, nodeId, 0x6060, 0x00, 1, 0,
//					&Homing_mode, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 43: { /*Set Homing_Speed_Fast 6099 01*/
//			/***Use if to allocate
//			 * specific move parameters for each rotation_motor
//			 */
//			UNS32 Homing_Speed_Fast = 0x00161a80;
//			writeNetworkDictCallBack(d, nodeId, 0x6099, 0x01, 4, 0,
//					&Homing_Speed_Fast, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 44: { /*Set Homing_Speed_Slow 6099 02*/
//			UNS32 Homing_Speed_Slow = 0x00061a80;
//			writeNetworkDictCallBack(d, nodeId, 0x6099, 0x02, 4, 0,
//					&Homing_Speed_Slow, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 45: { /*Set Homing_Acceleration 609A*/
//			UNS32 Homing_Acceleration = 0x0000FFA0;
//			writeNetworkDictCallBack(d, nodeId, 0x609A, 0x00, 4, 0,
//					&Homing_Acceleration, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 46: { /*Set homing_position_method H23:Home is Current Position; Move to New Zero 35*/
//			UNS8 homing_position_method = 0x23;
//			writeNetworkDictCallBack(d, nodeId, 0x6098, 0x00, 1, 0,
//					&homing_position_method, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 47: { /*Set homing_offset 607C*/
//			UNS32 Homing_offset = 0x000F86A0;
//			writeNetworkDictCallBack(d, nodeId, 0x607C, 0x00, 4, 0,
//					&Homing_offset, CheckSDOAndContinue, 0);
//		}
//			break;
			//设置位置模式
			//梯形
			//速度
			//加速度+减速度
			//是否需要设置HOMING?????
			//enable
			/*Position mode parameter*/
		case 29: { /*Set Slave into position Mode*/
			UNS8 position_mode = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x6060, 0x00, 1, 0,
					&position_mode, CheckSDOAndContinue, 0);
		}
			break;

		case 30: { /*Set Slave Motion profile type--梯形*/
			UNS16 Motion_Profile_type = 0x0000;
			writeNetworkDictCallBack(d, nodeId, 0x6086, 0x00, 2, 0,
					&Motion_Profile_type, CheckSDOAndContinue, 0);
		}
			break;

		case 31: { /* Set Slave profile Velocity 6081
		 旋转电机转速=0.5转/s =655360 counts/s =0x000A0000*/
			writeNetworkDictCallBack(d, nodeId, 0x6081, 0x00, 4, 0,
					&RotateVelocity, CheckSDOAndContinue, 0);
		}
			break;

		case 32: { /*Set Slave Acceleration 6083 = 6×旋转速度=0.5*6=3rps=0x0000999A*/
			writeNetworkDictCallBack(d, nodeId, 0x6083, 0x00, 4, 0, &RotateAcc,
					CheckSDOAndContinue, 0);
		}
			break;

		case 33: { /*Set Slave Deceleration 6084与加速度相同=6×旋转速度=0.5*6=3rps=0x0000999A*/
			writeNetworkDictCallBack(d, nodeId, 0x6084, 0x00, 4, 0, &RotateAcc,
					CheckSDOAndContinue, 0);
		}
			break;

			/*位置模式使用：绝对值（bit6=0）+ change immediate（bit5=1）
			 * 使能	:012F  halt/absolute/change immediate/bit4=0/
			 *启动 	:003F  	  absolute/change immediate/bit4=1
			 *启动后bit4 1-0	:002F 	  absolute/change immediate/bit4=0
			 *停止  :012F  halt/absolute/change immediate/bit4=0/
			 */
		case 34: {
			/* enable motor halt/absolute/change imeadiate/bit4=0/ */
			UNS16 enable = 0x012F;
//			UNS16 enable = 0x0127;//断使能
			writeNetworkDictCallBack(d, nodeId, 0x6040, 0x00, 2, 0, &enable,
					CheckSDOAndContinue, 0);
		}
			break;

			/**************************设置从站心跳报文**************************/

		case 35: /*Third step: configure Slave Heart-beat interval*/
			writeNetworkDictCallBack(d, /*CO_Data* d*/
			nodeId, /*UNS8 nodeId*/
			0x1017, /*UNS16 index*/
			0x00, /*UNS16 index*/
			2, /*UNS8 count*/
			0, /*UNS8 dataType*/
			&heartdata,/*data must be a index*/
			CheckSDOAndContinue, /*SDOCallback_t Callback*/
			0); /* use block mode */
			break;

			/****************************** 启动从节点 *******************************/
		case 36: {
			/* Put the master in operational mode */
			setState(CANOpenShellOD_Data, Operational);

			/* Ask slave node to go in operational mode */
			StartNode(nodeId);

			/* reset counter in case the slave boot-up again */
//			init_step6 = 0;//不需要置0，避免电机重新上电从新配置。当电机重新上电此时init_step1==36，进入配置后++init_step1 ==37，并没有case36则跳过。
			num_slavebootup++;
			break;

		}
		}

	}

	if (nodeId == 0x07) {
		switch (++init_step7) {
		/************************** 配置TPDO **************************************/

		/* 运行模式(Mode of Operation Display 6061)	== 280 + nodeID 异步FF */
		case 1: { /*disable Slave's TPDO2 */
			UNS32 TPDO2_COBId = 0x80000280 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x01, 4, 0,
					&TPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 2: { /*Set Slave's TPDO 2 to be transmitted on FF*/
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 3: { /*Reset Slave's RPDO 2 Mapping number */
			UNS8 TPDO2_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x00, 1, 0,
					&TPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 4: { /*Maping Slave's TPDO 2 with mode_of_operation_Display in 6061 */
			UNS32 TPDO2_mode_of_operation_Display = 0x60610008;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x01, 4, 0,
					&TPDO2_mode_of_operation_Display, CheckSDOAndContinue, 0);
		}
			break;

		case 5: { /*Set Slave's RPDO 3 Mapping number */
			UNS8 TPDO2_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x00, 1, 0,
					&TPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 6: { /*Re-Ensable Slave's TPDO*/
			UNS32 TPDO2_COBId = 0x00000280 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x01, 4, 0,
					&TPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 当前速度+位置(Actual Velocity 606C,Position Acture value 6064) == 380 + nodeID 同步01 */
		case 7: { /*disable Slave's TPDO3 */
			UNS32 TPDO3_COBId = 0x80000380 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x01, 4, 0,
					&TPDO3_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 8: { /*Set Slave's TPDO 3 to be transmitted on 01*/
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x02, 1, 0,
					&Transmission_TPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 9: { /*Reset Slave's RPDO 3 Mapping number */
			UNS8 TPDO3_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x00, 1, 0,
					&TPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 10: { /*Maping Slave's TPDO 3 with speed in 606C */
			UNS32 TPDO3_MapSpeed = 0x606C0020;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x01, 4, 0,
					&TPDO3_MapSpeed, CheckSDOAndContinue, 0);
		}
			break;

		case 11: { /*Maping Slave's TPDO 3 with Position in 6064 */
			UNS32 TPDO3_Mapposition = 0x60640020;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x02, 4, 0,
					&TPDO3_Mapposition, CheckSDOAndContinue, 0);
		}
			break;

		case 12: { /*Set Slave's RPDO 3 Mapping number */
			UNS8 TPDO3_MapNum = 0x02;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x00, 1, 0,
					&TPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 13: { /*Re-Ensable Slave's TPDO*/
			UNS32 RPDO3_COBId = 0x00000380 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x01, 4, 0,
					&RPDO3_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 禁用默认TPDO4（Actural Velocity 606C）480+nodeID 1803-1A03；避免发送无畏信息 */
		case 14: { /*disable Slave's TPDO4 */
			UNS32 TPDO4_COBId = 0x80000480 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1803, 0x01, 4, 0,
					&TPDO4_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/************************** 配置RPDO **************************************/

			/* 控制字  (control words 6040)	== 220	同步00--改为FF */
		case 15: { /*set Slave's RPDO 1 cobid to 220*/
			UNS32 RPDO2_control_word = 0x00000220;
			writeNetworkDictCallBack(d, nodeId, 0x1400, 0x01, 4, 0,
					&RPDO2_control_word, CheckSDOAndContinue, 0);
		}
			break;

		case 16: { /*Set Slave's RPDO 1 to be transmitted on 00--改为FF!!! */
			writeNetworkDictCallBack(d, nodeId, 0x1400, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

			/* 运行模式(Mode of Operation 6060)	== 320	同步00 */
		case 17: { /*disable Slave's RPDO2 */
			UNS32 RPDO2_COBId = 0x80000320;
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x01, 4, 0,
					&RPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 18: { /*Set Slave's RPDO 2 to be transmitted on 00*/
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x02, 1, 0,
					&Transmission_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 19: { /*Reset Slave's RPDO 2 Mapping number */
			UNS8 RPDO2_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x00, 1, 0,
					&RPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 20: { /*Maping Slave's RPDO 2 with mode_of_operation in 6060 */
			UNS32 RPDO3_Map_mode_of_operation = 0x60600008;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x01, 4, 0,
					&RPDO3_Map_mode_of_operation, CheckSDOAndContinue, 0);
		}
			break;

		case 21: { /*Set Slave's RPDO 2 Mapping number */
			UNS8 RPDO2_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x00, 1, 0,
					&RPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 22: { /*Re-Ensable Slave's RPDO2*/
			UNS32 RPDO2_COBId = 0x00000320;
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x01, 4, 0,
					&RPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

//		/* 速度模式:目标速度(target velocity 60FF)	==	420 + nodeID 同步00 */
//		case 22: { /*disable Slave's RPDO3 */
//			UNS32 RPDO3_COBId = 0x80000420 + nodeId;
//			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x01, 4, 0,
//					&RPDO3_COBId, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 23: { /*Set Slave's RPDO 3 to be transmitted on 00*/
//			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x02, 1, 0,
//					&Transmission_RPDO, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 24: { /*Reset Slave's RPDO 3 Mapping number */
//			UNS8 RPDO3_MapNum = 0x00;
//			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x00, 1, 0,
//					&RPDO3_MapNum, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 25: { /*Maping Slave's RPDO 3 with Target_Velocity in 60FF */
//			UNS32 RPDO3_MapTarget_Velocity = 0x60FF0020;
//			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x01, 4, 0,
//					&RPDO3_MapTarget_Velocity, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 26: { /*Set Slave's RPDO 3 Mapping number */
//			UNS8 RPDO3_MapNum = 0x01;
//			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x00, 1, 0,
//					&RPDO3_MapNum, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 27: { /*Re-Ensable Slave's RPDO*/
//			UNS32 RPDO3_COBId = 0x00000420 + nodeId;
//			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x01, 4, 0,
//					&RPDO3_COBId, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		/* 加速度，减速度(Profile Acceleraion 6083,Profile Deceleration 6084)	==	520 + nodeID 异步FF */
//		case 28: { /*disable Slave's RPDO4 */
//			UNS32 RPDO4_COBId = 0x80000520 + nodeId;
//			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x01, 4, 0,
//					&RPDO4_COBId, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 29: { /*Set Slave's RPDO 4 to be transmitted on FF*/
//			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x02, 1, 0,
//					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 30: { /*Reset Slave's RPDO 4 Mapping number */
//			UNS8 RPDO4_MapNum = 0x00;
//			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x00, 1, 0,
//					&RPDO4_MapNum, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 31: { /*Maping Slave's RPDO 4 with profile acceleration */
//			UNS32 RPDO4_MapProfile_acceleration = 0x60830020;
//			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x01, 4, 0,
//					&RPDO4_MapProfile_acceleration, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 32: { /*Maping Slave's RPDO 4 with profile deceleration */
//			UNS32 RPDO4_MapProfile_deceleration = 0x60840020;
//			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x02, 4, 0,
//					&RPDO4_MapProfile_deceleration, CheckSDOAndContinue, 0);
//		}
//			break;
//		case 33: { /*Set Slave's RPDO 4 Mapping number */
//			UNS8 RPDO4_MapNum = 0x02;
//			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x00, 1, 0,
//					&RPDO4_MapNum, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 34: { /*Re-Ensable Slave's RPDO*/
//			UNS32 RPDO4_COBId = 0x00000520 + nodeId;
//			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x01, 4, 0,
//					&RPDO4_COBId, CheckSDOAndContinue, 0);
//		}
//			break;

			/* 位置模式:目标位置(target position 607A)== 620 + nodeID 异步FF */
		case 23: { /*disable Slave's RPDO5*/
			UNS32 RPDO5_COBId = 0x80000620 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x01, 4, 0,
					&RPDO5_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 24: { /*Set Slave's RPDO 5 to be transmitted on FF*/
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 25: { /*Reset Slave's RPDO 5 Mapping number */
			UNS8 RPDO5_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x00, 1, 0,
					&RPDO5_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 26: { /*Maping Slave's RPDO 5 with Target_position in 607A */
			UNS32 RPDO5_MapTarget_position = 0x607A0020;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x01, 4, 0,
					&RPDO5_MapTarget_position, CheckSDOAndContinue, 0);
		}
			break;

//		case 39: { /*Maping Slave's RPDO 5 with profile_velocity in 6081 */
//			UNS32 RPDO5_MapProfile_velocity = 0x60810020;
//			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x02, 4, 0,
//					&RPDO5_MapProfile_velocity, CheckSDOAndContinue, 0);
//		}
//			break;

		case 27: { /*Set Slave's RPDO 5 Mapping number */
			UNS8 RPDO4_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x00, 1, 0,
					&RPDO4_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 28: { /*Re-Ensable Slave's RPDO5*/
			UNS32 RPDO5_COBId = 0x00000620 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x01, 4, 0,
					&RPDO5_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/*************************设置电机运行参数**************************/

//			/*Homing mode parameter*/
//		case 42: { /*Set Slave into Homing Mode*/
//			eprintf("Master : set slave %2.2x into Velocity Mode\n", nodeId);
//			fflush(stdout);
//			UNS8 Homing_mode = 0x06;
//			writeNetworkDictCallBack(d, nodeId, 0x6060, 0x00, 1, 0,
//					&Homing_mode, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 43: { /*Set Homing_Speed_Fast 6099 01*/
//			/***Use if to allocate
//			 * specific move parameters for each rotation_motor
//			 */
//			UNS32 Homing_Speed_Fast = 0x00161a80;
//			writeNetworkDictCallBack(d, nodeId, 0x6099, 0x01, 4, 0,
//					&Homing_Speed_Fast, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 44: { /*Set Homing_Speed_Slow 6099 02*/
//			UNS32 Homing_Speed_Slow = 0x00061a80;
//			writeNetworkDictCallBack(d, nodeId, 0x6099, 0x02, 4, 0,
//					&Homing_Speed_Slow, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 45: { /*Set Homing_Acceleration 609A*/
//			UNS32 Homing_Acceleration = 0x0000FFA0;
//			writeNetworkDictCallBack(d, nodeId, 0x609A, 0x00, 4, 0,
//					&Homing_Acceleration, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 46: { /*Set homing_position_method H23:Home is Current Position; Move to New Zero 35*/
//			UNS8 homing_position_method = 0x23;
//			writeNetworkDictCallBack(d, nodeId, 0x6098, 0x00, 1, 0,
//					&homing_position_method, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 47: { /*Set homing_offset 607C*/
//			UNS32 Homing_offset = 0x000F86A0;
//			writeNetworkDictCallBack(d, nodeId, 0x607C, 0x00, 4, 0,
//					&Homing_offset, CheckSDOAndContinue, 0);
//		}
//			break;
			//设置位置模式
			//梯形
			//速度
			//加速度+减速度
			//是否需要设置HOMING?????
			//enable
			/*Position mode parameter*/
		case 29: { /*Set Slave into position Mode*/
			UNS8 position_mode = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x6060, 0x00, 1, 0,
					&position_mode, CheckSDOAndContinue, 0);
		}
			break;

		case 30: { /*Set Slave Motion profile type--梯形*/
			UNS16 Motion_Profile_type = 0x0000;
			writeNetworkDictCallBack(d, nodeId, 0x6086, 0x00, 2, 0,
					&Motion_Profile_type, CheckSDOAndContinue, 0);
		}
			break;

		case 31: { /* Set Slave profile Velocity 6081
		 旋转电机转速=0.5转/s =655360 counts/s =0x000A0000*/
			writeNetworkDictCallBack(d, nodeId, 0x6081, 0x00, 4, 0,
					&RotateVelocity, CheckSDOAndContinue, 0);
		}
			break;

		case 32: { /*Set Slave Acceleration 6083 = 6×旋转速度=0.5*6=3rps=0x0000999A*/
			writeNetworkDictCallBack(d, nodeId, 0x6083, 0x00, 4, 0, &RotateAcc,
					CheckSDOAndContinue, 0);
		}
			break;

		case 33: { /*Set Slave Deceleration 6084与加速度相同=6×旋转速度=0.5*6=3rps=0x0000999A*/
			writeNetworkDictCallBack(d, nodeId, 0x6084, 0x00, 4, 0, &RotateAcc,
					CheckSDOAndContinue, 0);
		}
			break;

			/*位置模式使用：绝对值（bit6=0）+ change immediate（bit5=1）
			 * 使能	:012F  halt/absolute/change immediate/bit4=0/
			 *启动 	:003F  	  absolute/change immediate/bit4=1
			 *启动后bit4 1-0	:002F 	  absolute/change immediate/bit4=0
			 *停止  :012F  halt/absolute/change immediate/bit4=0/
			 */
		case 34: {
			/* enable motor halt/absolute/change imeadiate/bit4=0/ */
			UNS16 enable = 0x012F;
//			UNS16 enable = 0x0127;//断使能
			writeNetworkDictCallBack(d, nodeId, 0x6040, 0x00, 2, 0, &enable,
					CheckSDOAndContinue, 0);
		}
			break;

			/**************************设置从站心跳报文**************************/

		case 35: /*Third step: configure Slave Heart-beat interval*/
			writeNetworkDictCallBack(d, /*CO_Data* d*/
			nodeId, /*UNS8 nodeId*/
			0x1017, /*UNS16 index*/
			0x00, /*UNS16 index*/
			2, /*UNS8 count*/
			0, /*UNS8 dataType*/
			&heartdata,/*data must be a index*/
			CheckSDOAndContinue, /*SDOCallback_t Callback*/
			0); /* use block mode */
			break;

			/****************************** 启动从节点 *******************************/
		case 36: {
			/* Put the master in operational mode */
			setState(CANOpenShellOD_Data, Operational);

			/* Ask slave node to go in operational mode */
			StartNode(nodeId);

			/* reset counter in case the slave boot-up again */
//			init_step7 = 0;//不需要置0，避免电机重新上电从新配置。当电机重新上电此时init_step1==36，进入配置后++init_step1 ==37，并没有case36则跳过。
			num_slavebootup++;
			break;

		}
		}

	}

	if (nodeId == 0x08) {
		switch (++init_step8) {
		/************************** 配置TPDO **************************************/

		/* 运行模式(Mode of Operation Display 6061)	== 280 + nodeID 异步FF */
		case 1: { /*disable Slave's TPDO2 */
			UNS32 TPDO2_COBId = 0x80000280 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x01, 4, 0,
					&TPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 2: { /*Set Slave's TPDO 2 to be transmitted on FF*/
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 3: { /*Reset Slave's RPDO 2 Mapping number */
			UNS8 TPDO2_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x00, 1, 0,
					&TPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 4: { /*Maping Slave's TPDO 2 with mode_of_operation_Display in 6061 */
			UNS32 TPDO2_mode_of_operation_Display = 0x60610008;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x01, 4, 0,
					&TPDO2_mode_of_operation_Display, CheckSDOAndContinue, 0);
		}
			break;

		case 5: { /*Set Slave's RPDO 3 Mapping number */
			UNS8 TPDO2_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x00, 1, 0,
					&TPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 6: { /*Re-Ensable Slave's TPDO*/
			UNS32 TPDO2_COBId = 0x00000280 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x01, 4, 0,
					&TPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 当前速度+位置(Actual Velocity 606C,Position Acture value 6064) == 380 + nodeID 同步01 */
		case 7: { /*disable Slave's TPDO3 */
			UNS32 TPDO3_COBId = 0x80000380 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x01, 4, 0,
					&TPDO3_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 8: { /*Set Slave's TPDO 3 to be transmitted on 01*/
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x02, 1, 0,
					&Transmission_TPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 9: { /*Reset Slave's RPDO 3 Mapping number */
			UNS8 TPDO3_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x00, 1, 0,
					&TPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 10: { /*Maping Slave's TPDO 3 with speed in 606C */
			UNS32 TPDO3_MapSpeed = 0x606C0020;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x01, 4, 0,
					&TPDO3_MapSpeed, CheckSDOAndContinue, 0);
		}
			break;

		case 11: { /*Maping Slave's TPDO 3 with Position in 6064 */
			UNS32 TPDO3_Mapposition = 0x60640020;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x02, 4, 0,
					&TPDO3_Mapposition, CheckSDOAndContinue, 0);
		}
			break;

		case 12: { /*Set Slave's RPDO 3 Mapping number */
			UNS8 TPDO3_MapNum = 0x02;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x00, 1, 0,
					&TPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 13: { /*Re-Ensable Slave's TPDO*/
			UNS32 RPDO3_COBId = 0x00000380 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x01, 4, 0,
					&RPDO3_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 禁用默认TPDO4（Actural Velocity 606C）480+nodeID 1803-1A03；避免发送无畏信息 */
		case 14: { /*disable Slave's TPDO4 */
			UNS32 TPDO4_COBId = 0x80000480 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1803, 0x01, 4, 0,
					&TPDO4_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/************************** 配置RPDO **************************************/

			/* 控制字  (control words 6040)	== 220	同步00--改为FF */
		case 15: { /*set Slave's RPDO 1 cobid to 220*/
			UNS32 RPDO2_control_word = 0x00000220;
			writeNetworkDictCallBack(d, nodeId, 0x1400, 0x01, 4, 0,
					&RPDO2_control_word, CheckSDOAndContinue, 0);
		}
			break;

		case 16: { /*Set Slave's RPDO 1 to be transmitted on 00--改为FF!!! */
			writeNetworkDictCallBack(d, nodeId, 0x1400, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

			/* 运行模式(Mode of Operation 6060)	== 320	同步00 */
		case 17: { /*disable Slave's RPDO2 */
			UNS32 RPDO2_COBId = 0x80000320;
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x01, 4, 0,
					&RPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 18: { /*Set Slave's RPDO 2 to be transmitted on 00*/
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x02, 1, 0,
					&Transmission_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 19: { /*Reset Slave's RPDO 2 Mapping number */
			UNS8 RPDO2_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x00, 1, 0,
					&RPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 20: { /*Maping Slave's RPDO 2 with mode_of_operation in 6060 */
			UNS32 RPDO3_Map_mode_of_operation = 0x60600008;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x01, 4, 0,
					&RPDO3_Map_mode_of_operation, CheckSDOAndContinue, 0);
		}
			break;

		case 21: { /*Set Slave's RPDO 2 Mapping number */
			UNS8 RPDO2_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x00, 1, 0,
					&RPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 22: { /*Re-Ensable Slave's RPDO2*/
			UNS32 RPDO2_COBId = 0x00000320;
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x01, 4, 0,
					&RPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

//		/* 速度模式:目标速度(target velocity 60FF)	==	420 + nodeID 同步00 */
//		case 22: { /*disable Slave's RPDO3 */
//			UNS32 RPDO3_COBId = 0x80000420 + nodeId;
//			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x01, 4, 0,
//					&RPDO3_COBId, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 23: { /*Set Slave's RPDO 3 to be transmitted on 00*/
//			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x02, 1, 0,
//					&Transmission_RPDO, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 24: { /*Reset Slave's RPDO 3 Mapping number */
//			UNS8 RPDO3_MapNum = 0x00;
//			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x00, 1, 0,
//					&RPDO3_MapNum, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 25: { /*Maping Slave's RPDO 3 with Target_Velocity in 60FF */
//			UNS32 RPDO3_MapTarget_Velocity = 0x60FF0020;
//			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x01, 4, 0,
//					&RPDO3_MapTarget_Velocity, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 26: { /*Set Slave's RPDO 3 Mapping number */
//			UNS8 RPDO3_MapNum = 0x01;
//			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x00, 1, 0,
//					&RPDO3_MapNum, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 27: { /*Re-Ensable Slave's RPDO*/
//			UNS32 RPDO3_COBId = 0x00000420 + nodeId;
//			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x01, 4, 0,
//					&RPDO3_COBId, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		/* 加速度，减速度(Profile Acceleraion 6083,Profile Deceleration 6084)	==	520 + nodeID 异步FF */
//		case 28: { /*disable Slave's RPDO4 */
//			UNS32 RPDO4_COBId = 0x80000520 + nodeId;
//			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x01, 4, 0,
//					&RPDO4_COBId, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 29: { /*Set Slave's RPDO 4 to be transmitted on FF*/
//			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x02, 1, 0,
//					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 30: { /*Reset Slave's RPDO 4 Mapping number */
//			UNS8 RPDO4_MapNum = 0x00;
//			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x00, 1, 0,
//					&RPDO4_MapNum, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 31: { /*Maping Slave's RPDO 4 with profile acceleration */
//			UNS32 RPDO4_MapProfile_acceleration = 0x60830020;
//			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x01, 4, 0,
//					&RPDO4_MapProfile_acceleration, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 32: { /*Maping Slave's RPDO 4 with profile deceleration */
//			UNS32 RPDO4_MapProfile_deceleration = 0x60840020;
//			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x02, 4, 0,
//					&RPDO4_MapProfile_deceleration, CheckSDOAndContinue, 0);
//		}
//			break;
//		case 33: { /*Set Slave's RPDO 4 Mapping number */
//			UNS8 RPDO4_MapNum = 0x02;
//			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x00, 1, 0,
//					&RPDO4_MapNum, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 34: { /*Re-Ensable Slave's RPDO*/
//			UNS32 RPDO4_COBId = 0x00000520 + nodeId;
//			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x01, 4, 0,
//					&RPDO4_COBId, CheckSDOAndContinue, 0);
//		}
//			break;

			/* 位置模式:目标位置(target position 607A)== 620 + nodeID 异步FF */
		case 23: { /*disable Slave's RPDO5*/
			UNS32 RPDO5_COBId = 0x80000620 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x01, 4, 0,
					&RPDO5_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 24: { /*Set Slave's RPDO 5 to be transmitted on FF*/
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 25: { /*Reset Slave's RPDO 5 Mapping number */
			UNS8 RPDO5_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x00, 1, 0,
					&RPDO5_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 26: { /*Maping Slave's RPDO 5 with Target_position in 607A */
			UNS32 RPDO5_MapTarget_position = 0x607A0020;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x01, 4, 0,
					&RPDO5_MapTarget_position, CheckSDOAndContinue, 0);
		}
			break;

//		case 39: { /*Maping Slave's RPDO 5 with profile_velocity in 6081 */
//			UNS32 RPDO5_MapProfile_velocity = 0x60810020;
//			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x02, 4, 0,
//					&RPDO5_MapProfile_velocity, CheckSDOAndContinue, 0);
//		}
//			break;

		case 27: { /*Set Slave's RPDO 5 Mapping number */
			UNS8 RPDO4_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x00, 1, 0,
					&RPDO4_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 28: { /*Re-Ensable Slave's RPDO5*/
			UNS32 RPDO5_COBId = 0x00000620 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x01, 4, 0,
					&RPDO5_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/*************************设置电机运行参数**************************/

			//设置位置模式
			//梯形
			//速度
			//加速度+减速度
			//是否需要设置HOMING?????
			//enable
			/*Position mode parameter*/
		case 29: { /*Set Slave into position Mode*/
			UNS8 position_mode = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x6060, 0x00, 1, 0,
					&position_mode, CheckSDOAndContinue, 0);
		}
			break;

		case 30: { /*Set Slave Motion profile type--梯形*/
			UNS16 Motion_Profile_type = 0x0000;
			writeNetworkDictCallBack(d, nodeId, 0x6086, 0x00, 2, 0,
					&Motion_Profile_type, CheckSDOAndContinue, 0);
		}
			break;

		case 31: { /* Set Slave profile Velocity 6081
		 旋转电机转速=0.5转/s =655360 counts/s =0x000A0000*/
			writeNetworkDictCallBack(d, nodeId, 0x6081, 0x00, 4, 0,
					&RotateVelocity, CheckSDOAndContinue, 0);
		}
			break;

		case 32: { /*Set Slave Acceleration 6083 = 6×旋转速度=0.5*6=3rps=0x0000999A*/
			writeNetworkDictCallBack(d, nodeId, 0x6083, 0x00, 4, 0, &RotateAcc,
					CheckSDOAndContinue, 0);
		}
			break;

		case 33: { /*Set Slave Deceleration 6084与加速度相同=6×旋转速度=0.5*6=3rps=0x0000999A*/
			writeNetworkDictCallBack(d, nodeId, 0x6084, 0x00, 4, 0, &RotateAcc,
					CheckSDOAndContinue, 0);
		}
			break;

			/*位置模式使用：绝对值（bit6=0）+ change immediate（bit5=1）
			 * 使能	:012F  halt/absolute/change immediate/bit4=0/
			 *启动 	:003F  	  absolute/change immediate/bit4=1
			 *启动后bit4 1-0	:002F 	  absolute/change immediate/bit4=0
			 *停止  :012F  halt/absolute/cif (nodeId ==0x02) {hange immediate/bit4=0/
			 */
		case 34: {
			/* enable motor halt/absolute/change imeadiate/bit4=0/ */
			UNS16 enable = 0x012F;
//			UNS16 enable = 0x0127;//断使能
			writeNetworkDictCallBack(d, nodeId, 0x6040, 0x00, 2, 0, &enable,
					CheckSDOAndContinue, 0);
		}
			break;

			/**************************设置从站心跳报文**************************/

		case 35: /*Third step: configure Slave Heart-beat interval*/
			writeNetworkDictCallBack(d, /*CO_Data* d*/
			nodeId, /*UNS8 nodeId*/
			0x1017, /*UNS16 index*/
			0x00, /*UNS16 index*/
			2, /*UNS8 count*/
			0, /*UNS8 dataType*/
			&heartdata,/*data must be a index*/
			CheckSDOAndContinue, /*SDOCallback_t Callback*/
			0); /* use block mode */
			break;

			/****************************** 启动从节点 *******************************/
		case 36: {
			/* Put the master in operational mode */
			setState(CANOpenShellOD_Data, Operational);

			/* Ask slave node to go in operational mode */
			StartNode(nodeId);

			/* reset counter in case the slave boot-up again */
//			init_step8 = 0;//不需要置0，避免电机重新上电从新配置。当电机重新上电此时init_step1==36，进入配置后++init_step1 ==37，并没有case36则跳过。
			num_slavebootup++;
			break;

		}
		}

	}

	/*第三组：举升电机 nodeID:09 位置模式 (有可能需要homing->position，上电找0点)
	 *
	 *TPDO:状态PDO 同步：01-240  异步：254-255  （十进制）
	 *		状态字  (status words 6041)				== 180 + nodeID 异步FF
	 *		运行模式(Mode of Operation Display 6061)	== 280 + nodeID 异步FF
	 *		当前速度+位置(Actual Velocity 606C,Position Acture value 6064) == 380 + nodeID 同步01
	 *		禁用默认TPDO4（Actural Velocity 606C）480+nodeID 1803-1A03；避免发送无畏信息
	 *
	 *RPDO:控制PDO 同步：00-240  异步：254-255  （十进制）
	 *		控制字  (control words 6040)		== 230	同步00
	 *		运行模式(Mode of Operation 6060)	== 330	同步00
	 *
	 *		速度模式:目标速度(target velocity 60FF)	==	430 同步00
	 *
	 *		加速度，减速度(Profile Acceleraion 6083,Profile Deceleration 6084)	==	530 异步FF
	 *
	 *		位置模式:目标位置(target position 607A),速度(Profile Velocity 6081)	== 630 异步FF
	 *
	 *
	 *		 */
	if (nodeId == 0x09) {
		switch (++init_step9) {
		/************************** 配置TPDO **************************************/
		/* 运行模式(Mode of Operation Display 6061)	== 280 + nodeID 异步FF */
		case 1: { /*disable Slave's TPDO2 */
			UNS32 TPDO2_COBId = 0x80000280 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x01, 4, 0,
					&TPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 2: { /*Set Slave's TPDO 2 to be transmitted on FF*/
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 3: { /*Reset Slave's RPDO 2 Mapping number */
			UNS8 TPDO2_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x00, 1, 0,
					&TPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 4: { /*Maping Slave's TPDO 2 with mode_of_operation_Display in 6061 */
			UNS32 TPDO2_mode_of_operation_Display = 0x60610008;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x01, 4, 0,
					&TPDO2_mode_of_operation_Display, CheckSDOAndContinue, 0);
		}
			break;

		case 5: { /*Set Slave's RPDO 3 Mapping number */
			UNS8 TPDO2_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1A01, 0x00, 1, 0,
					&TPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 6: { /*Re-Ensable Slave's TPDO*/
			UNS32 TPDO2_COBId = 0x00000280 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x01, 4, 0,
					&TPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 当前速度+位置(Actual Velocity 606C,Position Acture value 6064) == 380 + nodeID 同步01 */
		case 7: { /*disable Slave's TPDO3 */
			UNS32 TPDO3_COBId = 0x80000380 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x01, 4, 0,
					&TPDO3_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 8: { /*Set Slave's TPDO 3 to be transmitted on 01*/
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x02, 1, 0,
					&Transmission_TPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 9: { /*Reset Slave's RPDO 3 Mapping number */
			UNS8 TPDO3_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x00, 1, 0,
					&TPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 10: { /*Maping Slave's TPDO 3 with speed in 606C */
			UNS32 TPDO3_MapSpeed = 0x606C0020;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x01, 4, 0,
					&TPDO3_MapSpeed, CheckSDOAndContinue, 0);
		}
			break;

		case 11: { /*Maping Slave's TPDO 3 with Position in 6064 */
			UNS32 TPDO3_Mapposition = 0x60640020;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x02, 4, 0,
					&TPDO3_Mapposition, CheckSDOAndContinue, 0);
		}
			break;

		case 12: { /*Set Slave's RPDO 3 Mapping number */
			UNS8 TPDO3_MapNum = 0x02;
			writeNetworkDictCallBack(d, nodeId, 0x1A02, 0x00, 1, 0,
					&TPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 13: { /*Re-Ensable Slave's TPDO*/
			UNS32 TPDO3_COBId = 0x00000380 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1802, 0x01, 4, 0,
					&TPDO3_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/* 禁用默认TPDO4（Actural Velocity 606C）480+nodeID 1803-1A03；避免发送无畏信息 */
		case 14: { /*disable Slave's TPDO4 */
			UNS32 TPDO4_COBId = 0x80000480 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1803, 0x01, 4, 0,
					&TPDO4_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/************************** 配置RPDO **************************************/

			/* 控制字  (control words 6040) == 230	同步00 */
		case 15: { /*set Slave's RPDO 1 cobid to 230*/
			UNS32 RPDO1_COBId = 0x00000230;
			writeNetworkDictCallBack(d, nodeId, 0x1400, 0x01, 4, 0,
					&RPDO1_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 16: { /*Set Slave's RPDO 1 to be transmitted on 00 */
			writeNetworkDictCallBack(d, nodeId, 0x1400, 0x02, 1, 0,
					&Transmission_RPDO, CheckSDOAndContinue, 0);
		}
			break;

			/* 运行模式(Mode of Operation 6060)	== 330	同步00 */
		case 17: { /*disable Slave's RPDO3 */
			UNS32 RPDO2_COBId = 0x80000330;
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x01, 4, 0,
					&RPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 18: { /*Set Slave's RPDO 2 to be transmitted on 00*/
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x02, 1, 0,
					&Transmission_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 19: { /*Reset Slave's RPDO 2 Mapping number */
			UNS8 RPDO2_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x00, 1, 0,
					&RPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 20: { /*Maping Slave's RPDO 2 with mode_of_operation in 6060 */
			UNS32 RPDO3_Map_mode_of_operation = 0x60600008;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x01, 4, 0,
					&RPDO3_Map_mode_of_operation, CheckSDOAndContinue, 0);
		}
			break;

		case 21: { /*Set Slave's RPDO 2 Mapping number */
			UNS8 RPDO2_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1601, 0x00, 1, 0,
					&RPDO2_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 22: { /*Re-Ensable Slave's RPDO2*/
			UNS32 RPDO2_COBId = 0x00000330;
			writeNetworkDictCallBack(d, nodeId, 0x1401, 0x01, 4, 0,
					&RPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

//		/* 速度模式:目标速度(target velocity 60FF)	==	430 同步00 */
//		case 22: { /*disable Slave's RPDO3 */
//			UNS32 RPDO3_COBId = 0x80000430;
//			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x01, 4, 0,
//					&RPDO3_COBId, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 23: { /*Set Slave's RPDO 3 to be transmitted on 00*/
//			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x02, 1, 0,
//					&Transmission_RPDO, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 24: { /*Reset Slave's RPDO 3 Mapping number */
//			UNS8 RPDO3_MapNum = 0x00;
//			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x00, 1, 0,
//					&RPDO3_MapNum, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 25: { /*Maping Slave's RPDO 3 with Target_Velocity in 60FF */
//			UNS32 RPDO3_MapTarget_Velocity = 0x60FF0020;
//			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x01, 4, 0,
//					&RPDO3_MapTarget_Velocity, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 26: { /*Set Slave's RPDO 3 Mapping number */
//			UNS8 RPDO3_MapNum = 0x01;
//			writeNetworkDictCallBack(d, nodeId, 0x1602, 0x00, 1, 0,
//					&RPDO3_MapNum, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 27: { /*Re-Ensable Slave's RPDO*/
//			UNS32 RPDO3_COBId = 0x00000430;
//			writeNetworkDictCallBack(d, nodeId, 0x1402, 0x01, 4, 0,
//					&RPDO3_COBId, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		/* 加速度，减速度(Profile Acceleraion 6083,Profile Deceleration 6084)	==	530 异步FF */
//		case 28: { /*disable Slave's RPDO4 */
//			UNS32 RPDO4_COBId = 0x80000530;
//			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x01, 4, 0,
//					&RPDO4_COBId, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 29: { /*Set Slave's RPDO 4 to be transmitted on FF*/
//			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x02, 1, 0,
//					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 30: { /*Reset Slave's RPDO 4 Mapping number */
//			UNS8 RPDO4_MapNum = 0x00;
//			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x00, 1, 0,
//					&RPDO4_MapNum, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 31: { /*Maping Slave's RPDO 4 with profile acceleration */
//			UNS32 RPDO4_MapProfile_acceleration = 0x60830020;
//			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x01, 4, 0,
//					&RPDO4_MapProfile_acceleration, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 32: { /*Maping Slave's RPDO 4 with profile deceleration */
//			UNS32 RPDO4_MapProfile_deceleration = 0x60840020;
//			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x02, 4, 0,
//					&RPDO4_MapProfile_deceleration, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 33: { /*Set Slave's RPDO 4 Mapping number */
//			UNS8 RPDO4_MapNum = 0x02;
//			writeNetworkDictCallBack(d, nodeId, 0x1603, 0x00, 1, 0,
//					&RPDO4_MapNum, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 34: { /*Re-Ensable Slave's RPDO*/
//			UNS32 RPDO4_COBId = 0x00000530;
//			writeNetworkDictCallBack(d, nodeId, 0x1403, 0x01, 4, 0,
//					&RPDO4_COBId, CheckSDOAndContinue, 0);
//		}
//			break;

			/* 位置模式:目标位置(target position 607A)	== 630 异步FF */
		case 23: { /*disable Slave's RPDO5*/
			UNS32 RPDO5_COBId = 0x80000630;
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x01, 4, 0,
					&RPDO5_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 24: { /*Set Slave's RPDO 5 to be transmitted on FF*/
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x02, 1, 0,
					&Position_Mode_RPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 25: { /*Reset Slave's RPDO 5 Mapping number */
			UNS8 RPDO5_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x00, 1, 0,
					&RPDO5_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 26: { /*Maping Slave's RPDO 3 with Target_position in 607A */
			UNS32 RPDO3_MapTarget_position = 0x607A0020;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x01, 4, 0,
					&RPDO3_MapTarget_position, CheckSDOAndContinue, 0);
		}
			break;

		case 27: { /*Set Slave's RPDO 5 Mapping number */
			UNS8 RPDO4_MapNum = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x1604, 0x00, 1, 0,
					&RPDO4_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 28: { /*Re-Ensable Slave's RPDO5*/
			UNS32 RPDO5_COBId = 0x00000630;
			writeNetworkDictCallBack(d, nodeId, 0x1404, 0x01, 4, 0,
					&RPDO5_COBId, CheckSDOAndContinue, 0);
		}
			break;

			/*************************设置电机运行参数**************************/
			/*Homing mode parameter*/
//		case 29: { /*Set Slave into Homing Mode*/
//			UNS8 Homing_mode = 0x06;
//			writeNetworkDictCallBack(d, nodeId, 0x6060, 0x00, 1, 0,
//					&Homing_mode, CheckSDOAndContinue, 0);
//		}
//			break;
//
////		case 43: { /*Set Homing_Speed_Fast 6099 01*/
////			UNS32 Homing_Speed_Fast = 0x00161a80;
////			writeNetworkDictCallBack(d, nodeId, 0x6099, 0x01, 4, 0,
////					&Homing_Speed_Fast, CheckSDOAndContinue, 0);
////		}
////			break;
////
////		case 44: { /*Set Homing_Speed_Slow 6099 02*/
////			UNS32 Homing_Speed_Slow = 0x00061a80;
////			writeNetworkDictCallBack(d, nodeId, 0x6099, 0x02, 4, 0,
////					&Homing_Speed_Slow, CheckSDOAndContinue, 0);
////		}
////			break;
////
////		case 45: { /*Set Homing_Acceleration 609A*/
////			UNS32 Homing_Acceleration = 0x0000FFA0;
////			writeNetworkDictCallBack(d, nodeId, 0x609A, 0x00, 4, 0,
////					&Homing_Acceleration, CheckSDOAndContinue, 0);
////		}
////			break;
//
//		case 30: { /*Set homing_position_method 00:Home is Current Position;*/
//			UNS8 homing_position_method = 0x00;
//			writeNetworkDictCallBack(d, nodeId, 0x6098, 0x00, 1, 0,
//					&homing_position_method, CheckSDOAndContinue, 0);
//		}
//			break;
//
////		case 47: { /*Set homing_offset 607C*/
////			UNS32 Homing_offset = 0x000F86A0;
////			writeNetworkDictCallBack(d, nodeId, 0x607C, 0x00, 4, 0,
////					&Homing_offset, CheckSDOAndContinue, 0);
////		}
//
//			/* Ask slave node to go in operational mode */
//		case 31: {
//				/*Set homing_position_method 00:Home is Current Position;*/
//				StartNode(nodeId);
//		}
//			break;
//
//		case 32: { /*star Homing*/
//			UNS8 homing_start = 0x005F;
//			writeNetworkDictCallBack(d, nodeId, 0x6040, 0x00, 1, 0,
//					&homing_start, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 33: { /*star Homing*/
//			UNS8 homing_start = 0x005F;
//			writeNetworkDictCallBack(d, nodeId, 0x6040, 0x00, 1, 0,
//					&homing_start, CheckSDOAndContinue, 0);
//		}
//			break;
//
//		case 34: { /*enable motor with absolute/change imeadiate/halt/bit4=0 */
//			UNS16 enable = 0x012F;
//			writeNetworkDictCallBack(d, nodeId, 0x6040, 0x00, 2, 0, &enable,
//					CheckSDOAndContinue, 0);
//		}
//			break;
			//位置模式
			//Motion Profile梯形
			//homing当前位置
			//举升速度
			//举升加速度 减速度
			//enable
			/*Position mode parameter*/
		case 29: { /*Set Slave into position Mode*/
			UNS8 position_mode = 0x01;
			writeNetworkDictCallBack(d, nodeId, 0x6060, 0x00, 1, 0,
					&position_mode, CheckSDOAndContinue, 0);
		}
			break;

		case 30: { /*Set Slave Motion profile type--梯形*/
			UNS16 Motion_Profile_type = 0x0000;
			writeNetworkDictCallBack(d, nodeId, 0x6086, 0x00, 2, 0,
					&Motion_Profile_type, CheckSDOAndContinue, 0);
		}
			break;

		case 31: { /*Set homing position 00：Home is Current Position*/
			UNS8 home_current_position = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x6098, 0x00, 1, 0,
					&home_current_position, CheckSDOAndContinue, 0);
		}
			break;

		case 32: { /* Set Slave profile Velocity 6081
		 电机转速=2600rpm==43.3转/s =7099733.333 counts/s=0x006C5555
		 上电后以正常速度的 1/52== 50rpm==0x006C5555/52== */
			UNS32 LiftSpeed = 0x006C5555;
			UNS32 LiftInitSpeed=(LiftSpeed/52);
			writeNetworkDictCallBack(d, nodeId, 0x6081, 0x00, 4, 0, &LiftInitSpeed,
					CheckSDOAndContinue, 0);
		}
			break;

		case 33: { /*Set Slave Acceleration = 举升速度的1/2 = 0x00008AAB*/
			UNS32 LiftAcc = 0x00008AAB;
			UNS32 LiftAcc42=LiftAcc*2;
			writeNetworkDictCallBack(d, nodeId, 0x6083, 0x00, 4, 0, &LiftAcc42,
					CheckSDOAndContinue, 0);
		}
			break;

		case 34: { /*Set Slave Deceleration 与加速度相同=0x00008AAB*/
			UNS32 LiftDec = 0x00008AAB;
			UNS32 LiftDec42=LiftDec*2;
			writeNetworkDictCallBack(d, nodeId, 0x6084, 0x00, 4, 0, &LiftDec42,
					CheckSDOAndContinue, 0);
		}
			break;

			/*位置模式使用：绝对值（bit6=0）+ change immediate（bit5=1）
			 * 使能	:012F  halt/absolute/change immediate/bit4=0/
			 *启动 	:003F  	  absolute/change immediate/bit4=1
			 *启动后bit4 1-0	:002F 	  absolute/change immediate/bit4=0
			 *停止  :012F  halt/absolute/change immediate/bit4=0/
			 */
		case 35: { /*enable motor with absolute/change imeadiate/halt/bit4=0 */
			UNS16 enable = 0x012F;
//			UNS16 enable = 0x0127;//短时能
			writeNetworkDictCallBack(d, nodeId, 0x6040, 0x00, 2, 0, &enable,
					CheckSDOAndContinue, 0);
		}
			break;

			/**************************设置从站心跳报文**************************/

		case 36: {/*Third step: configure Slave Heart-beat interval*/
			writeNetworkDictCallBack(d, /*CO_Data* d*/
			nodeId, /*UNS8 nodeId*/
			0x1017, /*UNS16 index*/
			0x00, /*UNS16 index*/
			2, /*UNS8 count*/
			0, /*UNS8 dataType*/
			&heartdata,/*data must be a index*/
			CheckSDOAndContinue, /*SDOCallback_t Callback*/
			0); /* use block mode */
		}
			break;

			/****************************** 启动从节点 *******************************/
		case 37: {
			/* Put the master in operational mode */
			setState(CANOpenShellOD_Data, Operational);

			/* Ask slave node to go in operational mode */
			StartNode(nodeId);

			/* reset counter in case the slave boot-up again */
//			init_step9 = 0;//不需要置0，避免电机重新上电从新配置。当电机重新上电此时init_step1==37，进入配置后++init_step1 ==38，并没有case36则跳过。
			num_slavebootup++;
			break;

		}
		}

	}

	/****************************************PGV***********************************/
	if (nodeId == 0x0A) {
		switch (++init_stepA) {

		case 1: { /*disable Slave's TPDO1 */
			UNS32 TPDO2_COBId = 0x80000180 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1800, 0x01, 4, 0,
					&TPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 2: { /*disable Slave's TPDO2 */
			UNS32 TPDO2_COBId = 0x80000280 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1801, 0x01, 4, 0,
					&TPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 3: { /*disable Slave's TPDO4 */
			UNS32 TPDO2_COBId = 0x80000480 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1803, 0x01, 4, 0,
					&TPDO2_COBId, CheckSDOAndContinue, 0);
		}
			break;

//		case 10: /*Third step: configure Slave Heart-beat interval*/
//			writeNetworkDictCallBack(d, /*CO_Data* d*/
//			nodeId, /*UNS8 nodeId*/
//			0x1017, /*UNS16 index*/
//			0x00, /*UNS16 index*/
//			2, /*UNS8 count*/
//			0, /*UNS8 dataType*/
//			&heartdata,/*data must be a index*/
//			CheckSDOAndContinue, /*SDOCallback_t Callback*/
//			0); /* use block mode */
//			break;

			/*启动PGV站*/
		case 4: {
			/* Ask slave node to go in operational mode */
			StartNode(nodeId);

			/* reset counter in case the slave boot-up again */
//			init_stepA = 0;//不需要置0，避免电机重新上电从新配置。当电机重新上电此时init_step1==11，进入配置后++init_step1 ==12，并没有case36则跳过。
			num_slavebootup++;
			break;
		}

		}
	} //end with configure 0A


	/*IO站：nodeID:0F
	 *每8bit=最小单位=1组HEX
	 *
	 *TPDO:输入模块 4*16DI-N(磁导航，Lo触发) + 2*16DI-P(避障输入+遥控器,H触发)
	 *		4*16DI-N(磁导航，Lo触发)(6000 01-08)： 180+nodeID  异步FF(默认，无需配置)
	 *		2*16DI-P(限位开关输入+避障输入/遥控器,H触发)(6000 09-0C): 190+nodeID 异步FF
	 *
	 *RPDO:输出模块 1*16DO-P(避障区域选择) + 1*8DO-P(继电器输出)
	 *		1*16DO-P(避障区域选择)：200+nodeID  异步FF (默认，无需配置)
	 *		1*8DO-P	(继电器输出)	：200+nodeID  异步FF	(默认，无需配置)
	 *		 */

	if (nodeId == 0x0f) {
		switch (++init_stepF) {

		/**************************Set I/O TPDO **************************************/
		/* To configure last 32bits(begin from module 5)
		 * use TPDO 6 : 1805-1A05------>6000 09-0C 08 (09-0C: 4*8=32BIT)
		 * cobID:			190+NODEID
		 * Transmit type: 	FF
		 * Index        :	6000 ReadInput8Bit
		 * SubIndex 	:	09-0c
		 * Type			:	unsigned 8 = 8bit = 1byte = 0X00
		 ********************************************************/

		case 1: { /*Reset I/O TPDO 6 Mapping number */
			UNS8 TPDO3_MapNum = 0x00;
			writeNetworkDictCallBack(d, nodeId, 0x1A05, 0x00, 1, 0,
					&TPDO3_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 2: { /*Maping Slave's TPDO 6 with ReadInput8Bit in 6000 09 */
			UNS32 ReadInput8Bit = 0x60000908; //bit 65=8*8+1----bit 72=65+8
			writeNetworkDictCallBack(d, nodeId, 0x1A05, 0x01, 4, 0,
					&ReadInput8Bit, CheckSDOAndContinue, 0);
		}
			break;

		case 3: { /*Maping Slave's TPDO 6 in ReadInput8Bit in 6000 0A */
			UNS32 ReadInput8Bit = 0x60000A08;
			writeNetworkDictCallBack(d, nodeId, 0x1A05, 0x02, 4, 0,
					&ReadInput8Bit, CheckSDOAndContinue, 0);
		}
			break;

		case 4: { /*Maping Slave's TPDO 6 in ReadInput8Bit in 6000 0B */
			UNS32 ReadInput8Bit = 0x60000B08;
			writeNetworkDictCallBack(d, nodeId, 0x1A05, 0x03, 4, 0,
					&ReadInput8Bit, CheckSDOAndContinue, 0);
		}
			break;

		case 5: { /*Maping Slave's TPDO 6 in ReadInput8Bit in 6000 0C */
			UNS32 ReadInput8Bit = 0x60000C08;
			writeNetworkDictCallBack(d, nodeId, 0x1A05, 0x04, 4, 0,
					&ReadInput8Bit, CheckSDOAndContinue, 0);
		}
			break;

		case 6: { /*Set Slave's RPDO 6 Mapping number = 4 */
			UNS8 TPDO6_MapNum = 0x04;
			writeNetworkDictCallBack(d, nodeId, 0x1A05, 0x00, 1, 0,
					&TPDO6_MapNum, CheckSDOAndContinue, 0);
		}
			break;

		case 7: { /*Set TPDO6 in 01 Sync */
			writeNetworkDictCallBack(d, nodeId, 0x1805, 0x02, 1, 0,
					&Transmission_TPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 8: { /*Set TPDO1 in 01 Sync  1800-1A00  maping 01-08*/
			writeNetworkDictCallBack(d, nodeId, 0x1800, 0x02, 1, 0,
					&Transmission_TPDO, CheckSDOAndContinue, 0);
		}
			break;

		case 9: { /*Enable Slave's TPDO6 */
			UNS32 TPDO6_COBId = 0x00000190 + nodeId;
			writeNetworkDictCallBack(d, nodeId, 0x1805, 0x01, 4, 0,
					&TPDO6_COBId, CheckSDOAndContinue, 0);
		}
			break;

		case 10: /*Third step: configure Slave Heart-beat interval*/
			writeNetworkDictCallBack(d, /*CO_Data* d*/
			nodeId, /*UNS8 nodeId*/
			0x1017, /*UNS16 index*/
			0x00, /*UNS16 index*/
			2, /*UNS8 count*/
			0, /*UNS8 dataType*/
			&heartdata,/*data must be a index*/
			CheckSDOAndContinue, /*SDOCallback_t Callback*/
			0); /* use block mode */
			break;

			/*启动IO站*/
		case 11: {
			/* Put the master in operational mode */
			setState(CANOpenShellOD_Data, Operational);

			/* Ask slave node to go in operational mode */
			StartNode(nodeId);

			/* reset counter in case the slave boot-up again */
//			init_stepF = 0;//不需要置0，避免电机重新上电从新配置。当电机重新上电此时init_step1==11，进入配置后++init_step1 ==12，并没有case36则跳过。
			num_slavebootup++;
			break;
		}

		}
	} //end with configure 0F

	//	if (num_slavebootup==1){  //the number of motor has been booted up
	//		self_test(CANOpenShellOD_Data);
	//
	//	}

//	if (num_slavebootup == 1) {
//		sleep(2);			//important delay for self_test
//		boot_success(1);
//	}
}

int UpMotorZero(CO_Data* d, UNS8 nodeId)
{
	switch (++inti_stepUpMotor) {
	case 1: { /* Set Slave profile Velocity 6081
		 	 电机转速=2600rpm==43.3转/s =7099733.333 counts/s=0x006C5555
		 	 上电后以正常速度的 1/52== 50rpm==0x006C5555/52== */
			UNS32 LiftSpeed = 0x006C5555;//2600
			UNS32 LiftSLow=(LiftSpeed/2);//1300rpm
			writeNetworkDictCallBack(d, nodeId, 0x6081, 0x00, 4, 0,
					&LiftSLow,CheckUpMotorNomalSpeed, 0);
			}
			break;
	}
	printf("举升电机重设正常速度成功\n");
	Log("举升电机重设正常速度成功");
	return 0;
}

