#include<stdio.h>
#include"CANOpenShellMasterOD.h"
//初始化和自检相关函数声明
int InitRov();//初始化程序
int SelfTest();//自检程序
int SenserInit();//总线初始化
int PowerManageInit();//电源管理初始化
int CanInit();//Can总线初始化
int LimitSwitchInit();//限位开关初始化
void InitStruct();
int CLoseGreen();
int InitZeroPosition();
int GetUpMotorInitPosition();
int UpMotorGoZero();
int CloseAndOpenK2();
int ConfigMotor();
int ConfigPGV();


extern INTEGER32 UpMotorInitPosition;
