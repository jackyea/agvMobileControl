
#include "CanOpenShell.h"
#include "CANOpenShellMasterOD.h"


/*电机位置回调*/
UNS32 Call_R1_Position(CO_Data* d,const indextable * unsused_indextable, UNS8 unsused_bSubindex) {
	eprintf("R1_position: %x\n", R1_position);
	fflush(stdout);
	return 0;
}

UNS32 Call_R2_Position(CO_Data* d, const indextable * unsused_indextable,
		UNS8 unsused_bSubindex) {
	eprintf("R2_position: %x\n", R2_position);
	fflush(stdout);
	return 0;
}

UNS32 Call_R3_Position(CO_Data* d, const indextable * unsused_indextable,
		UNS8 unsused_bSubindex) {
	eprintf("R3_position: %x\n", R3_position);
	fflush(stdout);
	return 0;
}

UNS32 Call_R4_Position(CO_Data* d, const indextable * unsused_indextable,
		UNS8 unsused_bSubindex) {
	eprintf("R4_position: %x\n", R4_position);
	fflush(stdout);
	return 0;
}

UNS32 Call_R5_Position(CO_Data* d, const indextable * unsused_indextable,
		UNS8 unsused_bSubindex) {
	eprintf("R5_position: %x\n", R5_position);
	fflush(stdout);
	return 0;
}

UNS32 Call_R6_Position(CO_Data* d, const indextable * unsused_indextable,
		UNS8 unsused_bSubindex) {
	eprintf("R6_position: %x\n", R6_position);
	fflush(stdout);
	return 0;
}

UNS32 Call_R7_Position(CO_Data* d, const indextable * unsused_indextable,
		UNS8 unsused_bSubindex) {
	eprintf("R7_position: %x\n", R7_position);
	fflush(stdout);
	return 0;
}

UNS32 Call_R8_Position(CO_Data* d, const indextable * unsused_indextable,
		UNS8 unsused_bSubindex) {
	eprintf("R8_position: %x\n", R8_position);
	fflush(stdout);
	return 0;
}

UNS32 Call_R9_Position(CO_Data* d, const indextable * unsused_indextable,
		UNS8 unsused_bSubindex) {
	eprintf("R9_position: %x\n", R9_position);
	fflush(stdout);
	return 0;
}

/*电机速度回调*/
UNS32 Call_R1_Speed(CO_Data* d,const indextable * unsused_indextable,
		UNS8 unsused_bSubindex) {
	eprintf("R1_speed: %x\n", R1_speed);
	fflush(stdout);
	return 0;
}

UNS32 Call_R2_Speed(CO_Data* d, const indextable * unsused_indextable,
		UNS8 unsused_bSubindex) {
	eprintf("R2_speed: %x\n", R2_speed);
	fflush(stdout);
	return 0;
}

UNS32 Call_R3_Speed(CO_Data* d, const indextable * unsused_indextable,
		UNS8 unsused_bSubindex) {
	eprintf("R3_speed: %x\n", R3_speed);
	fflush(stdout);
	return 0;
}

UNS32 Call_R4_Speed(CO_Data* d, const indextable * unsused_indextable,
		UNS8 unsused_bSubindex) {
	eprintf("R4_speed: %x\n", R4_speed);
	fflush(stdout);
	return 0;
}

UNS32 Call_R5_Speed(CO_Data* d, const indextable * unsused_indextable,
		UNS8 unsused_bSubindex) {
	eprintf("R5_speed: %x\n", R5_speed);
	fflush(stdout);
	return 0;
}

UNS32 Call_R6_Speed(CO_Data* d, const indextable * unsused_indextable,
		UNS8 unsused_bSubindex) {
	eprintf("R6_speed: %x\n", R6_speed);
	fflush(stdout);
	return 0;
}

UNS32 Call_R7_Speed(CO_Data* d, const indextable * unsused_indextable,
		UNS8 unsused_bSubindex) {
	eprintf("R7_speed: %x\n", R7_speed);
	fflush(stdout);
	return 0;
}

UNS32 Call_R8_Speed(CO_Data* d, const indextable * unsused_indextable,
		UNS8 unsused_bSubindex) {
	eprintf("R8_speed: %x\n", R8_speed);
	fflush(stdout);
	return 0;
}

UNS32 Call_R9_Speed(CO_Data* d, const indextable * unsused_indextable,
		UNS8 unsused_bSubindex) {
	eprintf("R9_speed: %x\n", R9_speed);
	fflush(stdout);
	return 0;
}

/*电机状态回调*/
//UNS32 Call_R1_status_word(CO_Data* d, const indextable * unsused_indextable,
//		UNS8 unsused_bSubindex) {
//	eprintf("R1_status_word: %x\n", Call_R1_status_word);
//	fflush(stdout);
//	return 0;
//}
//
//UNS32 Call_R2_status_word(CO_Data* d, const indextable * unsused_indextable,
//		UNS8 unsused_bSubindex) {
//	eprintf("R2_status_word: %x\n", Call_R2_status_word);
//	fflush(stdout);
//	return 0;
//}
//
//UNS32 Call_R3_status_word(CO_Data* d, const indextable * unsused_indextable,
//		UNS8 unsused_bSubindex) {
//	eprintf("R3_status_word: %x\n", Call_R3_status_word);
//	fflush(stdout);
//	return 0;
//}
//
//UNS32 Call_R4_status_word(CO_Data* d, const indextable * unsused_indextable,
//		UNS8 unsused_bSubindex) {
//	eprintf("R4_status_word: %x\n", Call_R4_status_word);
//	fflush(stdout);
//	return 0;
//}
//
//UNS32 Call_R5_status_word(CO_Data* d, const indextable * unsused_indextable,
//		UNS8 unsused_bSubindex) {
//	eprintf("R5_status_word: %x\n", Call_R5_status_word);
//	fflush(stdout);
//	return 0;
//}
//
//UNS32 Call_R6_status_word(CO_Data* d, const indextable * unsused_indextable,
//		UNS8 unsused_bSubindex) {
//	eprintf("R6_status_word: %x\n", Call_R6_status_word);
//	fflush(stdout);
//	return 0;
//}
//
//UNS32 Call_R7_status_word(CO_Data* d, const indextable * unsused_indextable,
//		UNS8 unsused_bSubindex) {
//	eprintf("R7_status_word: %x\n", Call_R7_status_word);
//	fflush(stdout);
//	return 0;
//}
//
//UNS32 Call_R8_status_word(CO_Data* d, const indextable * unsused_indextable,
//		UNS8 unsused_bSubindex) {
//	eprintf("R8_status_word: %x\n", Call_R8_status_word);
//	fflush(stdout);
//	return 0;
//}
//
//UNS32 Call_R9_status_word(CO_Data* d, const indextable * unsused_indextable,
//		UNS8 unsused_bSubindex) {
//	eprintf("R9_status_word: %x\n", Call_R9_status_word);
//	fflush(stdout);
//	return 0;
//}

/*IO站回调*/
UNS32 Call_IO_InputData(CO_Data* d, const indextable * unsused_indextable,UNS8 unsused_bSubindex)
{
	int count =0;
	/*printf will print four times in each change
	 * becasue pdo has many subindex, and each subindex need be written into OD
	 * so,callback function will be called in each written!!! so there are 4 print.
	 * */
//	eprintf("Call_IO_InputData: %x %x %x %x %x %x %x %x %x %x %x %x\n",
//			DI12_IO_Input_8bit, DI11_IO_Input_8bit, DI10_IO_Input_8bit,
//			DI9_IO_Input_8bit, DI8_IO_Input_8bit, DI7_IO_Input_8bit,
//			DI6_IO_Input_8bit, DI5_IO_Input_8bit, DI4_IO_Input_8bit,
//			DI3_IO_Input_8bit, DI2_IO_Input_8bit, DI1_IO_Input_8bit);

	eprintf("Call_IO_InputData: %x %x %x %x %x %x %x %x %x %x %x %x\n",
			IN6_Remoter_bit9_16, IN6_DLS1_4_bit1_8, IN5_Limit_Switch_bit9_10,
			IN5_Limit_Switch_bit1_8, IN4_NAVI4_bit9_16, IN4_NAVI4_bit1_8,
			IN3_NAVI3_bit9_16, IN3_NAVI3_bit1_8, IN2_NAVI2_bit9_16,
			IN2_NAVI2_bit1_8, IN1_NAVI1_bit9_16, IN1_NAVI1_bit1_8);

//	eprintf("Call_IO_InputData: %x %x \n",
//			DI10_IO_Input_8bit,DI9_IO_Input_8bit);
	fflush(stdout);
	return 0;
}

/*****************************注册回调函数******************************/
void	RegisterODandCallback(CO_Data* CANOpenShellOD_Data){

	/*注册位置回调函数*/
	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2000, 0,
			&Call_R1_Position);
	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2001, 0,
			&Call_R2_Position);
	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2002, 0,
			&Call_R3_Position);
	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2003, 0,
			&Call_R4_Position);
	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2004, 0,
			&Call_R5_Position);
	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2005, 0,
			&Call_R6_Position);
	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2006, 0,
			&Call_R7_Position);
	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2007, 0,
			&Call_R8_Position);
	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2008, 0,
			&Call_R9_Position);

	/*注册速度回调函数*/
	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2100, 0, &Call_R1_Speed);
	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2101, 0, &Call_R2_Speed);
	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2102, 0, &Call_R3_Speed);
	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2103, 0, &Call_R4_Speed);
	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2104, 0, &Call_R5_Speed);
	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2105, 0, &Call_R6_Speed);
	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2106, 0, &Call_R7_Speed);
	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2107, 0, &Call_R8_Speed);
	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2108, 0, &Call_R9_Speed);

	/*注册状态回调函数*/
//	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2010, 0,
//			&Call_R1_status_word);
//	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2011, 0,
//			&Call_R2_status_word);
//	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2012, 0,
//			&Call_R3_status_word);
//	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2013, 0,
//			&Call_R4_status_word);
//	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2014, 0,
//			&Call_R5_status_word);
//	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2015, 0,
//			&Call_R6_status_word);
//	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2016, 0,
//			&Call_R7_status_word);
//	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2017, 0,
//			&Call_R8_status_word);
//	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x2018, 0,
//			&Call_R9_status_word);

	/*注册IO站回调函数*/
	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x3001, 0x00,
			&Call_IO_InputData);

	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x3002, 0x00,
			&Call_IO_InputData);

	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x3003, 0x00,
			&Call_IO_InputData);

	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x3004, 0x00,
			&Call_IO_InputData);

	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x3005, 0x00,
			&Call_IO_InputData);

	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x3006, 0x00,
			&Call_IO_InputData);

	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x3007, 0x00,
			&Call_IO_InputData);

	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x3008, 0x00,
			&Call_IO_InputData);

	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x3009, 0x00,
			&Call_IO_InputData);

	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x300A, 0x00,
			&Call_IO_InputData);

	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x300B, 0x00,
			&Call_IO_InputData);

	RegisterSetODentryCallBack(CANOpenShellOD_Data, 0x300C, 0x00,
			&Call_IO_InputData);
}
