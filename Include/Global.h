#ifndef _GLOBAL_H
#define _GLOBAL_H

typedef unsigned char uchar;
typedef unsigned int uint;
typedef unsigned long ulong;

union LongData
{
	long longD;
	int		intD[2];
	uchar Byte[4];
};

union uLongData
{
	ulong ulongD;
	uint uintD[2];
	uchar Byte[4];
};

union intData
{
	int intD;
	uchar Byte[2];
};

union uintData
{
	uint uintD;
	uchar Byte[2];
};

//上位机电机序号
//#define		MOTOR_Z			6		//Z
//#define		MOTOR_Y			5		//y
//#define		MOTOR_X			4		//x
//#define		MOTOR_Mix		0		//mix
//#define		MOTOR_ARM_C		1		//sample C
//#define		MOTOR_ARM_Z		2		//sample Z
//#define		MOTOR_PUMP		3		//pump
//#define		MOTOR_PLATE		7		//plate

#define		MOTOR_Z				0x00		//Z
#define		MOTOR_Y				0x01		//y
#define		MOTOR_X				0x02		//x
#define		MOTOR_Mix			0x03		//mix
#define		MOTOR_ARM_C		0x04		//sample C
#define		MOTOR_ARM_Z		0x05		//sample Z
#define		MOTOR_PUMP		0x06		//pump
#define		MOTOR_PLATE		0x07		//plate

#define 	ARMMIX_OTHER 		0x10
#define 	ARMSA_OTHER 		0x11
#define 	PLATE_OTHER 		0x12
#define 	MICROXYZ_OTHER 	0x13

#define		LIANGTONG3		0x00		//mix water
#define		QIBENG1				0x01
#define		SHUIBENG1			0x02
#define		SANTONG2			0x03
#define		SANTONG3			0x04
#define		LIANGTONG2		0x05
#define		LIANGTONG1		0x06
#define		SHUIBENG2			0x07
#define		LIANGTONG4		0x08		//mix gas

#define PART_PLATE		0
#define PART_ARM1			1
#define PART_ARM2			2
#define PART_ARM3			3		
#define PART_XYZ			4		
#define PART_IO				5

#define	TOTAL_SLAVER	5

struct	PartPara
{
	uchar	Type;
	uchar	Errcode;
	uchar	TotalAxis;
	uchar	StartAxisNum;
	uchar	TotalX;
	uchar	TotalY;
	uchar	RunStep;
};

/*轴接口参数*/
struct	AxisPara
{
	union	intData 	 refpos;
	union	intData 	 minpos;
	union	intData 	 maxpos;
	union	intData 	 workpos0;
	union	intData 	 workpos1;
	union	intData 	 workpos2;
	union intData 	 workpos3;
	
	uchar	 MannualSpeed;
	uchar	 AutoSpeed;
	uchar	 AccConP;
	uchar	 AccSpeed;
	uchar	 DecConP;
	uchar	 DecSpeed;
	uchar	 BitPara;	//bit0=1: (MovDir = 1) + dir of returning reference point;
};

/*轴控制变量*/
struct	AxisVar
{
	uchar			Xin;		//bit0: MO;		bit1: Protect;		bit2: RefDone;		bit3: Ref
	uchar			Yout;		//bit0: Tq1;	bit1: Tq2; 				bit2: DirRev; 		bit3: DirCmd; 		bit4: nStandBy; 		bit5: DRVRst; 		bit6: Positioning; 		bit7 =1;
	uchar			Yout2;	//bit0：RefEn_MCU
	uchar			Status;	//bit4: ParameterRS_OK
	uchar			RunStep;	
	uchar			Acon;
	uchar			Dcon;	
	uchar			Speed;
	uchar			TargetV;
	uchar			Errcode;
	union intData	Pos;				//=CurPos
	union intData	TargetPos;	//=TargPos
	uint			RunCnt;
};

/*Global Address*/
#define		gPLSCMD				0x00
#define		gSTATECMD			0x01
#define		PLC_I					0x10
#define		PLC_OL				0x20
#define		PLC_OH				0x21

/*Base Address*/
#define		ADDR_AXIS0			0x30
#define		ADDR_AXIS1			0x40
#define		ADDR_AXIS2			0x50
#define		ADDR_AXIS3			0x60
#define		ADDR_AXIS4			0x70
#define		ADDR_AXIS5			0x80
#define		ADDR_AXIS6			0x90
#define		ADDR_AXIS7			0xa0

/*Axis read*/
#define		PLS_CNT_L 			0x00
#define		PLS_CNT_H 			0x01
#define		DRV_STATE 			0x02

/*Axis write*/
#define		STATECMD				0x00
#define		PLSCMD					0x01
#define		SPEEDCMD				0x02
#define		YOUT2						0x03
#define		TARGETPLS_L			0x04
#define		TARGETPLS_H			0x05
#define		REFPOS_L				0x06
#define		REFPOS_H				0x07

#define 	MOTORNUM 	8
#define 	RELAYNUM 	16		//7个预留
#define 	CUPNUM 		24

/*universal global variable*/
extern	bit			ContinueRun;
extern	bit			ContinueDir;
extern	uchar  	gBuf[];
extern	uchar 	data 	gT1msCnt,gT1msCntOld;
extern	uchar 	data	Errcode,ErrcodeB;
extern	uint		data	testcnt;

extern	uchar * data gPtr;
extern	union uLongData data uLi;			
#define ui0 uLi.Byte[0]
#define ui1 uLi.Byte[1]
#define ui2 uLi.Byte[2]
#define ui3 uLi.Byte[3]
#define uii0 uLi.uintD[0]
#define uii1 uLi.uintD[1]

extern	union LongData data  sLi;
#define si0 sLi.Byte[0]
#define si1 sLi.Byte[1]
#define si2 sLi.Byte[2]
#define si3 sLi.Byte[3]
#define sii0 sLi.intD[0]
#define sii1 sLi.intD[1]

extern	uchar	bdata	gByteM;						
extern	bit			m0;
extern	bit			m1;
extern	bit			m2;
extern	bit			m3;
extern	bit			m4;
extern	bit			m5;
extern	bit			m6;
extern	bit			m7;

/*IO variable*/
extern 	uchar bdata XX;
extern  uchar	bdata	YY0;
extern  bit	Relay1;
extern  bit	Relay2;
extern  bit	Relay3;
extern  bit	Relay4;
extern  bit	Relay5;
extern  bit	Relay6;
extern  bit	Relay7;
extern  bit	Relay8;
extern  uchar	bdata	YY1;
extern  bit	Relay9;
extern  bit	Relay10;
extern  bit	Relay11;
extern  bit	Relay12;
extern  bit	Relay13;
extern  bit	Relay14;
extern  bit	Relay15;
extern  bit	Relay16;
extern  uchar 	data 	relayTdelay[RELAYNUM];
extern	uchar		data 	RelayOffCnt[RELAYNUM];

/*part variable*/
extern	struct	PartPara Part[];

/*motor variable*/
extern  struct	AxisPara	AxisP[MOTORNUM];
extern  struct	AxisVar		AxisV[MOTORNUM];
extern	uchar	data	gPartSel;
extern	union	uintData data RemDis;
extern	union	intData  data TargPos,CurPos;
extern	uchar 	data CurV;
extern	uchar 	bdata KeyFromHost;
extern	bit		K_CW;
extern	bit		K_CCW;
extern	bit		K_F;

extern	uchar bdata MotorY,MotorX;
extern	bit		MO;
extern	bit		Protect;
extern	bit		RefDone;
extern	bit		Ref;

extern	bit		Torque1;
extern	bit		Torque2;
extern	bit		DirRev;
extern	bit		MovDir;

extern	bit		DecFlag;
extern	bit		Positioning;
extern	bit		EMS_Stop;
extern	bit		Y_CW;
extern	bit		Y_CCW;
extern	bit		Y_F;

extern	bit		testFlag;
extern	uchar data 	testStep;

extern 	uchar IO_XX[2];	//数组长度要与struct PartPara Part[TOTAL_SLAVER]对应
extern 	uchar IO_YY[2];

void  MotorIO_Init();
void  Variable_Init();
void	Parameter_Init();
void	UserTimer();

#endif




