#include	"Git_Repository_Fecal\Include\Global.h"
#include	"Git_Repository_Fecal\Include\RS422.h"
#include  "Git_Repository_Fecal\Include\IO_Expand.h"
#include	"Git_Repository_Fecal\Include\Motor.h"
#include	<c8051F340.h>

/*universal global variable*/
uchar xdata gBuf[256];
uchar data 	gT1msCnt,gT1msCntOld;	//T =1ms
uchar data 	Errcode,ErrcodeB;
uint  data	testcnt;

uchar * data gPtr;						//global pointer
union uLongData data  uLi;		//temp variable
union  LongData data  sLi;		//temp variable
uchar	bdata	gByteM;						//bit variable
sbit m0 = gByteM^0;
sbit m1 = gByteM^1;
sbit m2 = gByteM^2;
sbit m3 = gByteM^3;
sbit m4 = gByteM^4;
sbit m5 = gByteM^5;
sbit m6 = gByteM^6;
sbit m7 = gByteM^7;

/*IO variable*/
uchar bdata 	XX;
uchar	bdata		YY0;
sbit	Relay1 = YY0^0;
sbit	Relay2 = YY0^1;
sbit	Relay3 = YY0^2;
sbit	Relay4 = YY0^3;
sbit	Relay5 = YY0^4;
sbit	Relay6 = YY0^5;
sbit	Relay7 = YY0^6;
sbit	Relay8 = YY0^7;
uchar	bdata		YY1;
sbit	Relay9 = YY1^0;
sbit	Relay10 = YY1^1;
sbit	Relay11 = YY1^2;
sbit	Relay12 = YY1^3;
sbit	Relay13 = YY1^4;
sbit	Relay14 = YY1^5;
sbit	Relay15 = YY1^6;
sbit	Relay16 = YY1^7;
uchar 	data 	relayTdelay[RELAYNUM];
uchar		data 	RelayOffCnt[RELAYNUM];

/*part variable*/
struct	PartPara Part[TOTAL_SLAVER]	=
{
	{	PART_ARM1,	0,	1,	MOTOR_Mix,		0,	0,	0	},
	{	PART_ARM3,	0,	3,	MOTOR_ARM_C,	0,	0,	0	},
	{	PART_PLATE,	0,	1,	MOTOR_PLATE,	0,	0,	0	},
	{	PART_XYZ,	0,	3,	MOTOR_Z,		0,	0,	0	},
	{	PART_IO,	0,	0,	0,				2,	2,  0	}
};

/*motor variable*/
struct	AxisPara	AxisP[MOTORNUM];
struct	AxisVar		AxisV[MOTORNUM];
uchar	data	gPartSel;
uchar data 	CurV;
union	uintData data RemDis;
union	intData  data TargPos,CurPos;
uchar bdata KeyFromHost;
sbit		K_CW =KeyFromHost^0;
sbit		K_CCW =KeyFromHost^1;
sbit		K_F =KeyFromHost^2;

uchar bdata MotorY,MotorX;
sbit		MO = MotorX^0;
sbit		Protect = MotorX^1;
sbit		RefDone = MotorX^2;
sbit		Ref = MotorX^3;

sbit		Torque1 = MotorY^0;
sbit		Torque2 = MotorY^1;
sbit		DirRev = MotorY^2;
sbit		MovDir = MotorY^3;
sbit		Positioning = MotorY^6;	//add by tt
sbit		DecFlag = MotorY^7;	//上位机不可见

bit		EMS_Stop;
bit		ContinueRun;
bit		ContinueDir;
bit		Y_CW;
bit		Y_CCW;
bit		Y_F;

bit		testFlag;
uchar data testStep;
uchar IO_XX[2]; //数组长度要与struct	PartPara Part[TOTAL_SLAVER]对应，每一个子站所包含的IO分量
uchar IO_YY[2];

void  MotorIO_Init()
{
	/*the motor var*/
	AxisV[0].Yout =0x14;
	AxisV[1].Yout =0x14;
	AxisV[2].Yout =0x10;
	AxisV[3].Yout =0x14;
	AxisV[4].Yout =0x10;
	AxisV[5].Yout =0x14;	
	AxisV[6].Yout =0x14;
	AxisV[7].Yout =0x14;
	for(ui0 = 0;ui0<MOTORNUM;ui0++)
	{
		AxisV[ui0].Xin =0x00;
		AxisV[ui0].RunStep =0;
		AxisV[ui0].Speed =0x00;
		AxisV[ui0].TargetV =0x00;
		AxisV[ui0].Acon =1;
		AxisV[ui0].Dcon =1;
		AxisV[ui0].Pos.intD =0x0000;
		AxisV[ui0].TargetPos.intD =0x0000;
		AxisV[ui0].RunCnt =0;
		AxisV[ui0].Status =0;
		
		gwrite_Port( ADDR_AXIS0 +ui0*16 +STATECMD , 0);		
		gwrite_Port( ADDR_AXIS0 +ui0*16 +SPEEDCMD , 0);
	}
	
	/*the output var*/
	for(ui0 = 0;ui0 <RELAYNUM;ui0++)
	{
		relayTdelay[ui0] =20;
		RelayOffCnt[ui0] =0;
	}
	tmpRelay[0] = 0x00;
	tmpRelay[1] = 0x00;
	YY0 = 0x00;
	YY1 = 0x00;
	gwrite_Port(PLC_OL, ~YY0);	
	gwrite_Port(PLC_OH, ~YY1);
	IO_YY[0] =YY0;
	IO_YY[1] =YY1;
}


void  Variable_Init()
{
	gT1msCnt =0;
	gT1msCntOld =0;
	Errcode =0;
	gPtr =0x00;
//	testcnt =0x1388;
	gPartSel =0;
	RemDis.uintD =0x0000;
	TargPos.intD =0x0000;
	CurPos.intD =0x0000;
	CurV =0;
//	Y_CW =0;
//	Y_CCW =0;
//	Y_F =0;
	EMS_Stop =0;
	DecFlag =0;
	StepRunFlag =0;
//	testFlag =0;
//	testStep =0x00;
	mixFlag =0;
	sampleFlag =0;
	cleanFlag =0;
//	default:ContinueRun
	ContinueRun = 1;
	StepRunFlag =0;
}

void	Parameter_Init()
{
	/*the motor para*/
	for(ui0 = 0;ui0<MOTORNUM;ui0++)
	{
		AxisP[ui0].refpos.intD =0x0000;
		AxisP[ui0].minpos.intD =0x0000;
		AxisP[ui0].maxpos.intD =0x2ee0;
		AxisP[ui0].workpos0.intD =0x0100;
		AxisP[ui0].workpos1.intD =0x2710;
		AxisP[ui0].workpos2.intD =0x0100;
		AxisP[ui0].workpos3.intD =0x2710;		
		AxisP[ui0].MannualSpeed =0x1b;
		AxisP[ui0].AutoSpeed =0x1b;
		AxisP[ui0].AccConP =1;
		AxisP[ui0].AccSpeed =1;
		AxisP[ui0].DecConP =1;
		AxisP[ui0].DecSpeed =1;
		AxisP[ui0].BitPara =0;
	}
	AxisP[MOTOR_Z].BitPara =1;
	AxisP[MOTOR_Mix].BitPara =1;
	AxisP[MOTOR_ARM_Z].BitPara =1;
	/*Global reset signal*/
	gwrite_Port( gPLSCMD , 0x80);

	for(ui0 =0;ui0 <MOTORNUM;ui0 ++)
	{
		/*ZeroSpeed*/
		gwrite_Port( ADDR_AXIS0 + ui0*16 +SPEEDCMD , 0x00);
		/*Tq1 & Tq2 & DirRev & DirCmd & nStandby & DRVrst*/
		gwrite_Port( ADDR_AXIS0 + ui0*16 +STATECMD , AxisV[ui0].Yout);
		/*PlsCntClr & StepPls & RefDoneClr & RePositioning*/
		gwrite_Port( ADDR_AXIS0 + ui0*16 +PLSCMD , 0x01);
		gwrite_Port( ADDR_AXIS0 + ui0*16 +PLSCMD , 0x00);
		/*set the Ref Position*/
		gwrite_Port( ADDR_AXIS0 + ui0*16 +REFPOS_L, AxisP[ui0].refpos.Byte[1]);
		gwrite_Port( ADDR_AXIS0 + ui0*16 +REFPOS_H, AxisP[ui0].refpos.Byte[0]);
	}
	/*set the YOUT2*/
	gwrite_Port( ADDR_AXIS0 +YOUT2 , 0x00);
	gwrite_Port( ADDR_AXIS1 +YOUT2 , 0x00);
	gwrite_Port( ADDR_AXIS2 +YOUT2 , 0x00);
	gwrite_Port( ADDR_AXIS3 +YOUT2 , 0x00);
	gwrite_Port( ADDR_AXIS4 +YOUT2 , 0x00);
	gwrite_Port( ADDR_AXIS5 +YOUT2 , 0x00);
	gwrite_Port( ADDR_AXIS6 +YOUT2 , 0x00);
	gwrite_Port( ADDR_AXIS7 +YOUT2 , 0x00);	
}

void UserTimer()
{
	if((gT1msCnt %100) ==0)
	{
		for(ui0 =0;ui0 <RELAYNUM;ui0 ++)
		{
			 if(RelayOffCnt[ui0] !=0x00)
			 {
					RelayOffCnt[ui0] --;
					if(RelayOffCnt[ui0] ==0x00)
					{
						if(ui0 <8)
						{
							ui1 =1;
							ui1 <<=ui0;
							ui2 =~ui1;
							if(ui0 ==QIBENG1 || ui0 ==SHUIBENG1 || ui0 ==SHUIBENG2)
							{
								YY0 &=ui2;	
							}							
						}
						else
						{
							//YY1 &=ui2;
						}
					}
			 }
		}			 
	}
}

















