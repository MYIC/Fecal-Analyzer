#ifndef	_MOTOR_H
#define	_MOTOR_H

#include	"Git_Repository_Fecal\Include\Global.h"

#define		ADCON_LEVEL		20

//与细分方式无关，与FPGA各脉冲发生模块的时钟频率相关
#define		KsMap(mNo)	{												\		
		Ks =KsTab_25M[AxisP[mNo].DecConP -1];	  	\
}

#define		KvMap(axis)	{							\
		if(axis ==MOTOR_Z)							\
			Kv =4;												\
		else if(axis ==MOTOR_PUMP)			\
			Kv =2;												\
		else														\
			Kv =1;												\
}

extern float code 	KsTab_25M[];
extern float data	Ks;
extern unsigned char  data    Kv;
extern unsigned char	data 		tmpRelay[];	
extern 	bit	 mixFlag;
extern 	bit	 sampleFlag;
extern 	bit	 cleanFlag;	

void	ReadAxis();
void	IOCtrl();
//void	PositionCtrl();
//void	SpeedCtrl();
void	MotorRst();
void	MixCtrl();
void	SampleCtrl();
void 	CleanCtrl();
void	PositionCtrlInit(int targposition,unsigned char mNo);
void	SpeedCtrlInit(bit K_CW, bit K_CCW,bit K_F);
unsigned char ChkSpeed(unsigned char axis,unsigned char targetV);
void SpeedModeRun(unsigned char axis,unsigned char targetV);
void PositioningStart(unsigned char axis);


#endif