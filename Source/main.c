//create time: 2016/11/16
//author: Yangsen
//Function: 8-motor ctrl

#include	"Git_Repository_Fecal\Include\main.h"
#include	"Git_Repository_Fecal\Include\Mcu_Init.h"
#include	"Git_Repository_Fecal\Include\RS422.h"
#include	"Git_Repository_Fecal\Include\Motor.h"
#include	"Git_Repository_Fecal\Include\Global.h"
#include  "Git_Repository_Fecal\Include\IO_Expand.h"

void main()
{
	MCU_Init();
	MotorIO_Init();
	Variable_Init();
	Parameter_Init();
	while(1)
	{
		if(gT1msCntOld != gT1msCnt)
		{
			UserTimer();
			ReadAxis();
			
			/*Motor Speed Refresh*/
			for(ui2=0;ui2<MOTORNUM;ui2++)
			{
				if(AxisV[ui2].Yout &0x40)//位置控制模式，找到参考点后可以执行
				{
					CurV =ChkSpeed(ui2,AxisP[ui2].AutoSpeed);
					gwrite_Port(ADDR_AXIS0 + ui2*16 + STATECMD, AxisV[ui2].Yout);
					gwrite_Port(ADDR_AXIS0 + ui2*16 + SPEEDCMD, CurV);
				}
				else
				{
				  //AxisV[ui2].TargetV = PosLimit(ui2);
					SpeedModeRun(ui2,AxisV[ui2].TargetV);
				}
			}
			
			/*IO Refresh*/
			IOCtrl();
			
			if((gT1msCnt & 0x03) ==0x00)		
			{
				MotorRst();
				MixCtrl();
				SampleCtrl();
				CleanCtrl();
			}
			else if((gT1msCnt & 0x03) ==0x02)
			{
			}
			RS422_Interface();
			gT1msCntOld	=gT1msCnt;
		}
	}		
}







