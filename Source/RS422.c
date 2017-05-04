#include	"Git_Repository_Fecal\Include\RS422.h"
#include	"Git_Repository_Fecal\Include\Global.h"
#include  "Git_Repository_Fecal\Include\IO_Expand.h"
#include	"Git_Repository_Fecal\Include\MCU_Init.h"
#include	"Git_Repository_Fecal\Include\Motor.h"
#include	<c8051f340.h>

uchar test_buf[256];
uchar test_rxerr =0;

#define ID_MASK
uchar 	data 		myID;						//contorl panel ID
uchar 	data 		ReceivedID;
uint 		data  	TxLen;					//TX data length
uint 		data  	ChkSum;					//value of xor
uchar 	xdata 	rxBuffer[RXBUFLEN];		//array of TX data
uchar 	xdata 	txBuffer[TXBUFLEN];		//array of RX data
uchar 	data		RxStatus,TxStatus; 		//RS422 status while receiving/sending data
uchar		data 		RxCnt,TxCnt;		//TX/RX data number
uchar		data		BroadcastCMD;
uchar		data		Cur_CMD;
uchar		data		Rs_ID;
uchar 	xdata   RxErr,TxErr;

bit gRS422RxOK;	//RS422 receive data ok flag
uchar bdata RSstatus;				    	//RX data check 
sbit ACKflag = RSstatus^0;				//本次传输OK
sbit CurCMD_rOK	= RSstatus^1;			//若等于0；本次接收的数据无法响应，如电机序号超出范围、屏蔽广播命令等。
sbit BroadcastCMD_rOK =	RSstatus^2;	//模态标志，本次广播命令传输OK

sbit StepRunFlag = RSstatus^5;	
sbit Parameter_OK = RSstatus^6;			
sbit RSstatus_bit7 = RSstatus^7; //must =1，防止出现取值出现命令字节

sbit	RS422TxPortState = P0^3;
sbit	RS422RxPortState = P0^2;
sbit	node0 = P2^7;	//将端口寄存器声明为特殊功能位再进行使用
sbit	node1 = P2^6;
sbit	node2 = P2^5;
sbit	node3 = P2^4;
void RS422_Init (void)
{
	RS422TxPortState = 0;
	RS422RxPortState = 0;
	gByteM =0xf0;
	m3 = node0;
	m2 = node1;
	m1 = node2;
	m0 = node3;
	myID = ~gByteM;		
	RSstatus = 0x80;
}

void SaveMotorPara(unsigned char ch)
{
	gPtr =&AxisP[ch];
	for(ui1 = 3;ui1<30;ui1+=2)
	{
		*gPtr = (AtoNum(rxBuffer[ui1])<<4)+ AtoNum(rxBuffer[ui1+1]);
		//ui1+=2;
		gPtr++;
	}
	while(ui1<37)
	{
		*gPtr =rxBuffer[ui1] - 0x20;
		ui1++;
		gPtr++;
	}
	
//	AxisV[ch].Xin |=0x10;
	AxisV[ch].Status =0x10;
	
//	if(AxisV[ch].Speed ==0)	//reload the ref position
	{
//		gwrite_Port( ADDR_AXIS0 +ch*16 +REFPOS_L, AxisP[ch].refpos.Byte[1]);
//		gwrite_Port( ADDR_AXIS0 +ch*16 +REFPOS_H, AxisP[ch].refpos.Byte[0]);
//		gwrite_Port( ADDR_AXIS0 +ch*16 +REFPOS_L, 0);
//		gwrite_Port( ADDR_AXIS0 +ch*16 +REFPOS_H, 0);	
	}
}

void SaveRelayPara()
{
	gPtr = &relayTdelay[0];
	for(ui1 =0;ui1 <RELAYNUM;ui1 ++)
	{
		*gPtr =rxBuffer[ui1+3] -0x20;
		gPtr++;
	}
}

void  ReceiveData()
{
	if(Rs_ID !=0xdf)  //非广播命令
		ProcessData();
	
	if(BroadcastCMD) //广播命令
	{
		switch(BroadcastCMD)
		{
			case 0x01: //全局复位
			{
				for(ui0 =0; ui0 <MOTORNUM; ui0++)
				{
					AxisV[ui0].RunStep =2;
					gwrite_Port( ADDR_AXIS0 + ui0*16 +PLSCMD , 0x04);
					gwrite_Port( ADDR_AXIS0 + ui0*16 +PLSCMD , 0x00);	
				}
			};break;
			case 0x02: //急停与解除急停
			{
				if(EMS_Stop ==0)
				{
					for(ui0 =0; ui0 <MOTORNUM; ui0++)
					{
						AxisV[ui0].RunStep =1; //急停状态
						AxisV[ui0].TargetV =0;
						//nStandby & Drvrst
						AxisV[ui0].Yout &= 0xef;
						gwrite_Port( ADDR_AXIS0 + ui0*16 +STATECMD, AxisV[ui0].Yout);
					}
					EMS_Stop =1;
				}
				else
				{
					for(ui0 =0; ui0 <MOTORNUM; ui0++)
					{
						AxisV[ui0].RunStep =0; //急停解除
						//nStandby = 1;
						AxisV[ui0].Yout |= 0x10;
						gwrite_Port( ADDR_AXIS0 + ui0*16 +STATECMD, AxisV[ui0].Yout);
						//DRVRst = 1;
//						AxisV[ui0].Yout |= 0x20;
//						gwrite_Port( ADDR_AXIS0 + ui0*16 +STATECMD, AxisV[ui0].Yout);
						//DRVRst = 0;
//						AxisV[ui0].Yout &= 0xdf;
//						gwrite_Port( ADDR_AXIS0 + ui0*16 +STATECMD, AxisV[ui0].Yout);
					}
					EMS_Stop =0;
				}
			};break;
			case 0x10: //启动连续运行
			{
				ContinueRun =1;
				StepRunFlag =0;		
			};break;
			case 0x11: //启动单步运行
			{
				StepRunFlag =1;
			};break;
			case 0x12: //单步运行下一步
			{
				ContinueRun =1;
				ContinueDir =1;
			};break;
			case 0x13: //单步运行上一步???
			{
				ContinueRun =1;
				ContinueDir =0;
			};break;
			default:
			{
			//Errcode =
			};break;
		}
		BroadcastCMD =0;
	}	
}

uchar MotorMap(uchar ID,uchar MotorIndex)
{
	switch(ID)
	{
		case ID_ARMMIX:
		{
			if(MotorIndex ==0)
			{
				gPartSel = MOTOR_Mix;
			}
			else
			{
				gPartSel = ARMMIX_OTHER;
			}			
		};break;
		case ID_ARMSA:
		{
			if(MotorIndex ==0)
			{
				gPartSel = MOTOR_ARM_C;
			}
			else if(MotorIndex ==1)
			{
				gPartSel = MOTOR_ARM_Z;
			}
			else if(MotorIndex ==2)
			{
				gPartSel = MOTOR_PUMP;
			}
			else
			{
				gPartSel = ARMSA_OTHER;
			}
		};break;
		case ID_PLATE:
		{
			if(MotorIndex ==0)
			{
				gPartSel = MOTOR_PLATE;
			}
			else
			{
				gPartSel = PLATE_OTHER;
			}
		};break;
		case ID_MICROXYZ:
		{
			if(MotorIndex ==2)
			{
				gPartSel = MOTOR_Z;
			}
			else if(MotorIndex ==1)
			{
				gPartSel = MOTOR_Y;
			}
			else if(MotorIndex ==0)
			{
				gPartSel = MOTOR_X;
			}
			else
			{
				gPartSel = 0xff;
			}
		};break;
		default:
		{
			gPartSel = 0xff;
		};break;
	}
	return gPartSel;
}

void ProcessData()
{
	CurCMD_rOK =0;
	Cur_CMD =rxBuffer[2];
	if(Rs_ID>ID_MICROXYZ)	//IO操作
	{
		CurCMD_rOK =1;
		if(Cur_CMD ==RSCMD_RELAY)	//继电器控制命令
		{
			ui1 =rxBuffer[3];
			ui2 =rxBuffer[4];
			ui3 =ui2<<4;
			tmpRelay[0] =(ui1 &0x0f)|(ui3 &0xf0);
			ui1 =rxBuffer[5];
			ui2 =rxBuffer[6];
			ui3 =ui2<<4;
			tmpRelay[1] =((ui1 &0x0f)|(ui3 &0xf0));
			
			ui1 =rxBuffer[7];			//增加一路两通阀，双字节变量控制泵阀
			ui2 =rxBuffer[8];
			ui3 =ui2<<4;
			tmpRelay[2] =(ui1 &0x0f)|(ui3 &0xf0);
			ui1 =rxBuffer[9];
			ui2 =rxBuffer[10];
			ui3 =ui2<<4;
			tmpRelay[3] =((ui1 &0x0f)|(ui3 &0xf0));			
		}
		else if((Cur_CMD &0xf0) ==RSCMD_RELAYPARA)//继电器延时参数
		{
			SaveRelayPara();
		}
		else
		{
			CurCMD_rOK =0;
		}
	}
	else
	{
		gPartSel =MotorMap(Rs_ID,Cur_CMD&3);
//		if(gPartSel <MOTORNUM)//电机命令
		{
			CurCMD_rOK =1;
			if((Cur_CMD &0xf0) ==RSCMD_POS_A)//电机定位命令
			{
				if(Cur_CMD &8)
				{
					//32位置定位命令
				}
				else
				{
					AtoInt(rxBuffer,3);
					/*
					for(ui0 =0;ui0 <8;ui0++)	//test
					{
						PositioningStart(ui0);
						sii1 += 1000;
					}
					*/
					PositioningStart(gPartSel);
				}
			}			
			else if((Cur_CMD &0xf0) ==RSCMD_MOTORPARA)//电机参数
			{		
				SaveMotorPara(gPartSel);
			}
			else if((Cur_CMD &0xf0) ==RSCMD_AXIS_REF) //电机复位
			{
				if(AxisV[gPartSel].Speed ==0)
				{
					gwrite_Port( ADDR_AXIS0 + gPartSel*16 +PLSCMD , 0x04);
					gwrite_Port( ADDR_AXIS0 + gPartSel*16 +PLSCMD , 0x00);	
					AxisV[gPartSel].RunStep =2;			
				}
				else
					CurCMD_rOK =0;
			}
			else if((Cur_CMD &0xf0) ==RSCMD_ACTION)
			{
				switch(Cur_CMD)
				{
					case ACTION_MIX:
					{
						AxisV[MOTOR_Mix].RunStep =18;
					};break;
					case ACTION_SAMPLE:
					{
						AxisV[MOTOR_PUMP].RunStep =18;
					};break;
					case ACTION_CLEAN:
					{
						AxisV[MOTOR_ARM_C].RunStep =18;
					};break;				
					default:
						break;
				}
			}
			else
			{
				CurCMD_rOK =0;
			}
		}
		/*
		else
		{
			//Error
			CurCMD_rOK =0;
		}
		*/
	}
}


void SendData(unsigned char * buf,unsigned char len)
{
	TxLen = len + 5;
	TxCnt = 0;
	gPtr = &txBuffer;			
#ifdef ID_MASK
	*gPtr = Rs_ID +0x20;
#else
	*gPtr =myID +0x20;	 //slave
#endif
	ChkSum = *gPtr;
	gPtr++;
	*gPtr = TxLen +0x20; //(去掉头尾)
	ChkSum ^= *gPtr;
	gPtr++;
	*gPtr=RSstatus;		 //HandShake
	ChkSum ^= *gPtr;
	gPtr++;	
	for(ui0=0;ui0<len;ui0++) //D1-Dn
	{
		*gPtr = buf[ui0];
		ChkSum ^= *gPtr;
		gPtr++;
	}
	*gPtr = NtoA(ChkSum&0xf);
	gPtr++;		
	*gPtr = NtoA(ChkSum/0x10);	
	RS422STARTSEND;
}

void StartTxData()											
{
	gPtr = &gBuf;
	*gPtr = Part[Rs_ID].Type +0x20;	
	gPtr ++;
	*gPtr = Part[Rs_ID].Errcode +0x20;//Errcode
	gPtr ++;
	*gPtr = Part[Rs_ID].RunStep +0x20;
	gPtr ++;
	*gPtr = 0x20;//DataCase
	gPtr ++;
	
	ui0 = Part[Rs_ID].TotalAxis;
	if(ui0)
		ui1 = Part[Rs_ID].StartAxisNum;
	while(ui0!=0)
	{
		*gPtr = AxisV[ui1].RunStep + 0x20;
		gPtr ++;
		*gPtr = AxisV[ui1].Xin|0x80|AxisV[ui1].Status;
		gPtr ++;
		*gPtr = AxisV[ui1].Yout|0x80;
		gPtr ++;
		*gPtr = AxisV[ui1].Speed +0x20;
		gPtr ++;
		*gPtr = NtoA(AxisV[ui1].Pos.Byte[0]>>4);
		gPtr ++;
		*gPtr = NtoA(AxisV[ui1].Pos.Byte[0]);
		gPtr ++;
		*gPtr = NtoA(AxisV[ui1].Pos.Byte[1]>>4);
		gPtr ++;
		*gPtr = NtoA(AxisV[ui1].Pos.Byte[1]);
		gPtr ++;
		
		ui0 --;
		ui1 ++;
	}
	
	//以后扩展用
	ui0 = Part[Rs_ID].TotalX;
	ui1 = 0;
	while (ui0!=0)
	{
		*gPtr = NtoA(IO_XX[ui1]>>4);
		gPtr ++;
		*gPtr = NtoA(IO_XX[ui1]);
		gPtr ++;
		ui0 --;
		ui1++;
	}
	ui0 = Part[Rs_ID].TotalY;
	ui1 = 0;
	while (ui0!=0)
	{
		*gPtr = NtoA(IO_YY[ui1]>>4);
		gPtr ++;
		*gPtr = NtoA(IO_YY[ui1]);
		gPtr ++;
		ui0 --;
		ui1++;
	}

	uii0 = gPtr - &gBuf;
	SendData(gBuf,ui1);
}

void RS422_Interface(void)
{
	if(gRS422RxOK)
	{
		gRS422RxOK =0;
		RxErr =0;
		if(RxCnt!=(rxBuffer[1]-0x20))	//check number 
		{
			RxErr =1;
		}
		else							//check xor
		{
			RxCnt--;
			ChkSum = AtoNum(rxBuffer[RxCnt])*0x10;
			ChkSum += AtoNum(rxBuffer[--RxCnt]);
			ui1 = rxBuffer[0];
			for(ui0=1;ui0<RxCnt;ui0++)
			{
				ui1 ^=rxBuffer[ui0];
			}
			if(ChkSum!=ui1)
			{
				RxErr =2;
			}				
		}			
		if(RxErr==0)
		{
			ACKflag =1;
			ReceiveData();				
		}
		else
		{
			ACKflag =0;
//			test_buf[test_rxerr] =RxCnt;
//			test_rxerr ++;
		}
		if(Rs_ID !=0xdf)
			StartTxData();
	}
}

void UART0_ISR (void) interrupt RS422INTNUM
{
	static unsigned char data buf=0;
	if(RS422_RI)								//receive interrupt
	{
		buf =RS422_SBUF;
		CLR_RS422_RI;
		if(buf==STX) 
		{
			RxCnt =0;
			RxStatus =1;
		}
		else if(buf ==ETX)
		{
			if(RxStatus>1)
			{
				if(Rs_ID ==0xdf)
				{
					if(rxBuffer[2] ==~rxBuffer[3]) //broadcastcmd不进入ReceiveData函数，直接在中断进行校验
					{
						BroadcastCMD = rxBuffer[2] -0x20;
						BroadcastCMD_rOK = 1;
					}
					else
					{
						BroadcastCMD_rOK = 0;
					}
				}
				else
				{
					BroadcastCMD_rOK = 0;	
					gRS422RxOK = 1;
				}
			}
		}
		else if(RxStatus)
		{
			if(RxStatus ==1)
			{
				#ifdef ID_MASK
				{
					RxStatus = 2;
					rxBuffer[RxCnt] = buf;			//receive data
				}
				#else
				if(buf ==myID +0x20 || buf==0xff)	//buf==0x20
				{
					RxStatus =2;
					rxBuffer[RxCnt] =buf;			//receive data
				}
				else
					RxStatus = 0;
				#endif
				Rs_ID =buf-0x20;					//Rs_ID = rxBuffer[0]-0x20;
			}
			else if(RxCnt <RXBUFLEN)
			{
				rxBuffer[RxCnt] =buf;
			}
			RxCnt++;
		}
	}  
	if(RS422_TI) 									//send interrupt
	{  
		if(TxCnt<TxLen)
		{ 
			 RS422_SBUF = txBuffer[TxCnt];			//send data
			 TxCnt++;
		}
		else
		{
			if(TxCnt==TxLen)
			{
				RS422_SBUF =ETX;
				TxCnt++;
			}
			else
			{
				RS422TxPortState = 0;
			}	
		}
		CLR_RS422_TI;								//clear send flag
	} 
}

unsigned char NtoA(unsigned char chr)
{
	chr&=0xf;
	if(chr>9)
	{
		chr+=('A'-10);//A ASC 65
	}
	else
		chr+='0';	 //0 ASC 48
	return chr;
}
unsigned char AtoNum(unsigned char chr)
{
	if(chr>='0' && chr<='9')
	{
		chr-='0';
	}
	else if(chr>='a'&&chr<='f')
	{
		chr-=('a'-10);
	}
	else if(chr>='A'&&chr<='F')
	{
		chr-=('A'-10);
	}
	else
		return 0;
	return chr;
}

void AtoInt(unsigned char buf[],unsigned char index)
{
	si2 = AtoNum(buf[index]);
	si2 = (si2<<4) + AtoNum(buf[index+1]);
	si3 = AtoNum(buf[index+2]);
	si3 = (si3<<4) + AtoNum(buf[index+3]);
}