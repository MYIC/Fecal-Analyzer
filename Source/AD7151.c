#include	<c8051f340.h>
#include	"Include\RS422.h"
#include	"Include\Global.h"
#include	"Include\MCU_Init.h"
#include	"Include\AD7151.h"

#define  SLAVE_ADDR     0x90           // Device address for slave target
// SMBus states:
// MT = Master Transmitter
// MR = Master Receiver
#define  SMB_MTSTA      0xE0           // (MT) STA transmitted
#define  SMB_MTDB       0xC0           // (MT) data byte transmitted
#define  SMB_MRDB       0x88           // (MR) data byte received
#define  SMB_BUSY       SMB0CF&0x20

unsigned char gSMB_CMD,mSMB_CMD;
unsigned char idata SMB_TxBuf[20];	
unsigned char idata SMB_RxBuf[24]; 	
unsigned char data SMB_Rcnt,SMB_Wcnt,SMB_Index;
bit SMB_Err;
bit SMB_CmdW;
bit SMB_CmdR;
bit SMB_wOK;
bit SMB_rOK;

union uintData CurD_7151;

void	SendDtoSMBus()
{
	mSMB_CMD =gSMB_CMD;
	SMB_CmdW = 1;
	switch(mSMB_CMD)
	{   
   case 1:
			SMB_TxBuf[0] = (SLAVE_ADDR & 0xfe); 
			SMB_TxBuf[1] = 0x09;
			SMB_TxBuf[2] = 0x75; //0x09寄存器设置值（固定阈值高位）
      SMB_TxBuf[3] = 0x00; //0x0A寄存器设置值（固定阈值低位）
      SMB_TxBuf[4] = 0x5B; //0x0B寄存器设置值（CDC输入范围：0.5pF；禁用迟滞）
      SMB_TxBuf[5] = 0x00; //0x0C寄存器设置值（无效寄存器）
      SMB_TxBuf[6] = 0x00; //0x0D寄存器设置值（无效寄存器）
      SMB_TxBuf[7] = 0x00; //0x0E寄存器设置值（无效寄存器）
      SMB_TxBuf[8] = 0x91; //0x0F寄存器设置值（设置固定阈值模式；设置阈值为负数据<阈值；使能通道转换；连续转换）
      SMB_TxBuf[9] = 0x00; //0x10寄存器设置值（禁用关断超时）
			SMB_TxBuf[10] = 0x8D;//0x11寄存器设置值（禁止自动偏置 设置偏置值）
			SMB_TxBuf[11] = 0xAA;
      SMB_Wcnt =12;
			SMB_Index =0;
			STA = 1;
			break;
	 case 2:
		  SMB_TxBuf[0] = (SLAVE_ADDR & 0xfe); 
			SMB_TxBuf[1] = 0x11;
			SMB_TxBuf[2] = 0x8D;
			SMB_Wcnt =3;
			SMB_Index =0;
			STA = 1;
			break;
	 default:
			break;		  
	}	
}

void	Read7151D()
{
	SMB_TxBuf[0] = (SLAVE_ADDR & 0xfe); 	//WR
	SMB_TxBuf[1] = 0x00;
	SMB_TxBuf[2] = (SLAVE_ADDR | 0x01);		//RD
	SMB_Wcnt =3;
	SMB_Rcnt =24;
	SMB_Index =0;
	SMB_CmdR = 1;
	STA = 1;	
}

void 	SMBus_Interface()
{
	if(!(SMB_CmdW|SMB_CmdR))
  {
     Read7151D();
     return;
  }
  if(mSMB_CMD)	
	{
		if(SMB_wOK)//一帧写数据完成
		{
			if(gSMB_CMD==2)
			{
				SMB_wOK =0;
				SMB_CmdW =0;
				mSMB_CMD = 0;
				gSMB_CMD = 0;
			}
			else
			{
				gSMB_CMD=2;
				SendDtoSMBus();
			}          
		}
	}
	else
	{
		if(SMB_rOK)//一帧读数据完成
		{
      SMB_rOK = 0;
			SMB_CmdR =0;
			//Process the data
			CurD_7151.Byte[0] =SMB_RxBuf[1];
			CurD_7151.Byte[1] =SMB_RxBuf[2];			
			if(gSMB_CMD)
			{
				SendDtoSMBus();
			}
			else
			{
				Read7151D();
			}
		}   
	}
}

void 	SMBus_ISR (void) interrupt 7
{
	switch (SMB0CN & 0xfc)	// Status code for the SMBus
	{
		// (MT) STA transmitted
		case SMB_MTSTA:				
       STA = 0;
			 SMB0DAT =SMB_TxBuf[SMB_Index];// Load the slave Address +W bit
			 SMB_Index ++;
			 if(SMB_Wcnt--==0)	//初始化SMB_Wcnt=3，减完SMB_Wcnt=2
			 {
				SMB_Index = 0;
			 }
			 break;		
		// (MT) data byte transmitted &ACK  
    case SMB_MTDB:
			 if(ACK)
			 {
				 if(SMB_Wcnt)				//第一次进来SMB_Wcnt=2
				 {
						if(SMB_Wcnt ==1 && SMB_CmdR)              
						{
							STA = 1;
						}
						else
						{
							SMB0DAT = SMB_TxBuf[SMB_Index];				//发送数据；加载寄存器地址0xXX
							SMB_Index ++;
							SMB_Wcnt --;
						}
				 }
				 else if(SMB_CmdW)
				 {
					 STO = 1;
					 SMB_wOK = 1;
				 }
			 }
			 else
			 {
				 	 SMB_Err = 1;
			 }
			break;
		// (MR) data byte received
		case SMB_MRDB:        
			 SMB_RxBuf[SMB_Index-3] =SMB0DAT;
       SMB_Index++;
		   if(SMB_Rcnt)
			 {             
				if(--SMB_Rcnt==0)	//准备发送停止位
				{	
				 ACK = 0;
				 STO = 1;
				 SMB_rOK = 1;
				}
				else
        {				
           ACK = 1;
        }                    
			 }
       break;
		default:
			 SMB_Err = 1;
       break;
	}
  if(SMB_Err)                         // If the transfer failed,
  {
		SMB0CF &= ~0x80;                  // Reset communication
    SMB0CF |= 0x80;
    STA = 0;
    STO = 0;
    ACK = 0;
    SMB_Err =0;
	}
  SI = 0;                             // Clear interrupt flag
}









