#ifndef RS422_H
#define	RS422_H

//#define RxErr ui2						//receive data error
//#define TxErr ui3						//send data error

#define	STX 		0x02
#define	ETX 		0x03
#define	RXBUFLEN 	64
#define	TXBUFLEN 	128

//RS_CMD >=0x20
#define RSCMD_REQ 				0x20
#define RSCMD_RES 				0x21
#define RSCMD_EMS					0x22
#define RSCMD_AXISTOZERO 	0x28
#define RSCMD_POS_A 			0x30
#define RSCMD_POS_A_D			0x38
#define RSCMD_AXIS_REF 		0x40
#define RSCMD_RELAY		 		0x50
//#define RSCMD_RELAY_S 		0x50
//#define RSCMD_RELAY_R 		0x51
#define RSCMD_RELAYPARA 	0x60
#define RSCMD_ACTION 					0x70
#define ACTION_MIX 						0x71
#define ACTION_SAMPLE					0x72
#define ACTION_CLEAN					0x73
#define RSCMD_MOTORPARA 			0x80

#define ID_ARMMIX 			0x0
#define ID_ARMSA 				0x1
#define ID_PLATE 				0x2
#define ID_MICROXYZ 		0x3
#define ID_IO						0x4

#define	RS422_UART0
#ifdef RS422_UART0
	#define	RS422_RI	RI0
	#define	RS422_TI	TI0
	#define	CLR_RS422_RI RI0=0;
	#define	CLR_RS422_TI TI0=0;
   	#define	RS422_SBUF SBUF0
   	#define	RS422INTNUM	  4
	#define RS422STARTSEND  {RS422TxPortState=1; SBUF0 = STX;}
#else
	#define	RS422_RI	((SCON1&0x1)==1)
	#define	RS422_TI	((SCON1&0x2)==2)
	#define	CLR_RS422_RI  SCON1&=0xfe;
	#define	CLR_RS422_TI  SCON1&=0xfd;
   	#define	RS422_SBUF SBUF1
   	#define	RS422INTNUM	  16
	#define RS422STARTSEND  {RS422TxPortState=1; SBUF1 = STX;}
#endif
	
extern bit StepRunFlag;	
extern bit gRS422RxOK;
//extern unsigned char data	 IOCtrlCMD;
extern unsigned char xdata   RxErr,TxErr;
extern unsigned char xdata 	 rxBuffer[];			//array of RX data
//extern unsigned char xdata txBuffer[];			//array of TX data
extern void  RS422_Init();
extern void	 RS422_Interface();
extern void  ProcessData();
extern unsigned char MotorMap(unsigned char ID,unsigned char MotorIndex);
extern unsigned char AtoNum(unsigned char chr);
extern unsigned char NtoA(unsigned char chr);
extern void AtoInt(unsigned char buf[],unsigned char);
#endif


