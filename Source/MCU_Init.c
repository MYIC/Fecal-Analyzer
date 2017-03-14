#include	"Git_Repository_Fecal\Include\MCU_Init.h"
#include	"Git_Repository_Fecal\Include\RS422.h"
#include	"Git_Repository_Fecal\Include\Global.h"

sfr16  TMR2RL   = 0xCA;
sfr16  TMR2     = 0xCC;
sfr16  TMR3RL   = 0x92;
sfr16  TMR3     = 0x94;
sbit   SCL      = P0^1;

void	Oscillator_Init();
void	RSTSOURCE_Init();
void	Port_Init();
void 	Timer0_Init();
//void 	Timer2_Init();
void  Timer3_Init();

void MCU_Init(void)
{
	PCA0MD &= ~0x40;					 //Disable the watch dog
	CLKSEL  =  0x00;
	Oscillator_Init();				 //时钟选择寄存器,使用内部振荡器
	RSTSOURCE_Init();
	Port_Init();
	Timer0_Init();
//	Timer2_Init();
	
	UART0_Init();
	RS422_Init();
	EA = 1;	
}

/*SYSCLK = 12*4/2= 24MHz*/
void Oscillator_Init()
{
	OSCICN |= 0x80; 								//使能内部高频振荡器
	CLKMUL = 0x00; 									//复位时钟乘法器,以及选择时钟乘法器时钟来源为内部高频振荡器
	CLKMUL |= 0x80; 								//使能时钟乘法器
	for(ui0 = 0;ui0 < 100;ui0 ++);	//延时5us	
		CLKMUL |= 0xC0; 				    	//初始化时钟乘法器
	while(!(CLKMUL & 0x20)); 				//等待4倍时钟乘法器准备好
		CLKSEL = 0x02;              
}

void RSTSOURCE_Init()
{
	VDM0CN	|= 0x80;
	for(ui0 = 0; ui0<200; ui0++);			   //Delay 160us
	RSTSRC	=  0x02;
}

void Port_Init()
{
	P0MDIN	= 0xFF;
 	P1MDIN	= 0xFF;
	P2MDIN 	= 0xFF;
	P3MDIN 	= 0xFF;

	P0MDOUT = 0x9e;	 //0:open-drain, 1:push-pull// 1001 1110
	P1MDOUT = 0x03;	 //open-drain output,the current will be less
	P2MDOUT = 0x0c;
	P3MDOUT	= 0xFF;

	XBR0	  = 0x05;  //enable SMBus and UART0
	XBR1    = 0x40;  //XBAR = 1
	XBR2    = 0x00;
	
	P0		= 0xFF;
	P1		= 0xFF;
	P2		= 0xFF;
	P3		= 0xFF;
}

void UART0_Init (void)
{
	SCON0     = 0x10;       						// Timer 1 support baud rate
	CKCON    |= 0x08;      							// Timer 1 use system clock/12
	TH1       = 0x98;       						// baud rate 115200
	TL1       = TH1;                        
	TMOD     |= 0x20;       						// timer1 in 8-bit auto-reload mode
	TR1       = 1;                           
	TI0       = 0;											// Transmit interrupt flag 
	RI0       = 0;  										// Receive interrupt flag
	ES0       = 1;  										// UART0 interrupt enable
}

void 	Timer0_Init(void)
{
	 CKCON |= 0x00;	//T0clk =SYSCLK/12                           
	 TMOD  |= 0x01;                      
			 
	 TH0 = 0xf8;
	 TL0 = 0x30;			//1ms
	 TR0 = 1;				//Enable the Timer0
	 ET0 = 1;				//Enable the Timer0 interrupt
}

void Timer0_ISR (void) interrupt 1
{  
   TH0 = 0xf8;
   TL0 = 0x30;
	 gT1msCnt ++;
}

/*
void Timer2_Init (void)			     
{
   CKCON  = 0x00;                          
   TMR2CN = 0x00;
   TMR2RL = 65536 - 20000;			//10ms
   TMR2   = TMR2RL;
   TR2    = 1;
   ET2	  = 1;                                 
}
*/
/*
 void Timer2_ISR (void) interrupt 5
{  
  TF2H = 0;                 // Clear Timer2 interrupt-pending flag 
}
*/




