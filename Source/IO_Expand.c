#include  "Git_Repository_Fecal\Include\IO_Expand.h"
#include	"Git_Repository_Fecal\Include\Global.h"
#include	<intrins.h>

sbit ALE = P1^1;
sbit RD = P1^0;
sbit WR = P0^7;

uchar bdata  AD;
sbit	Databit0 =	AD^0;
sbit	Databit1 =	AD^1;
sbit	Databit2 =	AD^2;
sbit	Databit3 =	AD^3;
sbit	Databit4 =	AD^4;
sbit	Databit5 =	AD^5;
sbit	Databit6 =	AD^6;
sbit	Databit7 =	AD^7;

sbit	PortDatabit0 =P1^2;
sbit	PortDatabit1 =P1^3;
sbit	PortDatabit2 =P1^4;
sbit	PortDatabit3 =P1^5;
sbit	PortDatabit4 =P1^6;
sbit	PortDatabit5 =P1^7;
sbit	PortDatabit6 =P2^0;
sbit	PortDatabit7 =P2^1;

void PortMapWrite()
{
	PortDatabit0 =Databit0;
	PortDatabit1 =Databit1;
	PortDatabit2 =Databit2;
	PortDatabit3 =Databit3;
	PortDatabit4 =Databit4;
	PortDatabit5 =Databit5;
	PortDatabit6 =Databit6;
	PortDatabit7 =Databit7;
}

void PortMapRead()
{
	Databit0 =PortDatabit0;
	Databit1 =PortDatabit1;
	Databit2 =PortDatabit2;
	Databit3 =PortDatabit3;	
	Databit4 =PortDatabit4;
	Databit5 =PortDatabit5;
	Databit6 =PortDatabit6;
	Databit7 =PortDatabit7;	
}
uchar	gread_Port(uchar ADDR)
{
	RD = 1;
	ALE = 0;
	AD = ADDR;					//write the address on the bus 
	PortMapWrite();
	ALE = 1;
	ALE = 0;
	AD = 0xff;          //×¼Ë«Ïò¿Ú
	PortMapWrite();
	RD = 0;
	PortMapRead();
	RD = 1;
	return (AD);		//read the data from the bus 
}

void	gwrite_Port( uchar ADDR, uchar DATA )
{	
	WR = 1;
	ALE = 0;
	AD = ADDR;
	PortMapWrite();
	ALE = 1;
	ALE = 0;
	AD = DATA;
	PortMapWrite();
	WR = 0;
	WR = 1;	
}
