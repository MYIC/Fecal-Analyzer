#ifndef AD7151_H
#define	AD7151_H

#define 	MINC		0x5000
#define		MAXC		0x6300

extern	unsigned char gSMB_CMD,mSMB_CMD;
extern	union uintData CurD_7151;
extern	bit SMB_Err;
extern	bit SMB_CmdW;
extern	bit SMB_CmdR;
extern	bit SMB_wOK;
extern	bit SMB_rOK;


void	SMBus_Interface();
void	SendDtoSMBus();
void	Read7151D();

#endif





