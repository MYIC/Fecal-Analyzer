#include	<c8051F340.h>
#include	"Git_Repository_Fecal\Include\Global.h"

#ifndef IO_EXPAND_H
#define IO_EXPAND_H

uchar gread_Port(uchar ADDR);
void	gwrite_Port( uchar ADDR, uchar DATA );

#endif