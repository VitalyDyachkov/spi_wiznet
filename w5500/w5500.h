#ifndef W5500_H
#define W5500_H
#include "stdint.h"
//=========================Common Register Block===================================//
#define MODE_REG 	(uint16_t)0x0000
#define GAR0 			(uint16_t)0x0001
#define GAR1 			(uint16_t)0x0002
#define GAR2 			(uint16_t)0x0003
#define GAR3 			(uint16_t)0x0004
#define SUBR0 		(uint16_t)0x0005
#define SUBR1 		(uint16_t)0x0006
#define SUBR2 		(uint16_t)0x0007
#define SUBR3 		(uint16_t)0x0008
#define SHAR0 		(uint16_t)0x0009
#define SHAR1 		(uint16_t)0x000A
#define SHAR2 		(uint16_t)0x000B
#define SHAR3 		(uint16_t)0x000C
#define SHAR4 		(uint16_t)0x000D
#define SHAR5 		(uint16_t)0x000E
#define SIPR0 		(uint16_t)0x000F
#define SIPR1 		(uint16_t)0x0010
#define SIPR2 		(uint16_t)0x0011
#define SIPR3 		(uint16_t)0x0012
#define INTLEVEL0 (uint16_t)0x0013
#define INTLEVEL1 (uint16_t)0x0014
#define IR 				(uint16_t)0x0015
#define W_IMR 		(uint16_t)0x0016
#define SIR 			(uint16_t)0x0017
#define SIMR 			(uint16_t)0x0018
#define RTR0 			(uint16_t)0x0019
#define RTR1 			(uint16_t)0x001A
#define W_RCR 		(uint16_t)0x001B
#define PTIMER 		(uint16_t)0x001C
#define PMAGIC 		(uint16_t)0x001D
#define PHAR0 		(uint16_t)0x001E
#define PHAR1     (uint16_t)0x001F
#define PHAR2 		(uint16_t)0x0020
#define PHAR3 		(uint16_t)0x0021
#define PHAR4 		(uint16_t)0x0022
#define PHAR5 		(uint16_t)0x0023
#define PSID0 		(uint16_t)0x0024
#define PSID1 		(uint16_t)0x0025
#define PMRUR0 		(uint16_t)0x0026
#define PMRUR1 		(uint16_t)0x0027
#define UIPR0 		(uint16_t)0x0028
#define UIPR1 		(uint16_t)0x0029
#define UIPR2 		(uint16_t)0x002A
#define UIPR3 		(uint16_t)0x002B
#define UPORTR0 	(uint16_t)0x002C
#define UPORTR1 	(uint16_t)0x002D
#define PHYCFGR 	(uint16_t)0x002E
#define VERSIONR 	(uint16_t)0x0039
//=========================Socket Register Block (Offset Address)====================//
#define SOCK_x_MR 			(uint16_t)0x0000
#define SOCK_x_CR 			(uint16_t)0x0001
#define SOCK_x_IR 			(uint16_t)0x0002
#define SOCK_x_SR 			(uint16_t)0x0003
#define SOCK_x_PORT0 		(uint16_t)0x0004
#define SOCK_x_PORT1 		(uint16_t)0x0005
#define SOCK_x_DHAR0 		(uint16_t)0x0006
#define SOCK_x_DHAR1 		(uint16_t)0x0007
#define SOCK_x_DHAR2 		(uint16_t)0x0008
#define SOCK_x_DHAR3 		(uint16_t)0x0009
#define SOCK_x_DHAR4 		(uint16_t)0x000A
#define SOCK_x_DHAR5 		(uint16_t)0x000B
#define SOCK_x_DIPR0 		(uint16_t)0x000C
#define SOCK_x_DIPR1 		(uint16_t)0x000D
#define SOCK_x_DIPR2 		(uint16_t)0x000E
#define SOCK_x_DIPR3 		(uint16_t)0x000F
#define SOCK_x_DPORT0 	(uint16_t)0x0010
#define SOCK_x_DPORT1 	(uint16_t)0x0011
#define SOCK_x_MSSR0  	(uint16_t)0x0012
#define SOCK_x_MSSR1 	  (uint16_t)0x0013
#define SOCK_x_TOS    	(uint16_t)0x0015
#define SOCK_x_TTL 		  (uint16_t)0x0016
#define SOCK_x_RXBS 		(uint16_t)0x001E
#define SOCK_x_TXBS     (uint16_t)0x001F
#define SOCK_x_TX_FSR0 	(uint16_t)0x0020
#define SOCK_x_TX_FSR1 	(uint16_t)0x0021
#define SOCK_x_TX_RD0 	(uint16_t)0x0022
#define SOCK_x_TX_RD1 	(uint16_t)0x0023
#define SOCK_x_TX_WR0 	(uint16_t)0x0024
#define SOCK_x_TX_WR1 	(uint16_t)0x0025
#define SOCK_x_RX_RSR0 	(uint16_t)0x0026
#define SOCK_x_RX_RSR1 	(uint16_t)0x0027
#define SOCK_x_RX_RD0 	(uint16_t)0x0028
#define SOCK_x_RX_RD1 	(uint16_t)0x0029
#define SOCK_x_RX_WR0 	(uint16_t)0x002A
#define SOCK_x_RX_WR1 	(uint16_t)0x002B
#define SOCK_x_IMR 	    (uint16_t)0x002C
#define SOCK_x_FRAG0 	  (uint16_t)0x002D
#define SOCK_x_FRAG1 	  (uint16_t)0x002E
#define SOCK_x_KPALVTR 	(uint16_t)0x002F
//=========================Block Select Bits====================//
#define WSCR 			0x00

#define SS0R 			0x01
#define SS0TxB 		0x02
#define SS0RxB 		0x03

#define SS1R 			0x05
#define SS1TxB 		0x06
#define SS1RxB 		0x07

#define SS2R 			0x09
#define SS2TxB 		0x0A
#define SS2RxB 		0x0B

#define SS3R 			0x0D
#define SS3TxB 		0x0E
#define SS3RxB 		0x0F

#define SS4R 			0x11
#define SS4TxB 		0x12
#define SS4RxB 		0x13

#define SS5R 			0x15
#define SS5TxB 		0x16
#define SS5RxB  	0x17

#define SS6R 	  	0x19
#define SS6TxB    0x1A
#define SS6RxB 		0x1B

#define SS7R 			0x1D
#define SS7TxB    0x1E
#define SS7RxB 		0x1F


#define WRITE_CHIP 0x04
#define READ_CHIP 0x00

#define OM_VDLM 0x0
#define OM_DLM_1 0x1
#define OM_DLM_2 0x2
#define OM_DLM_4 0x3
	
//=========================Operating socket's====================//
#define OPEN_SOCKET 0x01
#define CLOSE_SOCKET 0x10
#define SOCKET_SEND 0x20
#define SET_SOCKET_CLOSED 0x00
#define SET_SOCKET_TCP 0x01
#define SET_SOCKET_UDP 0x02
#define SET_SOCKET_MACRAW 0x04

#define RESET_CHIP 0x90
#endif
