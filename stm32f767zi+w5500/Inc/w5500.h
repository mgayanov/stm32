
#ifndef __W5500__
#define __W5500__

#include "stm32f7xx_hal.h"

#define CHIPVERSION_0X04 0x04

#define MAC_ADDR {0x00,0x15,0x42,0xBF,0xF0,0x51}
#define IP_ADDR {192,168,1,196}
#define IP_GATE {192,168,1,2}
#define IP_MASK {255,255,255,0}
#define LOCAL_PORT_80 80
#define LOCAL_PORT_68 68

#define DHCP_SERVER_IP {255,255,255,255}
#define DHCP_SERVER_PORT_67 67

/* Gateway */
#define GAR0 0x0001
#define GAR1 0x0002
#define GAR2 0x0003
#define GAR3 0x0004

/* Subnet */
#define SUBR0 0x0005
#define SUBR1 0x0006
#define SUBR2 0x0007
#define SUBR3 0x0008

/* MAC */
#define SHAR0 0x0009
#define SHAR1 0x000A
#define SHAR2 0x000B
#define SHAR3 0x000C
#define SHAR4 0x000D
#define SHAR5 0x000E

/* IP */
#define SIPR0 0x000F
#define SIPR1 0x0010
#define SIPR2 0x0011
#define SIPR3 0x0012

/* Mode */
#define TCP_MODE 0x1
#define UDP_MODE 0x2
#define MACRAW   0x4


/* Socket number */
#define SOCKET_0 0

/* Socket status */
#define SOCK_CLOSED      0x00
#define SOCK_INIT        0x13
#define SOCK_LISTEN      0x14
#define SOCK_ESTABLISHED 0x17
#define SOCK_CLOSE_WAIT  0x1C
#define SOCK_UDP         0x22
#define SOCK_MACRAW      0x42

/* Socket command */
#define OPEN      0x01
#define LISTEN    0x02
#define CONNECT   0x04
#define DISCON    0x08
#define CLOSE     0x10
#define SEND      0x20
#define SEND_MAC  0x21
#define SEND_KEEP 0x22
#define RECV      0x40


#define MR       0x0000 //Mode register
#define VERSIONR 0x0039

#define BSB_COMMON 0x00

#define SOCKET_0 0
#define SOCKET_1 1
#define SOCKET_2 2
#define SOCKET_3 3
#define SOCKET_4 4
#define SOCKET_5 5
#define SOCKET_6 6
#define SOCKET_7 7

#define BSB_Sn(n) 0x01 + 0x04 * n
#define BSB_Sn_TX(n) BSB_Sn(n) + 0x01
#define BSB_Sn_RX(n) BSB_Sn(n) + 0x02

#define Sn_PORT0      0x0004
#define Sn_PORT1      0x0005
#define Sn_MR         0x0000
#define Sn_CR         0x0001
#define Sn_SR         0x0003
#define Sn_RX_RSR0    0x0026
#define Sn_RX_RSR1    0x0027
#define Sn_RX_RD0     0x0028
#define Sn_RX_RD1     0x0029
#define Sn_TX_FSR0    0x0020
#define Sn_TX_FSR1    0x0021
#define Sn_TXBUF_SIZE 0x001F
#define Sn_TX_WR0     0x0024
#define Sn_TX_WR1     0x0025

#define Sn_TX_RD0     0x0022
#define Sn_TX_RD1     0x0023

#define Sn_DIPR0      0x000C
#define Sn_DIPR1      0x000D
#define Sn_DIPR2      0x000E
#define Sn_DIPR3      0x000F
#define Sn_DPORT0     0x0010
#define Sn_DPORT1     0x0011

#define RWB_READ 0
#define RWB_WRITE 1

#define OM_FDM1 0x01

/*
typedef enum{
	TCP_MODE = 0x1,
	UDP_MODE = 0x2
} Socket_ModeTypeDef;
*/

typedef struct{
	uint8_t *mac_address;
	uint8_t *gw;
	uint8_t *subnet_mask;
	uint8_t *ip;
} W5500_InitTypeDef;

void w5500_writeReg(uint8_t op, uint16_t address, uint8_t data);
uint8_t w5500_readReg(uint8_t op, uint16_t address);

void w5500_softReset(void);
void w5500_setMACAddr(uint8_t *mac_address);
void w5500_setGateway(uint8_t *gw);
void w5500_setSubNetMask(uint8_t *subnet_mask);

void w5500_setSourceIP(uint8_t ip[4]);
void w5500_setSourcePort(uint8_t Socket, uint8_t port);

void w5500_setDestinationIP(uint8_t Socket, uint8_t ip[4]);
void w5500_setDestinationPort(uint8_t Socket, uint8_t port);

uint16_t w5500_getRXReadPointer(uint8_t Socket);

uint16_t w5500_getTXWritePointer(uint8_t Socket);
uint16_t w5500_getTXReadPointer(uint8_t Socket);

uint16_t w5500_getTXFreeSize(uint8_t Socket);
uint8_t w5500_getTXBufSize(uint8_t Socket);

uint16_t w5500_getRXReceivedSize(uint8_t Socket);

void w5500_setWritePointer(uint8_t Socket, uint8_t write_pointer);

void w5500_send(uint8_t Socket);
void w5500_closeSocket(uint8_t Socket);
void w5500_disconnSocket(uint8_t Socket);

uint8_t w5500_getSocketStatus(uint8_t Socket);
void w5500_openSocket(uint8_t Socket, uint16_t mode);
void w5500_listenSocket(uint8_t Socket);
void w5500_checkChipVersion(void);
void ErrorHandler(void);
void w5500_init(W5500_InitTypeDef *source);
#endif /* __MAIN_H__ */
