/*
 * w5500.c
 *
 *  Created on: Sep 30, 2018
 *      Author: ioz
 */
#include "w5500.h"

extern SPI_HandleTypeDef hspi1;

void w5500_init(W5500_InitTypeDef *source){

	CheckChipVersion();

	w5500_softReset();
	w5500_setMACAddr(source->mac_address);
	w5500_setGateway(source->gw);
	w5500_setSubNetMask(source->subnet_mask);
	w5500_setSourceIP(source->ip);

}

void w5500_softReset(){
	w5500_writeReg(BSB_COMMON, MR, 0x80);
}

void w5500_setMACAddr(uint8_t *mac_address){
	w5500_writeReg(BSB_COMMON, SHAR0, mac_address[0]);
	w5500_writeReg(BSB_COMMON, SHAR1, mac_address[1]);
	w5500_writeReg(BSB_COMMON, SHAR2, mac_address[2]);
	w5500_writeReg(BSB_COMMON, SHAR3, mac_address[3]);
	w5500_writeReg(BSB_COMMON, SHAR4, mac_address[4]);
	w5500_writeReg(BSB_COMMON, SHAR5, mac_address[5]);
}

void w5500_setGateway(uint8_t *gw){
	w5500_writeReg(BSB_COMMON, GAR0, gw[0]);
	w5500_writeReg(BSB_COMMON, GAR1, gw[1]);
	w5500_writeReg(BSB_COMMON, GAR2, gw[2]);
	w5500_writeReg(BSB_COMMON, GAR3, gw[3]);
}

void w5500_setSubNetMask(uint8_t *subnet_mask){
	w5500_writeReg(BSB_COMMON, SUBR0, subnet_mask[0]);
	w5500_writeReg(BSB_COMMON, SUBR1, subnet_mask[1]);
	w5500_writeReg(BSB_COMMON, SUBR2, subnet_mask[2]);
	w5500_writeReg(BSB_COMMON, SUBR3, subnet_mask[3]);
}

void w5500_setSourceIP(uint8_t *ip){
	w5500_writeReg(BSB_COMMON, SIPR0, ip[0]);
	w5500_writeReg(BSB_COMMON, SIPR1, ip[1]);
	w5500_writeReg(BSB_COMMON, SIPR2, ip[2]);
	w5500_writeReg(BSB_COMMON, SIPR3, ip[3]);
}

void w5500_setSourcePort(uint8_t port){
	w5500_writeReg(BSB_S0, Sn_PORT0, port>>8);
	w5500_writeReg(BSB_S0, Sn_PORT1, port);
}

void w5500_setDestinationIP(uint8_t ip[4]){
	w5500_writeReg(BSB_S0, Sn_DIPR0, ip[0]);
	w5500_writeReg(BSB_S0, Sn_DIPR1, ip[1]);
	w5500_writeReg(BSB_S0, Sn_DIPR2, ip[2]);
	w5500_writeReg(BSB_S0, Sn_DIPR3, ip[3]);
}

void w5500_setDestinationPort(uint8_t port){
	w5500_writeReg(BSB_S0, Sn_DPORT0, port>>8);
	w5500_writeReg(BSB_S0, Sn_DPORT1, port);
}

uint16_t w5500_getReadPointer(){
	uint16_t read_pointer;
	uint8_t rd0, rd1;
	rd0 = w5500_readReg(BSB_S0, Sn_RX_RD0);
	rd1 = w5500_readReg(BSB_S0, Sn_RX_RD1);
	read_pointer = rd0<<8|rd1;
	return read_pointer;
}

uint16_t w5500_getWritePointer(){
	uint16_t write_pointer;
	uint8_t wr0, wr1;
	wr0 = w5500_readReg(BSB_S0, Sn_TX_WR0);
	wr1 = w5500_readReg(BSB_S0, Sn_TX_WR1);
	write_pointer = wr0<<8|wr1;
	return write_pointer;
}

uint16_t w5500_getTXFreeSize(){
	uint16_t size;
	uint8_t fsr0, fsr1;
	fsr0 = w5500_readReg(BSB_S0, Sn_TX_FSR0);
	fsr1 = w5500_readReg(BSB_S0, Sn_TX_FSR1);
	size = fsr0<<8|fsr1;

	return size;
}

uint8_t w5500_getTXBufSize(){
	return w5500_readReg(BSB_S0, Sn_TXBUF_SIZE);
}

uint16_t w5500_getRXReceivedSize(){
	uint16_t size;
	uint8_t rsr0, rsr1;
	rsr0 = w5500_readReg(BSB_S0, Sn_RX_RSR0);
	rsr1 = w5500_readReg(BSB_S0, Sn_RX_RSR1);
	size = rsr0<<8|rsr1;

	return size;
}

void w5500_setWritePointer(uint8_t write_pointer){
	w5500_writeReg(BSB_S0, Sn_TX_WR1, write_pointer);
}

void CheckChipVersion(){
	uint8_t v = w5500_readReg(BSB_COMMON, VERSIONR);
	if(v != CHIPVERSION_0X04){
		ErrorHandler();
	}
}

uint8_t w5500_SocketStatus(uint8_t op, uint8_t sock_num){
	return w5500_readReg(op, Sn_SR);
}

void w5500_openSocket(uint8_t sock_num, uint16_t mode){
	w5500_writeReg(BSB_S0, Sn_MR, mode);
	w5500_writeReg(BSB_S0, Sn_CR, OPEN);
	HAL_Delay(1000);
	uint8_t status;

	while(1){
		status = w5500_SocketStatus(BSB_S0, SOCKET_0);
		if(status == SOCK_INIT){
			break;
		}
	}
}

void w5500_listenSocket(uint8_t sock_num){
	w5500_writeReg(BSB_S0, Sn_CR, LISTEN); //LISTEN SOCKET
	HAL_Delay(1000);
	uint8_t status;

	while(1){
		status = w5500_SocketStatus(BSB_S0, SOCKET_0);
		if(status == SOCK_LISTEN){
			break;
		}
	}
}

void w5500_send(){
	w5500_writeReg(BSB_S0, Sn_CR, SEND);
	HAL_Delay(1000);
}

void w5500_writeReg(uint8_t block, uint16_t address, uint8_t data){
	uint8_t opcode = (block<<3)|OM_FDM1;
	uint8_t buf[] = {address >> 8, address, opcode|(RWB_WRITE<<2), data};
	HAL_StatusTypeDef write_stat;

	write_stat = HAL_SPI_Transmit(&hspi1, buf, 4, 10);
	HAL_Delay(100);
}

uint8_t w5500_readReg(uint8_t block, uint16_t address){
	uint8_t data;
	uint8_t opcode = (block<<3)|OM_FDM1;
	uint8_t wbuf[] = {address >> 8, address, opcode|RWB_READ<<2, 0xff};
	uint8_t rbuf[4];

	HAL_StatusTypeDef read_stat;

	read_stat = HAL_SPI_TransmitReceive(&hspi1, wbuf, rbuf, 4, 10);
	//HAL_Delay(1000);
	data = rbuf[3];
	return data;
}

void ErrorHandler(){
	while(1){

	}
}
