/*
 * w5500.c
 *
 *  Created on: Sep 30, 2018
 *      Author: ioz
 */
#include "w5500.h"

extern SPI_HandleTypeDef hspi1;

void w5500_init(W5500_InitTypeDef *source){

	w5500_checkChipVersion();

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

void w5500_setSourcePort(uint8_t Socket, uint8_t port){
	w5500_writeReg(BSB_Sn(Socket), Sn_PORT0, port>>8);
	w5500_writeReg(BSB_Sn(Socket), Sn_PORT1, port);
}

void w5500_setDestinationIP(uint8_t Socket, uint8_t ip[4]){
	w5500_writeReg(BSB_Sn(Socket), Sn_DIPR0, ip[0]);
	w5500_writeReg(BSB_Sn(Socket), Sn_DIPR1, ip[1]);
	w5500_writeReg(BSB_Sn(Socket), Sn_DIPR2, ip[2]);
	w5500_writeReg(BSB_Sn(Socket), Sn_DIPR3, ip[3]);
}

void w5500_setDestinationPort(uint8_t Socket, uint8_t port){
	w5500_writeReg(BSB_Sn(Socket), Sn_DPORT0, port>>8);
	w5500_writeReg(BSB_Sn(Socket), Sn_DPORT1, port);
}

uint16_t w5500_getRXReadPointer(uint8_t Socket){
	uint16_t read_pointer;
	uint8_t rd0, rd1;
	rd0 = w5500_readReg(BSB_Sn(Socket), Sn_RX_RD0);
	rd1 = w5500_readReg(BSB_Sn(Socket), Sn_RX_RD1);
	read_pointer = rd0<<8|rd1;
	return read_pointer;
}

uint16_t w5500_getRXWritePointer(uint8_t Socket){
	uint16_t write_pointer;
	uint8_t wr0, wr1;
	wr0 = w5500_readReg(BSB_Sn(Socket), Sn_RX_WR0);
	wr1 = w5500_readReg(BSB_Sn(Socket), Sn_RX_WR1);
	write_pointer = wr0<<8|wr1;
	return write_pointer;
}

uint16_t w5500_getTXWritePointer(uint8_t Socket){
	uint16_t write_pointer;
	uint8_t wr0, wr1;
	wr0 = w5500_readReg(BSB_Sn(Socket), Sn_TX_WR0);
	wr1 = w5500_readReg(BSB_Sn(Socket), Sn_TX_WR1);
	write_pointer = wr0<<8|wr1;
	return write_pointer;
}

uint16_t w5500_getTXReadPointer(uint8_t Socket){
	uint16_t read_pointer;
	uint8_t rd0, rd1;
	rd0 = w5500_readReg(BSB_Sn(Socket), Sn_TX_RD0);
	rd1 = w5500_readReg(BSB_Sn(Socket), Sn_TX_RD1);
	read_pointer = rd0<<8|rd1;
	return read_pointer;
}

uint16_t w5500_getTXFreeSize(uint8_t Socket){
	uint16_t size;
	uint8_t fsr0, fsr1;
	fsr0 = w5500_readReg(BSB_Sn(Socket), Sn_TX_FSR0);
	fsr1 = w5500_readReg(BSB_Sn(Socket), Sn_TX_FSR1);
	size = fsr0<<8|fsr1;

	return size;
}

uint8_t w5500_getTXBufSize(uint8_t Socket){
	return w5500_readReg(BSB_Sn(Socket), Sn_TXBUF_SIZE);
}

uint16_t w5500_getRXReceivedSize(uint8_t Socket){
	uint16_t size;
	uint8_t rsr0, rsr1;
	rsr0 = w5500_readReg(BSB_Sn(Socket), Sn_RX_RSR0);
	rsr1 = w5500_readReg(BSB_Sn(Socket), Sn_RX_RSR1);
	size = rsr0<<8|rsr1;

	return size;
}

void w5500_setTXWritePointer(uint8_t Socket, uint16_t txwritepointer){
	w5500_writeReg(BSB_Sn(Socket), Sn_TX_WR0, txwritepointer >> 8);
	w5500_writeReg(BSB_Sn(Socket), Sn_TX_WR1, txwritepointer & 0xFF);
}

void w5500_setTXReadPointer(uint8_t Socket, uint16_t txreadpointer){
	w5500_writeReg(BSB_Sn(Socket), Sn_TX_RD0, txreadpointer >> 8);
	w5500_writeReg(BSB_Sn(Socket), Sn_TX_RD1, txreadpointer & 0xFF);
}

uint8_t w5500_getRXBufByte(uint8_t Socket, uint16_t rxreadpointer){
	return w5500_readReg(BSB_Sn_RX(Socket), rxreadpointer);
}

void w5500_checkChipVersion(){
	uint8_t v = w5500_readReg(BSB_COMMON, VERSIONR);
	if(v != CHIPVERSION_0X04){
		ErrorHandler();
	}
}

uint8_t w5500_getSocketStatus(uint8_t Socket){
	return w5500_readReg(BSB_Sn(Socket), Sn_SR);
}

void w5500_openSocket(uint8_t Socket, uint16_t mode){
	w5500_writeReg(BSB_Sn(Socket), Sn_MR, mode);
	w5500_writeReg(BSB_Sn(Socket), Sn_CR, OPEN);
	HAL_Delay(1000);
	uint8_t status;

	while(1){
		status = w5500_getSocketStatus(Socket);
		if(status == SOCK_INIT){
			break;
		}
	}
}

void w5500_closeSocket(uint8_t Socket){
	w5500_writeReg(BSB_Sn(Socket), Sn_CR, CLOSE);
	HAL_Delay(1000);
}

void w5500_disconnSocket(uint8_t Socket){
	w5500_writeReg(BSB_Sn(Socket), Sn_CR, DISCON);
	HAL_Delay(1000);
}

void w5500_listenSocket(uint8_t Socket){
	w5500_writeReg(BSB_Sn(Socket), Sn_CR, LISTEN); //LISTEN SOCKET
	HAL_Delay(1000);
	uint8_t status;

	while(1){
		status = w5500_getSocketStatus(Socket);
		if(status == SOCK_LISTEN){
			break;
		}
	}
}

void w5500_send(uint8_t Socket){
	w5500_writeReg(BSB_Sn(Socket), Sn_CR, SEND);
	HAL_Delay(1000);
}

void w5500_recv(uint8_t Socket){
	w5500_writeReg(BSB_Sn(Socket), Sn_CR, RECV);
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
	data = rbuf[3];
	return data;
}

void ErrorHandler(){
	while(1){

	}
}
