/*
 * Copyright 2018 Paul Stoffregen
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include <Arduino.h>
#include "Ethernet.h"
#include "w5500.h"


/***************************************************/
/**            Default SS pin setting             **/
/***************************************************/

#define SS_PIN_DEFAULT  10




// W5500 controller instance
uint8_t  W5500Class::CH_BASE_MSB;
uint8_t  W5500Class::ss_pin = SS_PIN_DEFAULT;
#ifdef ETHERNET_LARGE_BUFFERS
uint16_t W5500Class::SSIZE = 2048;
uint16_t W5500Class::SMASK = 0x07FF;
#endif
W5500Class W5500;


uint8_t W5500Class::init(int spiMisoPin, int spiMosiPin, int spiSclkPin)
{
	static bool initialized = false;
	uint8_t i;

	if (initialized) return 1;

	// Many Ethernet shields have a CAT811 or similar reset chip
	// connected to W5100 or W5200 chips.  The W5200 will not work at
	// all, and may even drive its MISO pin, until given an active low
	// reset pulse!  The CAT811 has a 240 ms typical pulse length, and
	// a 400 ms worst case maximum pulse length.  MAX811 has a worst
	// case maximum 560 ms pulse length.  This delay is meant to wait
	// until the reset pulse is ended.  If your hardware has a shorter
	// reset time, this can be edited or removed.
	delay(560);
	//Serial.println("w5100 init");
	if ((spiMisoPin == -1) || (spiMosiPin == -1) || (spiSclkPin == -1)) {
		SPI.begin();
	} else {
		SPI.begin(spiSclkPin, spiMisoPin, spiMosiPin);
	}
	initSS();
	resetSS();
	SPI.beginTransaction(SPI_ETHERNET_SETTINGS);

	CH_BASE_MSB = 0x10;
#ifdef ETHERNET_LARGE_BUFFERS
#if MAX_SOCK_NUM <= 1
	SSIZE = 16384;
#elif MAX_SOCK_NUM <= 2
	SSIZE = 8192;
#elif MAX_SOCK_NUM <= 4
	SSIZE = 4096;
#else
	SSIZE = 2048;
#endif
	SMASK = SSIZE - 1;
	for (i=0; i<MAX_SOCK_NUM; i++) {
		writeSnRX_SIZE(i, SSIZE >> 10);
		writeSnTX_SIZE(i, SSIZE >> 10);
	}
	for (; i<8; i++) {
		writeSnRX_SIZE(i, 0);
		writeSnTX_SIZE(i, 0);
	}
#endif

	SPI.endTransaction();
	initialized = true;
	return 1; // successful init
}

uint8_t W5500Class::init() {
	return init(-1, -1, -1);
}

// Soft reset the WIZnet chip, by writing to its MR register reset bit
uint8_t W5500Class::softReset(void)
{
	uint16_t count=0;

	//Serial.println("WIZnet soft reset");
	// write to reset bit
	writeMR(0x80);
	// then wait for soft reset to complete
	do {
		uint8_t mr = readMR();
		//Serial.print("mr=");
		//Serial.println(mr, HEX);
		if (mr == 0) return 1;
		delay(1);
	} while (++count < 20);
	return 0;
}

W5500Linkstatus W5500Class::getLinkStatus()
{
	uint8_t phystatus;

	if (!init()) return UNKNOWN;
	SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
	phystatus = readPHYCFGR();
	SPI.endTransaction();
	if (phystatus & 0x01) return LINK_ON;
	return LINK_OFF;
}

uint16_t W5500Class::write(uint16_t addr, const uint8_t *buf, uint16_t len)
{
	uint8_t cmd[8];
	
		setSS();
		if (addr < 0x100) {
			// common registers 00nn
			cmd[0] = 0;
			cmd[1] = addr & 0xFF;
			cmd[2] = 0x04;
		} else if (addr < 0x8000) {
			// socket registers  10nn, 11nn, 12nn, 13nn, etc
			cmd[0] = 0;
			cmd[1] = addr & 0xFF;
			cmd[2] = ((addr >> 3) & 0xE0) | 0x0C;
		} else if (addr < 0xC000) {
			// transmit buffers  8000-87FF, 8800-8FFF, 9000-97FF, etc
			//  10## #nnn nnnn nnnn
			cmd[0] = addr >> 8;
			cmd[1] = addr & 0xFF;
			#if defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 1
			cmd[2] = 0x14;                       // 16K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 2
			cmd[2] = ((addr >> 8) & 0x20) | 0x14; // 8K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 4
			cmd[2] = ((addr >> 7) & 0x60) | 0x14; // 4K buffers
			#else
			cmd[2] = ((addr >> 6) & 0xE0) | 0x14; // 2K buffers
			#endif
		} else {
			// receive buffers
			cmd[0] = addr >> 8;
			cmd[1] = addr & 0xFF;
			#if defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 1
			cmd[2] = 0x1C;                       // 16K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 2
			cmd[2] = ((addr >> 8) & 0x20) | 0x1C; // 8K buffers
			#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 4
			cmd[2] = ((addr >> 7) & 0x60) | 0x1C; // 4K buffers
			#else
			cmd[2] = ((addr >> 6) & 0xE0) | 0x1C; // 2K buffers
			#endif
		}
		if (len <= 5) {
			for (uint8_t i=0; i < len; i++) {
				cmd[i + 3] = buf[i];
			}
			SPI.transfer(cmd, len + 3);
		} else {
			SPI.transfer(cmd, 3);
			SPI.transferBytes(buf, NULL, len);
		}
		resetSS();
	return len;
}

uint16_t W5500Class::read(uint16_t addr, uint8_t *buf, uint16_t len)
{
	uint8_t cmd[4];

	setSS();
	if (addr < 0x100) {
		// common registers 00nn
		cmd[0] = 0;
		cmd[1] = addr & 0xFF;
		cmd[2] = 0x00;
	} else if (addr < 0x8000) {
		// socket registers  10nn, 11nn, 12nn, 13nn, etc
		cmd[0] = 0;
		cmd[1] = addr & 0xFF;
		cmd[2] = ((addr >> 3) & 0xE0) | 0x08;
	} else if (addr < 0xC000) {
		// transmit buffers  8000-87FF, 8800-8FFF, 9000-97FF, etc
		//  10## #nnn nnnn nnnn
		cmd[0] = addr >> 8;
		cmd[1] = addr & 0xFF;
		#if defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 1
		cmd[2] = 0x10;                       // 16K buffers
		#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 2
		cmd[2] = ((addr >> 8) & 0x20) | 0x10; // 8K buffers
		#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 4
		cmd[2] = ((addr >> 7) & 0x60) | 0x10; // 4K buffers
		#else
		cmd[2] = ((addr >> 6) & 0xE0) | 0x10; // 2K buffers
		#endif
	} else {
		// receive buffers
		cmd[0] = addr >> 8;
		cmd[1] = addr & 0xFF;
		#if defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 1
		cmd[2] = 0x18;                       // 16K buffers
		#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 2
		cmd[2] = ((addr >> 8) & 0x20) | 0x18; // 8K buffers
		#elif defined(ETHERNET_LARGE_BUFFERS) && MAX_SOCK_NUM <= 4
		cmd[2] = ((addr >> 7) & 0x60) | 0x18; // 4K buffers
		#else
		cmd[2] = ((addr >> 6) & 0xE0) | 0x18; // 2K buffers
		#endif
	}
	SPI.transfer(cmd, 3);
	memset(buf, 0, len);
	SPI.transfer(buf, len);
	resetSS();
	return len;
}

void W5500Class::execCmdSn(SOCKET s, SockCMD _cmd)
{
	// Send command to socket
	writeSnCR(s, _cmd);
	// Wait for command to complete
	while (readSnCR(s)) ;
}