/* Copyright 2018 Paul Stoffregen
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this
 * software and associated documentation files (the "Software"), to deal in the Software
 * without restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <Arduino.h>
#include "Ethernet.h"
#include "utility/w5500.h"

volatile bool INTnFlag;

int EthernetClass::begin(uint8_t *mac, unsigned long timeout, unsigned long responseTimeout)
{

	// Initialise the basic info
	if (W5500.init() == 0) return 0;
	SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
	W5500.setMACAddress(mac);
	W5500.setIPAddress(IPAddress(0,0,0,0).raw_address());
	SPI.endTransaction();
	return 0;
}

void EthernetClass::begin(uint8_t *mac, IPAddress ip)
{
	begin(mac, ip, -1, -1, -1);
}
void EthernetClass::begin(uint8_t *mac, IPAddress ip, int spiMisoPin, int spiMosiPin, int spiSclkPin, int ETH_intN)
{
	// Assume the DNS server will be the machine on the same network as the local IP
	// but with last octet being '1'
	IPAddress dns = ip;
	dns[3] = 1;
	begin(mac, ip, dns, spiMisoPin, spiMosiPin, spiSclkPin, ETH_intN);
}

void EthernetClass::begin(uint8_t *mac, IPAddress ip, IPAddress dns)
{
	begin(mac, ip, dns, -1, -1, -1, -1);
}

void EthernetClass::begin(uint8_t *mac, IPAddress ip, IPAddress dns, int spiMisoPin, int spiMosiPin, int spiSclkPin, int ETH_intN)
{
	// Assume the gateway will be the machine on the same network as the local IP
	// but with last octet being '1'
	IPAddress gateway = ip;
	gateway[3] = 1;
	begin(mac, ip, dns, gateway, spiMisoPin, spiMosiPin, spiSclkPin, ETH_intN);
}

void EthernetClass::begin(uint8_t *mac, IPAddress ip, IPAddress dns, IPAddress gateway)
{
	begin(mac, ip, dns, gateway, -1, -1, -1, -1);
}

void EthernetClass::begin(uint8_t *mac, IPAddress ip, IPAddress dns, IPAddress gateway, int spiMisoPin, int spiMosiPin, int spiSclkPin, int ETH_intN)
{
	IPAddress subnet(255, 255, 255, 0);
	begin(mac, ip, dns, gateway, subnet, spiMisoPin, spiMosiPin, spiSclkPin, ETH_intN);
}

void setRecvFlag() {
	// This method will reset the INTn pin to 0x00 after a write has been acknowledged
	// Serial.printf("SIMR %i SIR %i IR %i SnIR %i", W5500.readSIMR(),W5500.readSIR(), W5500.readIR(), W5500.readSnIR(0));
	// if (W5500.readSIR() > 0) {
	// 	// Reset the register and pull the INTn back down
	// 	W5500.writeSnIR(0, 0xff);
		
	// }	
	INTnFlag = true;
	// otherwise don't do anything
}

void EthernetClass::begin(uint8_t *mac, IPAddress ip, IPAddress dns, IPAddress gateway, IPAddress subnet)
{
	begin(mac, ip, dns, gateway, subnet, -1, -1, -1, -1);
}

void EthernetClass::begin(uint8_t *mac, IPAddress ip, IPAddress dns, IPAddress gateway, IPAddress subnet, int spiMisoPin, int spiMosiPin, int spiSclkPin, int ETH_intN)
{
	if (W5500.init(spiMisoPin, spiMosiPin, spiSclkPin) == 0) return;
	W5500.softReset();
	SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
	W5500.setMACAddress(mac);
	W5500.setIPAddress(ip._address.bytes);
	W5500.setGatewayIp(gateway._address.bytes);
	W5500.setSubnetMask(subnet._address.bytes);
	W5500.writeSIMR(0x01);
	W5500.writeSnIMR(0, 0x04);
	// Set Interupprt
	if (ETH_intN == -1) {
		ETH_intN = 9;
	}
	pinMode(ETH_intN, INPUT);
	//attachInterrupt(ETH_intN, setRecvFlag, FALLING);
	SPI.endTransaction();
}

bool EthernetClass::detectRead() {
	//if (INTnFlag) {
	if (!digitalRead(9)) {
		W5500.writeSnIR(0, 0xff);
		INTnFlag = false;
		return true;
	} else {
		return false;
	}
}

void EthernetClass::init(uint8_t sspin)
{
	W5500.setSS(sspin);
}

EthernetLinkStatus EthernetClass::linkStatus()
{
	switch (W5500.getLinkStatus()) {
		case UNKNOWN:  return Unknown;
		case LINK_ON:  return LinkON;
		case LINK_OFF: return LinkOFF;
		default:       return Unknown;
	}
}

int EthernetClass::maintain()
{
	return 0;
}


void EthernetClass::MACAddress(uint8_t *mac_address)
{
	SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
	W5500.getMACAddress(mac_address);
	SPI.endTransaction();
}

IPAddress EthernetClass::localIP()
{
	IPAddress ret;
	SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
	W5500.getIPAddress(ret.raw_address());
	SPI.endTransaction();
	return ret;
}

IPAddress EthernetClass::subnetMask()
{
	IPAddress ret;
	SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
	W5500.getSubnetMask(ret.raw_address());
	SPI.endTransaction();
	return ret;
}

IPAddress EthernetClass::gatewayIP()
{
	IPAddress ret;
	SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
	W5500.getGatewayIp(ret.raw_address());
	SPI.endTransaction();
	return ret;
}

void EthernetClass::setMACAddress(const uint8_t *mac_address)
{
	SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
	W5500.setMACAddress(mac_address);
	SPI.endTransaction();
}

void EthernetClass::setLocalIP(const IPAddress local_ip)
{
	SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
	IPAddress ip = local_ip;
	W5500.setIPAddress(ip.raw_address());
	SPI.endTransaction();
}

void EthernetClass::setSubnetMask(const IPAddress subnet)
{
	SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
	IPAddress ip = subnet;
	W5500.setSubnetMask(ip.raw_address());
	SPI.endTransaction();
}

void EthernetClass::setGatewayIP(const IPAddress gateway)
{
	SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
	IPAddress ip = gateway;
	W5500.setGatewayIp(ip.raw_address());
	SPI.endTransaction();
}

void EthernetClass::setRetransmissionTimeout(uint16_t milliseconds)
{
	if (milliseconds > 6553) milliseconds = 6553;
	SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
	W5500.setRetransmissionTime(milliseconds * 10);
	SPI.endTransaction();
}

void EthernetClass::setRetransmissionCount(uint8_t num)
{
	SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
	W5500.setRetransmissionCount(num);
	SPI.endTransaction();
}


EthernetClass Ethernet;