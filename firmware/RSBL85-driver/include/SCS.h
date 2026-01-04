
#ifndef _SCS_H
#define _SCS_H

#include "INST.h"

class SCS
{
public:
	SCS();
	SCS(uint8_t End);
	SCS(uint8_t End, uint8_t Level);
	int genWrite(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);					 // Normal write command
	int regWrite(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);					 // Asynchronous write command
	int RegWriteAction(uint8_t ID = 0xfe);													 // Asynchronous write execution command
	void syncWrite(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen); // Synchronous write command
	int writeByte(uint8_t ID, uint8_t MemAddr, uint8_t bDat);								 // Write 1 byte
	int writeWord(uint8_t ID, uint8_t MemAddr, uint16_t wDat);								 // Write 2 bytes
	int Read(uint8_t ID, uint8_t MemAddr, uint8_t *nData, uint8_t nLen);					 // Read command
	int readByte(uint8_t ID, uint8_t MemAddr);												 // Read 1 byte
	int readWord(uint8_t ID, uint8_t MemAddr);												 // Read 2 bytes
	int Ping(uint8_t ID);																	 // Ping command
	int syncReadPacketTx(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t nLen);			 // Synchronous read packet transmission
	int syncReadPacketRx(uint8_t ID, uint8_t *nDat);										 // Synchronous read return packet decoding, returns number of memory bytes on success, 0 on failure
	int syncReadRxPacketToByte();															 // Decode one byte
	int syncReadRxPacketToWrod(uint8_t negBit = 0);											 // Decode two bytes, negBit is direction bit, negBit=0 means no direction
	void syncReadBegin(uint8_t IDN, uint8_t rxLen);											 // Synchronous read begin
	void syncReadEnd();																		 // Synchronous read end
public:
	uint8_t Level; // Servo response level
	uint8_t End;   // Processor endianness structure
	uint8_t Error; // Servo status
	uint8_t syncReadRxPacketIndex;
	uint8_t syncReadRxPacketLen;
	uint8_t *syncReadRxPacket;
	uint8_t *syncReadRxBuff;
	uint16_t syncReadRxBuffLen;
	uint16_t syncReadRxBuffMax;

protected:
	virtual int writeSCS(unsigned char *nDat, int nLen) = 0;
	virtual int readSCS(unsigned char *nDat, int nLen) = 0;
	virtual int writeSCS(unsigned char bDat) = 0;
	virtual void rFlushSCS() = 0;
	virtual void wFlushSCS() = 0;

protected:
	void writeBuf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen, uint8_t Fun);
	void Host2SCS(uint8_t *DataL, uint8_t *DataH, uint16_t Data); // Split one 16-bit number into two 8-bit numbers
	uint16_t SCS2Host(uint8_t DataL, uint8_t DataH);			  // Combine two 8-bit numbers into one 16-bit number
	int Ack(uint8_t ID);										  // Return acknowledgement
};
#endif