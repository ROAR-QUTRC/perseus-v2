#include "SCS.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

SCS::SCS()
{
    Level = 1;  // All commands return acknowledgement except broadcast commands
    Error = 0;
}

SCS::SCS(uint8_t End)
{
    Level = 1;
    this->End = End;
    Error = 0;
}

SCS::SCS(uint8_t End, uint8_t Level)
{
    this->Level = Level;
    this->End = End;
    Error = 0;
}

// Split one 16-bit number into two 8-bit numbers
// DataL is low byte, DataH is high byte
void SCS::Host2SCS(uint8_t* DataL, uint8_t* DataH, uint16_t Data)
{
    if (End)
    {
        *DataL = (Data >> 8);
        *DataH = (Data & 0xff);
    }
    else
    {
        *DataH = (Data >> 8);
        *DataL = (Data & 0xff);
    }
}

// Combine two 8-bit numbers into one 16-bit number
// DataL is low byte, DataH is high byte
uint16_t SCS::SCS2Host(uint8_t DataL, uint8_t DataH)
{
    uint16_t Data;
    if (End)
    {
        Data = DataL;
        Data <<= 8;
        Data |= DataH;
    }
    else
    {
        Data = DataH;
        Data <<= 8;
        Data |= DataL;
    }
    return Data;
}

void SCS::writeBuf(uint8_t ID, uint8_t MemAddr, uint8_t* nDat, uint8_t nLen, uint8_t Fun)
{
    uint8_t msgLen = 2;
    uint8_t bBuf[6];
    uint8_t CheckSum = 0;
    bBuf[0] = 0xff;
    bBuf[1] = 0xff;
    bBuf[2] = ID;
    bBuf[4] = Fun;
    if (nDat)
    {
        msgLen += nLen + 1;
        bBuf[3] = msgLen;
        bBuf[5] = MemAddr;
        writeSCS(bBuf, 6);
    }
    else
    {
        bBuf[3] = msgLen;
        writeSCS(bBuf, 5);
    }
    CheckSum = ID + msgLen + Fun + MemAddr;
    uint8_t i = 0;
    if (nDat)
    {
        for (i = 0; i < nLen; i++)
        {
            CheckSum += nDat[i];
        }
        writeSCS(nDat, nLen);
    }
    writeSCS(~CheckSum);
}

// Normal write command
// Servo ID, MemAddr memory table address, data to write, write length
int SCS::genWrite(uint8_t ID, uint8_t MemAddr, uint8_t* nDat, uint8_t nLen)
{
    rFlushSCS();
    writeBuf(ID, MemAddr, nDat, nLen, INST_WRITE);
    wFlushSCS();
    return Ack(ID);
}

// Asynchronous write command
// Servo ID, MemAddr memory table address, data to write, write length
int SCS::regWrite(uint8_t ID, uint8_t MemAddr, uint8_t* nDat, uint8_t nLen)
{
    rFlushSCS();
    writeBuf(ID, MemAddr, nDat, nLen, INST_REG_WRITE);
    wFlushSCS();
    return Ack(ID);
}

// Asynchronous write execution command
// Servo ID
int SCS::RegWriteAction(uint8_t ID)
{
    rFlushSCS();
    writeBuf(ID, 0, NULL, 0, INST_REG_ACTION);
    wFlushSCS();
    return Ack(ID);
}

// Synchronous write command
// Servo ID[] array, IDN array length, MemAddr memory table address, data to write, write length
void SCS::syncWrite(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t* nDat, uint8_t nLen)
{
    rFlushSCS();
    uint8_t mesLen = ((nLen + 1) * IDN + 4);
    uint8_t Sum = 0;
    uint8_t bBuf[7];
    bBuf[0] = 0xff;
    bBuf[1] = 0xff;
    bBuf[2] = 0xfe;
    bBuf[3] = mesLen;
    bBuf[4] = INST_SYNC_WRITE;
    bBuf[5] = MemAddr;
    bBuf[6] = nLen;
    writeSCS(bBuf, 7);

    Sum = 0xfe + mesLen + INST_SYNC_WRITE + MemAddr + nLen;
    uint8_t i, j;
    for (i = 0; i < IDN; i++)
    {
        writeSCS(ID[i]);
        writeSCS(nDat + i * nLen, nLen);
        Sum += ID[i];
        for (j = 0; j < nLen; j++)
        {
            Sum += nDat[i * nLen + j];
        }
    }
    writeSCS(~Sum);
    wFlushSCS();
}

int SCS::writeByte(uint8_t ID, uint8_t MemAddr, uint8_t bDat)
{
    rFlushSCS();
    writeBuf(ID, MemAddr, &bDat, 1, INST_WRITE);
    wFlushSCS();
    return Ack(ID);
}

int SCS::writeWord(uint8_t ID, uint8_t MemAddr, uint16_t wDat)
{
    uint8_t bBuf[2];
    Host2SCS(bBuf + 0, bBuf + 1, wDat);
    rFlushSCS();
    writeBuf(ID, MemAddr, bBuf, 2, INST_WRITE);
    wFlushSCS();
    return Ack(ID);
}

// Read command
// Servo ID, MemAddr memory table address, return data nData, data length nLen
int SCS::Read(uint8_t ID, uint8_t MemAddr, uint8_t* nData, uint8_t nLen)
{
    rFlushSCS();
    writeBuf(ID, MemAddr, &nLen, 1, INST_READ);
    wFlushSCS();

    uint8_t bBuf[255];
    uint8_t i;
    uint8_t calSum = 0;
    int Size = readSCS(bBuf, nLen + 6);
    // printf("nLen+6 = %d, Size = %d\n", nLen+6, Size);
    if (Size != (nLen + 6))
    {
        return 0;
    }
    // for(i=0; i<Size; i++){
    // printf("%x\n", bBuf[i]);
    //}
    if (bBuf[0] != 0xff || bBuf[1] != 0xff)
    {
        return 0;
    }
    for (i = 2; i < (Size - 1); i++)
    {
        calSum += bBuf[i];
    }
    calSum = ~calSum;
    if (calSum != bBuf[Size - 1])
    {
        return 0;
    }
    memcpy(nData, bBuf + 5, nLen);
    Error = bBuf[4];
    return nLen;
}

// Read 1 byte, return -1 on timeout
int SCS::readByte(uint8_t ID, uint8_t MemAddr)
{
    uint8_t bDat;
    int Size = Read(ID, MemAddr, &bDat, 1);
    if (Size != 1)
    {
        return -1;
    }
    else
    {
        return bDat;
    }
}

// Read 2 bytes, return -1 on timeout
int SCS::readWord(uint8_t ID, uint8_t MemAddr)
{
    uint8_t nDat[2];
    int Size;
    uint16_t wDat;
    Size = Read(ID, MemAddr, nDat, 2);
    if (Size != 2)
        return -1;
    wDat = SCS2Host(nDat[0], nDat[1]);
    return wDat;
}

// Ping command, return servo ID, return -1 on timeout
int SCS::Ping(uint8_t ID)
{
    rFlushSCS();
    writeBuf(ID, 0, NULL, 0, INST_PING);
    wFlushSCS();
    Error = 0;

    uint8_t bBuf[6];
    uint8_t i;
    uint8_t calSum = 0;
    int Size = readSCS(bBuf, 6);
    if (Size != 6)
    {
        return -1;
    }
    if (bBuf[0] != 0xff || bBuf[1] != 0xff)
    {
        return -1;
    }
    if (bBuf[2] != ID && ID != 0xfe)
    {
        return -1;
    }
    if (bBuf[3] != 2)
    {
        return -1;
    }
    for (i = 2; i < (Size - 1); i++)
    {
        calSum += bBuf[i];
    }
    calSum = ~calSum;
    if (calSum != bBuf[Size - 1])
    {
        return -1;
    }
    Error = bBuf[2];
    return Error;
}

int SCS::Ack(uint8_t ID)
{
    Error = 0;
    if (ID != 0xfe && Level)
    {
        uint8_t bBuf[6];
        uint8_t i;
        uint8_t calSum = 0;
        int Size = readSCS(bBuf, 6);
        if (Size != 6)
        {
            return 0;
        }
        if (bBuf[0] != 0xff || bBuf[1] != 0xff)
        {
            return 0;
        }
        if (bBuf[2] != ID)
        {
            return 0;
        }
        if (bBuf[3] != 2)
        {
            return 0;
        }
        for (i = 2; i < (Size - 1); i++)
        {
            calSum += bBuf[i];
        }
        calSum = ~calSum;
        if (calSum != bBuf[Size - 1])
        {
            return 0;
        }
        Error = bBuf[4];
    }
    return 1;
}

int SCS::syncReadPacketTx(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint8_t nLen)
{
    rFlushSCS();
    syncReadRxPacketLen = nLen;
    uint8_t checkSum = (4 + 0xfe) + IDN + MemAddr + nLen + INST_SYNC_READ;
    uint8_t i;
    writeSCS(0xff);
    writeSCS(0xff);
    writeSCS(0xfe);
    writeSCS(IDN + 4);
    writeSCS(INST_SYNC_READ);
    writeSCS(MemAddr);
    writeSCS(nLen);
    for (i = 0; i < IDN; i++)
    {
        writeSCS(ID[i]);
        checkSum += ID[i];
    }
    checkSum = ~checkSum;
    writeSCS(checkSum);
    wFlushSCS();

    syncReadRxBuffLen = readSCS(syncReadRxBuff, syncReadRxBuffMax);
    return syncReadRxBuffLen;
}

void SCS::syncReadBegin(uint8_t IDN, uint8_t rxLen)
{
    syncReadRxBuffMax = IDN * (rxLen + 6);
    syncReadRxBuff = new uint8_t[syncReadRxBuffMax];
}

void SCS::syncReadEnd()
{
    if (syncReadRxBuff)
    {
        delete syncReadRxBuff;
        syncReadRxBuff = NULL;
    }
}

int SCS::syncReadPacketRx(uint8_t ID, uint8_t* nDat)
{
    uint16_t syncReadRxBuffIndex = 0;
    syncReadRxPacket = nDat;
    syncReadRxPacketIndex = 0;
    while ((syncReadRxBuffIndex + 6 + syncReadRxPacketLen) <= syncReadRxBuffLen)
    {
        uint8_t bBuf[] = {0, 0, 0};
        uint8_t calSum = 0;
        while (syncReadRxBuffIndex < syncReadRxBuffLen)
        {
            bBuf[0] = bBuf[1];
            bBuf[1] = bBuf[2];
            bBuf[2] = syncReadRxBuff[syncReadRxBuffIndex++];
            if (bBuf[0] == 0xff && bBuf[1] == 0xff && bBuf[2] != 0xff)
            {
                break;
            }
        }
        if (bBuf[2] != ID)
        {
            continue;
        }
        if (syncReadRxBuff[syncReadRxBuffIndex++] != (syncReadRxPacketLen + 2))
        {
            continue;
        }
        Error = syncReadRxBuff[syncReadRxBuffIndex++];
        calSum = ID + (syncReadRxPacketLen + 2) + Error;
        for (uint8_t i = 0; i < syncReadRxPacketLen; i++)
        {
            syncReadRxPacket[i] = syncReadRxBuff[syncReadRxBuffIndex++];
            calSum += syncReadRxPacket[i];
        }
        calSum = ~calSum;
        if (calSum != syncReadRxBuff[syncReadRxBuffIndex++])
        {
            return 0;
        }
        return syncReadRxPacketLen;
    }
    return 0;
}

int SCS::syncReadRxPacketToByte()
{
    if (syncReadRxPacketIndex >= syncReadRxPacketLen)
    {
        return -1;
    }
    return syncReadRxPacket[syncReadRxPacketIndex++];
}

int SCS::syncReadRxPacketToWord(uint8_t negBit)
{
    if ((syncReadRxPacketIndex + 1) >= syncReadRxPacketLen)
    {
        return -1;
    }
    int Word = SCS2Host(syncReadRxPacket[syncReadRxPacketIndex], syncReadRxPacket[syncReadRxPacketIndex + 1]);
    syncReadRxPacketIndex += 2;
    if (negBit)
    {
        if (Word & (1 << negBit))
        {
            Word = -(Word & ~(1 << negBit));
        }
    }
    return Word;
}