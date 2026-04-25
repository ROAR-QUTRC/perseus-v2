#ifndef _SCSERIAL_H
#define _SCSERIAL_H

#include <Arduino.h>
#include <HardwareSerial.h>

#include "SCS.h"

class SCSerial : public SCS
{
public:
    SCSerial();
    SCSerial(uint8_t End);
    SCSerial(uint8_t End, uint8_t Level);

protected:
    int writeSCS(unsigned char* nDat, int nLen);  // Output nLen bytes
    int readSCS(unsigned char* nDat, int nLen);   // Input nLen bytes
    int writeSCS(unsigned char bDat);             // Output 1 byte
    void rFlushSCS();
    void wFlushSCS();

public:
    unsigned long IOTimeOut;  // Input/output timeout in milliseconds
    int Err;

public:
    virtual int getErr() { return Err; }
    virtual int setBaudRate(int baudRate);

    //   - txPin: TX pin (typically GPIO17 for UART2)
    //   - rxPin: RX pin (typically GPIO16 for UART2)
    //   - dirPin: RS485 direction control pin (-1 if auto-controlled by HAT)
    virtual bool begin(int baudRate, int8_t rxPin = -1, int8_t txPin = -1, int8_t dirPin = -1);
    virtual bool begin(HardwareSerial* serial, int baudRate, int8_t rxPin = -1, int8_t txPin = -1, int8_t dirPin = -1);
    virtual void end();

protected:
    HardwareSerial* pSerial;
    int8_t directionPin;  // RS485 direction control pin (-1 if not used)
    unsigned char txBuf[255];
    int txBufLen;
    bool ownSerial;  // True if we created the HardwareSerial instance

    void setTxMode();  // Set RS485 to transmit mode
    void setRxMode();  // Set RS485 to receive mode
};

#endif
