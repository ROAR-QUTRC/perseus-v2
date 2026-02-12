#include "SCSerial.h"

SCSerial::SCSerial()
{
    IOTimeOut = 10;  // 10ms timeout
    pSerial = nullptr;
    directionPin = -1;
    txBufLen = 0;
    ownSerial = false;
    Err = 0;
}

SCSerial::SCSerial(uint8_t End)
    : SCS(End)
{
    IOTimeOut = 10;  // 10ms timeout
    pSerial = nullptr;
    directionPin = -1;
    txBufLen = 0;
    ownSerial = false;
    Err = 0;
}

SCSerial::SCSerial(uint8_t End, uint8_t Level)
    : SCS(End, Level)
{
    IOTimeOut = 10;  // 10ms timeout
    pSerial = nullptr;
    directionPin = -1;
    txBufLen = 0;
    ownSerial = false;
    Err = 0;
}

void SCSerial::setTxMode()
{
    if (directionPin >= 0)
    {
        digitalWrite(directionPin, HIGH);  // HIGH = Transmit mode
        delayMicroseconds(2);              // 2μs
    }
}

void SCSerial::setRxMode()
{
    if (directionPin >= 0)
    {
        digitalWrite(directionPin, LOW);  // LOW = Receive mode
        delayMicroseconds(2);             // 2μs
    }
}

bool SCSerial::begin(int baudRate, int8_t rxPin, int8_t txPin, int8_t dirPin)
{
    // Use Serial2 by default on ESP32
    pSerial = &Serial2;
    ownSerial = false;
    return begin(pSerial, baudRate, rxPin, txPin, dirPin);
}

bool SCSerial::begin(HardwareSerial* serial, int baudRate, int8_t rxPin, int8_t txPin, int8_t dirPin)
{
    if (serial == nullptr)
    {
        return false;
    }

    // Use provided serial port
    pSerial = serial;
    directionPin = dirPin;

    // Setup direction control pin if specified
    if (directionPin >= 0)
    {
        pinMode(directionPin, OUTPUT);
        setRxMode();  // Start in receive mode
    }

    // Configure serial port
    // For RS485 servos: 8N1 (8 data bits, no parity, 1 stop bit)
    if (rxPin >= 0 && txPin >= 0)
    {
        pSerial->begin(baudRate, SERIAL_8N1, rxPin, txPin);
    }
    else
    {
        pSerial->begin(baudRate, SERIAL_8N1);
    }

    // Set timeout for reads
    pSerial->setTimeout(IOTimeOut);

    Serial.printf("SCSerial initialised at %d baud\n", baudRate);
    if (directionPin >= 0)
    {
        Serial.printf("RS485 direction pin: GPIO%d\n", directionPin);
    }

    return true;
}

int SCSerial::setBaudRate(int baudRate)
{
    if (pSerial == nullptr)
    {
        return -1;
    }

    pSerial->updateBaudRate(baudRate);
    return 1;
}

int SCSerial::readSCS(unsigned char* nDat, int nLen)
{
    if (pSerial == nullptr)
    {
        return 0;
    }

    int rvLen = 0;
    unsigned long startTime = millis();

    while (rvLen < nLen)
    {
        if (pSerial->available())
        {
            nDat[rvLen++] = pSerial->read();
        }

        // Check for timeout
        if ((millis() - startTime) >= IOTimeOut)
        {
            break;
        }

        // Small yield to prevent watchdog issues on ESP32
        if (rvLen < nLen && !pSerial->available())
        {
            delayMicroseconds(50);  // 50μs yield
        }
    }

    return rvLen;
}

int SCSerial::writeSCS(unsigned char* nDat, int nLen)
{
    while (nLen--)
    {
        txBuf[txBufLen++] = *nDat++;
    }
    return txBufLen;
}

int SCSerial::writeSCS(unsigned char bDat)
{
    txBuf[txBufLen++] = bDat;
    return txBufLen;
}

void SCSerial::rFlushSCS()
{
    if (pSerial != nullptr)
    {
        // Flush receive buffer
        while (pSerial->available())
        {
            pSerial->read();
        }
    }
}

void SCSerial::wFlushSCS()
{
    if (txBufLen && pSerial != nullptr)
    {
        setTxMode();  // Switch to transmit mode

        pSerial->write(txBuf, txBufLen);
        pSerial->flush();  // Wait for transmission to complete

        setRxMode();  // Switch back to receive mode
        txBufLen = 0;
    }
}

void SCSerial::end()
{
    if (pSerial != nullptr)
    {
        pSerial->end();
        pSerial = nullptr;
    }

    if (directionPin >= 0)
    {
        pinMode(directionPin, INPUT);
        directionPin = -1;
    }
}
