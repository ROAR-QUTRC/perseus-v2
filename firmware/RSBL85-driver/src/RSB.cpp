#include "RSB.h"

SMS_STS::SMS_STS()
{
    End = 0;
}

SMS_STS::SMS_STS(uint8_t End)
    : SCSerial(End)
{
}

SMS_STS::SMS_STS(uint8_t End, uint8_t Level)
    : SCSerial(End, Level)
{
}

int SMS_STS::WritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC)
{
    if (Position < 0)
    {
        Position = -Position;
        Position |= (1 << 15);
    }
    uint8_t bBuf[7];
    bBuf[0] = ACC;
    Host2SCS(bBuf + 1, bBuf + 2, Position);
    Host2SCS(bBuf + 3, bBuf + 4, 0);
    Host2SCS(bBuf + 5, bBuf + 6, Speed);

    return genWrite(ID, SMS_STS_ACC, bBuf, 7);
}

int SMS_STS::RegWritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC)
{
    if (Position < 0)
    {
        Position = -Position;
        Position |= (1 << 15);
    }
    uint8_t bBuf[7];
    bBuf[0] = ACC;
    Host2SCS(bBuf + 1, bBuf + 2, Position);
    Host2SCS(bBuf + 3, bBuf + 4, 0);
    Host2SCS(bBuf + 5, bBuf + 6, Speed);

    return regWrite(ID, SMS_STS_ACC, bBuf, 7);
}

void SMS_STS::SyncWritePosEx(uint8_t ID[], uint8_t IDN, int16_t Position[], uint16_t Speed[], uint8_t ACC[])
{
    uint8_t offbuf[IDN][7];
    for (uint8_t i = 0; i < IDN; i++)
    {
        if (Position[i] < 0)
        {
            Position[i] = -Position[i];
            Position[i] |= (1 << 15);
        }
        uint8_t bBuf[7];
        uint16_t V;
        if (Speed)
        {
            V = Speed[i];
        }
        else
        {
            V = 0;
        }
        if (ACC)
        {
            bBuf[0] = ACC[i];
        }
        else
        {
            bBuf[0] = 0;
        }
        Host2SCS(bBuf + 1, bBuf + 2, Position[i]);
        Host2SCS(bBuf + 3, bBuf + 4, 0);
        Host2SCS(bBuf + 5, bBuf + 6, V);
        memcpy(offbuf[i], bBuf, 7);
    }
    syncWrite(ID, IDN, SMS_STS_ACC, (uint8_t*)offbuf, 7);
}

int SMS_STS::WritePosWithTime(uint8_t ID, int16_t Position, uint16_t Time, uint8_t ACC)
{
    if (Position < 0)
    {
        Position = -Position;
        Position |= (1 << 15);
    }
    uint8_t bBuf[7];
    bBuf[0] = ACC;
    Host2SCS(bBuf + 1, bBuf + 2, Position);
    Host2SCS(bBuf + 3, bBuf + 4, Time);
    Host2SCS(bBuf + 5, bBuf + 6, 0);

    return genWrite(ID, SMS_STS_ACC, bBuf, 7);
}

void SMS_STS::SyncWritePosWithTime(uint8_t ID[], uint8_t IDN, int16_t Position[], uint16_t Time, uint8_t ACC[])
{
    uint8_t offbuf[IDN][7];
    for (uint8_t i = 0; i < IDN; i++)
    {
        if (Position[i] < 0)
        {
            Position[i] = -Position[i];
            Position[i] |= (1 << 15);
        }
        uint8_t bBuf[7];
        bBuf[0] = ACC ? ACC[i] : 0;
        Host2SCS(bBuf + 1, bBuf + 2, Position[i]);
        Host2SCS(bBuf + 3, bBuf + 4, Time);
        Host2SCS(bBuf + 5, bBuf + 6, 0);
        memcpy(offbuf[i], bBuf, 7);
    }
    syncWrite(ID, IDN, SMS_STS_ACC, (uint8_t*)offbuf, 7);
}

int SMS_STS::WheelMode(uint8_t ID)
{
    return writeByte(ID, SMS_STS_MODE, 1);
}

int SMS_STS::WriteSpe(uint8_t ID, int16_t Speed, uint8_t ACC)
{
    if (Speed < 0)
    {
        Speed = -Speed;
        Speed |= (1 << 15);
    }
    uint8_t bBuf[2];
    bBuf[0] = ACC;
    genWrite(ID, SMS_STS_ACC, bBuf, 1);
    Host2SCS(bBuf + 0, bBuf + 1, Speed);

    return genWrite(ID, SMS_STS_GOAL_SPEED_L, bBuf, 2);
}

int SMS_STS::MoveToAngle(uint8_t ID, int16_t TargetAngle, uint16_t Time, uint8_t ACC)
{
    if (TargetAngle == 0 || Time == 0)
    {
        return WriteSpe(ID, 0, ACC);
    }

    uint16_t absAngle = abs(TargetAngle);
    uint16_t targetSteps = ((uint32_t)absAngle * 4096) / 360;

    int16_t Speed = SpeCalc(targetSteps, Time);

    if (TargetAngle < 0)
    {
        Speed = -Speed;
    }

    int result = WriteSpe(ID, Speed, ACC);
    if (result == -1)
    {
        return -1;
    }

    delay(Time);

    return WriteSpe(ID, 0, ACC);
}

int SMS_STS::SpeCalc(uint16_t Steps, uint16_t Time)
{
    if (Time == 0)
    {
        return 0;
    }

    uint32_t Speed = ((uint32_t)Steps * 1000) / (uint32_t)Time;
    if (Speed > 0x7FFF)
    {
        Speed = 0x7FFF;
    }
    return (int16_t)Speed;
}

// angle in degrees, time in milliseconds
void SMS_STS::SyncRotateByAngle(uint8_t ID[], uint8_t IDN, int16_t RotationAngle[], uint16_t Time, uint8_t ACC[])
{
    int16_t Speed[IDN];

    for (uint8_t i = 0; i < IDN; i++)
    {
        if (RotationAngle[i] == 0 || Time == 0)
        {
            Speed[i] = 0;
            continue;
        }

        uint16_t absAngle = abs(RotationAngle[i]);
        uint16_t targetSteps = ((uint32_t)absAngle * 4096) / 360;

        int16_t speed = SpeCalc(targetSteps, Time);

        if (RotationAngle[i] < 0)
        {
            speed = -speed;
        }

        Speed[i] = speed;
    }

    uint8_t accBuf[IDN][1];
    for (uint8_t i = 0; i < IDN; i++)
    {
        accBuf[i][0] = ACC[i];
    }
    syncWrite(ID, IDN, SMS_STS_ACC, (uint8_t*)accBuf, 1);

    uint8_t speedBuf[IDN][2];
    for (uint8_t i = 0; i < IDN; i++)
    {
        int16_t speed = Speed[i];
        if (speed < 0)
        {
            speed = -speed;
            speed |= (1 << 15);
        }
        Host2SCS(&speedBuf[i][0], &speedBuf[i][1], speed);
    }
    syncWrite(ID, IDN, SMS_STS_GOAL_SPEED_L, (uint8_t*)speedBuf, 2);

    if (Time > 0)
    {
        delay(Time);

        uint8_t stopBuf[IDN][2];
        for (uint8_t i = 0; i < IDN; i++)
        {
            Host2SCS(&stopBuf[i][0], &stopBuf[i][1], 0);
        }
        syncWrite(ID, IDN, SMS_STS_GOAL_SPEED_L, (uint8_t*)stopBuf, 2);
    }
}

// Move TO target angle position with goal time (position + time control)
// angle in degrees, time in milliseconds
void SMS_STS::SyncMoveToAngle(uint8_t ID[], uint8_t IDN, int16_t TargetAngle[], uint16_t Time, uint8_t ACC[])
{
    uint8_t offbuf[IDN][7];

    for (uint8_t i = 0; i < IDN; i++)
    {
        int32_t position = ((int32_t)TargetAngle[i] * 4096) / 360;

        if (position < 0)
        {
            position = -position;
            position |= (1 << 15);
        }

        uint8_t bBuf[7];
        bBuf[0] = ACC[i];
        Host2SCS(bBuf + 1, bBuf + 2, (int16_t)position);  
        Host2SCS(bBuf + 3, bBuf + 4, Time);               
        Host2SCS(bBuf + 5, bBuf + 6, 0);                  
        memcpy(offbuf[i], bBuf, 7);
    }

    syncWrite(ID, IDN, SMS_STS_ACC, (uint8_t*)offbuf, 7);
}

int SMS_STS::EnableTorque(uint8_t ID, uint8_t Enable)
{
    return writeByte(ID, SMS_STS_TORQUE_ENABLE, Enable);
}

int SMS_STS::unLockEprom(uint8_t ID)
{
    return writeByte(ID, SMS_STS_LOCK, 0);
}

int SMS_STS::LockEprom(uint8_t ID)
{
    return writeByte(ID, SMS_STS_LOCK, 1);
}

int SMS_STS::CalibrationOfs(uint8_t ID)
{
    return writeByte(ID, SMS_STS_TORQUE_ENABLE, 128);
}

int SMS_STS::FeedBack(int ID)
{
    int nLen = Read(ID, SMS_STS_PRESENT_POSITION_L, Mem, sizeof(Mem));
    if (nLen != sizeof(Mem))
    {
        Err = 1;
        return -1;
    }
    Err = 0;
    return nLen;
}

int SMS_STS::ReadPos(int ID)
{
    int Pos = -1;
    if (ID == -1)
    {
        Pos = Mem[SMS_STS_PRESENT_POSITION_H - SMS_STS_PRESENT_POSITION_L];
        Pos <<= 8;
        Pos |= Mem[SMS_STS_PRESENT_POSITION_L - SMS_STS_PRESENT_POSITION_L];
    }
    else
    {
        Err = 0;
        Pos = readWord(ID, SMS_STS_PRESENT_POSITION_L);
        if (Pos == -1)
        {
            Err = 1;
        }
    }
    if (!Err && (Pos & (1 << 15)))
    {
        Pos = -(Pos & ~(1 << 15));
    }

    return Pos;
}

int SMS_STS::ReadSpeed(int ID)
{
    int Speed = -1;
    if (ID == -1)
    {
        Speed = Mem[SMS_STS_PRESENT_SPEED_H - SMS_STS_PRESENT_POSITION_L];
        Speed <<= 8;
        Speed |= Mem[SMS_STS_PRESENT_SPEED_L - SMS_STS_PRESENT_POSITION_L];
    }
    else
    {
        Err = 0;
        Speed = readWord(ID, SMS_STS_PRESENT_SPEED_L);
        if (Speed == -1)
        {
            Err = 1;
            return -1;
        }
    }
    if (!Err && (Speed & (1 << 15)))
    {
        Speed = -(Speed & ~(1 << 15));
    }
    return Speed;
}

int SMS_STS::ReadLoad(int ID)
{
    int Load = -1;
    if (ID == -1)
    {
        Load = Mem[SMS_STS_PRESENT_LOAD_H - SMS_STS_PRESENT_POSITION_L];
        Load <<= 8;
        Load |= Mem[SMS_STS_PRESENT_LOAD_L - SMS_STS_PRESENT_POSITION_L];
    }
    else
    {
        Err = 0;
        Load = readWord(ID, SMS_STS_PRESENT_LOAD_L);
        if (Load == -1)
        {
            Err = 1;
        }
    }
    if (!Err && (Load & (1 << 10)))
    {
        Load = -(Load & ~(1 << 10));
    }
    return Load;
}

int SMS_STS::ReadVoltage(int ID)
{
    int Voltage = -1;
    if (ID == -1)
    {
        Voltage = Mem[SMS_STS_PRESENT_VOLTAGE - SMS_STS_PRESENT_POSITION_L];
    }
    else
    {
        Err = 0;
        Voltage = readByte(ID, SMS_STS_PRESENT_VOLTAGE);
        if (Voltage == -1)
        {
            Err = 1;
        }
    }
    return Voltage;
}

int SMS_STS::ReadTemper(int ID)
{
    int Temper = -1;
    if (ID == -1)
    {
        Temper = Mem[SMS_STS_PRESENT_TEMPERATURE - SMS_STS_PRESENT_POSITION_L];
    }
    else
    {
        Err = 0;
        Temper = readByte(ID, SMS_STS_PRESENT_TEMPERATURE);
        if (Temper == -1)
        {
            Err = 1;
        }
    }
    return Temper;
}

int SMS_STS::ReadMove(int ID)
{
    int Move = -1;
    if (ID == -1)
    {
        Move = Mem[SMS_STS_MOVING - SMS_STS_PRESENT_POSITION_L];
    }
    else
    {
        Err = 0;
        Move = readByte(ID, SMS_STS_MOVING);
        if (Move == -1)
        {
            Err = 1;
        }
    }
    return Move;
}

int SMS_STS::ReadCurrent(int ID)
{
    int Current = -1;
    if (ID == -1)
    {
        Current = Mem[SMS_STS_PRESENT_CURRENT_H - SMS_STS_PRESENT_POSITION_L];
        Current <<= 8;
        Current |= Mem[SMS_STS_PRESENT_CURRENT_L - SMS_STS_PRESENT_POSITION_L];
    }
    else
    {
        Err = 0;
        Current = readWord(ID, SMS_STS_PRESENT_CURRENT_L);
        if (Current == -1)
        {
            Err = 1;
            return -1;
        }
    }
    if (!Err && (Current & (1 << 15)))
    {
        Current = -(Current & ~(1 << 15));
    }
    return Current;
}