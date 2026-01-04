#ifndef _SMS_STS_H
#define _SMS_STS_H

// Baud Rate Definitions
#define SMS_STS_1M 0
#define SMS_STS_0_5M 1
#define SMS_STS_250K 2
#define SMS_STS_128K 3
#define SMS_STS_115200 4
#define SMS_STS_76800 5
#define SMS_STS_57600 6
#define SMS_STS_38400 7

// Memory Table Definitions
//-------EPROM (Read Only)--------
#define SMS_STS_MODEL_L 3
#define SMS_STS_MODEL_H 4

//-------EPROM (Read/Write)--------
#define SMS_STS_ID 5
#define SMS_STS_BAUD_RATE 6
#define SMS_STS_MIN_ANGLE_LIMIT_L 9
#define SMS_STS_MIN_ANGLE_LIMIT_H 10
#define SMS_STS_MAX_ANGLE_LIMIT_L 11
#define SMS_STS_MAX_ANGLE_LIMIT_H 12
#define SMS_STS_CW_DEAD 26
#define SMS_STS_CCW_DEAD 27
#define SMS_STS_OFS_L 31
#define SMS_STS_OFS_H 32
#define SMS_STS_MODE 33

//-------SRAM (Read/Write)--------
#define SMS_STS_TORQUE_ENABLE 40
#define SMS_STS_ACC 41
#define SMS_STS_GOAL_POSITION_L 42
#define SMS_STS_GOAL_POSITION_H 43
#define SMS_STS_GOAL_TIME_L 44
#define SMS_STS_GOAL_TIME_H 45
#define SMS_STS_GOAL_SPEED_L 46
#define SMS_STS_GOAL_SPEED_H 47
#define SMS_STS_LOCK 55

//-------SRAM (Read Only)--------
#define SMS_STS_PRESENT_POSITION_L 56
#define SMS_STS_PRESENT_POSITION_H 57
#define SMS_STS_PRESENT_SPEED_L 58
#define SMS_STS_PRESENT_SPEED_H 59
#define SMS_STS_PRESENT_LOAD_L 60
#define SMS_STS_PRESENT_LOAD_H 61
#define SMS_STS_PRESENT_VOLTAGE 62
#define SMS_STS_PRESENT_TEMPERATURE 63
#define SMS_STS_MOVING 66
#define SMS_STS_PRESENT_CURRENT_L 69
#define SMS_STS_PRESENT_CURRENT_H 70

#include "SCSerial.h"

class SMS_STS : public SCSerial
{
public:
	SMS_STS();
	SMS_STS(uint8_t End);
	SMS_STS(uint8_t End, uint8_t Level);

	// Ordinary write single servo position command
	virtual int WritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC = 0);

	// Asynchronous write single servo position command (requires RegWriteAction to trigger)
	virtual int RegWritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC = 0);

	// Synchronous write multiple servo position command
	virtual void SyncWritePosEx(uint8_t ID[], uint8_t IDN, int16_t Position[], uint16_t Speed[], uint8_t ACC[]);

	// Wheel Mode (Constant Speed Mode)
	virtual int WheelMode(uint8_t ID);

	// Wheel Mode Control Command
	virtual int WriteSpe(uint8_t ID, int16_t Speed, uint8_t ACC = 0);

	// Torque Control Command
	virtual int EnableTorque(uint8_t ID, uint8_t Enable);

	// EPROM Unlock
	virtual int unLockEprom(uint8_t ID);

	// EPROM Lock
	virtual int LockEprom(uint8_t ID);

	// Center/Offset Calibration
	virtual int CalibrationOfs(uint8_t ID);

	// Feedback Servo Information
	virtual int FeedBack(int ID);

	// Read Position
	virtual int ReadPos(int ID);

	// Read Speed
	virtual int ReadSpeed(int ID);

	// Read Load (Output voltage percentage to motor 0~1000)
	virtual int ReadLoad(int ID);

	// Read Voltage
	virtual int ReadVoltage(int ID);

	// Read Temperature
	virtual int ReadTemper(int ID);

	// Read Move Status
	virtual int ReadMove(int ID);

	// Read Current
	virtual int ReadCurrent(int ID);

private:
	uint8_t Mem[SMS_STS_PRESENT_CURRENT_H - SMS_STS_PRESENT_POSITION_L + 1];
};

#endif