#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <stdexcept>
#include <type.hpp>
#include <type_traits>
#include <vector>

class bq76942
{
public:
    enum class direct_command : uint8_t
    {
        CONTROL_STATUS = 0x00,
        SAFETY_ALERT_A = 0x02,
        SAFETY_STATUS_A = 0x03,
        SAFETY_ALERT_B = 0x04,
        SAFETY_STATUS_B = 0x05,
        SAFETY_ALERT_C = 0x06,
        SAFETY_STATUS_C = 0x07,
        PF_ALERT_A = 0x0A,
        PF_STATUS_A = 0x0B,
        PF_ALERT_B = 0x0C,
        PF_STATUS_B = 0x0D,
        PF_ALERT_C = 0x0E,
        PF_STATUS_C = 0x0F,
        PF_ALERT_D = 0x10,
        PF_STATUS_D = 0x11,
        BATTERY_STATUS = 0x12,
        CELL_1_VOLTAGE = 0x14,
        CELL_2_VOLTAGE = 0x16,
        CELL_3_VOLTAGE = 0x18,
        CELL_4_VOLTAGE = 0x1A,
        CELL_5_VOLTAGE = 0x1C,
        CELL_6_VOLTAGE = 0x1E,
        CELL_7_VOLTAGE = 0x20,
        CELL_8_VOLTAGE = 0x22,
        CELL_9_VOLTAGE = 0x24,
        CELL_10_VOLTAGE = 0x26,
        CELL_11_VOLTAGE = 0x28,
        CELL_12_VOLTAGE = 0x2A,
        CELL_13_VOLTAGE = 0x2C,
        CELL_14_VOLTAGE = 0x2E,
        CELL_15_VOLTAGE = 0x30,
        CELL_16_VOLTAGE = 0x32,
        STACK_VOLTAGE = 0x34,
        PACK_PIN_VOLTAGE = 0x36,
        LD_PIN_VOLTAGE = 0x38,
        CC2_CURRENT = 0x3A,
        SUBCOMMAND = 0x3E,
        TRANSFER_BUF = 0x40,
        TRANSFER_CHECKSUM = 0x60,
        TRANSFER_LEN = 0x61,
        ALARM_STATUS = 0x62,
        ALARM_RAW_STATUS = 0x64,
        ALARM_ENABLE = 0x66,
        INT_TEMPERATURE = 0x68,
        CFETOFF_TEMPERATURE = 0x6A,
        DFETOFF_TEMPERATURE = 0x6C,
        ALERT_TEMPERATURE = 0x6E,
        TS1_TEMPERATURE = 0x70,
        TS2_TEMPERATURE = 0x72,
        TS3_TEMPERATURE = 0x74,
        HDQ_TEMPERATURE = 0x76,
        DCHG_TEMPERATURE = 0x78,
        DDSG_TEMPERATURE = 0x7A,
        FET_STATUS = 0x7F,
    };
    enum class subcommand : uint16_t
    {
        DEVICE_NUMBER = 0x0001,
        FW_VERSION = 0x0002,
        HW_VERSION = 0x0003,
        IROM_SIG = 0x0004,
        STATIC_CFG_SIG = 0x0005,
        PREV_MACWRITE = 0x0007,
        DROM_SIG = 0x0009,
        SECURITY_KEYS = 0x0035,
        SAVED_PF_STATUS = 0x0053,
        MANUFACTURING_STATUS = 0x0057,
        MANU_DATA = 0x0070,
        DA_STATUS_1 = 0x0071,
        DA_STATUS_2 = 0x0072,
        DA_STATUS_3 = 0x0073,
        DA_STATUS_4 = 0x0074,
        DA_STATUS_5 = 0x0075,
        DA_STATUS_6 = 0x0076,
        DA_STATUS_7 = 0x0077,
        CUV_SNAPSHOT = 0x0080,
        COV_SNAPSHOT = 0X0081,
        CB_ACTIVE_CELLS = 0x0083,
        CB_SET_LVL = 0x0084,
        CB_STATUS_1 = 0x0085,
        CB_STATUS_2 = 0x0086,
        CB_STATUS_3 = 0x0087,
        FET_CONTROL = 0x0097,
        REG12_CONTROL = 0x0098,
        OTP_WR_CHECK = 0x00A0,
        OTP_WRITE = 0x00A1,
        READ_CAL1 = 0xF081,
        CAL_CUV = 0xF090,
        CAL_COV = 0xF091,
    };
    enum class cmd_only_subcommand : uint16_t
    {
        EXIT_DEEPSLEEP = 0x000E,
        DEEPSLEEP = 0x000F,
        SHUTDOWN = 0x0010,
        BQ769x2_RESET = 0x0012,  //"RESET" in documentation
        RESET = 0x0012,
        PDSG_TEST = 0x001C,
        FUSE_TOGGLE = 0x001D,
        PCHG_TEST = 0x001E,
        CHG_TEST = 0x001F,
        DSG_TEST = 0x0020,
        FET_ENABLE = 0x0022,
        PF_ENABLE = 0x0024,
        PF_RESET = 0x0029,
        SEAL = 0x0030,
        RESET_PASSQ = 0x0082,
        PTO_RECOVER = 0x008A,
        SET_CFGUPDATE = 0x0090,
        EXIT_CFGUPDATE = 0x0092,
        DSG_PDSG_OFF = 0x0093,
        CHG_PCHG_OFF = 0x0094,
        ALL_FETS_OFF = 0x0095,
        ALL_FETS_ON = 0x0096,
        SLEEP_ENABLE = 0x0099,
        SLEEP_DISABLE = 0x009A,
        OCDL_RECOVER = 0x009B,
        SCDL_RECOVER = 0x009C,
        LOAD_DETECT_RESTART = 0x009D,
        LOAD_DETECT_ON = 0x009E,
        LOAD_DETECT_OFF = 0x009F,
        CFETOFF_LO = 0x2800,
        DFETOFF_LO = 0x2801,
        ALERT_LO = 0x2802,
        HDQ_LO = 0x2806,
        DCHG_LO = 0x2807,
        DDSG_LO = 0x2808,
        CFETOFF_HI = 0x2810,
        DFETOFF_HI = 0x2811,
        ALERT_HI = 0x2812,
        HDQ_HI = 0x2816,
        DCHG_HI = 0x2817,
        DDSG_HI = 0x2818,
        PF_FORCE_A = 0x2857,
        PF_FORCE_B = 0x29A3,
        SWAP_COMM_MODE = 0x29BC,
        SWAP_TO_I2C = 0x29E7,
        SWAP_TO_SPI = 0x7C35,
        SWAP_TO_HDQ = 0x7C40,
    };
    enum class data_register : uint16_t
    {
        CELL_1_GAIN = 0x9180,                      // Calibration:Voltage:Cell 1 Gain
        CELL_2_GAIN = 0x9182,                      // Calibration:Voltage:Cell 2 Gain
        CELL_3_GAIN = 0x9184,                      // Calibration:Voltage:Cell 3 Gain
        CELL_4_GAIN = 0x9186,                      // Calibration:Voltage:Cell 4 Gain
        CELL_5_GAIN = 0x9188,                      // Calibration:Voltage:Cell 5 Gain
        CELL_6_GAIN = 0x918A,                      // Calibration:Voltage:Cell 6 Gain
        CELL_7_GAIN = 0x918C,                      // Calibration:Voltage:Cell 7 Gain
        CELL_8_GAIN = 0x918E,                      // Calibration:Voltage:Cell 8 Gain
        CELL_9_GAIN = 0x9190,                      // Calibration:Voltage:Cell 9 Gain
        CELL_10_GAIN = 0x9192,                     // Calibration:Voltage:Cell 10 Gain
        CELL_11_GAIN = 0x9194,                     // Calibration:Voltage:Cell 11 Gain
        CELL_12_GAIN = 0x9196,                     // Calibration:Voltage:Cell 12 Gain
        CELL_13_GAIN = 0x9198,                     // Calibration:Voltage:Cell 13 Gain
        CELL_14_GAIN = 0x919A,                     // Calibration:Voltage:Cell 14 Gain
        CELL_15_GAIN = 0x919C,                     // Calibration:Voltage:Cell 15 Gain
        CELL_16_GAIN = 0x919E,                     // Calibration:Voltage:Cell 16 Gain
        PACK_GAIN = 0x91A0,                        // Calibration:Voltage:Pack Gain
        TOS_GAIN = 0x91A2,                         // Calibration:Voltage:TOS Gain
        LD_GAIN = 0x91A4,                          // Calibration:Voltage:LD Gain
        ADC_GAIN = 0x91A6,                         // Calibration:Voltage:ADC Gain
        CC_GAIN = 0x91A8,                          // Calibration:Current:CC Gain
        CAPACITY_GAIN = 0x91AC,                    // Calibration:Current:Capacity Gain
        VCELL_OFFSET = 0x91B0,                     // Calibration:Vcell Offset:Vcell Offset
        VDIV_OFFSET = 0x91B2,                      // Calibration:V Divider Offset:Vdiv Offset
        COULOMB_COUNTER_OFFSET_SAMPLES = 0x91C6,   // Calibration:Current Offset:Coulomb Counter Offset Samples
        BOARD_OFFSET = 0x91C8,                     // Calibration:Current Offset:Board Offset
        INTERNAL_TEMP_OFFSET = 0x91CA,             // Calibration:Temperature:Internal Temp Offset
        CFETOFF_TEMP_OFFSET = 0x91CB,              // Calibration:Temperature:CFETOFF Temp Offset
        DFETOFF_TEMP_OFFSET = 0x91CC,              // Calibration:Temperature:DFETOFF Temp Offset
        ALERT_TEMP_OFFSET = 0x91CD,                // Calibration:Temperature:ALERT Temp Offset
        TS1_TEMP_OFFSET = 0x91CE,                  // Calibration:Temperature:TS1 Temp Offset
        TS2_TEMP_OFFSET = 0x91CF,                  // Calibration:Temperature:TS2 Temp Offset
        TS3_TEMP_OFFSET = 0x91D0,                  // Calibration:Temperature:TS3 Temp Offset
        HDQ_TEMP_OFFSET = 0x91D1,                  // Calibration:Temperature:HDQ Temp Offset
        DCHG_TEMP_OFFSET = 0x91D2,                 // Calibration:Temperature:DCHG Temp Offset
        DDSG_TEMP_OFFSET = 0x91D3,                 // Calibration:Temperature:DDSG Temp Offset
        INT_GAIN = 0x91E2,                         // Calibration:Internal Temp Model:Int Gain
        INT_BASE_OFFSET = 0x91E4,                  // Calibration:Internal Temp Model:Int base offset
        INT_MAXIMUM_AD = 0x91E6,                   // Calibration:Internal Temp Model:Int Maximum AD
        INT_MAXIMUM_TEMP = 0x91E8,                 // Calibration:Internal Temp Model:Int Maximum Temp
        T18K_COEFF_A1 = 0x91EA,                    // Calibration:18K Temperature Model:Coeff a1
        T18K_COEFF_A2 = 0x91EC,                    // Calibration:18K Temperature Model:Coeff a2
        T18K_COEFF_A3 = 0x91EE,                    // Calibration:18K Temperature Model:Coeff a3
        T18K_COEFF_A4 = 0x91F0,                    // Calibration:18K Temperature Model:Coeff a4
        T18K_COEFF_A5 = 0x91F2,                    // Calibration:18K Temperature Model:Coeff a5
        T18K_COEFF_B1 = 0x91F4,                    // Calibration:18K Temperature Model:Coeff b1
        T18K_COEFF_B2 = 0x91F6,                    // Calibration:18K Temperature Model:Coeff b2
        T18K_COEFF_B3 = 0x91F8,                    // Calibration:18K Temperature Model:Coeff b3
        T18K_COEFF_B4 = 0x91FA,                    // Calibration:18K Temperature Model:Coeff b4
        T18K_ADC0 = 0x91FE,                        // Calibration:18K Temperature Model:Adc0
        T180K_COEFF_A1 = 0x9200,                   // Calibration:180K Temperature Model:Coeff a1
        T180K_COEFF_A2 = 0x9202,                   // Calibration:180K Temperature Model:Coeff a2
        T180K_COEFF_A3 = 0x9204,                   // Calibration:180K Temperature Model:Coeff a3
        T180K_COEFF_A4 = 0x9206,                   // Calibration:180K Temperature Model:Coeff a4
        T180K_COEFF_A5 = 0x9208,                   // Calibration:180K Temperature Model:Coeff a5
        T180K_COEFF_B1 = 0x920A,                   // Calibration:180K Temperature Model:Coeff b1
        T180K_COEFF_B2 = 0x920C,                   // Calibration:180K Temperature Model:Coeff b2
        T180K_COEFF_B3 = 0x920E,                   // Calibration:180K Temperature Model:Coeff b3
        T180K_COEFF_B4 = 0x9210,                   // Calibration:180K Temperature Model:Coeff b4
        T180K_ADC0 = 0x9214,                       // Calibration:180K Temperature Model:Adc0
        CUSTOM_COEFF_A1 = 0x9216,                  // Calibration:Custom Temperature Model:Coeff a1
        CUSTOM_COEFF_A2 = 0x9218,                  // Calibration:Custom Temperature Model:Coeff a2
        CUSTOM_COEFF_A3 = 0x921A,                  // Calibration:Custom Temperature Model:Coeff a3
        CUSTOM_COEFF_A4 = 0x921C,                  // Calibration:Custom Temperature Model:Coeff a4
        CUSTOM_COEFF_A5 = 0x921E,                  // Calibration:Custom Temperature Model:Coeff a5
        CUSTOM_COEFF_B1 = 0x9220,                  // Calibration:Custom Temperature Model:Coeff b1
        CUSTOM_COEFF_B2 = 0x9222,                  // Calibration:Custom Temperature Model:Coeff b2
        CUSTOM_COEFF_B3 = 0x9224,                  // Calibration:Custom Temperature Model:Coeff b3
        CUSTOM_COEFF_B4 = 0x9226,                  // Calibration:Custom Temperature Model:Coeff b4
        CUSTOM_RC0 = 0x9228,                       // Calibration:Custom Temperature Model:Rc0
        CUSTOM_ADC0 = 0x922A,                      // Calibration:Custom Temperature Model:Adc0
        COULOMB_COUNTER_DEADBAND = 0x922D,         // Calibration:Current Deadband:Coulomb Counter Deadband
        CUV_THRESHOLD_OVERRIDE = 0x91D4,           // Calibration:CUV:CUV Threshold Override
        COV_THRESHOLD_OVERRIDE = 0x91D6,           // Calibration:COV:COV Threshold Override
        MIN_BLOW_FUSE_VOLTAGE = 0x9231,            // Settings:Fuse:Min Blow Fuse Voltage
        FUSE_BLOW_TIMEOUT = 0x9233,                // Settings:Fuse:Fuse Blow Timeout
        POWER_CONFIG = 0x9234,                     // Settings:Configuration:Power Config
        REG12_CONFIG = 0x9236,                     // Settings:Configuration:REG12 Config
        REG0_CONFIG = 0x9237,                      // Settings:Configuration:REG0 Config
        HWD_REGULATOR_OPTIONS = 0x9238,            // Settings:Configuration:HWD Regulator Options
        COMM_TYPE = 0x9239,                        // Settings:Configuration:Comm Type
        I2C_ADDRESS = 0x923A,                      // Settings:Configuration:I2C Address
        SPI_CONFIGURATION = 0x923C,                // Settings:Configuration:SPI Configuration
        COMM_IDLE_TIME = 0x923D,                   // Settings:Configuration:Comm Idle Time
        CFETOFF_PIN_CONFIG = 0x92FA,               // Settings:Configuration:CFETOFF Pin Config
        DFETOFF_PIN_CONFIG = 0x92FB,               // Settings:Configuration:DFETOFF Pin Config
        ALERT_PIN_CONFIG = 0x92FC,                 // Settings:Configuration:ALERT Pin Config
        TS1_CONFIG = 0x92FD,                       // Settings:Configuration:TS1 Config
        TS2_CONFIG = 0x92FE,                       // Settings:Configuration:TS2 Config
        TS3_CONFIG = 0x92FF,                       // Settings:Configuration:TS3 Config
        HDQ_PIN_CONFIG = 0x9300,                   // Settings:Configuration:HDQ Pin Config
        DCHG_PIN_CONFIG = 0x9301,                  // Settings:Configuration:DCHG Pin Config
        DDSG_PIN_CONFIG = 0x9302,                  // Settings:Configuration:DDSG Pin Config
        DA_CONFIGURATION = 0x9303,                 // Settings:Configuration:DA Configuration
        VCELL_MODE = 0x9304,                       // Settings:Configuration:Vcell Mode
        CC3_SAMPLES = 0x9307,                      // Settings:Configuration:CC3 Samples
        PROTECTION_CONFIGURATION = 0x925F,         // Settings:Protection:Protection Configuration
        ENABLED_PROTECTIONS_A = 0x9261,            // Settings:Protection:Enabled Protections A
        ENABLED_PROTECTIONS_B = 0x9262,            // Settings:Protection:Enabled Protections B
        ENABLED_PROTECTIONS_C = 0x9263,            // Settings:Protection:Enabled Protections C
        CHG_FET_PROTECTIONS_A = 0x9265,            // Settings:Protection:CHG FET Protections A
        CHG_FET_PROTECTIONS_B = 0x9266,            // Settings:Protection:CHG FET Protections B
        CHG_FET_PROTECTIONS_C = 0x9267,            // Settings:Protection:CHG FET Protections C
        DSG_FET_PROTECTIONS_A = 0x9269,            // Settings:Protection:DSG FET Protections A
        DSG_FET_PROTECTIONS_B = 0x926A,            // Settings:Protection:DSG FET Protections B
        DSG_FET_PROTECTIONS_C = 0x926B,            // Settings:Protection:DSG FET Protections C
        BODY_DIODE_THRESHOLD = 0x9273,             // Settings:Protection:Body Diode Threshold
        DEFAULT_ALARM_MASK = 0x926D,               // Settings:Alarm:Default Alarm Mask
        SF_ALERT_MASK_A = 0x926F,                  // Settings:Alarm:SF Alert Mask A
        SF_ALERT_MASK_B = 0x9270,                  // Settings:Alarm:SF Alert Mask B
        SF_ALERT_MASK_C = 0x9271,                  // Settings:Alarm:SF Alert Mask C
        PF_ALERT_MASK_A = 0x92C4,                  // Settings:Alarm:PF Alert Mask A
        PF_ALERT_MASK_B = 0x92C5,                  // Settings:Alarm:PF Alert Mask B
        PF_ALERT_MASK_C = 0x92C6,                  // Settings:Alarm:PF Alert Mask C
        PF_ALERT_MASK_D = 0x92C7,                  // Settings:Alarm:PF Alert Mask D
        ENABLED_PF_A = 0x92C0,                     // Settings:Permanent Failure:Enabled PF A
        ENABLED_PF_B = 0x92C1,                     // Settings:Permanent Failure:Enabled PF B
        ENABLED_PF_C = 0x92C2,                     // Settings:Permanent Failure:Enabled PF C
        ENABLED_PF_D = 0x92C3,                     // Settings:Permanent Failure:Enabled PF D
        FET_OPTIONS = 0x9308,                      // Settings:FET:FET Options
        CHG_PUMP_CONTROL = 0x9309,                 // Settings:FET:Chg Pump Control
        PRECHARGE_START_VOLTAGE = 0x930A,          // Settings:FET:Precharge Start Voltage
        PRECHARGE_STOP_VOLTAGE = 0x930C,           // Settings:FET:Precharge Stop Voltage
        PREDISCHARGE_TIMEOUT = 0x930E,             // Settings:FET:Predischarge Timeout
        PREDISCHARGE_STOP_DELTA = 0x930F,          // Settings:FET:Predischarge Stop Delta
        DSG_CURRENT_THRESHOLD = 0x9310,            // Settings:Current Thresholds:Dsg Current Threshold
        CHG_CURRENT_THRESHOLD = 0x9312,            // Settings:Current Thresholds:Chg Current Threshold
        CHECK_TIME = 0x9314,                       // Settings:Cell Open-Wire:Check Time
        CELL_1_INTERCONNECT = 0x9315,              // Settings:Interconnect Resistances:Cell 1 Interconnect
        CELL_2_INTERCONNECT = 0x9317,              // Settings:Interconnect Resistances:Cell 2 Interconnect
        CELL_3_INTERCONNECT = 0x9319,              // Settings:Interconnect Resistances:Cell 3 Interconnect
        CELL_4_INTERCONNECT = 0x931B,              // Settings:Interconnect Resistances:Cell 4 Interconnect
        CELL_5_INTERCONNECT = 0x931D,              // Settings:Interconnect Resistances:Cell 5 Interconnect
        CELL_6_INTERCONNECT = 0x931F,              // Settings:Interconnect Resistances:Cell 6 Interconnect
        CELL_7_INTERCONNECT = 0x9321,              // Settings:Interconnect Resistances:Cell 7 Interconnect
        CELL_8_INTERCONNECT = 0x9323,              // Settings:Interconnect Resistances:Cell 8 Interconnect
        CELL_9_INTERCONNECT = 0x9325,              // Settings:Interconnect Resistances:Cell 9 Interconnect
        CELL_10_INTERCONNECT = 0x9327,             // Settings:Interconnect Resistances:Cell 10 Interconnect
        CELL_11_INTERCONNECT = 0x9329,             // Settings:Interconnect Resistances:Cell 11 Interconnect
        CELL_12_INTERCONNECT = 0x932B,             // Settings:Interconnect Resistances:Cell 12 Interconnect
        CELL_13_INTERCONNECT = 0x932D,             // Settings:Interconnect Resistances:Cell 13 Interconnect
        CELL_14_INTERCONNECT = 0x932F,             // Settings:Interconnect Resistances:Cell 14 Interconnect
        CELL_15_INTERCONNECT = 0x9331,             // Settings:Interconnect Resistances:Cell 15 Interconnect
        CELL_16_INTERCONNECT = 0x9333,             // Settings:Interconnect Resistances:Cell 16 Interconnect
        MFG_STATUS_INIT = 0x9343,                  // Settings:Manufacturing:Mfg Status Init
        BALANCING_CONFIGURATION = 0x9335,          // Settings:Cell Balancing Config:Balancing Configuration
        MIN_CELL_TEMP = 0x9336,                    // Settings:Cell Balancing Config:Min Cell Temp
        MAX_CELL_TEMP = 0x9337,                    // Settings:Cell Balancing Config:Max Cell Temp
        MAX_INTERNAL_TEMP = 0x9338,                // Settings:Cell Balancing Config:Max Internal Temp
        CELL_BALANCE_INTERVAL = 0x9339,            // Settings:Cell Balancing Config:Cell Balance Interval
        CELL_BALANCE_MAX_CELLS = 0x933A,           // Settings:Cell Balancing Config:Cell Balance Max Cells
        CELL_BALANCE_MIN_CELL_V_CHARGE = 0x933B,   // Settings:Cell Balancing Config:Cell Balance Min Cell V (Charge)
        CELL_BALANCE_MIN_DELTA_CHARGE = 0x933D,    // Settings:Cell Balancing Config:Cell Balance Min Delta (Charge)
        CELL_BALANCE_STOP_DELTA_CHARGE = 0x933E,   // Settings:Cell Balancing Config:Cell Balance Stop Delta (Charge)
        CELL_BALANCE_MIN_CELL_V_RELAX = 0x933F,    // Settings:Cell Balancing Config:Cell Balance Min Cell V (Relax)
        CELL_BALANCE_MIN_DELTA_RELAX = 0x9341,     // Settings:Cell Balancing Config:Cell Balance Min Delta (Relax)
        CELL_BALANCE_STOP_DELTA_RELAX = 0x9342,    // Settings:Cell Balancing Config:Cell Balance Stop Delta (Relax)
        SHUTDOWN_CELL_VOLTAGE = 0x923F,            // Power:Shutdown:Shutdown Cell Voltage
        SHUTDOWN_STACK_VOLTAGE = 0x9241,           // Power:Shutdown:Shutdown Stack Voltage
        LOW_V_SHUTDOWN_DELAY = 0x9243,             // Power:Shutdown:Low V Shutdown Delay
        SHUTDOWN_TEMPERATURE = 0x9244,             // Power:Shutdown:Shutdown Temperature
        SHUTDOWN_TEMPERATURE_DELAY = 0x9245,       // Power:Shutdown:Shutdown Temperature Delay
        FET_OFF_DELAY = 0x9252,                    // Power:Shutdown:FET Off Delay
        SHUTDOWN_COMMAND_DELAY = 0x9253,           // Power:Shutdown:Shutdown Command Delay
        AUTO_SHUTDOWN_TIME = 0x9254,               // Power:Shutdown:Auto Shutdown Time
        RAM_FAIL_SHUTDOWN_TIME = 0x9255,           // Power:Shutdown:RAM Fail Shutdown Time
        SLEEP_CURRENT = 0x9248,                    // Power:Sleep:Sleep Current
        VOLTAGE_TIME = 0x924A,                     // Power:Sleep:Voltage Time
        WAKE_COMPARATOR_CURRENT = 0x924B,          // Power:Sleep:Wake Comparator Current
        SLEEP_HYSTERESIS_TIME = 0x924D,            // Power:Sleep:Sleep Hysteresis Time
        SLEEP_CHARGER_VOLTAGE_THRESHOLD = 0x924E,  // Power:Sleep:Sleep Charger Voltage Threshold
        SLEEP_CHARGER_PACK_TOS_DELTA = 0x9250,     // Power:Sleep:Sleep Charger PACK-TOS Delta
        CONFIG_RAM_SIGNATURE = 0x91E0,             // System Data:Integrity:Config RAM Signature
        CUV_THRESHOLD = 0x9275,                    // Protections:CUV:Threshold
        CUV_DELAY = 0x9276,                        // Protections:CUV:Delay
        CUV_RECOVERY_HYSTERESIS = 0x927B,          // Protections:CUV:Recovery Hysteresis
        COV_THRESHOLD = 0x9278,                    // Protections:COV:Threshold
        COV_DELAY = 0x9279,                        // Protections:COV:Delay
        COV_RECOVERY_HYSTERESIS = 0x927C,          // Protections:COV:Recovery Hysteresis
        COVL_LATCH_LIMIT = 0x927D,                 // Protections:COVL:Latch Limit
        COVL_COUNTER_DEC_DELAY = 0x927E,           // Protections:COVL:Counter Dec Delay
        COVL_RECOVERY_TIME = 0x927F,               // Protections:COVL:Recovery Time
        OCC_THRESHOLD = 0x9280,                    // Protections:OCC:Threshold
        OCC_DELAY = 0x9281,                        // Protections:OCC:Delay
        OCC_RECOVERY_THRESHOLD = 0x9288,           // Protections:OCC:Recovery Threshold
        OCC_PACK_TOS_DELTA = 0x92B0,               // Protections:OCC:PACK-TOS Delta
        OCD1_THRESHOLD = 0x9282,                   // Protections:OCD1:Threshold
        OCD1_DELAY = 0x9283,                       // Protections:OCD1:Delay
        OCD2_THRESHOLD = 0x9284,                   // Protections:OCD2:Threshold
        OCD2_DELAY = 0x9285,                       // Protections:OCD2:Delay
        SCD_THRESHOLD = 0x9286,                    // Protections:SCD:Threshold
        SCD_DELAY = 0x9287,                        // Protections:SCD:Delay
        SCD_RECOVERY_TIME = 0x9294,                // Protections:SCD:Recovery Time
        OCD3_THRESHOLD = 0x928A,                   // Protections:OCD3:Threshold
        OCD3_DELAY = 0x928C,                       // Protections:OCD3:Delay
        OCD_RECOVERY_THRESHOLD = 0x928D,           // Protections:OCD:Recovery Threshold
        OCDL_LATCH_LIMIT = 0x928F,                 // Protections:OCDL:Latch Limit
        OCDL_COUNTER_DEC_DELAY = 0x9290,           // Protections:OCDL:Counter Dec Delay
        OCDL_RECOVERY_TIME = 0x9291,               // Protections:OCDL:Recovery Time
        OCDL_RECOVERY_THRESHOLD = 0x9292,          // Protections:OCDL:Recovery Threshold
        SCDL_LATCH_LIMIT = 0x9295,                 // Protections:SCDL:Latch Limit
        SCDL_COUNTER_DEC_DELAY = 0x9296,           // Protections:SCDL:Counter Dec Delay
        SCDL_RECOVERY_TIME = 0x9297,               // Protections:SCDL:Recovery Time
        SCDL_RECOVERY_THRESHOLD = 0x9298,          // Protections:SCDL:Recovery Threshold
        OTC_THRESHOLD = 0x929A,                    // Protections:OTC:Threshold
        OTC_DELAY = 0x920B,                        // Protections:OTC:Delay
        OTC_RECOVERY = 0x929C,                     // Protections:OTC:Recovery
        OTD_THRESHOLD = 0x929D,                    // Protections:OTD:Threshold
        OTD_DELAY = 0x929E,                        // Protections:OTD:Delay
        OTD_RECOVERY = 0x929F,                     // Protections:OTD:Recovery
        OTF_THRESHOLD = 0x92A0,                    // Protections:OTF:Threshold
        OTF_DELAY = 0x92A1,                        // Protections:OTF:Delay
        OTF_RECOVERY = 0x92A2,                     // Protections:OTF:Recovery
        OTINT_THRESHOLD = 0x92A3,                  // Protections:OTINT:Threshold
        OTINT_DELAY = 0x92A4,                      // Protections:OTINT:Delay
        OTINT_RECOVERY = 0x92A5,                   // Protections:OTINT:Recovery
        UTC_THRESHOLD = 0x92A6,                    // Protections:UTC:Threshold
        UTC_DELAY = 0x92A7,                        // Protections:UTC:Delay
        UTC_RECOVERY = 0x92A8,                     // Protections:UTC:Recovery
        UTD_THRESHOLD = 0x92A9,                    // Protections:UTD:Threshold
        UTD_DELAY = 0x92AA,                        // Protections:UTD:Delay
        UTD_RECOVERY = 0x92AB,                     // Protections:UTD:Recovery
        UTINT_THRESHOLD = 0x92AC,                  // Protections:UTINT:Threshold
        UTINT_DELAY = 0x92AD,                      // Protections:UTINT:Delay
        UTINT_RECOVERY = 0x92AE,                   // Protections:UTINT:Recovery
        PROTECTIONS_RECOVERY_TIME = 0x92AF,        // Protections:Recovery:Time
        HWD_DELAY = 0x92B2,                        // Protections:HWD:Delay
        LOAD_DETECT_ACTIVE_TIME = 0x92B4,          // Protections:Load Detect:Active Time
        LOAD_DETECT_RETRY_DELAY = 0x92B5,          // Protections:Load Detect:Retry Delay
        LOAD_DETECT_TIMEOUT = 0x92B6,              // Protections:Load Detect:Timeout
        PTO_CHARGE_THRESHOLD = 0x92BA,             // Protections:PTO:Charge Threshold
        PTO_DELAY = 0x92BC,                        // Protections:PTO:Delay
        PTO_RESET = 0x92BE,                        // Protections:PTO:Reset
        CU_DEP_THRESHOLD = 0x92C8,                 // Permanent Fail:CUDEP:Threshold
        CU_DEP_DELAY = 0x92CA,                     // Permanent Fail:CUDEP:Delay
        SUV_THRESHOLD = 0x92CB,                    // Permanent Fail:SUV:Threshold
        SUV_DELAY = 0x92CD,                        // Permanent Fail:SUV:Delay
        SOV_THRESHOLD = 0x92CE,                    // Permanent Fail:SOV:Threshold
        SOV_DELAY = 0x92D0,                        // Permanent Fail:SOV:Delay
        TOS_THRESHOLD = 0x92D1,                    // Permanent Fail:TOS:Threshold
        TOS_DELAY = 0x92D3,                        // Permanent Fail:TOS:Delay
        SOCC_THRESHOLD = 0x92D4,                   // Permanent Fail:SOCC:Threshold
        SOCC_DELAY = 0x92D6,                       // Permanent Fail:SOCC:Delay
        SOCD_THRESHOLD = 0x92D7,                   // Permanent Fail:SOCD:Threshold
        SOCD_DELAY = 0x92D9,                       // Permanent Fail:SOCD:Delay
        SOT_THRESHOLD = 0x92DA,                    // Permanent Fail:SOT:Threshold
        SOT_DELAY = 0x92DB,                        // Permanent Fail:SOT:Delay
        SOTF_THRESHOLD = 0x92DC,                   // Permanent Fail:SOTF:Threshold
        SOTF_DELAY = 0x92DD,                       // Permanent Fail:SOTF:Delay
        VIMR_CHECK_VOLTAGE = 0x92DE,               // Permanent Fail:VIMR:Check Voltage
        VIMR_MAX_RELAX_CURRENT = 0x92E0,           // Permanent Fail:VIMR:Max Relax Current
        VIMR_THRESHOLD = 0x92E2,                   // Permanent Fail:VIMR:Threshold
        VIMR_DELAY = 0x92E4,                       // Permanent Fail:VIMR:Delay
        VIMR_RELAX_MIN_DURATION = 0x92E5,          // Permanent Fail:VIMR:Relax Min Duration
        VIMA_CHECK_VOLTAGE = 0x92E7,               // Permanent Fail:VIMA:Check Voltage
        VIMA_MIN_ACTIVE_CURRENT = 0x92E9,          // Permanent Fail:VIMA:Min Active Current
        VIMA_THRESHOLD = 0x92EB,                   // Permanent Fail:VIMA:Threshold
        VIMA_DELAY = 0x92ED,                       // Permanent Fail:VIMA:Delay
        CFETF_OFF_THRESHOLD = 0x92EE,              // Permanent Fail:CFETF:OFF Threshold
        CFETF_OFF_DELAY = 0x92F0,                  // Permanent Fail:CFETF:OFF Delay
        DFETF_OFF_THRESHOLD = 0x92F1,              // Permanent Fail:DFETF:OFF Threshold
        DFETF_OFF_DELAY = 0x92F3,                  // Permanent Fail:DFETF:OFF Delay
        VSSF_FAIL_THRESHOLD = 0x92F4,              // Permanent Fail:VSSF:Fail Threshold
        VSSF_DELAY = 0x92F6,                       // Permanent Fail:VSSF:Delay
        PF_2LVL_DELAY = 0x92F7,                    // Permanent Fail:2LVL:Delay
        LFOF_DELAY = 0x92F8,                       // Permanent Fail:LFOF:Delay
        HWMX_DELAY = 0x92F9,                       // Permanent Fail:HWMX:Delay
        SECURITY_SETTINGS = 0x9256,                // Security:Settings:Security Settings
        UNSEAL_KEY_STEP1 = 0x9257,                 // Security:Keys:Unseal Key Step 1
        UNSEAL_KEY_STEP2 = 0x9259,                 // Security:Keys:Unseal Key Step 2
        FULL_ACCESS_KEY_STEP1 = 0x925B,            // Security:Keys:Full Access Key Step 1
        FULL_ACCESS_KEY_STEP2 = 0x925D,            // Security:Keys:Full Access Key Step 2
    };

    enum class security_state : uint8_t
    {
        NOT_INITIALIZED = 0x00,
        FULLACCESS = 0x01,
        UNSEALED = 0x02,
        SEALED = 0x03,
    };
    enum class regulator_voltage : uint8_t
    {
        VOLTAGE_1_8 = 3,
        VOLTAGE_2_5 = 4,
        VOLTAGE_3_0 = 5,
        VOLTAGE_3_3 = 6,
        VOLTAGE_5_0 = 7,
    };
    enum class temperature_sensor : uint8_t
    {
        INTERNAL = static_cast<uint8_t>(direct_command::INT_TEMPERATURE),
        CFETOFF = static_cast<uint8_t>(direct_command::CFETOFF_TEMPERATURE),
        DFETOFF = static_cast<uint8_t>(direct_command::DFETOFF_TEMPERATURE),
        ALERT = static_cast<uint8_t>(direct_command::ALERT_TEMPERATURE),
        TS1 = static_cast<uint8_t>(direct_command::TS1_TEMPERATURE),
        TS2 = static_cast<uint8_t>(direct_command::TS2_TEMPERATURE),
        TS3 = static_cast<uint8_t>(direct_command::TS3_TEMPERATURE),
        HDQ = static_cast<uint8_t>(direct_command::HDQ_TEMPERATURE),
        DCHG = static_cast<uint8_t>(direct_command::DCHG_TEMPERATURE),
        DDSG = static_cast<uint8_t>(direct_command::DDSG_TEMPERATURE),
    };
    enum class gpo_state : uint16_t
    {
        CFETOFF_LO = static_cast<uint16_t>(cmd_only_subcommand::CFETOFF_LO),
        CFETOFF_HI = static_cast<uint16_t>(cmd_only_subcommand::CFETOFF_HI),
        DFETOFF_LO = static_cast<uint16_t>(cmd_only_subcommand::DFETOFF_LO),
        DFETOFF_HI = static_cast<uint16_t>(cmd_only_subcommand::DFETOFF_HI),
        ALERT_LO = static_cast<uint16_t>(cmd_only_subcommand::ALERT_LO),
        ALERT_HI = static_cast<uint16_t>(cmd_only_subcommand::ALERT_HI),
        HDQ_LO = static_cast<uint16_t>(cmd_only_subcommand::HDQ_LO),
        HDQ_HI = static_cast<uint16_t>(cmd_only_subcommand::HDQ_HI),
        DCHG_LO = static_cast<uint16_t>(cmd_only_subcommand::DCHG_LO),
        DCHG_HI = static_cast<uint16_t>(cmd_only_subcommand::DCHG_HI),
        DDSG_LO = static_cast<uint16_t>(cmd_only_subcommand::DDSG_LO),
        DDSG_HI = static_cast<uint16_t>(cmd_only_subcommand::DDSG_HI),
    };
    enum class gpo : uint8_t
    {
        CFETOFF,
        DFETOFF,
        ALERT,
        HDQ,
        DCHG,
        DDSG,
    };
    enum class measurement_loop_speed : uint8_t
    {
        FULL_SPEED = 0,
        HALF_SPEED = 1,
        QUARTER_SPEED = 2,
        EIGHTH_SPEED = 3,
    };
    enum class coulomb_conversion_speed : uint8_t
    {
        T_48MS = 0x0,
        T_24MS = 0x1,
        // WARNING: This setting may exhibit a large offset and should be avoided
        T_12MS = 0x2,
        // WARNING: This setting results in ~100uV noise and should only be used when
        // Wake Comparator Current is set so that |VSRP-VSRN|>1000uV
        T_6MS = 0x3,
    };
    enum class hwd_toggle_option : uint8_t
    {
        NO_ACTION = 0,
        REGULATORS_OFF = 1,
        REGULATORS_OFF_THEN_ON = 2,
        RESERVED = 3,
    };
    enum class comm_type : uint8_t
    {
        // for some reason, naming this DEFAULT breaks the *entire* file (over 1.4k errors)
        DEFAULT_1 = 0x00,
        DEFAULT_2 = 0xFF,
        HDQ_ON_ALERT_PIN = 0x03,
        HDQ_ON_HDQ_PIN = 0x04,
        I2C = 0x07,
        I2C_FAST = 0x08,
        I2C_FAST_WITH_TIMEOUT = 0x09,
        SPI = 0x0F,
        SPI_WITH_CRC = 0x10,
        I2C_WITH_CRC = 0x11,
        I2C_FAST_WITH_CRC = 0x12,
        I2C_WITH_TIMEOUTS = 0x1E,
    };
    enum class pullup_config : uint8_t
    {
        PULLUP_18K = 0,
        PULLUP_180K = 1,
        PULLUP_NONE = 2,
    };
    enum class polynomial_selection : uint8_t
    {
        POLYNOMIAL_18K = 0,
        POLYNOMIAL_180K = 1,
        POLYNOMIAL_CUSTOM = 2,
        POLYNOMIAL_NONE = 3,
    };
    enum class measurement_type : uint8_t
    {
        ADC = 0,
        CELL_THERMISTOR = 1,
        GENERAL_THERMISTOR = 2,
        FET_THERMISTOR = 3,
    };
    enum class adc_pin_function : uint8_t
    {
        UNUSED = 0,
        ADC_INPUT_OR_THERMISTOR = 3,
    };
    enum class cfetoff_pin_function : uint8_t
    {
        SPI_CS_OR_UNUSED = 0,
        GPO = 1,
        CFETOFF = 2,
    };
    enum class dfetoff_pin_function : uint8_t
    {
        UNUSED = 0,
        GPO = 1,
        DFETOFF_OR_BOTHOFF = 2,
    };
    enum class alert_pin_function : uint8_t
    {
        HDQ_OR_UNUSED = 0,
        GPO = 1,
        ALERT = 2,
    };
    enum class hdq_pin_function : uint8_t
    {
        HDQ_OR_MOSI = 0,
        GPO = 1,
        UNUSED = 2,
    };
    enum class dchg_pin_function : uint8_t
    {
        UNUSED = 0,
        GPO = 1,
        DCHG = 2,
    };
    enum class ddsg_pin_function : uint8_t
    {
        UNUSED = 0,
        GPO = 1,
        DDSG = 2,
    };
    enum class user_amps : uint8_t
    {
        DECIMILLIAMP = 0,
        MILLIAMP = 1,
        CENTIAMP = 2,
        DECIAMP = 3,
    };
    enum class short_circuit_discharge_threshold : uint8_t
    {
        THRESHOLD_10MV = 0,
        THRESHOLD_20MV = 1,
        THRESHOLD_40MV = 2,
        THRESHOLD_60MV = 3,
        THRESHOLD_80MV = 4,
        THRESHOLD_100MV = 5,
        THRESHOLD_125MV = 6,
        THRESHOLD_150MV = 7,
        THRESHOLD_175MV = 8,
        THRESHOLD_200MV = 9,
        THRESHOLD_250MV = 10,
        THRESHOLD_300MV = 11,
        THRESHOLD_350MV = 12,
        THRESHOLD_400MV = 13,
        THRESHOLD_450MV = 14,
        THRESHOLD_500MV = 15,
    };

#pragma pack(push, 1)
    union control_status_t
    {
        struct
        {
            uint16_t : 15;                   // RSVD_0
            bool inDeepsleep : 1;            // DEEPSLEEP
            bool hasLoadDetectTimedOut : 1;  // LD_TIMEOUT
            // was pullup active during previous measurement?
            bool wasLoadDetectOn : 1;  // LD_ON
        };
        uint16_t raw = 0x0000;
    };
    union protections_a_t
    {
        struct
        {
            bool shortCircuitDischarge : 1;  // SCD
            bool overcurrentDischarge2 : 1;  // OCD2
            bool overcurrentDischarge1 : 1;  // OCD1
            bool overcurrentCharge : 1;      // OCC
            bool cellOvervoltage : 1;        // COV
            bool cellUndervoltage : 1;       // CUV
            bool : 2;                        // RSVD_0
        };
        uint8_t raw = 0x00;
    };
    union protections_b_t
    {
        struct
        {
            bool fetOvertemp : 1;         // OTF
            bool internalOvertemp : 1;    // OTINT
            bool overtempDischarge : 1;   // OTD
            bool overtempCharge : 1;      // OTC
            bool : 1;                     // RSVD_0
            bool internalUndertemp : 1;   // UTINT
            bool undertempDischarge : 1;  // UTD
            bool undertempCharge : 1;     // UTC
        };
        uint8_t raw = 0x00;
    };
    union safety_alert_c_t
    {
        struct
        {
            bool overcurrentDischarge3 : 1;       // OCD3
            bool shortCircuitDischargeLatch : 1;  // SCDL
            bool overcurrentDischargeLatch : 1;   // OCDL
            bool cellOvervoltageLatch : 1;        // COVL
            bool prechargeTimeoutSuspend : 1;     // PTOS
            bool : 3;                             // RSVD_0
        };
        uint8_t raw = 0x00;
    };
    union protections_c_t
    {
        struct
        {
            bool overcurrentDischarge3 : 1;       // OCD3
            bool shortCircuitDischargeLatch : 1;  // SCDL
            bool overcurrentDischargeLatch : 1;   // OCDL
            bool cellOvervoltageLatch : 1;        // COVL
            // NOTE: When used in Settings:Protection:Enabled Protections C,
            // this bit is only RSVD not RSVD_0
            bool : 1;                    // RSVD_0
            bool prechargeTimeout : 1;   // PTO
            bool hostWatchdogFault : 1;  // HWDF
            bool : 1;                    // RSVD_0
        };
        uint8_t raw = 0x00;
    };
    union permanent_fail_a_t
    {
        struct
        {
            bool copperDeposition : 1;            // CUDEP
            bool safetyOvertempFet : 1;           // SOTF
            bool : 1;                             // RSVD_0
            bool safetyOvertemp : 1;              // SOT
            bool safetyOvercurrentDischarge : 1;  // SOCD
            bool safetyOvercurrentCharge : 1;     // SOCC
            bool safetyCellOvervoltage : 1;       // SOV
            bool safetyCellUndervoltage : 1;      // SUV
        };
        uint8_t raw = 0x00;
    };
    union permanent_fail_b_t
    {
        struct
        {
            bool shortCircuitDischargeLatch : 1;  // SCDL
            bool : 2;                             // RSVD_0
            bool voltageImbalanceActive : 1;      // VIMA
            bool voltageImbalanceRelax : 1;       // VIMR
            bool secondLevelProtector : 1;        // 2LVL
            bool dischargeFet : 1;                // DFETF
            bool chargeFet : 1;                   // CFETF
        };
        uint8_t raw = 0x00;
    };
    union permanent_fail_alert_c_t
    {
        struct
        {
            bool : 1;                           // RSVD_0
            bool hardwareMux : 1;               // HWMX
            bool internalVssMeasurement : 1;    // VSSF
            bool internalVoltageReference : 1;  // VREF
            bool internalLFO : 1;               // LFOF
            bool : 3;                           // RSVD_0
        };
        uint8_t raw = 0x00;
    };
    union permanent_fail_c_t
    {
        struct
        {
            bool commanded : 1;                 // CMDF
            bool hardwareMux : 1;               // HWMX
            bool internalVssMeasurement : 1;    // VSSF
            bool internalVoltageReference : 1;  // VREF
            bool internalLFO : 1;               // LFOF
            bool instructionROM : 1;            // IRMF
            bool dataROM : 1;                   // DRMF
            bool otpMemory : 1;                 // OTPF
        };
        uint8_t raw = 0x00;
    };
    union permanent_fail_d_t
    {
        struct
        {
            bool : 7;  // RSVD_0
            // top of stack vs cell sum
            bool topStackVsCell : 1;  // TOSF
        };
        uint8_t raw = 0x00;
    };
    union battery_status_t
    {
        struct
        {
            bool inSleep : 1;                           // SLEEP
            bool : 1;                                   // RSVD_0
            bool shutdownPending : 1;                   // SD_CMD
            bool permanentFailActive : 1;               // PF
            bool safetyFaultActive : 1;                 // SS
            bool fuseActive : 1;                        // FUSE
            bq76942::security_state securityState : 2;  // SEC[1:0]
            bool otpWriteBlocked : 1;                   // OTPB
            bool pendingOtpWrite : 1;                   // OTPW
            bool checkingCellOpenWire : 1;              // COW_CHK
            // whether or not the previous reset was due to the watchdog timer
            // NOTE: Independent of Host Watchdog settings
            bool wasWatchdogReset : 1;  // WD
            // AKA: Power-On Reset
            // Whether or not a full reset has occurred since last CONFIG_UPDATE exit
            bool fullResetOccurred : 1;   // POR
            bool sleepAllowed : 1;        // SLEEP_EN
            bool inPrechargeMode : 1;     // PCHG_MODE
            bool inConfigUpdateMode : 1;  // CFGUPDATE
        };
        uint16_t raw = 0x0000;
    };
    union alarm_status_t
    {
        struct
        {
            bool safetyStatusBOrC : 1;  // SSBC
            bool safetyStatusA : 1;     // SSA
            bool permanentFail : 1;     // PF
            // anything in the alarm SF A, B, or C masks triggered
            bool alarmSFAlert : 1;  // MSK_SFALERT
            // anything in the alarm PF A, B, C, or D masks triggered
            bool alarmPFAlert : 1;                 // MSK_PFALERT
            bool initializationStarted : 1;        // INITSTART
            bool initializationComplete : 1;       // INITCOMP
            bool : 1;                              // RSVD_0
            bool fullVoltageScanComplete : 1;      // FULLSCAN
            bool chargeFetOff : 1;                 // XCHG
            bool dischargeFetOff : 1;              // XDSG
            bool stackReachedShutdownVoltage : 1;  // SHUTV
            bool isFuseDriven : 1;                 // FUSE
            bool isBalancingCells : 1;             // CB
            bool adcScanComplete : 1;              // ADSCAN
            bool hasWokenFromSleep : 1;            // WAKE
        };
        uint16_t raw = 0x0000;
    };
    union fet_status_t
    {
        struct
        {
            bool : 1;                    // RSVD_0
            bool alertAsserted : 1;      // ALRT_PIN
            bool ddsgAsserted : 1;       // DDSG_PIN
            bool dchtAsserted : 1;       // DCHG_PIN
            bool predischargeFetOn : 1;  // PDSG_FET
            bool dischargeFetOn : 1;     // DSG_FET
            bool prechargeFetOn : 1;     // PCHG_FET
            bool chargeFetOn : 1;        // CHG_FET
        };
        uint8_t raw = 0x0000;
    };
    union manufacturing_status_t
    {
        struct
        {
            bool : 8;                         // RSVD_0
            bool isOtpWriteEnabled : 1;       // OTPW_EN
            bool isPermanentFailEnabled : 1;  // PF_EN
            bool predischargeTesting : 1;     // PDSG_TEST
            bool fetTestMode : 1;             // FET_EN
            bool : 1;                         // RSVD_0
            bool dischargeTesting : 1;        // DSG_TEST
            bool chargeTesting : 1;           // CHG_TEST
            bool prechargeTesting : 1;        // PCHG_TEST
        };
        uint16_t raw = 0x0000;
    };
    union fet_control_t
    {
        struct
        {
            bool : 4;                       // RSVD_0
            bool forcePrechargeOff : 1;     // PCHG_OFF
            bool forceChargeOff : 1;        // CHG_OFF
            bool forcePredischargeOff : 1;  // PDSG_OFF
            bool forceDischargeOff : 1;     // DSG_OFF
        };
        uint8_t raw = 0x00;
    };
    union regulator_control_t
    {
        struct
        {
            struct
            {
                bq76942::regulator_voltage voltage : 3;  // REG2V_[2:0]
                bool enable : 1;                         // REG2_EN
            } reg1;
            struct
            {
                bq76942::regulator_voltage voltage : 3;  // REG1V_[2:0]
                bool enable : 1;                         // REG1_EN
            } reg2;
        };
        uint8_t raw = 0x00;
    };
    struct otp_write_result_t
    {
        union resultRegister
        {
            struct
            {
                bool otpProgrammingOk : 1;    // OK
                bool : 1;                     // RSVD_0
                bool otpLocked : 1;           // LOCK
                bool signatureWriteFail : 1;  // NOSIG
                bool dataWriteFail : 1;       // NODATA
                // note: The following are NOT errors for the whole chip,
                // OTP programming just has stricter requirements
                bool overTemp : 1;      // HT
                bool underVoltage : 1;  // LV
                bool overVoltage : 1;   // HV
            };
            uint8_t raw = 0x00;
        };
        uint16_t failAddress;
    };
    struct firmware_version_t
    {
        union
        {
            uint16_t deviceNumber;
            uint8_t deviceNumberBytes[2];
        };
        union
        {
            uint16_t firmwareVersion;
            uint8_t firmwareVersionBytes[2];
        };
        union
        {
            uint16_t buildNumber;
            struct
            {
                uint8_t bcdHighOfHighByte : 4;
                uint8_t bcdLowOfHighByte : 4;
                uint8_t bcdHighOfLowByte : 4;
                uint8_t bcdLowOfLowByte : 4;
            };
        };
    };
    struct security_key_pair_t
    {
        union
        {
            uint16_t step1;
            uint8_t step1Bytes[2];
        };
        union
        {
            uint16_t step2;
            uint8_t step2Bytes[2];
        };
    };
    union security_keys_t
    {
        struct
        {
            security_key_pair_t unseal;
            security_key_pair_t fullAccess;
        };
        uint64_t raw = 0x0000000000000000;
    };
    struct saved_pf_status_t
    {
        permanent_fail_a_t pfStatusA;
        permanent_fail_b_t pfStatusB;
        permanent_fail_c_t pfStatusC;
        permanent_fail_d_t pfStatusD;
    };
    struct da_status_1_t
    {
        int32_t cell1VoltageCounts;
        int32_t cell1CurrentCounts;
        int32_t cell2VoltageCounts;
        int32_t cell2CurrentCounts;
        int32_t cell3VoltageCounts;
        int32_t cell3CurrentCounts;
        int32_t cell4VoltageCounts;
        int32_t cell4CurrentCounts;
    };
    struct da_status_2_t
    {
        int32_t cell5VoltageCounts;
        int32_t cell5CurrentCounts;
        int32_t cell6VoltageCounts;
        int32_t cell6CurrentCounts;
        int32_t cell7VoltageCounts;
        int32_t cell7CurrentCounts;
        int32_t cell8VoltageCounts;
        int32_t cell8CurrentCounts;
    };
    struct da_status_3_t
    {
        int32_t cell9VoltageCounts;
        int32_t cell9CurrentCounts;
        int32_t cell10VoltageCounts;
        int32_t cell10CurrentCounts;
    };
    struct raw_da_status_5_t
    {
        int16_t vreg18AdcCounts;
        int16_t vssAdcCounts;
        int16_t maxCellVoltage;
        int16_t minCellVoltage;
        int16_t batteryVoltageSum;
        int16_t cellTemperature;
        int16_t fetTemperature;
        int16_t maxCellTemperature;
        int16_t minCellTemperature;
        int16_t avgCellTemperature;
        int16_t cc3Current;
        int16_t cc1Current;
        int32_t cc2Counts;
        int32_t cc3Counts;
    };
    struct raw_da_status_6_t
    {
        int32_t accumulatedCharge;
        uint32_t accumulatedChargeFraction;
        uint32_t accumulatedChargeTime;
        int32_t cfetoffCounts;
        int32_t dfetoffCounts;
        int32_t alertCounts;
        int32_t ts1Counts;
        int32_t ts2Counts;
    };
    struct da_status_7_t
    {
        int32_t ts3Counts;
        int32_t hdqCounts;
        int32_t dchgCounts;
        int32_t ddsgCounts;
    };
    union voltage_snapshot_t
    {
        struct
        {
            int16_t cell1Voltage;
            int16_t cell2Voltage;
            int16_t cell3Voltage;
            int16_t cell4Voltage;
            int16_t cell5Voltage;
            int16_t cell6Voltage;
            int16_t cell7Voltage;
            int16_t cell8Voltage;
            int16_t cell9Voltage;
            int16_t cell10Voltage;
        };
        int16_t voltages[10];
    };
    union cb_active_cells_t
    {
        struct
        {
            bool : 6;  // Unused, likely RSVD_0
            bool cell10 : 1;
            bool cell9 : 1;
            bool cell8 : 1;
            bool cell7 : 1;
            bool cell6 : 1;
            bool cell5 : 1;
            bool cell4 : 1;
            bool cell3 : 1;
            bool cell2 : 1;
            bool cell1 : 1;
        };
        uint16_t raw = 0x0000;
    };
    struct cb_status_2_t
    {
        uint32_t cell1BalancingTime;
        uint32_t cell2BalancingTime;
        uint32_t cell3BalancingTime;
        uint32_t cell4BalancingTime;
        uint32_t cell5BalancingTime;
        uint32_t cell6BalancingTime;
        uint32_t cell7BalancingTime;
        uint32_t cell8BalancingTime;
    };
    struct cb_status_3_t
    {
        uint32_t cell9BalancingTime;
        uint32_t cell10BalancingTime;
    };
    struct cal1_t
    {
        int16_t calibrationDataCounter;
        int32_t cc2Counts;
        int16_t packCounts;
        int16_t topOfStackCounts;
        int16_t ldCounts;
    };

    // --- DATA REGISTERS ---
    union power_config_t
    {
        struct
        {
            bool : 2;                                  // RSVD_0
            bool enableDeepSleepOvertempShutdown : 1;  // DPSLP_OT
            bool disableShutdownTS2Wake : 1;           // SHUT_TS2
            bool enableDeepSleepChargerWake : 1;       // DPSLP_PD
            bool enableDeepSleepLDO : 1;               // DPSLP_LDO
            bool enableDeepSleepLFO : 1;               // DPSLP_LFO
            bool enableSleep : 1;                      // SLEEP
            bool enableOvertempShutdown : 1;           // OTSD
            // cleared: 3ms per conversion
            // set: 1.5ms per conversion (but lower accuracy)
            bool useFastAdc : 1;  // FASTADC
            // measurement loop speed during cell balancing
            bq76942::measurement_loop_speed cellBalanceLoopSpeed : 2;  // CB_LOOP_SLOW_[1:0]
            // measurement loop speed during normal operation
            bq76942::measurement_loop_speed loopSpeed : 2;    // LOOP_SLOW_[1:0]
            bq76942::coulomb_conversion_speed wakeSpeed : 2;  // WK_SPD_[1:0]
        };
        uint16_t raw = 0x0000;
    };
    union reg0_config_t
    {
        struct
        {
            bool : 6;             // RSVD_0
            bool : 1;             // RSVD
            bool reg0Enable : 1;  // REG0_EN
        };
        uint8_t raw = 0x00;
    };
    union hwd_regulator_options_t
    {
        struct
        {
            bool : 2;                                  // RSVD_0
            bq76942::hwd_toggle_option hwdAction : 2;  // TOGGLE_OPT_[1:0]
            // Regulator off time in seconds before turning back on
            uint8_t toggleTime : 4;  // TOGGLE_TIME_[3:0]
        };
        uint8_t raw = 0x00;
    };
    union spi_configuration_t
    {
        struct
        {
            bool : 1;  // RSVD_0
            // clear: MISO uses REG18 voltage
            // set: MISO uses REG1 voltage
            bool misoUsesReg1 : 1;  // MISO_REG1
            // use digital filters (recommend for high-freq operation)
            bool enableFilters : 1;  // FILT
            bool : 5;                // RSVD_0
        };
        uint8_t raw = 0x00;
    };
    union cfetoff_pin_configuration_t
    {
        struct
        {
            union
            {
                struct
                {
                    bool isActiveLow : 1;  // OPT[5]
                    bool : 1;              // OPT[4]
                    // clear: High uses REG18
                    // set: High uses REG1
                    bool driveHighUsesReg1 : 1;  // OPT[3]
                    // NOTE: Should be cleared when isActiveLow is set
                    bool pullupToReg1 : 1;  // OPT[2]
                    // clear: Driving high drives to HI-Z (unavailable with isActiveLow)
                    // set: Driving high drives to selected regulator
                    bool disableHighZDrive : 1;         // OPT[1]
                    bool enablePulldown : 1;            // OPT[0]
                    cfetoff_pin_function function : 2;  // PINFXN[1:0]
                } function;
                struct
                {
                    pullup_config pullupConfig : 2;        // OPT[5:4]
                    polynomial_selection polynomial : 2;   // OPT[3:2]
                    measurement_type measurementType : 2;  // OPT[1:0]
                    adc_pin_function function : 2;         // PINFXN[1:0]
                } adc;
            };
        };
        uint8_t raw = 0x00;
    };
    union dfetoff_pin_configuration_t
    {
        struct
        {
            union
            {
                struct
                {
                    bool isActiveLow : 1;  // OPT[5]
                    // clear: acts as DFETOFF
                    // set: acts as BOTHOFF
                    bool isBothOff : 1;  // OPT[4]
                    // clear: High uses REG18
                    // set: High uses REG1
                    bool driveHighUsesReg1 : 1;  // OPT[3]
                    // NOTE: Should be cleared when isActiveLow is set
                    bool pullupToReg1 : 1;  // OPT[2]
                    // clear: Driving high drives to HI-Z (unavailable with isActiveLow)
                    // set: Driving high drives to selected regulator
                    bool disableHighZDrive : 1;         // OPT[1]
                    bool enablePulldown : 1;            // OPT[0]
                    dfetoff_pin_function function : 2;  // PINFXN[1:0]
                } function;
                struct
                {
                    pullup_config pullupConfig : 2;        // OPT[5:4]
                    polynomial_selection polynomial : 2;   // OPT[3:2]
                    measurement_type measurementType : 2;  // OPT[1:0]
                    adc_pin_function function : 2;         // PINFXN[1:0]
                } adc;
            };
        };
        uint8_t raw = 0x00;
    };
    union alert_pin_configuration_t
    {
        struct
        {
            union
            {
                struct
                {
                    bool isActiveLow : 1;  // OPT[5]
                    bool : 1;              // OPT[4]
                    // clear: High uses REG18
                    // set: High uses REG1
                    bool driveHighUsesReg1 : 1;  // OPT[3]
                    // NOTE: Should be cleared when isActiveLow is set
                    bool pullupToReg1 : 1;  // OPT[2]
                    // clear: Driving high drives to HI-Z (unavailable with isActiveLow)
                    // set: Driving high drives to selected regulator
                    bool disableHighZDrive : 1;       // OPT[1]
                    bool enablePulldown : 1;          // OPT[0]
                    alert_pin_function function : 2;  // PINFXN[1:0]
                } function;
                struct
                {
                    pullup_config pullupConfig : 2;        // OPT[5:4]
                    polynomial_selection polynomial : 2;   // OPT[3:2]
                    measurement_type measurementType : 2;  // OPT[1:0]
                    adc_pin_function function : 2;         // PINFXN[1:0]
                } adc;
            };
        };
        uint8_t raw = 0x00;
    };
    union ts_pin_configuration_t
    {
        struct
        {
            union
            {
                struct
                {
                    pullup_config pullupConfig : 2;        // OPT[5:4]
                    polynomial_selection polynomial : 2;   // OPT[3:2]
                    measurement_type measurementType : 2;  // OPT[1:0]
                    adc_pin_function function : 2;         // PINFXN[1:0]
                } adc;
            };
        };
        uint8_t raw = 0x00;
    };
    union hdq_pin_configuration_t
    {
        struct
        {
            union
            {
                struct
                {
                    bool isActiveLow : 1;  // OPT[5]
                    bool : 1;              // OPT[4]
                    // clear: High uses REG18
                    // set: High uses REG1
                    bool driveHighUsesReg1 : 1;  // OPT[3]
                    // NOTE: Should be cleared when isActiveLow is set
                    bool pullupToReg1 : 1;  // OPT[2]
                    // clear: Driving high drives to HI-Z (unavailable with isActiveLow)
                    // set: Driving high drives to selected regulator
                    bool disableHighZDrive : 1;     // OPT[1]
                    bool enablePulldown : 1;        // OPT[0]
                    hdq_pin_function function : 2;  // PINFXN[1:0]
                } function;
                struct
                {
                    pullup_config pullupConfig : 2;        // OPT[5:4]
                    polynomial_selection polynomial : 2;   // OPT[3:2]
                    measurement_type measurementType : 2;  // OPT[1:0]
                    adc_pin_function function : 2;         // PINFXN[1:0]
                } adc;
            };
        };
        uint8_t raw = 0x00;
    };
    union dchg_pin_configuration_t
    {
        struct
        {
            union
            {
                struct
                {
                    bool isActiveLow : 1;  // OPT[5]
                    bool : 1;              // OPT[4]
                    // clear: High uses REG18
                    // set: High uses REG1
                    bool driveHighUsesReg1 : 1;  // OPT[3]
                    // NOTE: Should be cleared when isActiveLow is set
                    bool pullupToReg1 : 1;  // OPT[2]
                    // clear: Driving high drives to HI-Z (unavailable with isActiveLow)
                    // set: Driving high drives to selected regulator
                    bool disableHighZDrive : 1;      // OPT[1]
                    bool enablePulldown : 1;         // OPT[0]
                    dchg_pin_function function : 2;  // PINFXN[1:0]
                } function;
                struct
                {
                    pullup_config pullupConfig : 2;        // OPT[5:4]
                    polynomial_selection polynomial : 2;   // OPT[3:2]
                    measurement_type measurementType : 2;  // OPT[1:0]
                    adc_pin_function function : 2;         // PINFXN[1:0]
                } adc;
            };
        };
        uint8_t raw = 0x00;
    };
    union ddsg_pin_configuration_t
    {
        struct
        {
            union
            {
                struct
                {
                    bool isActiveLow : 1;  // OPT[5]
                    bool : 1;              // OPT[4]
                    // clear: High uses REG18
                    // set: High uses REG1
                    bool driveHighUsesReg1 : 1;  // OPT[3]
                    // NOTE: Should be cleared when isActiveLow is set
                    bool pullupToReg1 : 1;  // OPT[2]
                    // clear: Driving high drives to HI-Z (unavailable with isActiveLow)
                    // set: Driving high drives to selected regulator
                    bool disableHighZDrive : 1;      // OPT[1]
                    bool enablePulldown : 1;         // OPT[0]
                    ddsg_pin_function function : 2;  // PINFXN[1:0]
                } function;
                struct
                {
                    pullup_config pullupConfig : 2;        // OPT[5:4]
                    polynomial_selection polynomial : 2;   // OPT[3:2]
                    measurement_type measurementType : 2;  // OPT[1:0]
                    adc_pin_function function : 2;         // PINFXN[1:0]
                } adc;
            };
        };
        uint8_t raw = 0x00;
    };
    union da_configuration_t
    {
        struct
        {
            bool : 3;                               // RSVD_0
            bool useInternalAsFetTemperature : 1;   // TINT_FETT
            bool useInternalAsCellTemperature : 1;  // TINT_EN
            // clear: User volts is mV
            // set: User volts is cV (10mV)
            bool userVoltsIsCentivolts : 1;  // USER_VOLTS_CV
            user_amps userAmps : 2;          // USER_AMPS_[1:0]
        };
        uint8_t raw = 0x00;
    };
    union vcell_mode_t
    {
        struct
        {
            bool : 6;  // RSVD_0
            bool isCell10Present : 1;
            bool isCell9Present : 1;
            bool isCell8Present : 1;
            bool isCell7Present : 1;
            bool isCell6Present : 1;
            bool isCell5Present : 1;
            bool isCell4Present : 1;
            bool isCell3Present : 1;
            bool isCell2Present : 1;
            bool isCell1Present : 1;
        };
        uint16_t raw = 0x0000;
    };
    union protection_configuration_t
    {
        struct
        {
            bool : 5;  // RSVD_0
            //
            bool useShortCircuitChargeRecovery : 1;  // SCDL_CURR_RECOVERY
            bool useOvercurrentChargeRecovery : 1;   // OCDL_CURR_RECOVERY
            // a FET Permanent Failure will ignore the min blow fuse voltage if this is set
            bool fetFaultIgnoresFuseVoltage : 1;  // FETF_FUSE
            // if set, will use PACK voltage instead of TOS (Top Of Stack)
            bool fuseVoltageUsesPackVoltage : 1;  // PACK_FUSE
            bool : 1;                             // RSVD_0
            // if set, will write PF status to the OTP
            bool permanentFailWritesOtp : 1;  // PF_OTP
            // if set, will blow the fuse after a PF event
            bool permanentFailBlowsFuse : 1;  // PF_FUSE
            // if set, will enter deepsleep after writing OTP (if applicable) in a PF event
            bool permanentFailCausesDeepSleep : 1;  // PF_DPSLP
            // if set, a PF will turn off the regulators
            bool permanentFailStopsRegulators : 1;  // PF_REGS
            // if set, a PF will turn off the FETs
            bool permanentFailStopsFets : 1;  // PF_FETS
            bool : 1;                         // RSVD_0
        };
        uint16_t raw = 0x0000;
    };
    union chg_fet_protections_a_t
    {
        struct
        {
            bool shortCircuitDischarge : 1;  // SCD
            bool : 2;                        // RSVD_0
            bool overcurrentCharge : 1;      // OCC
            bool cellOvervoltage : 1;        // COV
            bool : 3;                        // RSVD_0
        };
        uint8_t raw = 0x00;
    };
    union chg_fet_protections_b_t
    {
        struct
        {
            bool fetOvertemperature : 1;        // OTF
            bool internalOvertemperature : 1;   // OTINT
            bool : 1;                           // RSVD_0
            bool chargeOvertemperature : 1;     // OTC
            bool : 1;                           // RSVD_0
            bool internalUndertemperature : 1;  // UTINT
            bool : 1;                           // RSVD_0
            bool chargeUndertemperature : 1;    // UTC
        };
        uint8_t raw = 0x00;
    };
    union chg_fet_protections_c_t
    {
        struct
        {
            bool : 1;                             // RSVD_0
            bool shortCircuitDischargeLatch : 1;  // SCDL
            bool : 1;                             // RSVD_0
            bool cellOvervoltageLatch : 1;        // COVL
            bool : 1;                             // RSVD_0
            bool prechargeTimeout : 1;            // PTO
            bool hostWatchdogFault : 1;           // HWDF
            bool : 1;                             // RSVD_0
        };
        uint8_t raw = 0x00;
    };
    union dsg_fet_protections_a_t
    {
        struct
        {
            bool shortCircuitDischarge : 1;  // SCD
            bool overcurrentDischarge2 : 1;  // OCD2
            bool overcurrentDischarge1 : 1;  // OCD1
            bool : 2;                        // RSVD_0
            bool cellUndervoltage : 1;       // CUV
            bool : 2;                        // RSVD_0
        };
        uint8_t raw = 0x00;
    };
    union dsg_fet_protections_b_t
    {
        struct
        {
            bool fetOvertemperature : 1;         // OTF
            bool internalOvertemperature : 1;    // OTINT
            bool dischargeOvertemperature : 1;   // OTD
            bool : 2;                            // RSVD_0
            bool internalUndertemperature : 1;   // UTINT
            bool dischargeUndertemperature : 1;  // UTD
            bool : 1;                            // RSVD_0
        };
        uint8_t raw = 0x00;
    };
    union dsg_fet_protections_c_t
    {
        struct
        {
            bool overcurrentDischarge3 : 1;       // OCD3
            bool shortCircuitDischargeLatch : 1;  // SCDL
            bool overcurrentDischargeLatch : 1;   // OCDL
            bool : 3;                             // RSVD
            bool hostWatchdogFault : 1;           // HWDF
            bool : 1;                             // RSVD_0
        };
        uint8_t raw = 0x00;
    };
    // TODO: check that this sets RSVD_1 correctly
    union alarm_sf_alert_mask_c_t
    {
        struct
        {
            bool overcurrentDischarge3 : 1;       // OCD3
            bool shortCircuitDischargeLatch : 1;  // SCDL
            bool overcurrentDischargeLatch : 1;   // OCDL
            bool cellOvervoltageLatch : 1;        // COVL
            bool : 1;                             // RSVD_0
            bool prechargeTimeout : 1;            // PTO
            bool : 1;                             // RSVD_1
            bool : 1;                             // RSVD_0
        };
        uint8_t raw = 0x02;
    };
    union fet_options_t
    {
        struct
        {
            bool : 2;                             // RSVD_0
            bool hostControlDefaultsFetsOff : 1;  // FET_INIT_OFF
            bool enablePredischarge : 1;          // PDSG_EN
            bool enableFetControl : 1;            // FET_CTRL_EN
            bool allowHostFetControl : 1;         // HOST_FET_EN
            bool allowChargeInSleep : 1;          // SLEEPCHG
            bool enableBodyDiodeProtection : 1;   // SFET (Series FET)
        };
        uint8_t raw = 0x00;
    };
    union fet_charge_pump_control_t
    {
        struct
        {
            bool : 5;  // RSVD_0
            // NOTE: Normally only used when allowChargeInSleep is cleared
            bool enableSleepDsgSourceFollower : 1;  // SFMODE_SLEEP
            // clear: Charge pump uses 11V overdrive
            // set: Charge pump uses 5.5V overdrive
            bool useLowOverdrive : 1;  // LVEN
            // NOTE: If cleared, enableFetControl should likely also be cleared
            bool enableChargePump : 1;  // CP_EN
        };
        uint8_t raw = 0x00;
    };
    union manufacturing_status_init_t
    {
        struct
        {
            bool : 8;                         // RSVD_0
            bool isOtpWriteEnabled : 1;       // OTPW_EN
            bool isPermanentFailEnabled : 1;  // PF_EN
            bool : 1;                         // PDSG_TEST
            bool fetTestMode : 1;             // FET_EN
            bool : 4;                         // RSVD_0
        };
        uint16_t raw = 0x0000;
    };
    union balancing_configuration_t
    {
        struct
        {
            bool : 3;                                // RSVD_0
            bool ignoreHostControlledBalancing : 1;  // CB_NO_CMD
            bool balancingExitsSleep : 1;            // CB_NOSLEEP
            bool allowBalancingInSleep : 1;          // CB_SLEEP
            bool allowRelaxedCellBalancing : 1;      // CB_RELAX
            bool allowChargingCellBalancing : 1;     // CB_CHG
        };
        uint8_t raw = 0x00;
    };
    union security_settings_t
    {
        struct
        {
            bool : 5;                  // RSVD_0
            bool permanentSeal : 1;    // PERM_SEAL
            bool lockConfig : 1;       // LOCK_CFG
            bool defaultToSealed : 1;  // SEAL
        };
        uint8_t raw = 0x00;
    };
#pragma pack(pop)

    typedef std::array<uint8_t, 32> manufacturer_data_t;
    typedef voltage_snapshot_t cuv_snapshot_t;
    typedef voltage_snapshot_t cov_snapshot_t;

    struct da_status_5_t
    {
        int16_t vreg18AdcCounts;
        int16_t vssAdcCounts;
        int16_t maxCellVoltage;
        int16_t minCellVoltage;
        int32_t batteryVoltageSum;
        float cellTemperature;
        float fetTemperature;
        float maxCellTemperature;
        float minCellTemperature;
        float avgCellTemperature;
        float cc3Current;
        float cc1Current;
        int32_t cc2Counts;
        int32_t cc3Counts;
    };
    struct da_status_6_t
    {
        double accumulatedCharge;
        std::chrono::seconds accumulatedChargeTime;
        int32_t cfetoffCounts;
        int32_t dfetoffCounts;
        int32_t alertCounts;
        int32_t ts1Counts;
        int32_t ts2Counts;
    };

    constexpr static auto SUBCOMMAND_TIMEOUT = std::chrono::milliseconds(10);
    constexpr static int MAX_RETRIES = 3;

    bq76942(uint8_t address = 0x08);

    // --- DIRECT COMMANDS ---
    control_status_t readControlStatus() { return readDirect<control_status_t>(direct_command::CONTROL_STATUS); }
    protections_a_t readSafetyAlertA() { return readDirect<protections_a_t>(direct_command::SAFETY_ALERT_A); }
    protections_a_t readSafetyStatusA() { return readDirect<protections_a_t>(direct_command::SAFETY_STATUS_A); }
    protections_b_t readSafetyAlertB() { return readDirect<protections_b_t>(direct_command::SAFETY_ALERT_B); }
    protections_b_t readSafetyStatusB() { return readDirect<protections_b_t>(direct_command::SAFETY_STATUS_B); }
    safety_alert_c_t readSafetyAlertC() { return readDirect<safety_alert_c_t>(direct_command::SAFETY_ALERT_C); }
    protections_c_t readSafetyStatusC() { return readDirect<protections_c_t>(direct_command::SAFETY_STATUS_C); }
    permanent_fail_a_t readPermanentFailAlertA() { return readDirect<permanent_fail_a_t>(direct_command::PF_ALERT_A); }
    permanent_fail_a_t readPermanentFailStatusA() { return readDirect<permanent_fail_a_t>(direct_command::PF_STATUS_A); }
    permanent_fail_b_t readPermanentFailAlertB() { return readDirect<permanent_fail_b_t>(direct_command::PF_ALERT_B); }
    permanent_fail_b_t readPermanentFailStatusB() { return readDirect<permanent_fail_b_t>(direct_command::PF_STATUS_B); }
    permanent_fail_alert_c_t readPermanentFailAlertC() { return readDirect<permanent_fail_alert_c_t>(direct_command::PF_ALERT_C); }
    permanent_fail_c_t readPermanentFailStatusC() { return readDirect<permanent_fail_c_t>(direct_command::PF_STATUS_C); }
    permanent_fail_d_t readPermanentFailAlertD() { return readDirect<permanent_fail_d_t>(direct_command::PF_ALERT_D); }
    permanent_fail_d_t readPermanentFailStatusD() { return readDirect<permanent_fail_d_t>(direct_command::PF_STATUS_D); }
    battery_status_t readBatteryStatus() { return readDirect<battery_status_t>(direct_command::BATTERY_STATUS); }
    /// @brief Read the voltage of one of the cells
    /// @return The voltage of the cell in mV
    int16_t readCellVoltage(const uint8_t cell);
    /// @brief Read the voltage at the top of the battery stack
    /// @return The voltage of the stack in mV
    int32_t readStackVoltage();
    int32_t readPackVoltage();
    int32_t readLdVoltage();
    float readCC2Current();
    alarm_status_t readAlarmStatus() { return readDirect<alarm_status_t>(direct_command::ALARM_STATUS); }
    void clearAlarmStatus(const alarm_status_t& alarmStatus) { writeDirect(direct_command::ALARM_STATUS, alarmStatus); }
    alarm_status_t readAlarmRawStatus() { return readDirect<alarm_status_t>(direct_command::ALARM_RAW_STATUS); }
    alarm_status_t readAlarmEnable() { return readDirect<alarm_status_t>(direct_command::ALARM_ENABLE); }
    void writeAlarmEnable(const alarm_status_t& alarmEnable) { writeDirect(direct_command::ALARM_ENABLE, alarmEnable); }
    float readTemperature(const temperature_sensor& sensor) { return rawTempToCelsius(readDirect<int16_t>(static_cast<direct_command>(sensor))); }
    fet_status_t readFetStatus() { return readDirect<fet_status_t>(direct_command::FET_STATUS); }

    // --- COMMAND ONLY SUBCOMMANDS ---
    void enterDeepSleep();
    void exitDeepSleep() { writeSubcommand(cmd_only_subcommand::EXIT_DEEPSLEEP); }
    // WARNING: THIS WILL POWER DOWN THE BMS COMPLETELY!
    // It can only be woken up by either power cycling or pulling down the TS2 pin!
    /// @brief Put the BMS into SHUTDOWN mode
    void shutdown();
    void reset() { writeSubcommand(cmd_only_subcommand::RESET); }
    void predischargeTest() { writeSubcommand(cmd_only_subcommand::PDSG_TEST); }
    void toggleFuse() { writeSubcommand(cmd_only_subcommand::FUSE_TOGGLE); }
    void prechargeTest() { writeSubcommand(cmd_only_subcommand::PCHG_TEST); }
    void chargeTest() { writeSubcommand(cmd_only_subcommand::CHG_TEST); }
    void dischargeTest() { writeSubcommand(cmd_only_subcommand::DSG_TEST); }
    void toggleFetTestMode() { writeSubcommand(cmd_only_subcommand::FET_ENABLE); }
    void togglePermanentFailEnabled() { writeSubcommand(cmd_only_subcommand::PF_ENABLE); }
    void seal() { writeSubcommand(cmd_only_subcommand::SEAL); }
    void resetChargeCounter() { writeSubcommand(cmd_only_subcommand::RESET_PASSQ); }
    void resetPassQ() { resetChargeCounter(); }
    void recoverPrechargeTimeout() { writeSubcommand(cmd_only_subcommand::PTO_RECOVER); }
    void enterConfigUpdateMode();
    void exitConfigUpdateMode() { writeSubcommand(cmd_only_subcommand::EXIT_CFGUPDATE); }
    void disableDischargeFets() { writeSubcommand(cmd_only_subcommand::DSG_PDSG_OFF); }
    void disableChargeFets() { writeSubcommand(cmd_only_subcommand::CHG_PCHG_OFF); }
    void disableAllFets() { writeSubcommand(cmd_only_subcommand::ALL_FETS_OFF); }
    void enableAllFets() { writeSubcommand(cmd_only_subcommand::ALL_FETS_ON); }
    void enableSleep() { writeSubcommand(cmd_only_subcommand::SLEEP_ENABLE); }
    void disableSleep() { writeSubcommand(cmd_only_subcommand::SLEEP_DISABLE); }
    void setSleepEnabled(const bool enabled) { enabled ? enableSleep() : disableSleep(); }
    void recoverDischargeOvercurrentLatch() { writeSubcommand(cmd_only_subcommand::OCDL_RECOVER); }
    void recoverShortCircuitDischargeLatch() { writeSubcommand(cmd_only_subcommand::SCDL_RECOVER); }
    void restartLoadDetect() { writeSubcommand(cmd_only_subcommand::LOAD_DETECT_RESTART); }
    void forceLoadDetectOn() { writeSubcommand(cmd_only_subcommand::LOAD_DETECT_ON); }
    void forceLoadDetectOff() { writeSubcommand(cmd_only_subcommand::LOAD_DETECT_OFF); }
    void setGPO(const gpo_state& state) { writeSubcommand(static_cast<cmd_only_subcommand>(state)); }
    void setGPO(const gpo& pin, const bool state);
    void forcePermanentFail();
    // Comm mode commands NOT IMPLEMENTED as this driver only supports I2C with CRC.
    // However, to support new modes would only require implementing the read/writeDirect functions for the new mode,
    // since subcommands use those under the hood.
    // void swapCommMode();
    // void swapToI2C();
    // void swapToSPI();
    // void swapToHDQ();

    // --- SUBCOMMANDS ---
    uint16_t readDeviceNumber() { return readSubcommand<uint16_t>(subcommand::DEVICE_NUMBER); };
    firmware_version_t readFirmwareVersion();
    uint16_t readHardwareVersion() { return readSubcommand<uint16_t>(subcommand::HW_VERSION); };
    uint16_t readIROMSignature() { return readSubcommand<uint16_t>(subcommand::IROM_SIG); };
    uint16_t readStaticConfigSignature() { return readSubcommand<uint16_t>(subcommand::STATIC_CFG_SIG); };
    // Intended for TI internal use
    // uint16_t readPrevMacWrite() { return readSubcommand<uint16_t>(subcommand::PREV_MAC_WRITE); };
    uint16_t readDROMSignature() { return readSubcommand<uint16_t>(subcommand::DROM_SIG); };
    security_keys_t readSecurityKeys();
    void writeSecurityKeys(const security_keys_t& keys);
    saved_pf_status_t readSavedPFStatus() { return readSubcommand<saved_pf_status_t>(subcommand::SAVED_PF_STATUS); };
    manufacturing_status_t readManufacturingStatus() { return readSubcommand<manufacturing_status_t>(subcommand::MANUFACTURING_STATUS); };
    manufacturer_data_t readManufacturerData();
    void writeManufacturerData(const manufacturer_data_t& data);
    da_status_1_t readDAStatus1() { return readSubcommand<da_status_1_t>(subcommand::DA_STATUS_1); };
    da_status_2_t readDAStatus2() { return readSubcommand<da_status_2_t>(subcommand::DA_STATUS_2); };
    da_status_3_t readDAStatus3() { return readSubcommand<da_status_3_t>(subcommand::DA_STATUS_3); };
    raw_da_status_5_t readRawDAStatus5() { return readSubcommand<raw_da_status_5_t>(subcommand::DA_STATUS_5); };
    da_status_5_t readDAStatus5();
    raw_da_status_6_t readRawDAStatus6() { return readSubcommand<raw_da_status_6_t>(subcommand::DA_STATUS_6); };
    da_status_6_t readDAStatus6();
    da_status_7_t readDAStatus7() { return readSubcommand<da_status_7_t>(subcommand::DA_STATUS_7); };
    voltage_snapshot_t readCuvSnapshot() { return readSubcommand<voltage_snapshot_t>(subcommand::CUV_SNAPSHOT); };
    voltage_snapshot_t readCovSnapshot() { return readSubcommand<voltage_snapshot_t>(subcommand::COV_SNAPSHOT); };
    cb_active_cells_t readCBActiveCells() { return readSubcommand<cb_active_cells_t>(subcommand::CB_ACTIVE_CELLS); };
    void writeCBActiveCells(const cb_active_cells_t& cells) { writeSubcommand(subcommand::CB_ACTIVE_CELLS, cells); }
    std::chrono::seconds readCellBalancingTime() { return std::chrono::seconds(readSubcommand<uint16_t>(subcommand::CB_STATUS_1)); };
    cb_status_2_t readCBStatus2() { return readSubcommand<cb_status_2_t>(subcommand::CB_STATUS_2); };
    cb_status_3_t readCBStatus3() { return readSubcommand<cb_status_3_t>(subcommand::CB_STATUS_3); };
    void writeFetControl(const fet_control_t& control) { writeSubcommand(subcommand::FET_CONTROL, control); }
    void writeRegulatorControl(const regulator_control_t& control) { writeSubcommand(subcommand::REG12_CONTROL, control); }
    otp_write_result_t readOtpWriteCheckResult() { return readSubcommand<otp_write_result_t>(subcommand::OTP_WR_CHECK); }
    otp_write_result_t readOtpWriteResult() { return readSubcommand<otp_write_result_t>(subcommand::OTP_WRITE); }
    cal1_t readCal1() { return readSubcommand<cal1_t>(subcommand::READ_CAL1); }
    uint16_t calibrateCuvThreshold() { return readSubcommand<uint16_t>(subcommand::CAL_CUV); }
    uint16_t calibrateCovThreshold() { return readSubcommand<uint16_t>(subcommand::CAL_COV); }

    // --- DATA REGISTERS ---
    class Calibration final
    {
    public:
        // Calibration:Voltage
        class Voltage
        {
        public:
            int16_t readCellGain(const uint8_t& cell) const;
            void writeCellGain(const uint8_t& cell, const int16_t& gain) const;
            uint16_t getPackGain() const { return _parent.readSubcommand<uint16_t>(data_register::PACK_GAIN); }
            void setPackGain(const uint16_t& gain) const { _parent.writeSubcommand(data_register::PACK_GAIN, gain); }
            uint16_t getStackGain() const { return _parent.readSubcommand<uint16_t>(data_register::TOS_GAIN); }
            void setStackGain(const uint16_t& gain) const { _parent.writeSubcommand(data_register::TOS_GAIN, gain); }
            uint16_t getLdGain() const { return _parent.readSubcommand<uint16_t>(data_register::LD_GAIN); }
            void setLdGain(const uint16_t& gain) const { _parent.writeSubcommand(data_register::LD_GAIN, gain); }
            int16_t getAdcGain() const { return _parent.readSubcommand<int16_t>(data_register::ADC_GAIN); }
            void setAdcGain(const int16_t& gain) const { _parent.writeSubcommand(data_register::ADC_GAIN, gain); }

        protected:
            Voltage(bq76942& parent) : _parent(parent) {}

            Voltage(Voltage&) = delete;
            Voltage& operator=(Voltage&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        // Calibration: Current
        class Current final
        {
        public:
            // TODO: VALIDATE THAT ESP32 USES IEEE 754 FLOATS
            float readCCGain() const { return _parent.readSubcommand<float>(data_register::CC_GAIN); }
            void writeCCGain(const float& gain) const { _parent.writeSubcommand(data_register::CC_GAIN, gain); }
            float readCapacityGain() const { return _parent.readSubcommand<float>(data_register::CAPACITY_GAIN); }
            void writeCapacityGain(const float& gain) const { _parent.writeSubcommand(data_register::CAPACITY_GAIN, gain); }

        protected:
            Current(bq76942& parent) : _parent(parent) {}

            Current(Current&) = delete;
            Current& operator=(Current&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        // Calibration:Vcell Offset
        int16_t readVcellOffset() const { return _parent.readSubcommand<int16_t>(data_register::VCELL_OFFSET); }
        void writeVcellOffset(const int16_t& offset) const { _parent.writeSubcommand(data_register::VCELL_OFFSET, offset); }

        // Calibration:V Divider Offset
        int16_t readVDividerOffset() const { return _parent.readSubcommand<int16_t>(data_register::VDIV_OFFSET); }
        void writeVDividerOffset(const int16_t& offset) const { _parent.writeSubcommand(data_register::VDIV_OFFSET, offset); }

        // Calibration:Current Offset
        class CurrentOffset final
        {
        public:
            uint16_t readCoulombCounterOffsetSamples() const { return _parent.readSubcommand<uint16_t>(data_register::COULOMB_COUNTER_OFFSET_SAMPLES); }
            void writeCoulombCounterOffsetSamples(const uint16_t& samples) const { _parent.writeSubcommand(data_register::COULOMB_COUNTER_OFFSET_SAMPLES, samples); }
            int16_t readBoardOffsetCurrent() const { return _parent.readSubcommand<int16_t>(data_register::BOARD_OFFSET); }
            void writeBoardOffsetCurrent(const int16_t& offset) const { _parent.writeSubcommand(data_register::BOARD_OFFSET, offset); }

        protected:
            CurrentOffset(bq76942& parent) : _parent(parent) {}

            CurrentOffset(CurrentOffset&) = delete;
            CurrentOffset& operator=(CurrentOffset&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        // Calibration:Temperature
        class Temperature final
        {
            int8_t readInternalOffset() const { return _parent.readSubcommand<int8_t>(data_register::INTERNAL_TEMP_OFFSET); }
            void writeInternalOffset(const int8_t& offset) const { _parent.writeSubcommand(data_register::INTERNAL_TEMP_OFFSET, offset); }
            int8_t readCfetoffOffset() const { return _parent.readSubcommand<int8_t>(data_register::CFETOFF_TEMP_OFFSET); }
            void writeCfetoffOffset(const int8_t& offset) const { _parent.writeSubcommand(data_register::CFETOFF_TEMP_OFFSET, offset); }
            int8_t readDfetoffOffset() const { return _parent.readSubcommand<int8_t>(data_register::DFETOFF_TEMP_OFFSET); }
            void writeDfetoffOffset(const int8_t& offset) const { _parent.writeSubcommand(data_register::DFETOFF_TEMP_OFFSET, offset); }
            int8_t readAlertOffset() const { return _parent.readSubcommand<int8_t>(data_register::ALERT_TEMP_OFFSET); }
            void writeAlertOffset(const int8_t& offset) const { _parent.writeSubcommand(data_register::ALERT_TEMP_OFFSET, offset); }
            int8_t readTs1Offset() const { return _parent.readSubcommand<int8_t>(data_register::TS1_TEMP_OFFSET); }
            void writeTs1Offset(const int8_t& offset) const { _parent.writeSubcommand(data_register::TS1_TEMP_OFFSET, offset); }
            int8_t readTs2Offset() const { return _parent.readSubcommand<int8_t>(data_register::TS2_TEMP_OFFSET); }
            void writeTs2Offset(const int8_t& offset) const { _parent.writeSubcommand(data_register::TS2_TEMP_OFFSET, offset); }
            int8_t readTs3Offset() const { return _parent.readSubcommand<int8_t>(data_register::TS3_TEMP_OFFSET); }
            void writeTs3Offset(const int8_t& offset) const { _parent.writeSubcommand(data_register::TS3_TEMP_OFFSET, offset); }
            int8_t readHdqOffset() const { return _parent.readSubcommand<int8_t>(data_register::HDQ_TEMP_OFFSET); }
            void writeHdqOffset(const int8_t& offset) const { _parent.writeSubcommand(data_register::HDQ_TEMP_OFFSET, offset); }
            int8_t readDchgOffset() const { return _parent.readSubcommand<int8_t>(data_register::DCHG_TEMP_OFFSET); }
            void writeDchgOffset(const int8_t& offset) const { _parent.writeSubcommand(data_register::DCHG_TEMP_OFFSET, offset); }
            int8_t readDdsgOffset() const { return _parent.readSubcommand<int8_t>(data_register::DDSG_TEMP_OFFSET); }
            void writeDdsgOffset(const int8_t& offset) const { _parent.writeSubcommand(data_register::DDSG_TEMP_OFFSET, offset); }

        protected:
            Temperature(bq76942& parent) : _parent(parent) {}

            Temperature(Temperature&) = delete;
            Temperature& operator=(Temperature&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        // Calibration:Internal Temp Model
        class InternalTempModel final
        {
        public:
            int16_t readGain() const { return _parent.readSubcommand<int16_t>(data_register::INT_GAIN); }
            void writeGain(const int16_t& gain) const { _parent.writeSubcommand(data_register::INT_GAIN, gain); }
            int16_t readBaseOffset() const { return _parent.readSubcommand<int16_t>(data_register::INT_BASE_OFFSET); }
            void writeBaseOffset(const int16_t& offset) const { _parent.writeSubcommand(data_register::INT_BASE_OFFSET, offset); }
            int16_t readMaximumAD() const { return _parent.readSubcommand<int16_t>(data_register::INT_MAXIMUM_AD); }
            void writeMaximumAD(const int16_t& ad) const { _parent.writeSubcommand(data_register::INT_MAXIMUM_AD, ad); }
            int16_t readMaximumTemp() const { return _parent.readSubcommand<int16_t>(data_register::INT_MAXIMUM_TEMP); }
            void writeMaximumTemp(const int16_t& temp) const { _parent.writeSubcommand(data_register::INT_MAXIMUM_TEMP, temp); }

        protected:
            InternalTempModel(bq76942& parent) : _parent(parent) {}

            InternalTempModel(InternalTempModel&) = delete;
            InternalTempModel& operator=(InternalTempModel&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        // Calibration:18K Temperature Model
        class T18KModel final
        {
        public:
            int16_t readCoeffA1() const { return _parent.readSubcommand<int16_t>(data_register::T18K_COEFF_A1); }
            void writeCoeffA1(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T18K_COEFF_A1, coeff); }
            int16_t readCoeffA2() const { return _parent.readSubcommand<int16_t>(data_register::T18K_COEFF_A2); }
            void writeCoeffA2(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T18K_COEFF_A2, coeff); }
            int16_t readCoeffA3() const { return _parent.readSubcommand<int16_t>(data_register::T18K_COEFF_A3); }
            void writeCoeffA3(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T18K_COEFF_A3, coeff); }
            int16_t readCoeffA4() const { return _parent.readSubcommand<int16_t>(data_register::T18K_COEFF_A4); }
            void writeCoeffA4(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T18K_COEFF_A4, coeff); }
            int16_t readCoeffA5() const { return _parent.readSubcommand<int16_t>(data_register::T18K_COEFF_A5); }
            void writeCoeffA5(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T18K_COEFF_A5, coeff); }
            int16_t readCoeffB1() const { return _parent.readSubcommand<int16_t>(data_register::T18K_COEFF_B1); }
            void writeCoeffB1(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T18K_COEFF_B1, coeff); }
            int16_t readCoeffB2() const { return _parent.readSubcommand<int16_t>(data_register::T18K_COEFF_B2); }
            void writeCoeffB2(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T18K_COEFF_B2, coeff); }
            int16_t readCoeffB3() const { return _parent.readSubcommand<int16_t>(data_register::T18K_COEFF_B3); }
            void writeCoeffB3(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T18K_COEFF_B3, coeff); }
            int16_t readCoeffB4() const { return _parent.readSubcommand<int16_t>(data_register::T18K_COEFF_B4); }
            void writeCoeffB4(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T18K_COEFF_B4, coeff); }
            int16_t readAdc0() const { return _parent.readSubcommand<int16_t>(data_register::T18K_ADC0); }
            void writeAdc0(const int16_t& adc) const { _parent.writeSubcommand(data_register::T18K_ADC0, adc); }

        protected:
            T18KModel(bq76942& parent) : _parent(parent) {}

            T18KModel(T18KModel&) = delete;
            T18KModel& operator=(T18KModel&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        // Calibration:180K Temperature Model
        class T180KModel final
        {
        public:
            int16_t readCoeffA1() const { return _parent.readSubcommand<int16_t>(data_register::T180K_COEFF_A1); }
            void writeCoeffA1(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T180K_COEFF_A1, coeff); }
            int16_t readCoeffA2() const { return _parent.readSubcommand<int16_t>(data_register::T180K_COEFF_A2); }
            void writeCoeffA2(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T180K_COEFF_A2, coeff); }
            int16_t readCoeffA3() const { return _parent.readSubcommand<int16_t>(data_register::T180K_COEFF_A3); }
            void writeCoeffA3(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T180K_COEFF_A3, coeff); }
            int16_t readCoeffA4() const { return _parent.readSubcommand<int16_t>(data_register::T180K_COEFF_A4); }
            void writeCoeffA4(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T180K_COEFF_A4, coeff); }
            int16_t readCoeffA5() const { return _parent.readSubcommand<int16_t>(data_register::T180K_COEFF_A5); }
            void writeCoeffA5(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T180K_COEFF_A5, coeff); }
            int16_t readCoeffB1() const { return _parent.readSubcommand<int16_t>(data_register::T180K_COEFF_B1); }
            void writeCoeffB1(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T180K_COEFF_B1, coeff); }
            int16_t readCoeffB2() const { return _parent.readSubcommand<int16_t>(data_register::T180K_COEFF_B2); }
            void writeCoeffB2(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T180K_COEFF_B2, coeff); }
            int16_t readCoeffB3() const { return _parent.readSubcommand<int16_t>(data_register::T180K_COEFF_B3); }
            void writeCoeffB3(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T180K_COEFF_B3, coeff); }
            int16_t readCoeffB4() const { return _parent.readSubcommand<int16_t>(data_register::T180K_COEFF_B4); }
            void writeCoeffB4(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T180K_COEFF_B4, coeff); }
            int16_t readAdc0() const { return _parent.readSubcommand<int16_t>(data_register::T180K_ADC0); }
            void writeAdc0(const int16_t& adc) const { _parent.writeSubcommand(data_register::T180K_ADC0, adc); }

        protected:
            T180KModel(bq76942& parent) : _parent(parent) {}

            T180KModel(T180KModel&) = delete;
            T180KModel& operator=(T180KModel&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        // Calibration:Custom Temperature Model
        class CustomTemperatureModel final
        {
        public:
            int16_t readCoeffA1() const { return _parent.readSubcommand<int16_t>(data_register::CUSTOM_COEFF_A1); }
            void writeCoeffA1(const int16_t& coeff) const { _parent.writeSubcommand(data_register::CUSTOM_COEFF_A1, coeff); }
            int16_t readCoeffA2() const { return _parent.readSubcommand<int16_t>(data_register::CUSTOM_COEFF_A2); }
            void writeCoeffA2(const int16_t& coeff) const { _parent.writeSubcommand(data_register::CUSTOM_COEFF_A2, coeff); }
            int16_t readCoeffA3() const { return _parent.readSubcommand<int16_t>(data_register::CUSTOM_COEFF_A3); }
            void writeCoeffA3(const int16_t& coeff) const { _parent.writeSubcommand(data_register::CUSTOM_COEFF_A3, coeff); }
            int16_t readCoeffA4() const { return _parent.readSubcommand<int16_t>(data_register::CUSTOM_COEFF_A4); }
            void writeCoeffA4(const int16_t& coeff) const { _parent.writeSubcommand(data_register::CUSTOM_COEFF_A4, coeff); }
            int16_t readCoeffA5() const { return _parent.readSubcommand<int16_t>(data_register::CUSTOM_COEFF_A5); }
            void writeCoeffA5(const int16_t& coeff) const { _parent.writeSubcommand(data_register::CUSTOM_COEFF_A5, coeff); }
            int16_t readCoeffB1() const { return _parent.readSubcommand<int16_t>(data_register::CUSTOM_COEFF_B1); }
            void writeCoeffB1(const int16_t& coeff) const { _parent.writeSubcommand(data_register::CUSTOM_COEFF_B1, coeff); }
            int16_t readCoeffB2() const { return _parent.readSubcommand<int16_t>(data_register::CUSTOM_COEFF_B2); }
            void writeCoeffB2(const int16_t& coeff) const { _parent.writeSubcommand(data_register::CUSTOM_COEFF_B2, coeff); }
            int16_t readCoeffB3() const { return _parent.readSubcommand<int16_t>(data_register::CUSTOM_COEFF_B3); }
            void writeCoeffB3(const int16_t& coeff) const { _parent.writeSubcommand(data_register::CUSTOM_COEFF_B3, coeff); }
            int16_t readCoeffB4() const { return _parent.readSubcommand<int16_t>(data_register::CUSTOM_COEFF_B4); }
            void writeCoeffB4(const int16_t& coeff) const { _parent.writeSubcommand(data_register::CUSTOM_COEFF_B4, coeff); }
            int16_t readRc0() const { return _parent.readSubcommand<int16_t>(data_register::CUSTOM_RC0); }
            void writeRc0(const int16_t& rc) const { _parent.writeSubcommand(data_register::CUSTOM_RC0, rc); }
            int16_t readAdc0() const { return _parent.readSubcommand<int16_t>(data_register::CUSTOM_ADC0); }
            void writeAdc0(const int16_t& adc) const { _parent.writeSubcommand(data_register::CUSTOM_ADC0, adc); }

        protected:
            CustomTemperatureModel(bq76942& parent) : _parent(parent) {}

            CustomTemperatureModel(const CustomTemperatureModel&) = delete;
            CustomTemperatureModel& operator=(const CustomTemperatureModel&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        // Calibration:Current Deadband
        int16_t readCoulombCounterDeadband() const { return _parent.readSubcommand<int16_t>(data_register::COULOMB_COUNTER_DEADBAND); }
        void writeCoulombCounterDeadband(const int16_t& deadband) const { _parent.writeSubcommand(data_register::COULOMB_COUNTER_DEADBAND, deadband); }

        // Calibration:CUV
        uint16_t readCuvThresholdOverride() const { return _parent.readSubcommand<uint16_t>(data_register::CUV_THRESHOLD_OVERRIDE); }
        void writeCuvThresholdOverride(const uint16_t& threshold) const { _parent.writeSubcommand(data_register::CUV_THRESHOLD_OVERRIDE, threshold); }

        // Calibration:COV
        uint16_t readCovThresholdOverride() const { return _parent.readSubcommand<uint16_t>(data_register::COV_THRESHOLD_OVERRIDE); }
        void writeCovThresholdOverride(const uint16_t& threshold) const { _parent.writeSubcommand(data_register::COV_THRESHOLD_OVERRIDE, threshold); }

        const Voltage voltage{_parent};
        const Current current{_parent};
        const CurrentOffset currentOffset{_parent};
        const Temperature temperature{_parent};
        const InternalTempModel internalTempModel{_parent};
        const T18KModel t18KModel{_parent};
        const T180KModel t180KModel{_parent};
        const CustomTemperatureModel customTemperatureModel{_parent};

    protected:
        Calibration(bq76942& parent) : _parent(parent) {}

        Calibration(Calibration&) = delete;
        Calibration& operator=(Calibration&) = delete;

    private:
        friend class bq76942;
        bq76942& _parent;
    };

    class Settings final
    {
    public:
        // Settings:Fuse
        class Fuse final
        {
        public:
            int16_t readMinBlowVoltage() const { return _parent.readSubcommand<int16_t>(data_register::MIN_BLOW_FUSE_VOLTAGE); }
            void writeMinBlowVoltage(const int16_t& voltage) const { _parent.writeSubcommand(data_register::MIN_BLOW_FUSE_VOLTAGE, voltage); }
            std::chrono::seconds readBlowTimeout() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::FUSE_BLOW_TIMEOUT)); }
            void writeBlowTimeout(const std::chrono::seconds& timeout) const { _parent.writeSubcommand<uint8_t>(data_register::FUSE_BLOW_TIMEOUT, timeout.count()); }

        protected:
            Fuse(bq76942& parent) : _parent(parent) {}

            Fuse(Fuse&) = delete;
            Fuse& operator=(Fuse&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        // Settings:Configuration
        class Configuration final
        {
        public:
            power_config_t readPowerConfig() const { return _parent.readSubcommand<power_config_t>(data_register::POWER_CONFIG); }
            void writePowerConfig(const power_config_t& config) const { _parent.writeSubcommand(data_register::POWER_CONFIG, config); }
            regulator_control_t readReg12Config() const { return _parent.readSubcommand<regulator_control_t>(data_register::REG12_CONFIG); }
            void writeReg12Config(const regulator_control_t& config) const { _parent.writeSubcommand(data_register::REG12_CONFIG, config); }
            reg0_config_t readReg0Config() const { return _parent.readSubcommand<reg0_config_t>(data_register::REG0_CONFIG); }
            void writeReg0Config(const reg0_config_t& config) const { _parent.writeSubcommand(data_register::REG0_CONFIG, config); }
            hwd_regulator_options_t readHwdRegulatorOptions() const { return _parent.readSubcommand<hwd_regulator_options_t>(data_register::HWD_REGULATOR_OPTIONS); }
            void writeHwdRegulatorOptions(const hwd_regulator_options_t& options) const { _parent.writeSubcommand(data_register::HWD_REGULATOR_OPTIONS, options); }
            comm_type readCommType() const { return static_cast<comm_type>(_parent.readSubcommand<uint8_t>(data_register::COMM_TYPE)); }
            // note: This will only apply on reset OR SWAP_COMM_MODE
            void writeCommType(const comm_type& type) const { _parent.writeSubcommand(data_register::COMM_TYPE, static_cast<uint8_t>(type)); }
            uint8_t readI2CAddress() const { return _parent.readSubcommand<uint8_t>(data_register::I2C_ADDRESS); }
            // note: This will only apply on reset OR SWAP_COMM_MODE
            void writeI2CAddress(const uint8_t& address) const { _parent.writeSubcommand(data_register::I2C_ADDRESS, address); }
            spi_configuration_t readSpiConfiguration() const { return _parent.readSubcommand<spi_configuration_t>(data_register::SPI_CONFIGURATION); }
            void writeSpiConfiguration(const spi_configuration_t& config) const { _parent.writeSubcommand(data_register::SPI_CONFIGURATION, config); }
            std::chrono::seconds readCommIdleTime() const { return std::chrono::seconds(_parent.readSubcommand<uint16_t>(data_register::COMM_IDLE_TIME)); }
            void writeCommIdleTime(const std::chrono::seconds& time) const { _parent.writeSubcommand(data_register::COMM_IDLE_TIME, time.count()); }
            cfetoff_pin_configuration_t readCfetoffPinConfig() const { return _parent.readSubcommand<cfetoff_pin_configuration_t>(data_register::CFETOFF_PIN_CONFIG); }
            void writeCfetoffPinConfig(const cfetoff_pin_configuration_t& config) const { _parent.writeSubcommand(data_register::CFETOFF_PIN_CONFIG, config); }
            dfetoff_pin_configuration_t readDfetoffPinConfig() const { return _parent.readSubcommand<dfetoff_pin_configuration_t>(data_register::DFETOFF_PIN_CONFIG); }
            void writeDfetoffPinConfig(const dfetoff_pin_configuration_t& config) const { _parent.writeSubcommand(data_register::DFETOFF_PIN_CONFIG, config); }
            alert_pin_configuration_t readAlertPinConfig() const { return _parent.readSubcommand<alert_pin_configuration_t>(data_register::ALERT_PIN_CONFIG); }
            void writeAlertPinConfig(const alert_pin_configuration_t& config) const { _parent.writeSubcommand(data_register::ALERT_PIN_CONFIG, config); }
            ts_pin_configuration_t readTs1PinConfig() const { return _parent.readSubcommand<ts_pin_configuration_t>(data_register::TS1_CONFIG); }
            void writeTs1PinConfig(const ts_pin_configuration_t& config) const { _parent.writeSubcommand(data_register::TS1_CONFIG, config); }
            ts_pin_configuration_t readTs2PinConfig() const { return _parent.readSubcommand<ts_pin_configuration_t>(data_register::TS2_CONFIG); }
            void writeTs2PinConfig(const ts_pin_configuration_t& config) const { _parent.writeSubcommand(data_register::TS2_CONFIG, config); }
            ts_pin_configuration_t readTs3PinConfig() const { return _parent.readSubcommand<ts_pin_configuration_t>(data_register::TS3_CONFIG); }
            void writeTs3PinConfig(const ts_pin_configuration_t& config) const { _parent.writeSubcommand(data_register::TS3_CONFIG, config); }
            hdq_pin_configuration_t readHdqPinConfig() const { return _parent.readSubcommand<hdq_pin_configuration_t>(data_register::HDQ_PIN_CONFIG); }
            void writeHdqPinConfig(const hdq_pin_configuration_t& config) const { _parent.writeSubcommand(data_register::HDQ_PIN_CONFIG, config); }
            dchg_pin_configuration_t readDchgPinConfig() const { return _parent.readSubcommand<dchg_pin_configuration_t>(data_register::DCHG_PIN_CONFIG); }
            void writeDchgPinConfig(const dchg_pin_configuration_t& config) const { _parent.writeSubcommand(data_register::DCHG_PIN_CONFIG, config); }
            ddsg_pin_configuration_t readDdsgPinConfig() const { return _parent.readSubcommand<ddsg_pin_configuration_t>(data_register::DDSG_PIN_CONFIG); }
            void writeDdsgPinConfig(const ddsg_pin_configuration_t& config) const { _parent.writeSubcommand(data_register::DDSG_PIN_CONFIG, config); }
            da_configuration_t readDAConfiguration() const { return _parent.readSubcommand<da_configuration_t>(data_register::DA_CONFIGURATION); }
            void writeDAConfiguration(const da_configuration_t& config) const { _parent.writeSubcommand(data_register::DA_CONFIGURATION, config); }
            vcell_mode_t readVcellMode() const { return _parent.readSubcommand<vcell_mode_t>(data_register::VCELL_MODE); }
            void writeVcellMode(const vcell_mode_t& mode) const { _parent.writeSubcommand(data_register::VCELL_MODE, mode); }
            uint8_t readCC3Samples() const { return _parent.readSubcommand<uint8_t>(data_register::CC3_SAMPLES); }
            void writeCC3Samples(const uint8_t& samples) const { _parent.writeSubcommand(data_register::CC3_SAMPLES, samples); }

        protected:
            Configuration(bq76942& parent) : _parent(parent) {}
            Configuration(Configuration&) = delete;
            Configuration& operator=(Configuration&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        // Settings:Protection
        class Protection final
        {
        public:
            protection_configuration_t readConfig() const { return _parent.readSubcommand<protection_configuration_t>(data_register::PROTECTION_CONFIGURATION); }
            void writeConfig(const protection_configuration_t& config) const { _parent.writeSubcommand(data_register::PROTECTION_CONFIGURATION, config); }
            protections_a_t readEnabledA() const { return _parent.readSubcommand<protections_a_t>(data_register::ENABLED_PROTECTIONS_A); }
            void writeEnabledA(const protections_a_t& protections) const { _parent.writeSubcommand(data_register::ENABLED_PROTECTIONS_A, protections); }
            protections_b_t readEnabledB() const { return _parent.readSubcommand<protections_b_t>(data_register::ENABLED_PROTECTIONS_B); }
            void writeEnabledB(const protections_b_t& protections) const { _parent.writeSubcommand(data_register::ENABLED_PROTECTIONS_B, protections); }
            protections_c_t readEnabledC() const { return _parent.readSubcommand<protections_c_t>(data_register::ENABLED_PROTECTIONS_C); }
            void writeEnabledC(const protections_c_t& protections) const { _parent.writeSubcommand(data_register::ENABLED_PROTECTIONS_C, protections); }
            chg_fet_protections_a_t readChgFetA() const { return _parent.readSubcommand<chg_fet_protections_a_t>(data_register::CHG_FET_PROTECTIONS_A); }
            void writeChgFetA(const chg_fet_protections_a_t& protections) const { _parent.writeSubcommand(data_register::CHG_FET_PROTECTIONS_A, protections); }
            chg_fet_protections_b_t readChgFetB() const { return _parent.readSubcommand<chg_fet_protections_b_t>(data_register::CHG_FET_PROTECTIONS_B); }
            void writeChgFetB(const chg_fet_protections_b_t& protections) const { _parent.writeSubcommand(data_register::CHG_FET_PROTECTIONS_B, protections); }
            chg_fet_protections_c_t readChgFetC() const { return _parent.readSubcommand<chg_fet_protections_c_t>(data_register::CHG_FET_PROTECTIONS_C); }
            void writeChgFetC(const chg_fet_protections_c_t& protections) const { _parent.writeSubcommand(data_register::CHG_FET_PROTECTIONS_C, protections); }
            dsg_fet_protections_a_t readDsgFetA() const { return _parent.readSubcommand<dsg_fet_protections_a_t>(data_register::DSG_FET_PROTECTIONS_A); }
            void writeDsgFetA(const dsg_fet_protections_a_t& protections) const { _parent.writeSubcommand(data_register::DSG_FET_PROTECTIONS_A, protections); }
            dsg_fet_protections_b_t readDsgFetB() const { return _parent.readSubcommand<dsg_fet_protections_b_t>(data_register::DSG_FET_PROTECTIONS_B); }
            void writeDsgFetB(const dsg_fet_protections_b_t& protections) const { _parent.writeSubcommand(data_register::DSG_FET_PROTECTIONS_B, protections); }
            dsg_fet_protections_c_t readDsgFetC() const { return _parent.readSubcommand<dsg_fet_protections_c_t>(data_register::DSG_FET_PROTECTIONS_C); }
            void writeDsgFetC(const dsg_fet_protections_c_t& protections) const { _parent.writeSubcommand(data_register::DSG_FET_PROTECTIONS_C, protections); }
            int16_t readBodyDiodeThreshold() const { return _parent.readSubcommand<int16_t>(data_register::BODY_DIODE_THRESHOLD); }
            void writeBodyDiodeThreshold(const int16_t& threshold) const { _parent.writeSubcommandClamped<int16_t>(data_register::BODY_DIODE_THRESHOLD, threshold, 0, std::numeric_limits<int16_t>::max()); }

        protected:
            Protection(bq76942& parent) : _parent(parent) {}
            Protection(Protection&) = delete;
            Protection& operator=(Protection&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        // Settings:Alarm
        class Alarm final
        {
        public:
            alarm_status_t readDefaultMask() const { return _parent.readSubcommand<alarm_status_t>(data_register::DEFAULT_ALARM_MASK); }
            void writeDefaultMask(const alarm_status_t& mask) const { _parent.writeSubcommand(data_register::DEFAULT_ALARM_MASK, mask); }
            protections_a_t readSafetyAlertMaskA() const { return _parent.readSubcommand<protections_a_t>(data_register::SF_ALERT_MASK_A); }
            void writeSafetyAlertMaskA(const protections_a_t& protections) const { _parent.writeSubcommand(data_register::SF_ALERT_MASK_A, protections); }
            protections_b_t readSafetyAlertMaskB() const { return _parent.readSubcommand<protections_b_t>(data_register::SF_ALERT_MASK_B); }
            void writeSafetyAlertMaskB(const protections_b_t& protections) const { _parent.writeSubcommand(data_register::SF_ALERT_MASK_B, protections); }
            protections_c_t readSafetyAlertMaskC() const { return _parent.readSubcommand<protections_c_t>(data_register::SF_ALERT_MASK_C); }
            void writeSafetyAlertMaskC(const protections_c_t& protections) const { _parent.writeSubcommand(data_register::SF_ALERT_MASK_C, protections); }

        protected:
            Alarm(bq76942& parent) : _parent(parent) {}
            Alarm(Alarm&) = delete;
            Alarm& operator=(Alarm&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        // Settings:Permanent Failure
        class PermanentFailure final
        {
        public:
            permanent_fail_a_t readEnabledA() const { return _parent.readSubcommand<permanent_fail_a_t>(data_register::ENABLED_PF_A); }
            void writeEnabledA(const permanent_fail_a_t& failures) const { _parent.writeSubcommand(data_register::ENABLED_PF_A, failures); }
            permanent_fail_b_t readEnabledB() const { return _parent.readSubcommand<permanent_fail_b_t>(data_register::ENABLED_PF_B); }
            void writeEnabledB(const permanent_fail_b_t& failures) const { _parent.writeSubcommand(data_register::ENABLED_PF_B, failures); }
            permanent_fail_c_t readEnabledC() const { return _parent.readSubcommand<permanent_fail_c_t>(data_register::ENABLED_PF_C); }
            void writeEnabledC(const permanent_fail_c_t& failures) const { _parent.writeSubcommand(data_register::ENABLED_PF_C, failures); }
            permanent_fail_d_t readEnabledD() const { return _parent.readSubcommand<permanent_fail_d_t>(data_register::ENABLED_PF_D); }
            void writeEnabledD(const permanent_fail_d_t& failures) const { _parent.writeSubcommand(data_register::ENABLED_PF_D, failures); }

        protected:
            PermanentFailure(bq76942& parent) : _parent(parent) {}
            PermanentFailure(PermanentFailure&) = delete;
            PermanentFailure& operator=(PermanentFailure&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        // Settings:FET
        class Fet final
        {
        public:
            fet_options_t readOptions() const { return _parent.readSubcommand<fet_options_t>(data_register::FET_OPTIONS); }
            void writeOptions(const fet_options_t& options) const { _parent.writeSubcommand(data_register::FET_OPTIONS, options); }
            fet_charge_pump_control_t readChargePumpControl() const { return _parent.readSubcommand<fet_charge_pump_control_t>(data_register::CHG_PUMP_CONTROL); }
            void writeChargePumpControl(const fet_charge_pump_control_t& control) const { _parent.writeSubcommand(data_register::CHG_PUMP_CONTROL, control); }
            int16_t readPrechargeStartVoltage() const { return _parent.readSubcommand<int16_t>(data_register::PRECHARGE_START_VOLTAGE); }
            void writePrechargeStartVoltage(const int16_t& voltage) const { _parent.writeSubcommandClamped<int16_t>(data_register::PRECHARGE_START_VOLTAGE, voltage, 0, std::numeric_limits<int16_t>::max()); }
            int16_t readPrechargeStopVoltage() const { return _parent.readSubcommand<int16_t>(data_register::PRECHARGE_STOP_VOLTAGE); }
            void writePrechargeStopVoltage(const int16_t& voltage) const { _parent.writeSubcommandClamped<int16_t>(data_register::PRECHARGE_STOP_VOLTAGE, voltage, 0, std::numeric_limits<int16_t>::max()); }
            std::chrono::milliseconds readPredischargeTimeout() const { return std::chrono::milliseconds(_parent.readSubcommand<uint16_t>(data_register::PREDISCHARGE_TIMEOUT) * 10); }
            void writePredischargeTimeout(const std::chrono::milliseconds& timeout) const
            {
                uint8_t roundedTimeout = static_cast<uint8_t>(std::round(timeout.count() / 10.0));
                _parent.writeSubcommand(data_register::PREDISCHARGE_TIMEOUT, roundedTimeout);
            }
            uint16_t readPredischargeStopDelta() const { return static_cast<uint16_t>(_parent.readSubcommand<uint8_t>(data_register::PREDISCHARGE_STOP_DELTA)) * 10; }
            void writePredischargeStopDelta(uint16_t voltage) const { _parent.writeSubcommand(data_register::PREDISCHARGE_STOP_DELTA, static_cast<uint8_t>(voltage / 10)); }

        protected:
            Fet(bq76942& parent) : _parent(parent) {}
            Fet(Fet&) = delete;
            Fet& operator=(Fet&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        // Settings:Current Thresholds
        uint32_t readDsgCurrentThreshold() const;
        void writeDsgCurrentThreshold(const uint32_t& threshold) const;
        uint32_t readChgCurrentThreshold() const;
        void writeChgCurrentThreshold(const uint32_t& threshold) const;

        // Settings:Cell Open-Wire
        std::chrono::seconds readCellOpenWireCheckTime() const { return std::chrono::seconds(_parent.readSubcommand<uint16_t>(data_register::CHECK_TIME)); }
        void writeCellOpenWireCheckTime(const std::chrono::seconds& time) const { _parent.writeSubcommand(data_register::CHECK_TIME, static_cast<uint8_t>(time.count())); }

        // Settings:Interconnect Resistances
        int16_t readCellInterconnectResistance(const uint8_t& cell);
        void writeCellInterconnectResistance(const uint8_t& cell, const int16_t& resistance);

        // Settings:Manufacturing
        manufacturing_status_init_t readManufacturingStatusInit() const { return _parent.readSubcommand<manufacturing_status_init_t>(data_register::MFG_STATUS_INIT); }
        void writeManufacturingStatusInit(const manufacturing_status_init_t& status) const { _parent.writeSubcommand(data_register::MFG_STATUS_INIT, status); }

        // Settings:Cell Balancing Config
        class CellBalancing final
        {
        public:
            balancing_configuration_t readConfig() const { return _parent.readSubcommand<balancing_configuration_t>(data_register::BALANCING_CONFIGURATION); }
            void writeConfig(const balancing_configuration_t& config) const { _parent.writeSubcommand(data_register::BALANCING_CONFIGURATION, config); }
            int8_t readMinCellTemp() const { return _parent.readSubcommand<int8_t>(data_register::MIN_CELL_TEMP); }
            void writeMinCellTemp(const int8_t& temp) const { _parent.writeSubcommand(data_register::MIN_CELL_TEMP, temp); }
            int8_t readMaxInternalTemp() const { return _parent.readSubcommand<int8_t>(data_register::MAX_INTERNAL_TEMP); }
            void writeMaxInternalTemp(const int8_t& temp) const { _parent.writeSubcommand(data_register::MAX_INTERNAL_TEMP, temp); }
            std::chrono::seconds readInterval() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::CELL_BALANCE_INTERVAL)); }
            void writeInterval(const std::chrono::seconds& interval) const { _parent.writeSubcommandClamped<uint8_t>(data_register::CELL_BALANCE_INTERVAL, static_cast<uint8_t>(interval.count()), 1, 255); }
            uint8_t readMaxCells() const { return _parent.readSubcommand<uint8_t>(data_register::CELL_BALANCE_MAX_CELLS); }
            void writeMaxCells(const uint8_t& cells) const { _parent.writeSubcommandClamped<uint8_t>(data_register::CELL_BALANCE_MAX_CELLS, cells, 0, 16); }
            int16_t readMinCellVCharge() const { return _parent.readSubcommand<int16_t>(data_register::CELL_BALANCE_MIN_CELL_V_CHARGE); }
            void writeMinCellVCharge(const int16_t& voltage) const { _parent.writeSubcommandClamped<int16_t>(data_register::CELL_BALANCE_MIN_CELL_V_CHARGE, voltage, 0, 5000); }
            uint8_t readMinCellDeltaCharge() const { return _parent.readSubcommand<uint8_t>(data_register::CELL_BALANCE_MIN_DELTA_CHARGE); }
            void writeMinCellDeltaCharge(const uint8_t& voltage) const { _parent.writeSubcommand(data_register::CELL_BALANCE_MIN_DELTA_CHARGE, voltage); }
            uint8_t readStopDeltaCharge() const { return _parent.readSubcommand<uint8_t>(data_register::CELL_BALANCE_STOP_DELTA_CHARGE); }
            void writeStopDeltaCharge(const uint8_t& voltage) const { _parent.writeSubcommand(data_register::CELL_BALANCE_STOP_DELTA_CHARGE, voltage); }
            int16_t readMinCellVRelax() const { return _parent.readSubcommand<int16_t>(data_register::CELL_BALANCE_MIN_CELL_V_RELAX); }
            void writeMinCellVRelax(const int16_t& voltage) const { _parent.writeSubcommandClamped<int16_t>(data_register::CELL_BALANCE_MIN_CELL_V_RELAX, voltage, 0, 5000); }
            uint8_t readMinCellDeltaRelax() const { return _parent.readSubcommand<uint8_t>(data_register::CELL_BALANCE_MIN_DELTA_RELAX); }
            void writeMinCellDeltaRelax(const uint8_t& voltage) const { _parent.writeSubcommand(data_register::CELL_BALANCE_MIN_DELTA_RELAX, voltage); }
            uint8_t readStopDeltaRelax() const { return _parent.readSubcommand<uint8_t>(data_register::CELL_BALANCE_STOP_DELTA_RELAX); }
            void writeStopDeltaRelax(const uint8_t& voltage) const { _parent.writeSubcommand(data_register::CELL_BALANCE_STOP_DELTA_RELAX, voltage); }

        protected:
            CellBalancing(bq76942& parent) : _parent(parent) {}
            CellBalancing(CellBalancing&) = delete;
            CellBalancing& operator=(CellBalancing&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        const Fuse fuse{_parent};
        const Configuration configuration{_parent};
        const Protection protection{_parent};
        const Alarm alarm{_parent};
        const PermanentFailure permanentFailure{_parent};
        const Fet fet{_parent};
        const CellBalancing cellBalancing{_parent};

    protected:
        Settings(bq76942& parent) : _parent(parent) {}

        Settings(Settings&) = delete;
        Settings& operator=(Settings&) = delete;

    private:
        friend class bq76942;
        bq76942& _parent;
    };

    class Power final
    {
    public:
        // Power:Shutdown
        class Shutdown final
        {
        public:
            int16_t readCellVoltage() const { return _parent.readSubcommand<int16_t>(data_register::SHUTDOWN_CELL_VOLTAGE); }
            void writeCellVoltage(const int16_t& voltage) const { _parent.writeSubcommandClamped<int16_t>(data_register::SHUTDOWN_CELL_VOLTAGE, voltage, 0, std::numeric_limits<int16_t>::max()); }
            int32_t readStackVoltage() const { return static_cast<int32_t>(_parent.readSubcommand<int16_t>(data_register::SHUTDOWN_STACK_VOLTAGE)) * 10; }
            void writeStackVoltage(const int32_t& voltage) const { _parent.writeSubcommandClamped<int16_t>(data_register::SHUTDOWN_STACK_VOLTAGE, static_cast<int16_t>(voltage / 10), 0, std::numeric_limits<int16_t>::max()); }
            std::chrono::seconds readLowVDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::LOW_V_SHUTDOWN_DELAY)); }
            void writeLowVDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommandClamped<uint8_t>(data_register::LOW_V_SHUTDOWN_DELAY, static_cast<uint8_t>(delay.count()), 0, 63); };
            uint8_t readTemperature() const { return _parent.readSubcommand<uint8_t>(data_register::SHUTDOWN_TEMPERATURE); }
            void writeTemperature(const uint8_t& temperature) const { _parent.writeSubcommandClamped<uint8_t>(data_register::SHUTDOWN_TEMPERATURE, temperature, 0, 255); }
            std::chrono::seconds readTemperatureDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::SHUTDOWN_TEMPERATURE_DELAY)); }
            void writeTemperatureDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommandClamped<uint8_t>(data_register::SHUTDOWN_TEMPERATURE_DELAY, static_cast<uint8_t>(delay.count()), 0, 254); }
            std::chrono::milliseconds readFetOffDelay() const { return std::chrono::milliseconds(_parent.readSubcommand<uint8_t>(data_register::FET_OFF_DELAY) * 250); }
            void writeFetOffDelay(const std::chrono::milliseconds& delay) const
            {
                const uint8_t roundedDelay = static_cast<uint8_t>(std::round(delay.count() / 250.0));
                _parent.writeSubcommandClamped<uint8_t>(data_register::FET_OFF_DELAY, roundedDelay, 0, 127);
            }
            std::chrono::milliseconds readCommandDelay() const { return std::chrono::milliseconds(_parent.readSubcommand<uint8_t>(data_register::SHUTDOWN_COMMAND_DELAY) * 250); }
            void writeCommandDelay(const std::chrono::milliseconds& delay) const
            {
                const uint8_t roundedDelay = static_cast<uint8_t>(std::round(delay.count() / 250.0));
                _parent.writeSubcommandClamped<uint8_t>(data_register::SHUTDOWN_COMMAND_DELAY, roundedDelay, 0, 254);
            }
            std::chrono::minutes readAutoShutdownTime() const { return std::chrono::minutes(_parent.readSubcommand<uint8_t>(data_register::AUTO_SHUTDOWN_TIME)); }
            void writeAutoShutdownTime(const std::chrono::minutes& time) const { _parent.writeSubcommandClamped<uint8_t>(data_register::AUTO_SHUTDOWN_TIME, static_cast<uint8_t>(time.count()), 0, 250); }
            std::chrono::seconds readRamFailShutdownTime() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::RAM_FAIL_SHUTDOWN_TIME)); }
            void writeRamFailShutdownTime(const std::chrono::seconds& time) const { _parent.writeSubcommand<uint8_t>(data_register::RAM_FAIL_SHUTDOWN_TIME, static_cast<uint8_t>(time.count())); }

        protected:
            Shutdown(bq76942& parent) : _parent(parent) {}
            Shutdown(Shutdown&) = delete;
            Shutdown& operator=(Shutdown&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        // Power:Sleep
        class Sleep final
        {
        public:
            int16_t readCurrent() const { return _parent.readSubcommand<int16_t>(data_register::SLEEP_CURRENT); }
            void writeCurrent(const int16_t& current) const { _parent.writeSubcommandClamped<int16_t>(data_register::SLEEP_CURRENT, current, 0, std::numeric_limits<int16_t>::max()); }
            std::chrono::seconds readVoltageReadDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::VOLTAGE_TIME)); }
            void writeVoltageReadDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommandClamped<uint8_t>(data_register::VOLTAGE_TIME, static_cast<uint8_t>(delay.count()), 1, 255); }
            int16_t readWakeComparatorCurrent() const { return _parent.readSubcommand<int16_t>(data_register::WAKE_COMPARATOR_CURRENT); }
            void writeWakeComparatorCurrent(const int16_t& current) const { _parent.writeSubcommandClamped<int16_t>(data_register::WAKE_COMPARATOR_CURRENT, current, 500, std::numeric_limits<int16_t>::max()); }
            std::chrono::seconds readHysteresisTime() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::SLEEP_HYSTERESIS_TIME)); }
            void writeHysteresisTime(const std::chrono::seconds& time) const { _parent.writeSubcommand(data_register::SLEEP_HYSTERESIS_TIME, static_cast<uint8_t>(time.count())); }
            int32_t readChargerVoltageThreshold() const { return static_cast<int32_t>(_parent.readSubcommand<int16_t>(data_register::SLEEP_CHARGER_VOLTAGE_THRESHOLD)) * 10; }
            void writeChargerVoltageThreshold(const int32_t& voltage) const { _parent.writeSubcommandClamped<int16_t>(data_register::SLEEP_CHARGER_VOLTAGE_THRESHOLD, static_cast<int16_t>(voltage / 10), 0, std::numeric_limits<int16_t>::max()); }
            int32_t readChargerPackTosDelta() const { return static_cast<int32_t>(_parent.readSubcommand<int16_t>(data_register::SLEEP_CHARGER_PACK_TOS_DELTA)) * 10; }
            void writeChargerPackTosDelta(const int32_t& voltage) const { _parent.writeSubcommandClamped<int16_t>(data_register::SLEEP_CHARGER_PACK_TOS_DELTA, static_cast<int16_t>(voltage / 10), 10, 8500); }

        protected:
            Sleep(bq76942& parent) : _parent(parent) {}
            Sleep(Sleep&) = delete;
            Sleep& operator=(Sleep&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        const Shutdown shutdown{_parent};
        const Sleep sleep{_parent};

    protected:
        Power(bq76942& parent) : _parent(parent) {}
        Power(Power&) = delete;
        Power& operator=(Power&) = delete;

    private:
        friend class bq76942;
        bq76942& _parent;
    };

    // System Data:Integrity
    uint16_t readConfigRamSignature() { return readSubcommand<uint16_t>(data_register::CONFIG_RAM_SIGNATURE); }
    void writeConfigRamSignature(const uint16_t& signature) { writeSubcommandClamped<uint16_t>(data_register::CONFIG_RAM_SIGNATURE, signature, 0, 0x7FFF); }

    class Protections final
    {
    public:
        float readCuvThreshold() const { return _parent.readSubcommand<uint8_t>(data_register::CUV_THRESHOLD) * 50.6; }
        void writeCuvThreshold(const float& threshold) const
        {
            const uint8_t roundedThreshold = static_cast<uint8_t>(std::round(threshold / 50.6));
            _parent.writeSubcommandClamped<uint8_t>(data_register::CUV_THRESHOLD, roundedThreshold, 20, 90);
        }
        std::chrono::microseconds readCuvDelay() const { return std::chrono::microseconds(_parent.readSubcommand<uint16_t>(data_register::CUV_DELAY) * 3300L); }
        void writeCuvDelay(const std::chrono::microseconds& delay) const
        {
            const uint16_t roundedDelay = static_cast<uint16_t>(std::round(delay.count() / 3300.0));
            _parent.writeSubcommandClamped<uint16_t>(data_register::CUV_DELAY, roundedDelay, 1, 2047);
        }
        float readCuvRecoveryHysteresis() const { return _parent.readSubcommand<uint8_t>(data_register::CUV_RECOVERY_HYSTERESIS) * 50.6; }
        void writeCuvRecoveryHysteresis(const float& hysteresis) const
        {
            const uint8_t roundedHysteresis = static_cast<uint8_t>(std::round(hysteresis / 50.6));
            _parent.writeSubcommandClamped<uint8_t>(data_register::CUV_RECOVERY_HYSTERESIS, roundedHysteresis, 2, 20);
        }

        float readCovThreshold() const { return _parent.readSubcommand<uint8_t>(data_register::COV_THRESHOLD) * 50.6; }
        void writeCovThreshold(const float& threshold) const
        {
            const uint8_t roundedThreshold = static_cast<uint8_t>(std::round(threshold / 50.6));
            _parent.writeSubcommandClamped<uint8_t>(data_register::COV_THRESHOLD, roundedThreshold, 20, 110);
        }
        std::chrono::microseconds readCovDelay() const { return std::chrono::microseconds(_parent.readSubcommand<uint16_t>(data_register::COV_DELAY) * 3300L); }
        void writeCovDelay(const std::chrono::microseconds& delay) const
        {
            const uint16_t roundedDelay = static_cast<uint16_t>(std::round(delay.count() / 3300.0));
            _parent.writeSubcommandClamped<uint16_t>(data_register::COV_DELAY, roundedDelay, 1, 2047);
        }
        float readCovRecoveryHysteresis() const { return _parent.readSubcommand<uint8_t>(data_register::COV_RECOVERY_HYSTERESIS) * 50.6; }
        void writeCovRecoveryHysteresis(const float& hysteresis) const
        {
            const uint8_t roundedHysteresis = static_cast<uint8_t>(std::round(hysteresis / 50.6));
            _parent.writeSubcommandClamped<uint8_t>(data_register::COV_RECOVERY_HYSTERESIS, roundedHysteresis, 2, 20);
        }

        uint8_t readCovlLatchLimit() const { return _parent.readSubcommand<uint8_t>(data_register::COVL_LATCH_LIMIT); }
        void writeCovlLatchLimit(const uint8_t& limit) const { _parent.writeSubcommand(data_register::COVL_LATCH_LIMIT, limit); }
        std::chrono::seconds readCovlCounterDecDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::COVL_COUNTER_DEC_DELAY)); }
        void writeCovlCounterDecDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::COVL_COUNTER_DEC_DELAY, static_cast<uint8_t>(delay.count())); }
        std::chrono::seconds readCovlRecoveryTime() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::COVL_RECOVERY_TIME)); }
        void writeCovlRecoveryTime(const std::chrono::seconds& time) const { _parent.writeSubcommand(data_register::COVL_RECOVERY_TIME, static_cast<uint8_t>(time.count())); }

        uint16_t readOccThresholdVoltage() const { return static_cast<uint16_t>(_parent.readSubcommand<uint8_t>(data_register::OCC_THRESHOLD)) * 2; }
        void writeOccThresholdVoltage(const uint16_t& voltage) const { _parent.writeSubcommandClamped<uint8_t>(data_register::OCC_THRESHOLD, static_cast<uint8_t>(voltage / 2), 2, 62); }
        std::chrono::microseconds readOccDelay() const { return std::chrono::microseconds((_parent.readSubcommand<uint8_t>(data_register::OCC_DELAY) * 3300L) + 6600L); };
        void writeOccDelay(const std::chrono::microseconds& delay) const
        {
            const uint8_t roundedDelay = static_cast<uint8_t>(std::round((delay.count() - 6600L) / 3300.0));
            _parent.writeSubcommandClamped<uint8_t>(data_register::OCC_DELAY, roundedDelay, 1, 127);
        }
        int16_t readOccRecoveryThreshold() const { return _parent.readSubcommand<int16_t>(data_register::OCC_RECOVERY_THRESHOLD); }
        void writeOccRecoveryThreshold(const int16_t& threshold) const { _parent.writeSubcommand(data_register::OCC_RECOVERY_THRESHOLD, threshold); }
        int32_t readOccPackTosDelta() const { return static_cast<int32_t>(_parent.readSubcommand<int16_t>(data_register::OCC_PACK_TOS_DELTA)) * 10; }
        void writeOccPackTosDelta(const int32_t& delta) const { _parent.writeSubcommandClamped<int16_t>(data_register::OCC_PACK_TOS_DELTA, static_cast<int32_t>(delta / 10), 10, 8500); }

        // note: no need to upgrade the type here, since the max of 100*2 = 200 is still within the uint8_t range
        uint8_t readOcd1Threshold() const { return _parent.readSubcommand<uint8_t>(data_register::OCD1_THRESHOLD) * 2; }
        void writeOcd1Threshold(const uint8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::OCD1_THRESHOLD, threshold / 2, 2, 100); }
        std::chrono::microseconds readOcd1Delay() const { return std::chrono::microseconds((_parent.readSubcommand<uint8_t>(data_register::OCD1_DELAY) * 3300L) + 6600L); }
        void writeOcd1Delay(const std::chrono::microseconds& delay) const
        {
            const uint8_t roundedDelay = static_cast<uint8_t>(std::round((delay.count() - 6600L) / 3300.0));
            _parent.writeSubcommandClamped<uint8_t>(data_register::OCD1_DELAY, roundedDelay, 1, 127);
        }
        uint8_t readOcd2Threshold() const { return _parent.readSubcommand<uint8_t>(data_register::OCD2_THRESHOLD) * 2; }
        void writeOcd2Threshold(const uint8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::OCD2_THRESHOLD, threshold / 2, 2, 100); }
        std::chrono::microseconds readOcd2Delay() const { return std::chrono::microseconds((_parent.readSubcommand<uint8_t>(data_register::OCD2_DELAY) * 3300L) + 6600L); }
        void writeOcd2Delay(const std::chrono::microseconds& delay) const
        {
            const uint8_t roundedDelay = static_cast<uint8_t>(std::round((delay.count() - 6600L) / 3300.0));
            _parent.writeSubcommandClamped<uint8_t>(data_register::OCD2_DELAY, roundedDelay, 1, 127);
        }

        short_circuit_discharge_threshold readShortCircuitThreshold() const { return _parent.readSubcommand<short_circuit_discharge_threshold>(data_register::SCD_THRESHOLD); }
        void writeShortCircuitThreshold(const short_circuit_discharge_threshold& threshold) const { _parent.writeSubcommand(data_register::SCD_THRESHOLD, threshold); }
        std::chrono::microseconds readShortCircuitDelay() const { return std::chrono::microseconds((_parent.readSubcommand<uint8_t>(data_register::SCD_DELAY) - 1) * 15); }
        void writeShortCircuitDelay(const std::chrono::microseconds& delay) const
        {
            const uint8_t roundedDelay = static_cast<uint8_t>(std::round(delay.count() / 15.0) + 1);
            _parent.writeSubcommandClamped<uint8_t>(data_register::SCD_DELAY, roundedDelay, 1, 31);
        }
        std::chrono::seconds readShortCircuitRecoveryTime() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::SCD_RECOVERY_TIME)); }
        void writeShortCircuitRecoveryTime(const std::chrono::seconds& time) const { _parent.writeSubcommand(data_register::SCD_RECOVERY_TIME, static_cast<uint8_t>(time.count())); }

        float readOcd3Threshold() const;
        void writeOcd3Threshold(const float& threshold);
        std::chrono::seconds readOcd3Delay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::OCD3_DELAY)); };
        void writeOcd3Delay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::OCD3_DELAY, static_cast<uint8_t>(delay.count())); }
        int16_t readOcdRecoveryThreshold() const { return _parent.readSubcommand<int16_t>(data_register::OCD_RECOVERY_THRESHOLD); }
        void writeOcdRecoveryThreshold(const int16_t& threshold) const { _parent.writeSubcommand(data_register::OCD_RECOVERY_THRESHOLD, threshold); }

        uint8_t readOcdlLatchLimit() const { return _parent.readSubcommand<uint8_t>(data_register::OCDL_LATCH_LIMIT); }
        void writeOcdlLatchLimit(const uint8_t& limit) const { _parent.writeSubcommand(data_register::OCDL_LATCH_LIMIT, limit); }
        std::chrono::seconds readOcdlCounterDecDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::OCDL_COUNTER_DEC_DELAY)); }
        void writeOcdlCounterDecDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::OCDL_COUNTER_DEC_DELAY, static_cast<uint8_t>(delay.count())); }
        std::chrono::seconds readOcdlRecoveryTime() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::OCDL_RECOVERY_TIME)); }
        void writeOcdlRecoveryTime(const std::chrono::seconds& time) const { _parent.writeSubcommand(data_register::OCDL_RECOVERY_TIME, static_cast<uint8_t>(time.count())); }
        int16_t readOcdlRecoveryThreshold() const { return _parent.readSubcommand<int16_t>(data_register::OCDL_RECOVERY_THRESHOLD); }
        void writeOcdlRecoveryThreshold(const int16_t& threshold) const { _parent.writeSubcommand(data_register::OCDL_RECOVERY_THRESHOLD, threshold); }

        uint8_t readShortCircuitLatchLimit() const { return _parent.readSubcommand<uint8_t>(data_register::SCDL_LATCH_LIMIT); }
        void writeShortCircuitLatchLimit(const uint8_t& limit) const { _parent.writeSubcommand(data_register::SCDL_LATCH_LIMIT, limit); }
        std::chrono::seconds readShortCircuitLatchCounterDecDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::SCDL_COUNTER_DEC_DELAY)); }
        void writeShortCircuitLatchCounterDecDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::SCDL_COUNTER_DEC_DELAY, static_cast<uint8_t>(delay.count())); }
        std::chrono::seconds readShortCircuitLatchRecoveryTime() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::SCDL_RECOVERY_TIME)); }
        void writeShortCircuitLatchRecoveryTime(const std::chrono::seconds& time) const { _parent.writeSubcommand(data_register::SCDL_RECOVERY_TIME, static_cast<uint8_t>(time.count())); }
        int16_t readShortCircuitLatchRecoveryThreshold() const { return _parent.readSubcommand<int16_t>(data_register::SCDL_RECOVERY_THRESHOLD); }
        void writeShortCircuitLatchRecoveryThreshold(const int16_t& threshold) const { _parent.writeSubcommand(data_register::SCDL_RECOVERY_THRESHOLD, threshold); }

        int8_t readOtcThreshold() const { return _parent.readSubcommand<int8_t>(data_register::OTC_THRESHOLD); }
        void writeOtcThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::OTC_THRESHOLD, threshold, -40, 120); }
        std::chrono::seconds readOtcDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::OTC_DELAY)); }
        void writeOtcDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::OTC_DELAY, static_cast<uint8_t>(delay.count())); }
        int8_t readOtcRecoveryThreshold() const { return _parent.readSubcommand<int8_t>(data_register::OTC_RECOVERY); }
        void writeOtcRecoveryThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::OTC_RECOVERY, threshold, -40, 120); }

        int8_t readOtdThreshold() const { return _parent.readSubcommand<int8_t>(data_register::OTD_THRESHOLD); }
        void writeOtdThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::OTD_THRESHOLD, threshold, -40, 120); }
        std::chrono::seconds readOtdDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::OTD_DELAY)); }
        void writeOtdDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::OTD_DELAY, static_cast<uint8_t>(delay.count())); }
        int8_t readOtdRecoveryThreshold() const { return _parent.readSubcommand<int8_t>(data_register::OTD_RECOVERY); }
        void writeOtdRecoveryThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::OTD_RECOVERY, threshold, -40, 120); }

        int8_t readOtfThreshold() const { return _parent.readSubcommand<int8_t>(data_register::OTF_THRESHOLD); }
        void writeOtfThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::OTF_THRESHOLD, threshold, 0, 150); }
        std::chrono::seconds readOtfDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::OTF_DELAY)); }
        void writeOtfDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::OTF_DELAY, static_cast<uint8_t>(delay.count())); }
        int8_t readOtfRecoveryThreshold() const { return _parent.readSubcommand<int8_t>(data_register::OTF_RECOVERY); }
        void writeOtfRecoveryThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::OTF_RECOVERY, threshold, 0, 150); }

        int8_t readOtintThreshold() const { return _parent.readSubcommand<int8_t>(data_register::OTINT_THRESHOLD); }
        void writeOtintThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::OTINT_THRESHOLD, threshold, -40, 120); }
        std::chrono::seconds readOtintDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::OTINT_DELAY)); }
        void writeOtintDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::OTINT_DELAY, static_cast<uint8_t>(delay.count())); }
        int8_t readOtintRecoveryThreshold() const { return _parent.readSubcommand<int8_t>(data_register::OTINT_RECOVERY); }
        void writeOtintRecoveryThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::OTINT_RECOVERY, threshold, -40, 120); }

        int8_t readUtcThreshold() const { return _parent.readSubcommand<int8_t>(data_register::UTC_THRESHOLD); }
        void writeUtcThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::UTC_THRESHOLD, threshold, -40, 120); }
        std::chrono::seconds readUtcDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::UTC_DELAY)); }
        void writeUtcDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::UTC_DELAY, static_cast<uint8_t>(delay.count())); }
        int8_t readUtcRecoveryThreshold() const { return _parent.readSubcommand<int8_t>(data_register::UTC_RECOVERY); }
        void writeUtcRecoveryThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::UTC_RECOVERY, threshold, -40, 120); }

        int8_t readUtdThreshold() const { return _parent.readSubcommand<int8_t>(data_register::UTD_THRESHOLD); }
        void writeUtdThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::UTD_THRESHOLD, threshold, -40, 120); }
        std::chrono::seconds readUtdDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::UTD_DELAY)); }
        void writeUtdDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::UTD_DELAY, static_cast<uint8_t>(delay.count())); }
        int8_t readUtdRecoveryThreshold() const { return _parent.readSubcommand<int8_t>(data_register::UTD_RECOVERY); }
        void writeUtdRecoveryThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::UTD_RECOVERY, threshold, -40, 120); }

        int8_t readUtintThreshold() const { return _parent.readSubcommand<int8_t>(data_register::UTINT_THRESHOLD); }
        void writeUtintThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::UTINT_THRESHOLD, threshold, -40, 120); }
        std::chrono::seconds readUtintDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::UTINT_DELAY)); }
        void writeUtintDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::UTINT_DELAY, static_cast<uint8_t>(delay.count())); }
        int8_t readUtintRecoveryThreshold() const { return _parent.readSubcommand<int8_t>(data_register::UTINT_RECOVERY); }
        void writeUtintRecoveryThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::UTINT_RECOVERY, threshold, -40, 120); }

        std::chrono::seconds readRecoveryTime() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::PROTECTIONS_RECOVERY_TIME)); }
        void writeRecoveryTime(const std::chrono::seconds& time) const { _parent.writeSubcommand(data_register::PROTECTIONS_RECOVERY_TIME, static_cast<uint8_t>(time.count())); }

        std::chrono::seconds readHwdTimeout() const { return std::chrono::seconds(_parent.readSubcommand<uint16_t>(data_register::HWD_DELAY)); }
        void writeHwdTimeout(const std::chrono::seconds& timeout) const { _parent.writeSubcommand(data_register::HWD_DELAY, static_cast<uint16_t>(timeout.count())); }

        std::chrono::seconds readLoadDetectTime() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::LOAD_DETECT_ACTIVE_TIME)); }
        void writeLoadDetectTime(const std::chrono::seconds& time) const { _parent.writeSubcommand(data_register::LOAD_DETECT_ACTIVE_TIME, static_cast<uint8_t>(time.count())); }
        std::chrono::seconds readLoadDetectRetryDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::LOAD_DETECT_RETRY_DELAY)); }
        void writeLoadDetectRetryDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::LOAD_DETECT_RETRY_DELAY, static_cast<uint8_t>(delay.count())); }
        std::chrono::hours readLoadDetectTimeout() const { return std::chrono::hours(_parent.readSubcommand<uint16_t>(data_register::LOAD_DETECT_TIMEOUT)); }
        void writeLoadDetectTimeout(const std::chrono::hours& timeout) const { _parent.writeSubcommand(data_register::LOAD_DETECT_TIMEOUT, static_cast<uint16_t>(timeout.count())); }

        int16_t readPrechargeTimeoutCurrentThreshold() const { return _parent.readSubcommand<int16_t>(data_register::PTO_CHARGE_THRESHOLD); }
        void writePrechargeTimeoutCurrentThreshold(const int16_t& threshold) const { _parent.writeSubcommandClamped<int16_t>(data_register::PTO_CHARGE_THRESHOLD, threshold, 0, 1000); }
        std::chrono::seconds readPrechargeTimeoutDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint16_t>(data_register::PTO_DELAY)); }
        void writePrechargeTimeoutDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::PTO_DELAY, static_cast<uint16_t>(delay.count())); }
        float readPrechargeResetCharge() const;
        void writePrechargeResetCurrent(const float& charge) const;

    protected:
        Protections(bq76942& parent) : _parent(parent) {}
        Protections(Protections&) = delete;
        Protections& operator=(Protections&) = delete;

    private:
        friend class bq76942;
        bq76942& _parent;
    };

    class PermanentFail final
    {
    public:
        int16_t readCopperDepositionThreshold() const { return _parent.readSubcommand<int16_t>(data_register::CU_DEP_THRESHOLD); }
        void writeCopperDepositionThreshold(const int16_t& threshold) const { _parent.writeSubcommandClamped<int16_t>(data_register::CU_DEP_THRESHOLD, threshold, 0, std::numeric_limits<int16_t>::max()); }
        std::chrono::seconds readCopperDepositionDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::CU_DEP_DELAY)); }
        void writeCopperDepositionDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::CU_DEP_DELAY, static_cast<uint8_t>(delay.count())); }

        int16_t readSuvThreshold() const { return _parent.readSubcommand<int16_t>(data_register::SUV_THRESHOLD); }
        void writeSuvThreshold(const int16_t& threshold) const { _parent.writeSubcommandClamped<int16_t>(data_register::SUV_THRESHOLD, threshold, 0, std::numeric_limits<int16_t>::max()); }
        std::chrono::seconds readSuvDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::SUV_DELAY)); }
        void writeSuvDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::SUV_DELAY, static_cast<uint8_t>(delay.count())); }

        int16_t readSovThreshold() const { return _parent.readSubcommand<int16_t>(data_register::SOV_THRESHOLD); }
        void writeSovThreshold(const int16_t& threshold) const { _parent.writeSubcommandClamped<int16_t>(data_register::SOV_THRESHOLD, threshold, 0, std::numeric_limits<int16_t>::max()); }
        std::chrono::seconds readSovDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::SOV_DELAY)); }
        void writeSovDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::SOV_DELAY, static_cast<uint8_t>(delay.count())); }

        int16_t readTosThreshold() const { return _parent.readSubcommand<int16_t>(data_register::TOS_THRESHOLD); }
        void writeTosThreshold(const int16_t& threshold) const { _parent.writeSubcommandClamped<int16_t>(data_register::TOS_THRESHOLD, threshold, 0, std::numeric_limits<int16_t>::max()); }
        std::chrono::seconds readTosDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::TOS_DELAY)); }
        void writeTosDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::TOS_DELAY, static_cast<uint8_t>(delay.count())); }

        float readSoccThreshold() const;
        void writeSoccThreshold(const float& threshold) const;

        float readSocdThreshold() const;
        void writeSocdThreshold(const float& threshold) const;

        int8_t readSotThreshold() const { return _parent.readSubcommand<int8_t>(data_register::SOT_THRESHOLD); }
        void writeSotThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<int8_t>(data_register::SOT_THRESHOLD, threshold, -40, 120); }
        std::chrono::seconds readSotDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::SOT_DELAY)); }
        void writeSotDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::SOT_DELAY, static_cast<uint8_t>(delay.count())); }

        int8_t readSotfThreshold() const { return _parent.readSubcommand<int8_t>(data_register::SOTF_THRESHOLD); }
        void writeSotfThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<int8_t>(data_register::SOTF_THRESHOLD, threshold, 0, 150); }
        std::chrono::seconds readSotfDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::SOTF_DELAY)); }
        void writeSotfDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::SOTF_DELAY, static_cast<uint8_t>(delay.count())); }

        int16_t readVimrCheckVoltage() const { return _parent.readSubcommand<int16_t>(data_register::VIMR_CHECK_VOLTAGE); }
        void writeVimrCheckVoltage(const int16_t& voltage) const { _parent.writeSubcommandClamped<int16_t>(data_register::VIMR_CHECK_VOLTAGE, voltage, 0, 5500); }
        int16_t readVimrMaxRelaxCurrent() const { return _parent.readSubcommand<int16_t>(data_register::VIMR_MAX_RELAX_CURRENT); }
        void writeVimrMaxRelaxCurrent(const int16_t& current) const { _parent.writeSubcommandClamped<int16_t>(data_register::VIMR_MAX_RELAX_CURRENT, current, 10, std::numeric_limits<int16_t>::max()); }
        int16_t readVimrThreshold() const { return _parent.readSubcommand<int16_t>(data_register::VIMR_THRESHOLD); }
        void writeVimrThreshold(const int16_t& threshold) const { _parent.writeSubcommandClamped<int16_t>(data_register::VIMR_THRESHOLD, threshold, 0, 5500); }
        std::chrono::seconds readVimrDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::VIMR_DELAY)); }
        void writeVimrDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::VIMR_DELAY, static_cast<uint8_t>(delay.count())); }
        std::chrono::seconds readVimrRelaxMinDuration() const { return std::chrono::seconds(_parent.readSubcommand<uint16_t>(data_register::VIMR_RELAX_MIN_DURATION)); }
        void writeVimrRelaxMinDuration(const std::chrono::seconds& duration) const { _parent.writeSubcommand(data_register::VIMR_RELAX_MIN_DURATION, static_cast<uint16_t>(duration.count())); }

        int16_t readVimaCheckVoltage() const { return _parent.readSubcommand<int16_t>(data_register::VIMA_CHECK_VOLTAGE); }
        void writeVimaCheckVoltage(const int16_t& voltage) const { _parent.writeSubcommandClamped<int16_t>(data_register::VIMA_CHECK_VOLTAGE, voltage, 0, 5500); }
        int16_t readVimaMinActiveCurrent() const { return _parent.readSubcommand<int16_t>(data_register::VIMA_MIN_ACTIVE_CURRENT); }
        void writeVimaMinActiveCurrent(const int16_t& current) const { _parent.writeSubcommandClamped<int16_t>(data_register::VIMA_MIN_ACTIVE_CURRENT, current, 10, std::numeric_limits<int16_t>::max()); }
        int16_t readVimaThreshold() const { return _parent.readSubcommand<int16_t>(data_register::VIMA_THRESHOLD); }
        void writeVimaThreshold(const int16_t& threshold) const { _parent.writeSubcommandClamped<int16_t>(data_register::VIMA_THRESHOLD, threshold, 0, 5500); }
        std::chrono::seconds readVimaDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::VIMA_DELAY)); }
        void writeVimaDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::VIMA_DELAY, static_cast<uint8_t>(delay.count())); }

        int16_t readCfetfThreshold() const { return _parent.readSubcommand<int16_t>(data_register::CFETF_OFF_THRESHOLD); }
        void writeCfetfThreshold(const int16_t& threshold) const { _parent.writeSubcommandClamped<int16_t>(data_register::CFETF_OFF_THRESHOLD, threshold, 10, 5000); }
        std::chrono::seconds readCfetfDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::CFETF_OFF_DELAY)); }
        void writeCfetfDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::CFETF_OFF_DELAY, static_cast<uint8_t>(delay.count())); }
        int16_t readDfetfThreshold() const { return _parent.readSubcommand<int16_t>(data_register::DFETF_OFF_THRESHOLD); }
        void writeDfetfThreshold(const int16_t& threshold) const { _parent.writeSubcommandClamped<int16_t>(data_register::DFETF_OFF_THRESHOLD, threshold, -5000, -10); }
        std::chrono::seconds readDfetfDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::DFETF_OFF_DELAY)); }
        void writeDfetfDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::DFETF_OFF_DELAY, static_cast<uint8_t>(delay.count())); }

        int16_t readVssfThreshold() const { return _parent.readSubcommand<int16_t>(data_register::VSSF_FAIL_THRESHOLD); }
        void writeVssfThreshold(const int16_t& threshold) const { _parent.writeSubcommandClamped<int16_t>(data_register::VSSF_FAIL_THRESHOLD, threshold, 1, std::numeric_limits<int16_t>::max()); };
        std::chrono::seconds readVssfDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::VSSF_DELAY)); }
        void writeVssfDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::VSSF_DELAY, static_cast<uint8_t>(delay.count())); }

        std::chrono::seconds read2LvlDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::PF_2LVL_DELAY)); }
        void write2LvlDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::PF_2LVL_DELAY, static_cast<uint8_t>(delay.count())); }
        std::chrono::seconds readLfofDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::LFOF_DELAY)); }
        void writeLfofDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::LFOF_DELAY, static_cast<uint8_t>(delay.count())); }
        std::chrono::seconds readHardwareMuxfDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::HWMX_DELAY)); }
        void writeHardwareMuxfDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::HWMX_DELAY, static_cast<uint8_t>(delay.count())); }

    protected:
        PermanentFail(bq76942& parent) : _parent(parent)
        {
        }
        PermanentFail(PermanentFail&) = delete;
        PermanentFail& operator=(PermanentFail&) = delete;

    private:
        friend class bq76942;
        bq76942& _parent;
    };

    uint16_t readUnsealKey1() { return readSubcommand<uint16_t>(data_register::UNSEAL_KEY_STEP1); }
    uint16_t readUnsealKey2() { return readSubcommand<uint16_t>(data_register::UNSEAL_KEY_STEP2); }
    uint16_t readFullAccessKey1() { return readSubcommand<uint16_t>(data_register::FULL_ACCESS_KEY_STEP1); }
    uint16_t readFullAccessKey2() { return readSubcommand<uint16_t>(data_register::FULL_ACCESS_KEY_STEP2); }

    const Calibration calibration{*this};
    const Settings settings{*this};
    const Power power{*this};
    const Protections protections{*this};
    const PermanentFail permanentFail{*this};

    std::vector<uint8_t> readDirect(const uint8_t registerAddr, const size_t bytes);
    template <typename T>
    T readDirect(const uint8_t registerAddr)
    {
        std::vector<uint8_t> dataBuf = readDirect(registerAddr, sizeof(T));
        if (dataBuf.size() != sizeof(T))
        {
            throw std::runtime_error(std::format("Data size mismatch - got {} bytes, expected {} for type {}",
                                                 dataBuf.size(), sizeof(T), demangle(typeid(T).name())));
        }
        // both the BMS and the ESP32 are little-endian so this works nicely
        // for all data types
        T data{};
        std::copy_n(dataBuf.begin(), sizeof(T), reinterpret_cast<uint8_t*>(&data));
        return data;
    }
    std::vector<uint8_t> readDirect(const direct_command registerAddr, const size_t bytes)
    {
        return readDirect(static_cast<uint8_t>(registerAddr), bytes);
    }
    template <typename T>
    T readDirect(const direct_command registerAddr)
    {
        return readDirect<T>(static_cast<uint8_t>(registerAddr));
    }

    void writeDirect(const uint8_t registerAddr, const std::vector<uint8_t>& data);
    template <typename T>
    void writeDirect(const uint8_t registerAddr, const T& data)
    {
        std::vector<uint8_t> dataBuf(sizeof(T));
        std::copy_n(reinterpret_cast<const uint8_t*>(&data), sizeof(T), dataBuf.begin());
        writeDirect(registerAddr, dataBuf);
    }
    void writeDirect(const direct_command registerAddr, const std::vector<uint8_t>& data)
    {
        writeDirect(static_cast<uint8_t>(registerAddr), data);
    }
    template <typename T>
    void writeDirect(const direct_command registerAddr, const T& data)
    {
        writeDirect(static_cast<uint8_t>(registerAddr), data);
    }

    std::vector<uint8_t> readSubcommand(const uint16_t registerAddr);
    template <typename T>
    T readSubcommand(const uint16_t registerAddr)
    {
        std::vector<uint8_t> dataBuf = readSubcommand(registerAddr);
        if (dataBuf.size() != sizeof(T))
        {
            throw std::runtime_error(std::format("Data size mismatch - got {} bytes, expected {} for type {}",
                                                 dataBuf.size(), sizeof(T), demangle(typeid(T).name())));
        }
        T data{};
        std::copy_n(dataBuf.begin(), sizeof(T), reinterpret_cast<uint8_t*>(&data));
        return data;
    }
    std::vector<uint8_t> readSubcommand(const data_register registerAddr) { return readSubcommand(static_cast<uint16_t>(registerAddr)); }
    template <typename T>
    T readSubcommand(const data_register registerAddr)
    {
        return readSubcommand<T>(static_cast<uint16_t>(registerAddr));
    }
    std::vector<uint8_t> readSubcommand(const subcommand registerAddr) { return readSubcommand(static_cast<uint16_t>(registerAddr)); }
    template <typename T>
    T readSubcommand(const subcommand registerAddr)
    {
        return readSubcommand<T>(static_cast<uint16_t>(registerAddr));
    }

    void writeSubcommand(const uint16_t registerAddr, const std::vector<uint8_t>& data = {});
    template <typename T>
    void writeSubcommand(const uint16_t registerAddr, const T& data)
    {
        std::vector<uint8_t> dataBuf(sizeof(T));
        std::copy_n(reinterpret_cast<const uint8_t*>(&data), sizeof(T), dataBuf.begin());
        writeSubcommand(registerAddr, dataBuf);
    }
    void writeSubcommand(const data_register registerAddr, const std::vector<uint8_t>& data = {}) { writeSubcommand(static_cast<uint16_t>(registerAddr), data); }
    template <typename T>
    void writeSubcommand(const data_register registerAddr, const T& data)
    {
        writeSubcommand(static_cast<uint16_t>(registerAddr), data);
    }
    void writeSubcommand(const subcommand registerAddr, const std::vector<uint8_t>& data = {}) { writeSubcommand(static_cast<uint16_t>(registerAddr), data); }
    template <typename T>
    void writeSubcommand(const subcommand registerAddr, const T& data)
    {
        writeSubcommand(static_cast<uint16_t>(registerAddr), data);
    }
    void writeSubcommand(const cmd_only_subcommand registerAddr) { writeSubcommand(static_cast<uint16_t>(registerAddr)); }

    template <typename T>
    void writeSubcommandClamped(const data_register registerAddr, const T& data, const T& minimum, const T& maximum)
    {
        static_assert(std::is_arithmetic<T>::value, "T must be an arithmetic type");
        if ((data < minimum) || (data > maximum))
        {
            throw std::out_of_range(std::format("Value {} is out of range for register {:#x} - must be between {} and {}",
                                                data, static_cast<uint16_t>(registerAddr), minimum, maximum));
        }
        writeSubcommand(registerAddr, data);
    }

    static float rawTempToCelsius(int16_t raw) { return kelvinToCelsius(raw / 10.0f); }
    static float kelvinToCelsius(float kelvin) { return kelvin - 273.15f; }
    static float getUserAmpsMultiplier(const da_configuration_t& config);
    static int16_t getUserVoltsMultiplier(const da_configuration_t& config);

private:
    uint8_t _address;

    std::vector<uint8_t> _readDirect(const uint8_t registerAddr, const size_t bytes);
    void _writeDirect(const uint8_t registerAddr, const std::vector<uint8_t>& data);
    std::vector<uint8_t> _readSubcommand(const uint16_t registerAddr);
    void _writeSubcommand(const uint16_t registerAddr, const std::vector<uint8_t>& data);
    static uint8_t _calculateChecksum(uint16_t registerAddr, const std::vector<uint8_t>& data);

    /// @brief Temporary buffer for storing data so CRCs can be calculated
    std::vector<uint8_t> _dataBuf;
};