#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <stdexcept>
#include <type_demangle.hpp>
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
    struct control_status_t
    {
        // was pullup active during previous measurement?
        bool wasLoadDetectOn : 1 = 0;        // LD_ON
        bool hasLoadDetectTimedOut : 1 = 0;  // LD_TIMEOUT
        bool inDeepsleep : 1 = 0;            // DEEPSLEEP
        uint16_t _rsvd4 : 15 = 0;            // RSVD_0
    };
    struct protections_a_t
    {
        uint8_t _rsvd0 : 2 = 0;              // RSVD_0
        bool cellUndervoltage : 1 = 0;       // CUV
        bool cellOvervoltage : 1 = 1;        // COV
        bool overcurrentCharge : 1 = 0;      // OCC
        bool overcurrentDischarge1 : 1 = 0;  // OCD1
        bool overcurrentDischarge2 : 1 = 0;  // OCD2
        bool shortCircuitDischarge : 1 = 1;  // SCD
    };
    struct protections_b_t
    {
        bool undertempCharge : 1 = 0;     // UTC
        bool undertempDischarge : 1 = 0;  // UTD
        bool internalUndertemp : 1 = 0;   // UTINT
        bool _rsvd3 : 1 = 0;              // RSVD_0
        bool overtempCharge : 1 = 0;      // OTC
        bool overtempDischarge : 1 = 0;   // OTD
        bool internalOvertemp : 1 = 0;    // OTINT
        bool fetOvertemp : 1 = 0;         // OTF
    };
    struct safety_alert_c_t
    {
        uint8_t _rsvd0 : 3 = 0;               // RSVD_0
        bool prechargeTimeoutSuspend : 1;     // PTOS
        bool cellOvervoltageLatch : 1;        // COVL
        bool overcurrentDischarge3 : 1;       // OCD3
        bool shortCircuitDischargeLatch : 1;  // SCDL
    };
    struct protections_c_t
    {
        bool _rsvd0 : 1 = 0;             // RSVD_0
        bool hostWatchdogFault : 1 = 0;  // HWDF
        bool prechargeTimeout : 1 = 0;   // PTO
        // NOTE: When used in Settings:Protection:Enabled Protections C,
        // this bit is only RSVD not RSVD_0
        bool _rsvd3 : 1 = 0;                      // RSVD_0
        bool cellOvervoltageLatch : 1 = 0;        // COVL
        bool overcurrentDischargeLatch : 1 = 0;   // OCDL
        bool shortCircuitDischargeLatch : 1 = 0;  // SCDL
        bool overcurrentDischarge3 : 1 = 0;       // OCD3
    };
    struct permanent_fail_a_t
    {
        bool safetyCellUndervoltage : 1;      // SUV
        bool safetyCellOvervoltage : 1;       // SOV
        bool safetyOvercurrentCharge : 1;     // SOCC
        bool safetyOvercurrentDischarge : 1;  // SOCD
        bool safetyOvertemp : 1;              // SOT
        bool _rsvd5 : 1 = 0;                  // RSVD_0
        bool safetyOvertempFet : 1;           // SOTF
        bool copperDeposition : 1;            // CUDEP
    };
    struct permanent_fail_b_t
    {
        bool chargeFet : 1;                   // CFETF
        bool dischargeFet : 1;                // DFETF
        bool secondLevelProtector : 1;        // 2LVL
        bool voltageImbalanceRelax : 1;       // VIMR
        bool voltageImbalanceActive : 1;      // VIMA
        uint8_t _rsvd5 : 2 = 0;               // RSVD_0
        bool shortCircuitDischargeLatch : 1;  // SCDL
    };
    struct permanent_fail_alert_c_t
    {
        uint8_t _rsvd0 : 3 = 0;             // RSVD_0
        bool internalLFO : 1;               // LFOF
        bool internalVoltageReference : 1;  // VREF
        bool internalVssMeasurement : 1;    // VSSF
        bool hardwareMux : 1;               // HWMX
        bool _rsvd7 : 1 = 0;                // RSVD_0
    };
    struct permanent_fail_c_t
    {
        bool otpMemory : 1;                 // OTPF
        bool dataROM : 1;                   // DRMF
        bool instructionROM : 1;            // IRMF
        bool internalLFO : 1;               // LFOF
        bool internalVoltageReference : 1;  // VREF
        bool internalVssMeasurement : 1;    // VSSF
        bool hardwareMux : 1;               // HWMX
        bool commanded : 1;                 // CMDF
    };
    struct permanent_fail_d_t
    {
        // top of stack vs cell sum
        bool topStackVsCell : 1;  // TOSF
        uint8_t _rsvd1 : 7 = 0;   // RSVD_0
    };
    struct battery_status_t
    {
        bool inConfigUpdateMode : 1;  // CFGUPDATE
        bool inPrechargeMode : 1;     // PCHG_MODE
        bool sleepAllowed : 1;        // SLEEP_EN
        // AKA: Power-On Reset
        // Whether or not a full reset has occurred since last CONFIG_UPDATE exit
        bool fullResetOccurred : 1;  // POR
        // whether or not the previous reset was due to the watchdog timer
        // NOTE: Independent of Host Watchdog settings
        bool wasWatchdogReset : 1;         // WD
        bool checkingCellOpenWire : 1;     // COW_CHK
        bool pendingOtpWrite : 1;          // OTPW
        bool otpWriteBlocked : 1;          // OTPB
        security_state securityState : 2;  // SEC[1:0]
        bool fuseActive : 1;               // FUSE
        bool safetyFaultActive : 1;        // SS
        bool permanentFailActive : 1;      // PF
        bool shutdownPending : 1;          // SD_CMD
        bool _rsvd14 : 1 = 0;              // RSVD_0
        bool inSleep : 1;                  // SLEEP
    };
    struct alarm_status_t
    {
        bool hasWokenFromSleep : 1 = 0;            // WAKE
        bool adcScanComplete : 1 = 0;              // ADSCAN
        bool isBalancingCells : 1 = 0;             // CB
        bool isFuseDriven : 1 = 0;                 // FUSE
        bool stackReachedShutdownVoltage : 1 = 0;  // SHUTV
        bool dischargeFetOff : 1 = 0;              // XDSG
        bool chargeFetOff : 1 = 0;                 // XCHG
        bool fullVoltageScanComplete : 1 = 0;      // FULLSCAN
        bool _rsvd8 : 1 = 0;                       // RSVD_0
        bool initializationComplete : 1 = 0;       // INITCOMP
        bool initializationStarted : 1 = 0;        // INITSTART
        // anything in the alarm PF A, B, C, or D masks triggered
        bool alarmPFAlert : 1 = 1;  // MSK_PFALERT
        // anything in the alarm SF A, B, or C masks triggered
        bool alarmSFAlert : 1 = 1;      // MSK_SFALERT
        bool permanentFail : 1 = 1;     // PF
        bool safetyStatusA : 1 = 1;     // SSA
        bool safetyStatusBOrC : 1 = 1;  // SSBC
    };
    struct fet_status_t
    {
        bool chargeFetOn : 1;        // CHG_FET
        bool prechargeFetOn : 1;     // PCHG_FET
        bool dischargeFetOn : 1;     // DSG_FET
        bool predischargeFetOn : 1;  // PDSG_FET
        bool dchgAsserted : 1;       // DCHG_PIN
        bool ddsgAsserted : 1;       // DDSG_PIN
        bool alertAsserted : 1;      // ALRT_PIN
        bool _rsvd7 : 1 = 0;         // RSVD_0
    };
    struct manufacturing_status_t
    {
        bool prechargeTesting : 1;        // PCHG_TEST
        bool chargeTesting : 1;           // CHG_TEST
        bool dischargeTesting : 1;        // DSG_TEST
        bool _rsvd3 : 1 = 0;              // RSVD_0
        bool autonomousFets : 1;          // FET_EN
        bool predischargeTesting : 1;     // PDSG_TEST
        bool isPermanentFailEnabled : 1;  // PF_EN
        bool isOtpWriteEnabled : 1;       // OTPW_EN
        uint8_t _rsvd8 : 8 = 0;           // RSVD_0
    };
    struct fet_control_t
    {
        bool forceDischargeOff : 1;     // DSG_OFF
        bool forcePredischargeOff : 1;  // PDSG_OFF
        bool forceChargeOff : 1;        // CHG_OFF
        bool forcePrechargeOff : 1;     // PCHG_OFF
        uint8_t _rsvd4 : 4 = 0;         // RSVD_0
    };
    struct regulator_control_t
    {
        bool reg1Enable : 1;                // REG1_EN
        regulator_voltage reg1Voltage : 3;  // REG1V_[2:0]
        bool reg2Enable : 1;                // REG2_EN
        regulator_voltage reg2Voltage : 3;  // REG2V_[2:0]
    };
    struct otp_write_result_t
    {
        struct resultRegister
        {
            // note: The following are NOT errors for the whole chip,
            // OTP programming just has stricter requirements
            bool overVoltage : 1;         // HV
            bool underVoltage : 1;        // LV
            bool overTemp : 1;            // HT
            bool dataWriteFail : 1;       // NODATA
            bool signatureWriteFail : 1;  // NOSIG
            bool otpLocked : 1;           // LOCK
            bool _rsvd6 : 1 = 0;          // RSVD_0
            bool otpProgrammingOk : 1;    // OK
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

    // --- SUBCOMMANDS ---
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
    struct power_config_t
    {
        coulomb_conversion_speed wakeSpeed : 2 = coulomb_conversion_speed::T_24MS;  // WK_SPD_[1:0]
        // measurement loop speed during normal operation
        measurement_loop_speed loopSpeed : 2 = measurement_loop_speed::FULL_SPEED;  // LOOP_SLOW_[1:0]
        // measurement loop speed during cell balancing
        measurement_loop_speed cellBalanceLoopSpeed : 2 = measurement_loop_speed::FULL_SPEED;  // CB_LOOP_SLOW_[1:0]
        // cleared: 3ms per conversion
        // set: 1.5ms per conversion (but lower accuracy)
        bool useFastAdc : 1 = false;                      // FASTADC
        bool enableOvertempShutdown : 1 = true;           // OTSD
        bool enableSleep : 1 = true;                      // SLEEP
        bool enableDeepSleepLFO : 1 = false;              // DPSLP_LFO
        bool enableDeepSleepLDO : 1 = false;              // DPSLP_LDO
        bool enableDeepSleepChargerWake : 1 = true;       // DPSLP_PD
        bool disableShutdownTS2Wake : 1 = false;          // SHUT_TS2
        bool enableDeepSleepOvertempShutdown : 1 = true;  // DPSLP_OT spellchecker:disable-line
        uint8_t _rsvd14 : 2 = 0;                          // RSVD_0
    };
    struct reg0_config_t
    {
        bool reg0Enable : 1 = true;  // REG0_EN
        bool _rsvd1 : 1;             // RSVD
        uint8_t _rsvd2 : 6 = 0;      // RSVD_0
    };
    struct hwd_regulator_options_t
    {
        // Regulator off time in seconds before turning back on
        uint8_t toggleTime : 4;           // TOGGLE_TIME_[3:0]
        hwd_toggle_option hwdAction : 2;  // TOGGLE_OPT_[1:0]
        uint8_t _rsvd5 : 2 = 0;           // RSVD_0
    };
    struct spi_configuration_t
    {
        uint8_t _rsvd0 : 5 = 0;  // RSVD_0
        // use digital filters (recommend for high-freq operation)
        bool enableFilters : 1;  // FILT
        // clear: MISO uses REG18 voltage
        // set: MISO uses REG1 voltage
        bool misoUsesReg1 : 1;  // MISO_REG1
        bool _rsvd7 : 1 = 0;    // RSVD_0
    };
    struct cfetoff_pin_configuration_t
    {
        union
        {
            struct
            {
                cfetoff_pin_function function : 2;  // PINFXN[1:0]
                bool enablePulldown : 1;            // OPT[0]
                // clear: Driving high drives to HI-Z (unavailable with isActiveLow)
                // set: Driving high drives to selected regulator
                bool disableHighZDrive : 1;  // OPT[1]
                // NOTE: Should be cleared when isActiveLow is set
                bool pullupToReg1 : 1;  // OPT[2]
                // clear: High uses REG18
                // set: High uses REG1
                bool driveHighUsesReg1 : 1;  // OPT[3]
                bool _rsvd6 : 1;             // OPT[4]
                bool isActiveLow : 1;        // OPT[5]
            } function;
            struct
            {
                adc_pin_function function : 2;         // PINFXN[1:0]
                measurement_type measurementType : 2;  // OPT[1:0]
                polynomial_selection polynomial : 2;   // OPT[3:2]
                pullup_config pullupConfig : 2;        // OPT[5:4]
            } adc;
        };
    };
    struct dfetoff_pin_configuration_t
    {
        union
        {
            struct
            {
                dfetoff_pin_function function : 2;  // PINFXN[1:0]
                bool enablePulldown : 1;            // OPT[0]
                // clear: Driving high drives to HI-Z (unavailable with isActiveLow)
                // set: Driving high drives to selected regulator
                bool disableHighZDrive : 1;  // OPT[1]
                // NOTE: Should be cleared when isActiveLow is set
                bool pullupToReg1 : 1;  // OPT[2]
                // clear: High uses REG18
                // set: High uses REG1
                bool driveHighUsesReg1 : 1;  // OPT[3]
                // set: acts as BOTHOFF
                bool isBothOff : 1;  // OPT[4]
                // _rsvd6 clear: acts as DFETOFF
                bool isActiveLow : 1;  // OPT[5]
            } function;
            struct
            {
                adc_pin_function function : 2;         // PINFXN[1:0]
                measurement_type measurementType : 2;  // OPT[1:0]
                polynomial_selection polynomial : 2;   // OPT[3:2]
                pullup_config pullupConfig : 2;        // OPT[5:4]
            } adc;
        };
    };
    struct alert_pin_configuration_t
    {
        union
        {
            struct
            {
                alert_pin_function function : 2;  // PINFXN[1:0]
                bool enablePulldown : 1;          // OPT[0]
                // clear: Driving high drives to HI-Z (unavailable with isActiveLow)
                // set: Driving high drives to selected regulator
                bool disableHighZDrive : 1;  // OPT[1]
                // NOTE: Should be cleared when isActiveLow is set
                bool pullupToReg1 : 1;  // OPT[2]
                // clear: High uses REG18
                // set: High uses REG1
                bool driveHighUsesReg1 : 1;  // OPT[3]
                bool _rsvd6 : 1;             // OPT[4]
                bool isActiveLow : 1;        // OPT[5]
            } function;
            struct
            {
                adc_pin_function function : 2;         // PINFXN[1:0]
                measurement_type measurementType : 2;  // OPT[1:0]
                polynomial_selection polynomial : 2;   // OPT[3:2]
                pullup_config pullupConfig : 2;        // OPT[5:4]
            } adc;
        };
    };
    struct ts_pin_configuration_t
    {
        adc_pin_function function : 2;         // PINFXN[1:0]
        measurement_type measurementType : 2;  // OPT[1:0]
        polynomial_selection polynomial : 2;   // OPT[3:2]
        pullup_config pullupConfig : 2;        // OPT[5:4]
    };
    struct hdq_pin_configuration_t
    {
        union
        {
            struct
            {
                hdq_pin_function function : 2;  // PINFXN[1:0]
                bool enablePulldown : 1;        // OPT[0]
                // clear: Driving high drives to HI-Z (unavailable with isActiveLow)
                // set: Driving high drives to selected regulator
                bool disableHighZDrive : 1;  // OPT[1]
                // NOTE: Should be cleared when isActiveLow is set
                bool pullupToReg1 : 1;  // OPT[2]
                // clear: High uses REG18
                // set: High uses REG1
                bool driveHighUsesReg1 : 1;  // OPT[3]
                bool _rsvd6 : 1;             // OPT[4]
                bool isActiveLow : 1;        // OPT[5]
            } function;
            struct
            {
                adc_pin_function function : 2;         // PINFXN[1:0]
                measurement_type measurementType : 2;  // OPT[1:0]
                polynomial_selection polynomial : 2;   // OPT[3:2]
                pullup_config pullupConfig : 2;        // OPT[5:4]
            } adc;
        };
    };
    struct dchg_pin_configuration_t
    {
        union
        {
            struct
            {
                dchg_pin_function function : 2;  // PINFXN[1:0]
                bool enablePulldown : 1;         // OPT[0]
                // clear: Driving high drives to HI-Z (unavailable with isActiveLow)
                // set: Driving high drives to selected regulator
                bool disableHighZDrive : 1;  // OPT[1]
                // NOTE: Should be cleared when isActiveLow is set
                bool pullupToReg1 : 1;  // OPT[2]
                // clear: High uses REG18
                // set: High uses REG1
                bool driveHighUsesReg1 : 1;  // OPT[3]
                bool _rsvd6 : 1;             // OPT[4]
                bool isActiveLow : 1;        // OPT[5]
            } function;
            struct
            {
                adc_pin_function function : 2;         // PINFXN[1:0]
                measurement_type measurementType : 2;  // OPT[1:0]
                polynomial_selection polynomial : 2;   // OPT[3:2]
                pullup_config pullupConfig : 2;        // OPT[5:4]
            } adc;
        };
    };
    struct ddsg_pin_configuration_t
    {
        union
        {
            struct
            {
                ddsg_pin_function function : 2;  // PINFXN[1:0]
                bool enablePulldown : 1;         // OPT[0]
                // clear: Driving high drives to HI-Z (unavailable with isActiveLow)
                // set: Driving high drives to selected regulator
                bool disableHighZDrive : 1;  // OPT[1]
                // NOTE: Should be cleared when isActiveLow is set
                bool pullupToReg1 : 1;  // OPT[2]
                // clear: High uses REG18
                // set: High uses REG1
                bool driveHighUsesReg1 : 1;  // OPT[3]
                bool _rsvd6 : 1 = 0;         // OPT[4]
                bool isActiveLow : 1;        // OPT[5]
            } function;
            struct
            {
                adc_pin_function function : 2;         // PINFXN[1:0]
                measurement_type measurementType : 2;  // OPT[1:0]
                polynomial_selection polynomial : 2;   // OPT[3:2]
                pullup_config pullupConfig : 2;        // OPT[5:4]
            } adc;
        };
    };
    struct da_configuration_t
    {
        user_amps userAmps : 2 = user_amps::MILLIAMP;  // USER_AMPS_[1:0]
        // clear: User volts is mV
        // set: User volts is cV (10mV)
        bool userVoltsIsCentivolts : 1 = 0;         // USER_VOLTS_CV
        bool useInternalAsCellTemperature : 1 = 0;  // TINT_EN
        bool useInternalAsFetTemperature : 1 = 0;   // TINT_FETT
        uint8_t _rsvd5 : 3 = 0;                     // RSVD_0
    };
    union selected_cells_t
    {
        struct
        {
            bool cell1 : 1;
            bool cell2 : 1;
            bool cell3 : 1;
            bool cell4 : 1;
            bool cell5 : 1;
            bool cell6 : 1;
            bool cell7 : 1;
            bool cell8 : 1;
            bool cell9 : 1;
            bool cell10 : 1;
            uint8_t _rsvd10 : 6;  // Unused, likely RSVD_0
        };
        uint16_t raw = 0x0000;
    };
    struct protection_configuration_t
    {
        bool _rsvd0 : 1 = 0;  // RSVD_0
        // if set, a PF will turn off the FETs
        bool permanentFailStopsFets : 1 = 1;  // PF_FETS
        // if set, a PF will turn off the regulators
        bool permanentFailStopsRegulators : 1 = 0;  // PF_REGS
        // if set, will enter deepsleep after writing OTP (if applicable) in a PF event
        bool permanentFailCausesDeepSleep : 1 = 0;  // PF_DPSLP
        // if set, will blow the fuse after a PF event
        bool permanentFailBlowsFuse : 1 = 0;  // PF_FUSE
        // if set, will write PF status to the OTP
        bool permanentFailWritesOtp : 1 = 0;  // PF_OTP
        bool _rsvd6 : 1 = 0;                  // RSVD_0
        // if set, will use PACK voltage instead of TOS (Top Of Stack)
        bool fuseVoltageUsesPackVoltage : 1 = 0;  // PACK_FUSE
        // a FET Permanent Failure will ignore the min blow fuse voltage if this is set
        bool fetFaultIgnoresFuseVoltage : 1 = 0;     // FETF_FUSE
        bool useOvercurrentChargeRecovery : 1 = 0;   // OCDL_CURR_RECOVERY
        bool useShortCircuitChargeRecovery : 1 = 0;  // SCDL_CURR_RECOVERY
        uint8_t _rsvd11 : 5 = 0;                     // RSVD_0
    };
    struct chg_fet_protections_a_t
    {
        uint8_t _rsvd0 : 3 = 0;          // RSVD_0
        bool cellOvervoltage : 1;        // COV
        bool overcurrentCharge : 1;      // OCC
        uint8_t _rsvd5 : 2 = 0;          // RSVD_0
        bool shortCircuitDischarge : 1;  // SCD
    };
    struct chg_fet_protections_b_t
    {
        bool undertempCharge : 1;    // UTC
        bool _rsvd1 : 1 = 0;         // RSVD_0
        bool internalUndertemp : 1;  // UTINT
        bool _rsvd3 : 1 = 0;         // RSVD_0
        bool overtempCharge : 1;     // OTC
        bool _rsvd5 : 1 = 0;         // RSVD_0
        bool internalOvertemp : 1;   // OTINT
        bool fetOvertemp : 1;        // OTF
    };
    struct chg_fet_protections_c_t
    {
        bool _rsvd0 : 1 = 0;                  // RSVD_0
        bool hostWatchdogFault : 1;           // HWDF
        bool prechargeTimeout : 1;            // PTO
        bool _rsvd3 : 1 = 0;                  // RSVD_0
        bool cellOvervoltageLatch : 1;        // COVL
        bool _rsvd5 : 1 = 0;                  // RSVD_0
        bool shortCircuitDischargeLatch : 1;  // SCDL
        bool _rsvd7 : 1 = 0;                  // RSVD_0
    };
    struct dsg_fet_protections_a_t
    {
        uint8_t _rsvd0 : 2 = 0;          // RSVD_0
        bool cellUndervoltage : 1;       // CUV
        uint8_t _rsvd3 : 2 = 0;          // RSVD_0
        bool overcurrentDischarge1 : 1;  // OCD1
        bool overcurrentDischarge2 : 1;  // OCD2
        bool shortCircuitDischarge : 1;  // SCD
    };
    struct dsg_fet_protections_b_t
    {
        bool _rsvd0 : 1 = 0;          // RSVD_0
        bool undertempDischarge : 1;  // UTD
        bool internalUndertemp : 1;   // UTINT
        uint8_t _rsvd3 : 2 = 0;       // RSVD_0
        bool overtempDischarge : 1;   // OTD
        bool internalOvertemp : 1;    // OTINT
        bool fetOvertemp : 1;         // OTF
    };
    struct dsg_fet_protections_c_t
    {
        bool _rsvd0 : 1 = 0;                  // RSVD_0
        bool hostWatchdogFault : 1;           // HWDF
        uint8_t _rsvd2 : 3 = 0;               // RSVD_0
        bool overcurrentDischargeLatch : 1;   // OCDL
        bool shortCircuitDischargeLatch : 1;  // SCDL
        bool overcurrentDischarge3 : 1;       // OCD3
    };
    struct alarm_sf_alert_mask_c_t
    {
        bool _rsvd0 : 1 = 0;                  // RSVD_0
        bool _rsvd1Set : 1 = 1;               // RSVD_1
        bool prechargeTimeout : 1;            // PTO
        bool _rsvd3 : 1 = 0;                  // RSVD_0
        bool cellOvervoltageLatch : 1;        // COVL
        bool overcurrentDischargeLatch : 1;   // OCDL
        bool shortCircuitDischargeLatch : 1;  // SCDL
        bool overcurrentDischarge3 : 1;       // OCD3
    };
    struct fet_options_t
    {
        bool enableBodyDiodeProtection : 1 = 1;   // SFET (Series FET)
        bool allowChargeInSleep : 1 = 0;          // SLEEPCHG
        bool allowHostFetControl : 1 = 1;         // HOST_FET_EN
        bool enableFetControl : 1 = 1;            // FET_CTRL_EN
        bool enablePredischarge : 1 = 0;          // PDSG_EN
        bool hostControlDefaultsFetsOff : 1 = 0;  // FET_INIT_OFF
        uint8_t _rsvd6 : 2 = 0;                   // RSVD_0
    };
    struct fet_charge_pump_control_t
    {
        // NOTE: If cleared, enableFetControl should likely also be cleared
        bool enableChargePump : 1;  // CP_EN
        // clear: Charge pump uses 11V overdrive
        // set: Charge pump uses 5.5V overdrive
        bool useLowOverdrive : 1;  // LVEN
        // NOTE: Normally only used when allowChargeInSleep is cleared
        bool enableSleepDsgSourceFollower : 1;  // SFMODE_SLEEP
        uint8_t : 5;                            // RSVD_0
    };
    struct manufacturing_status_init_t
    {
        uint8_t _rsvd0 : 4 = 0;           // RSVD_0
        bool autonomousFets : 1;          // FET_EN
        bool _rsvd6 : 1 = 0;              // RSVD_0
        bool isPermanentFailEnabled : 1;  // PF_EN
        bool isOtpWriteEnabled : 1;       // OTPW_EN
        uint8_t _rsvd8 : 8 = 0;           // RSVD_0
    };
    struct balancing_configuration_t
    {
        bool allowChargingCellBalancing : 1 = 0;     // CB_CHG
        bool allowRelaxedCellBalancing : 1 = 0;      // CB_RELAX
        bool allowBalancingInSleep : 1 = 0;          // CB_SLEEP
        bool balancingExitsSleep : 1 = 0;            // CB_NOSLEEP
        bool ignoreHostControlledBalancing : 1 = 0;  // CB_NO_CMD
        uint8_t _rsvd5 : 3 = 0;                      // RSVD_0
    };
    struct security_settings_t
    {
        bool defaultToSealed : 1;  // SEAL
        bool lockConfig : 1;       // LOCK_CFG
        bool permanentSeal : 1;    // PERM_SEAL
        uint8_t _rsvd3 : 5 = 0;    // RSVD_0
    };
#pragma pack(pop)

    typedef std::array<uint8_t, 32> manufacturer_data_t;

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
        float accumulatedCharge;
        std::chrono::seconds accumulatedChargeTime;
        int32_t cfetoffCounts;
        int32_t dfetoffCounts;
        int32_t alertCounts;
        int32_t ts1Counts;
        int32_t ts2Counts;
    };

    constexpr static auto SUBCOMMAND_TIMEOUT = std::chrono::milliseconds(20);
    constexpr static int MAX_RETRIES = 3;

    bq76942(uint8_t address = 0x08);

    // --- DIRECT COMMANDS ---
    control_status_t getControlStatus() { return readDirect<control_status_t>(direct_command::CONTROL_STATUS); }
    protections_a_t getSafetyAlertA() { return readDirect<protections_a_t>(direct_command::SAFETY_ALERT_A); }
    protections_a_t getSafetyStatusA() { return readDirect<protections_a_t>(direct_command::SAFETY_STATUS_A); }
    protections_b_t getSafetyAlertB() { return readDirect<protections_b_t>(direct_command::SAFETY_ALERT_B); }
    protections_b_t getSafetyStatusB() { return readDirect<protections_b_t>(direct_command::SAFETY_STATUS_B); }
    safety_alert_c_t getSafetyAlertC() { return readDirect<safety_alert_c_t>(direct_command::SAFETY_ALERT_C); }
    protections_c_t getSafetyStatusC() { return readDirect<protections_c_t>(direct_command::SAFETY_STATUS_C); }
    permanent_fail_a_t getPermanentFailAlertA() { return readDirect<permanent_fail_a_t>(direct_command::PF_ALERT_A); }
    permanent_fail_a_t getPermanentFailStatusA() { return readDirect<permanent_fail_a_t>(direct_command::PF_STATUS_A); }
    permanent_fail_b_t getPermanentFailAlertB() { return readDirect<permanent_fail_b_t>(direct_command::PF_ALERT_B); }
    permanent_fail_b_t getPermanentFailStatusB() { return readDirect<permanent_fail_b_t>(direct_command::PF_STATUS_B); }
    permanent_fail_alert_c_t getPermanentFailAlertC() { return readDirect<permanent_fail_alert_c_t>(direct_command::PF_ALERT_C); }
    permanent_fail_c_t getPermanentFailStatusC() { return readDirect<permanent_fail_c_t>(direct_command::PF_STATUS_C); }
    permanent_fail_d_t getPermanentFailAlertD() { return readDirect<permanent_fail_d_t>(direct_command::PF_ALERT_D); }
    permanent_fail_d_t getPermanentFailStatusD() { return readDirect<permanent_fail_d_t>(direct_command::PF_STATUS_D); }
    battery_status_t getBatteryStatus() { return readDirect<battery_status_t>(direct_command::BATTERY_STATUS); }
    /// @brief Read the voltage of one of the cells
    /// @return The voltage of the cell in mV
    int16_t getCellVoltage(const uint8_t cell);
    /// @brief Read the voltage at the top of the battery stack
    /// @return The voltage of the stack in mV
    int32_t getStackVoltage();
    int32_t getPackVoltage();
    int32_t getLdVoltage();
    float getCC2Current();
    alarm_status_t getAlarmStatus() { return readDirect<alarm_status_t>(direct_command::ALARM_STATUS); }
    void clearAlarmStatus(const alarm_status_t& alarmStatus) { writeDirect(direct_command::ALARM_STATUS, alarmStatus); }
    alarm_status_t getAlarmRawStatus() { return readDirect<alarm_status_t>(direct_command::ALARM_RAW_STATUS); }
    alarm_status_t getAlarmEnable() { return readDirect<alarm_status_t>(direct_command::ALARM_ENABLE); }
    void setAlarmEnable(const alarm_status_t& alarmEnable) { writeDirect(direct_command::ALARM_ENABLE, alarmEnable); }
    float getTemperature(const temperature_sensor& sensor) { return rawTempToCelsius(readDirect<int16_t>(static_cast<direct_command>(sensor))); }
    fet_status_t getFetStatus() { return readDirect<fet_status_t>(direct_command::FET_STATUS); }

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
    void exitConfigUpdateMode();
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
    uint16_t getDeviceNumber() { return readSubcommand<uint16_t>(subcommand::DEVICE_NUMBER); };
    firmware_version_t getFirmwareVersion();
    uint16_t getHardwareVersion() { return readSubcommand<uint16_t>(subcommand::HW_VERSION); };
    uint16_t getIROMSignature() { return readSubcommand<uint16_t>(subcommand::IROM_SIG); };
    uint16_t getStaticConfigSignature() { return readSubcommand<uint16_t>(subcommand::STATIC_CFG_SIG); };
    // Intended for TI internal use
    // uint16_t getPrevMacWrite() { return readSubcommand<uint16_t>(subcommand::PREV_MAC_WRITE); };
    uint16_t getDROMSignature() { return readSubcommand<uint16_t>(subcommand::DROM_SIG); };
    security_keys_t getSecurityKeys();
    void setSecurityKeys(const security_keys_t& keys);
    saved_pf_status_t getSavedPFStatus() { return readSubcommand<saved_pf_status_t>(subcommand::SAVED_PF_STATUS); };
    manufacturing_status_t getManufacturingStatus() { return readSubcommand<manufacturing_status_t>(subcommand::MANUFACTURING_STATUS); };
    manufacturer_data_t getManufacturerData();
    void setManufacturerData(const manufacturer_data_t& data);
    da_status_1_t getDAStatus1() { return readSubcommand<da_status_1_t>(subcommand::DA_STATUS_1); };
    da_status_2_t getDAStatus2() { return readSubcommand<da_status_2_t>(subcommand::DA_STATUS_2); };
    da_status_3_t getDAStatus3() { return readSubcommand<da_status_3_t>(subcommand::DA_STATUS_3); };
    raw_da_status_5_t getRawDAStatus5() { return readSubcommand<raw_da_status_5_t>(subcommand::DA_STATUS_5); };
    da_status_5_t getDAStatus5();
    raw_da_status_6_t getRawDAStatus6() { return readSubcommand<raw_da_status_6_t>(subcommand::DA_STATUS_6); };
    da_status_6_t getDAStatus6();
    da_status_7_t getDAStatus7() { return readSubcommand<da_status_7_t>(subcommand::DA_STATUS_7); };
    voltage_snapshot_t getCellUnderVoltSnapshot() { return readSubcommand<voltage_snapshot_t>(subcommand::CUV_SNAPSHOT); };
    voltage_snapshot_t getCellOverVoltSnapshot() { return readSubcommand<voltage_snapshot_t>(subcommand::COV_SNAPSHOT); };
    selected_cells_t getCBActiveCells() { return readSubcommand<selected_cells_t>(subcommand::CB_ACTIVE_CELLS); };
    void setCBActiveCells(const selected_cells_t& cells) { writeSubcommand(subcommand::CB_ACTIVE_CELLS, cells); }
    std::chrono::seconds readCellBalancingTime() { return std::chrono::seconds(readSubcommand<uint16_t>(subcommand::CB_STATUS_1)); };
    cb_status_2_t getCBStatus2() { return readSubcommand<cb_status_2_t>(subcommand::CB_STATUS_2); };
    cb_status_3_t getCBStatus3() { return readSubcommand<cb_status_3_t>(subcommand::CB_STATUS_3); };
    void setFetControl(const fet_control_t& control) { writeSubcommand(subcommand::FET_CONTROL, control); }
    void setRegulatorControl(const regulator_control_t& control) { writeSubcommand(subcommand::REG12_CONTROL, control); }
    otp_write_result_t getOtpWriteCheckResult() { return readSubcommand<otp_write_result_t>(subcommand::OTP_WR_CHECK); }
    otp_write_result_t getOtpWriteResult() { return readSubcommand<otp_write_result_t>(subcommand::OTP_WRITE); }
    cal1_t getCal1() { return readSubcommand<cal1_t>(subcommand::READ_CAL1); }
    uint16_t calibrateCellUnderVoltThreshold() { return readSubcommand<uint16_t>(subcommand::CAL_CUV); }
    uint16_t calibrateCellOverVoltThreshold() { return readSubcommand<uint16_t>(subcommand::CAL_COV); }

    // --- DATA REGISTERS ---
    class Calibration final
    {
    public:
        // Calibration:Voltage
        class Voltage
        {
        public:
            int16_t getCellGain(const uint8_t& cell) const;
            void setCellGain(const uint8_t& cell, const int16_t& gain) const;
            uint16_t getPackGain() const { return _parent.readSubcommand<uint16_t>(data_register::PACK_GAIN); }
            void setPackGain(const uint16_t& gain) const { _parent.writeSubcommand(data_register::PACK_GAIN, gain); }
            uint16_t getStackGain() const { return _parent.readSubcommand<uint16_t>(data_register::TOS_GAIN); }
            void setStackGain(const uint16_t& gain) const { _parent.writeSubcommand(data_register::TOS_GAIN, gain); }
            uint16_t getLdGain() const { return _parent.readSubcommand<uint16_t>(data_register::LD_GAIN); }
            void setLdGain(const uint16_t& gain) const { _parent.writeSubcommand(data_register::LD_GAIN, gain); }
            int16_t getAdcGain() const { return _parent.readSubcommand<int16_t>(data_register::ADC_GAIN); }
            void setAdcGain(const int16_t& gain) const { _parent.writeSubcommand(data_register::ADC_GAIN, gain); }

        protected:
            Voltage(bq76942& parent)
                : _parent(parent)
            {
            }

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
            /// @brief Sets CC gain and capacity gain based on the sense resistor value
            /// @param value Resistance in milliohms of the sense resistor
            void setSenseResistorValue(const float& value) const;
            float getCCGain() const { return _parent.readSubcommand<float>(data_register::CC_GAIN); }
            void setCCGain(const float& gain) const { _parent.writeSubcommand(data_register::CC_GAIN, gain); }
            float getCapacityGain() const { return _parent.readSubcommand<float>(data_register::CAPACITY_GAIN); }
            void setCapacityGain(const float& gain) const { _parent.writeSubcommand(data_register::CAPACITY_GAIN, gain); }

        protected:
            Current(bq76942& parent)
                : _parent(parent)
            {
            }

            Current(Current&) = delete;
            Current& operator=(Current&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        // Calibration:Vcell Offset
        int16_t getVcellOffset() const { return _parent.readSubcommand<int16_t>(data_register::VCELL_OFFSET); }
        void setVcellOffset(const int16_t& offset) const { _parent.writeSubcommand(data_register::VCELL_OFFSET, offset); }

        // Calibration:V Divider Offset
        int16_t getVDividerOffset() const { return _parent.readSubcommand<int16_t>(data_register::VDIV_OFFSET); }
        void setVDividerOffset(const int16_t& offset) const { _parent.writeSubcommand(data_register::VDIV_OFFSET, offset); }

        // Calibration:Current Offset
        class CurrentOffset final
        {
        public:
            uint16_t getCoulombCounterOffsetSamples() const { return _parent.readSubcommand<uint16_t>(data_register::COULOMB_COUNTER_OFFSET_SAMPLES); }
            void setCoulombCounterOffsetSamples(const uint16_t& samples) const { _parent.writeSubcommand(data_register::COULOMB_COUNTER_OFFSET_SAMPLES, samples); }
            int16_t getBoardOffsetCurrent() const { return _parent.readSubcommand<int16_t>(data_register::BOARD_OFFSET); }
            void setBoardOffsetCurrent(const int16_t& offset) const { _parent.writeSubcommand(data_register::BOARD_OFFSET, offset); }

        protected:
            CurrentOffset(bq76942& parent)
                : _parent(parent)
            {
            }

            CurrentOffset(CurrentOffset&) = delete;
            CurrentOffset& operator=(CurrentOffset&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        // Calibration:Temperature
        class Temperature final
        {
            int8_t getInternalOffset() const { return _parent.readSubcommand<int8_t>(data_register::INTERNAL_TEMP_OFFSET); }
            void setInternalOffset(const int8_t& offset) const { _parent.writeSubcommand(data_register::INTERNAL_TEMP_OFFSET, offset); }
            int8_t getCfetoffOffset() const { return _parent.readSubcommand<int8_t>(data_register::CFETOFF_TEMP_OFFSET); }
            void setCfetoffOffset(const int8_t& offset) const { _parent.writeSubcommand(data_register::CFETOFF_TEMP_OFFSET, offset); }
            int8_t getDfetoffOffset() const { return _parent.readSubcommand<int8_t>(data_register::DFETOFF_TEMP_OFFSET); }
            void setDfetoffOffset(const int8_t& offset) const { _parent.writeSubcommand(data_register::DFETOFF_TEMP_OFFSET, offset); }
            int8_t getAlertOffset() const { return _parent.readSubcommand<int8_t>(data_register::ALERT_TEMP_OFFSET); }
            void setAlertOffset(const int8_t& offset) const { _parent.writeSubcommand(data_register::ALERT_TEMP_OFFSET, offset); }
            int8_t getTs1Offset() const { return _parent.readSubcommand<int8_t>(data_register::TS1_TEMP_OFFSET); }
            void setTs1Offset(const int8_t& offset) const { _parent.writeSubcommand(data_register::TS1_TEMP_OFFSET, offset); }
            int8_t getTs2Offset() const { return _parent.readSubcommand<int8_t>(data_register::TS2_TEMP_OFFSET); }
            void setTs2Offset(const int8_t& offset) const { _parent.writeSubcommand(data_register::TS2_TEMP_OFFSET, offset); }
            int8_t getTs3Offset() const { return _parent.readSubcommand<int8_t>(data_register::TS3_TEMP_OFFSET); }
            void setTs3Offset(const int8_t& offset) const { _parent.writeSubcommand(data_register::TS3_TEMP_OFFSET, offset); }
            int8_t getHdqOffset() const { return _parent.readSubcommand<int8_t>(data_register::HDQ_TEMP_OFFSET); }
            void setHdqOffset(const int8_t& offset) const { _parent.writeSubcommand(data_register::HDQ_TEMP_OFFSET, offset); }
            int8_t getDchgOffset() const { return _parent.readSubcommand<int8_t>(data_register::DCHG_TEMP_OFFSET); }
            void setDchgOffset(const int8_t& offset) const { _parent.writeSubcommand(data_register::DCHG_TEMP_OFFSET, offset); }
            int8_t getDdsgOffset() const { return _parent.readSubcommand<int8_t>(data_register::DDSG_TEMP_OFFSET); }
            void setDdsgOffset(const int8_t& offset) const { _parent.writeSubcommand(data_register::DDSG_TEMP_OFFSET, offset); }

        protected:
            Temperature(bq76942& parent)
                : _parent(parent)
            {
            }

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
            int16_t getGain() const { return _parent.readSubcommand<int16_t>(data_register::INT_GAIN); }
            void setGain(const int16_t& gain) const { _parent.writeSubcommand(data_register::INT_GAIN, gain); }
            int16_t getBaseOffset() const { return _parent.readSubcommand<int16_t>(data_register::INT_BASE_OFFSET); }
            void setBaseOffset(const int16_t& offset) const { _parent.writeSubcommand(data_register::INT_BASE_OFFSET, offset); }
            int16_t getMaximumAD() const { return _parent.readSubcommand<int16_t>(data_register::INT_MAXIMUM_AD); }
            void setMaximumAD(const int16_t& ad) const { _parent.writeSubcommand(data_register::INT_MAXIMUM_AD, ad); }
            int16_t getMaximumTemp() const { return _parent.readSubcommand<int16_t>(data_register::INT_MAXIMUM_TEMP); }
            void setMaximumTemp(const int16_t& temp) const { _parent.writeSubcommand(data_register::INT_MAXIMUM_TEMP, temp); }

        protected:
            InternalTempModel(bq76942& parent)
                : _parent(parent)
            {
            }

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
            int16_t getCoeffA1() const { return _parent.readSubcommand<int16_t>(data_register::T18K_COEFF_A1); }
            void setCoeffA1(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T18K_COEFF_A1, coeff); }
            int16_t getCoeffA2() const { return _parent.readSubcommand<int16_t>(data_register::T18K_COEFF_A2); }
            void setCoeffA2(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T18K_COEFF_A2, coeff); }
            int16_t getCoeffA3() const { return _parent.readSubcommand<int16_t>(data_register::T18K_COEFF_A3); }
            void setCoeffA3(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T18K_COEFF_A3, coeff); }
            int16_t getCoeffA4() const { return _parent.readSubcommand<int16_t>(data_register::T18K_COEFF_A4); }
            void setCoeffA4(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T18K_COEFF_A4, coeff); }
            int16_t getCoeffA5() const { return _parent.readSubcommand<int16_t>(data_register::T18K_COEFF_A5); }
            void setCoeffA5(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T18K_COEFF_A5, coeff); }
            int16_t getCoeffB1() const { return _parent.readSubcommand<int16_t>(data_register::T18K_COEFF_B1); }
            void setCoeffB1(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T18K_COEFF_B1, coeff); }
            int16_t getCoeffB2() const { return _parent.readSubcommand<int16_t>(data_register::T18K_COEFF_B2); }
            void setCoeffB2(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T18K_COEFF_B2, coeff); }
            int16_t getCoeffB3() const { return _parent.readSubcommand<int16_t>(data_register::T18K_COEFF_B3); }
            void setCoeffB3(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T18K_COEFF_B3, coeff); }
            int16_t getCoeffB4() const { return _parent.readSubcommand<int16_t>(data_register::T18K_COEFF_B4); }
            void setCoeffB4(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T18K_COEFF_B4, coeff); }
            int16_t getAdc0() const { return _parent.readSubcommand<int16_t>(data_register::T18K_ADC0); }
            void setAdc0(const int16_t& adc) const { _parent.writeSubcommand(data_register::T18K_ADC0, adc); }

        protected:
            T18KModel(bq76942& parent)
                : _parent(parent)
            {
            }

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
            int16_t getCoeffA1() const { return _parent.readSubcommand<int16_t>(data_register::T180K_COEFF_A1); }
            void setCoeffA1(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T180K_COEFF_A1, coeff); }
            int16_t getCoeffA2() const { return _parent.readSubcommand<int16_t>(data_register::T180K_COEFF_A2); }
            void setCoeffA2(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T180K_COEFF_A2, coeff); }
            int16_t getCoeffA3() const { return _parent.readSubcommand<int16_t>(data_register::T180K_COEFF_A3); }
            void setCoeffA3(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T180K_COEFF_A3, coeff); }
            int16_t getCoeffA4() const { return _parent.readSubcommand<int16_t>(data_register::T180K_COEFF_A4); }
            void setCoeffA4(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T180K_COEFF_A4, coeff); }
            int16_t getCoeffA5() const { return _parent.readSubcommand<int16_t>(data_register::T180K_COEFF_A5); }
            void setCoeffA5(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T180K_COEFF_A5, coeff); }
            int16_t getCoeffB1() const { return _parent.readSubcommand<int16_t>(data_register::T180K_COEFF_B1); }
            void setCoeffB1(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T180K_COEFF_B1, coeff); }
            int16_t getCoeffB2() const { return _parent.readSubcommand<int16_t>(data_register::T180K_COEFF_B2); }
            void setCoeffB2(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T180K_COEFF_B2, coeff); }
            int16_t getCoeffB3() const { return _parent.readSubcommand<int16_t>(data_register::T180K_COEFF_B3); }
            void setCoeffB3(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T180K_COEFF_B3, coeff); }
            int16_t getCoeffB4() const { return _parent.readSubcommand<int16_t>(data_register::T180K_COEFF_B4); }
            void setCoeffB4(const int16_t& coeff) const { _parent.writeSubcommand(data_register::T180K_COEFF_B4, coeff); }
            int16_t getAdc0() const { return _parent.readSubcommand<int16_t>(data_register::T180K_ADC0); }
            void setAdc0(const int16_t& adc) const { _parent.writeSubcommand(data_register::T180K_ADC0, adc); }

        protected:
            T180KModel(bq76942& parent)
                : _parent(parent)
            {
            }

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
            int16_t getCoeffA1() const { return _parent.readSubcommand<int16_t>(data_register::CUSTOM_COEFF_A1); }
            void setCoeffA1(const int16_t& coeff) const { _parent.writeSubcommand(data_register::CUSTOM_COEFF_A1, coeff); }
            int16_t getCoeffA2() const { return _parent.readSubcommand<int16_t>(data_register::CUSTOM_COEFF_A2); }
            void setCoeffA2(const int16_t& coeff) const { _parent.writeSubcommand(data_register::CUSTOM_COEFF_A2, coeff); }
            int16_t getCoeffA3() const { return _parent.readSubcommand<int16_t>(data_register::CUSTOM_COEFF_A3); }
            void setCoeffA3(const int16_t& coeff) const { _parent.writeSubcommand(data_register::CUSTOM_COEFF_A3, coeff); }
            int16_t getCoeffA4() const { return _parent.readSubcommand<int16_t>(data_register::CUSTOM_COEFF_A4); }
            void setCoeffA4(const int16_t& coeff) const { _parent.writeSubcommand(data_register::CUSTOM_COEFF_A4, coeff); }
            int16_t getCoeffA5() const { return _parent.readSubcommand<int16_t>(data_register::CUSTOM_COEFF_A5); }
            void setCoeffA5(const int16_t& coeff) const { _parent.writeSubcommand(data_register::CUSTOM_COEFF_A5, coeff); }
            int16_t getCoeffB1() const { return _parent.readSubcommand<int16_t>(data_register::CUSTOM_COEFF_B1); }
            void setCoeffB1(const int16_t& coeff) const { _parent.writeSubcommand(data_register::CUSTOM_COEFF_B1, coeff); }
            int16_t getCoeffB2() const { return _parent.readSubcommand<int16_t>(data_register::CUSTOM_COEFF_B2); }
            void setCoeffB2(const int16_t& coeff) const { _parent.writeSubcommand(data_register::CUSTOM_COEFF_B2, coeff); }
            int16_t getCoeffB3() const { return _parent.readSubcommand<int16_t>(data_register::CUSTOM_COEFF_B3); }
            void setCoeffB3(const int16_t& coeff) const { _parent.writeSubcommand(data_register::CUSTOM_COEFF_B3, coeff); }
            int16_t getCoeffB4() const { return _parent.readSubcommand<int16_t>(data_register::CUSTOM_COEFF_B4); }
            void setCoeffB4(const int16_t& coeff) const { _parent.writeSubcommand(data_register::CUSTOM_COEFF_B4, coeff); }
            int16_t getRc0() const { return _parent.readSubcommand<int16_t>(data_register::CUSTOM_RC0); }
            void setRc0(const int16_t& rc) const { _parent.writeSubcommand(data_register::CUSTOM_RC0, rc); }
            int16_t getAdc0() const { return _parent.readSubcommand<int16_t>(data_register::CUSTOM_ADC0); }
            void setAdc0(const int16_t& adc) const { _parent.writeSubcommand(data_register::CUSTOM_ADC0, adc); }

        protected:
            CustomTemperatureModel(bq76942& parent)
                : _parent(parent)
            {
            }

            CustomTemperatureModel(const CustomTemperatureModel&) = delete;
            CustomTemperatureModel& operator=(const CustomTemperatureModel&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        // Calibration:Current Deadband
        int16_t getCoulombCounterDeadband() const { return _parent.readSubcommand<int16_t>(data_register::COULOMB_COUNTER_DEADBAND); }
        void setCoulombCounterDeadband(const int16_t& deadband) const { _parent.writeSubcommand(data_register::COULOMB_COUNTER_DEADBAND, deadband); }

        // Calibration:CUV
        uint16_t getCellUnderVoltThresholdOverride() const { return _parent.readSubcommand<uint16_t>(data_register::CUV_THRESHOLD_OVERRIDE); }
        void setCellUnderVoltThresholdOverride(const uint16_t& threshold) const { _parent.writeSubcommand(data_register::CUV_THRESHOLD_OVERRIDE, threshold); }

        // Calibration:COV
        uint16_t getCellOverVoltThresholdOverride() const { return _parent.readSubcommand<uint16_t>(data_register::COV_THRESHOLD_OVERRIDE); }
        void setCellOverVoltThresholdOverride(const uint16_t& threshold) const { _parent.writeSubcommand(data_register::COV_THRESHOLD_OVERRIDE, threshold); }

        const Voltage voltage;
        const Current current;
        const CurrentOffset currentOffset;
        const Temperature temperature;
        const InternalTempModel internalTempModel;
        const T18KModel t18KModel;
        const T180KModel t180KModel;
        const CustomTemperatureModel customTemperatureModel;

    protected:
        Calibration(bq76942& parent)
            : voltage(parent),
              current(parent),
              currentOffset(parent),
              temperature(parent),
              internalTempModel(parent),
              t18KModel(parent),
              t180KModel(parent),
              customTemperatureModel(parent),
              _parent(parent)
        {
        }

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
            int16_t getMinBlowVoltage() const { return _parent.readSubcommand<int16_t>(data_register::MIN_BLOW_FUSE_VOLTAGE); }
            void setMinBlowVoltage(const int16_t& voltage) const { _parent.writeSubcommand(data_register::MIN_BLOW_FUSE_VOLTAGE, voltage); }
            std::chrono::seconds readBlowTimeout() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::FUSE_BLOW_TIMEOUT)); }
            void setBlowTimeout(const std::chrono::seconds& timeout) const { _parent.writeSubcommand<uint8_t>(data_register::FUSE_BLOW_TIMEOUT, timeout.count()); }

        protected:
            Fuse(bq76942& parent)
                : _parent(parent)
            {
            }

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
            power_config_t getPowerConfig() const { return _parent.readSubcommand<power_config_t>(data_register::POWER_CONFIG); }
            void setPowerConfig(const power_config_t& config) const { _parent.writeSubcommand(data_register::POWER_CONFIG, config); }
            regulator_control_t getReg12Config() const { return _parent.readSubcommand<regulator_control_t>(data_register::REG12_CONFIG); }
            void setReg12Config(const regulator_control_t& config) const { _parent.writeSubcommand(data_register::REG12_CONFIG, config); }
            reg0_config_t getReg0Config() const { return _parent.readSubcommand<reg0_config_t>(data_register::REG0_CONFIG); }
            void setReg0Config(const reg0_config_t& config) const { _parent.writeSubcommand(data_register::REG0_CONFIG, config); }
            hwd_regulator_options_t getHwdRegulatorOptions() const { return _parent.readSubcommand<hwd_regulator_options_t>(data_register::HWD_REGULATOR_OPTIONS); }
            void setHwdRegulatorOptions(const hwd_regulator_options_t& options) const { _parent.writeSubcommand(data_register::HWD_REGULATOR_OPTIONS, options); }
            comm_type getCommType() const { return static_cast<comm_type>(_parent.readSubcommand<uint8_t>(data_register::COMM_TYPE)); }
            // note: This will only apply on reset OR SWAP_COMM_MODE
            void setCommType(const comm_type& type) const { _parent.writeSubcommand(data_register::COMM_TYPE, static_cast<uint8_t>(type)); }
            uint8_t getI2CAddress() const { return _parent.readSubcommand<uint8_t>(data_register::I2C_ADDRESS); }
            // note: This will only apply on reset OR SWAP_COMM_MODE
            void setI2CAddress(const uint8_t& address) const { _parent.writeSubcommand(data_register::I2C_ADDRESS, address); }
            spi_configuration_t getSpiConfiguration() const { return _parent.readSubcommand<spi_configuration_t>(data_register::SPI_CONFIGURATION); }
            void setSpiConfiguration(const spi_configuration_t& config) const { _parent.writeSubcommand(data_register::SPI_CONFIGURATION, config); }
            std::chrono::seconds readCommIdleTime() const { return std::chrono::seconds(_parent.readSubcommand<uint16_t>(data_register::COMM_IDLE_TIME)); }
            void setCommIdleTime(const std::chrono::seconds& time) const { _parent.writeSubcommand(data_register::COMM_IDLE_TIME, time.count()); }
            cfetoff_pin_configuration_t getCfetoffPinConfig() const { return _parent.readSubcommand<cfetoff_pin_configuration_t>(data_register::CFETOFF_PIN_CONFIG); }
            void setCfetoffPinConfig(const cfetoff_pin_configuration_t& config) const { _parent.writeSubcommand(data_register::CFETOFF_PIN_CONFIG, config); }
            dfetoff_pin_configuration_t getDfetoffPinConfig() const { return _parent.readSubcommand<dfetoff_pin_configuration_t>(data_register::DFETOFF_PIN_CONFIG); }
            void setDfetoffPinConfig(const dfetoff_pin_configuration_t& config) const { _parent.writeSubcommand(data_register::DFETOFF_PIN_CONFIG, config); }
            alert_pin_configuration_t getAlertPinConfig() const { return _parent.readSubcommand<alert_pin_configuration_t>(data_register::ALERT_PIN_CONFIG); }
            void setAlertPinConfig(const alert_pin_configuration_t& config) const { _parent.writeSubcommand(data_register::ALERT_PIN_CONFIG, config); }
            ts_pin_configuration_t getTs1PinConfig() const { return _parent.readSubcommand<ts_pin_configuration_t>(data_register::TS1_CONFIG); }
            void setTs1PinConfig(const ts_pin_configuration_t& config) const { _parent.writeSubcommand(data_register::TS1_CONFIG, config); }
            ts_pin_configuration_t getTs2PinConfig() const { return _parent.readSubcommand<ts_pin_configuration_t>(data_register::TS2_CONFIG); }
            void setTs2PinConfig(const ts_pin_configuration_t& config) const { _parent.writeSubcommand(data_register::TS2_CONFIG, config); }
            ts_pin_configuration_t getTs3PinConfig() const { return _parent.readSubcommand<ts_pin_configuration_t>(data_register::TS3_CONFIG); }
            void setTs3PinConfig(const ts_pin_configuration_t& config) const { _parent.writeSubcommand(data_register::TS3_CONFIG, config); }
            hdq_pin_configuration_t getHdqPinConfig() const { return _parent.readSubcommand<hdq_pin_configuration_t>(data_register::HDQ_PIN_CONFIG); }
            void setHdqPinConfig(const hdq_pin_configuration_t& config) const { _parent.writeSubcommand(data_register::HDQ_PIN_CONFIG, config); }
            dchg_pin_configuration_t getDchgPinConfig() const { return _parent.readSubcommand<dchg_pin_configuration_t>(data_register::DCHG_PIN_CONFIG); }
            void setDchgPinConfig(const dchg_pin_configuration_t& config) const { _parent.writeSubcommand(data_register::DCHG_PIN_CONFIG, config); }
            ddsg_pin_configuration_t getDdsgPinConfig() const { return _parent.readSubcommand<ddsg_pin_configuration_t>(data_register::DDSG_PIN_CONFIG); }
            void setDdsgPinConfig(const ddsg_pin_configuration_t& config) const { _parent.writeSubcommand(data_register::DDSG_PIN_CONFIG, config); }
            da_configuration_t getDAConfiguration() const { return _parent.readSubcommand<da_configuration_t>(data_register::DA_CONFIGURATION); }
            void setDAConfiguration(const da_configuration_t& config) const { _parent.writeSubcommand(data_register::DA_CONFIGURATION, config); }
            selected_cells_t getVcellMode() const { return _parent.readSubcommand<selected_cells_t>(data_register::VCELL_MODE); }
            void setVcellMode(const selected_cells_t& mode) const { _parent.writeSubcommand(data_register::VCELL_MODE, mode); }
            uint8_t getCC3Samples() const { return _parent.readSubcommand<uint8_t>(data_register::CC3_SAMPLES); }
            void setCC3Samples(const uint8_t& samples) const { _parent.writeSubcommand(data_register::CC3_SAMPLES, samples); }

        protected:
            Configuration(bq76942& parent)
                : _parent(parent)
            {
            }
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
            protection_configuration_t getConfig() const { return _parent.readSubcommand<protection_configuration_t>(data_register::PROTECTION_CONFIGURATION); }
            void setConfig(const protection_configuration_t& config) const { _parent.writeSubcommand(data_register::PROTECTION_CONFIGURATION, config); }
            protections_a_t getEnabledA() const { return _parent.readSubcommand<protections_a_t>(data_register::ENABLED_PROTECTIONS_A); }
            void setEnabledA(const protections_a_t& protections) const { _parent.writeSubcommand(data_register::ENABLED_PROTECTIONS_A, protections); }
            protections_b_t getEnabledB() const { return _parent.readSubcommand<protections_b_t>(data_register::ENABLED_PROTECTIONS_B); }
            void setEnabledB(const protections_b_t& protections) const { _parent.writeSubcommand(data_register::ENABLED_PROTECTIONS_B, protections); }
            protections_c_t getEnabledC() const { return _parent.readSubcommand<protections_c_t>(data_register::ENABLED_PROTECTIONS_C); }
            void setEnabledC(const protections_c_t& protections) const { _parent.writeSubcommand(data_register::ENABLED_PROTECTIONS_C, protections); }
            chg_fet_protections_a_t getChgFetA() const { return _parent.readSubcommand<chg_fet_protections_a_t>(data_register::CHG_FET_PROTECTIONS_A); }
            void setChgFetA(const chg_fet_protections_a_t& protections) const { _parent.writeSubcommand(data_register::CHG_FET_PROTECTIONS_A, protections); }
            chg_fet_protections_b_t getChgFetB() const { return _parent.readSubcommand<chg_fet_protections_b_t>(data_register::CHG_FET_PROTECTIONS_B); }
            void setChgFetB(const chg_fet_protections_b_t& protections) const { _parent.writeSubcommand(data_register::CHG_FET_PROTECTIONS_B, protections); }
            chg_fet_protections_c_t getChgFetC() const { return _parent.readSubcommand<chg_fet_protections_c_t>(data_register::CHG_FET_PROTECTIONS_C); }
            void setChgFetC(const chg_fet_protections_c_t& protections) const { _parent.writeSubcommand(data_register::CHG_FET_PROTECTIONS_C, protections); }
            dsg_fet_protections_a_t getDsgFetA() const { return _parent.readSubcommand<dsg_fet_protections_a_t>(data_register::DSG_FET_PROTECTIONS_A); }
            void setDsgFetA(const dsg_fet_protections_a_t& protections) const { _parent.writeSubcommand(data_register::DSG_FET_PROTECTIONS_A, protections); }
            dsg_fet_protections_b_t getDsgFetB() const { return _parent.readSubcommand<dsg_fet_protections_b_t>(data_register::DSG_FET_PROTECTIONS_B); }
            void setDsgFetB(const dsg_fet_protections_b_t& protections) const { _parent.writeSubcommand(data_register::DSG_FET_PROTECTIONS_B, protections); }
            dsg_fet_protections_c_t getDsgFetC() const { return _parent.readSubcommand<dsg_fet_protections_c_t>(data_register::DSG_FET_PROTECTIONS_C); }
            void setDsgFetC(const dsg_fet_protections_c_t& protections) const { _parent.writeSubcommand(data_register::DSG_FET_PROTECTIONS_C, protections); }
            int16_t getBodyDiodeThreshold() const { return _parent.readSubcommand<int16_t>(data_register::BODY_DIODE_THRESHOLD); }
            void setBodyDiodeThreshold(const int16_t& threshold) const { _parent.writeSubcommandClamped<int16_t>(data_register::BODY_DIODE_THRESHOLD, threshold, 0, std::numeric_limits<int16_t>::max()); }

        protected:
            Protection(bq76942& parent)
                : _parent(parent)
            {
            }
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
            alarm_status_t getDefaultMask() const { return _parent.readSubcommand<alarm_status_t>(data_register::DEFAULT_ALARM_MASK); }
            void setDefaultMask(const alarm_status_t& mask) const { _parent.writeSubcommand(data_register::DEFAULT_ALARM_MASK, mask); }
            protections_a_t getSafetyAlertMaskA() const { return _parent.readSubcommand<protections_a_t>(data_register::SF_ALERT_MASK_A); }
            void setSafetyAlertMaskA(const protections_a_t& protections) const { _parent.writeSubcommand(data_register::SF_ALERT_MASK_A, protections); }
            protections_b_t getSafetyAlertMaskB() const { return _parent.readSubcommand<protections_b_t>(data_register::SF_ALERT_MASK_B); }
            void setSafetyAlertMaskB(const protections_b_t& protections) const { _parent.writeSubcommand(data_register::SF_ALERT_MASK_B, protections); }
            alarm_sf_alert_mask_c_t getSafetyAlertMaskC() const { return _parent.readSubcommand<alarm_sf_alert_mask_c_t>(data_register::SF_ALERT_MASK_C); }
            void setSafetyAlertMaskC(alarm_sf_alert_mask_c_t protections) const { _parent.writeSubcommand(data_register::SF_ALERT_MASK_C, protections); }

            permanent_fail_a_t getPermanentFailMaskA() const { return _parent.readSubcommand<permanent_fail_a_t>(data_register::PF_ALERT_MASK_A); }
            void setPermanentFailMaskA(const permanent_fail_a_t& failures) const { _parent.writeSubcommand(data_register::PF_ALERT_MASK_A, failures); }
            permanent_fail_b_t getPermanentFailMaskB() const { return _parent.readSubcommand<permanent_fail_b_t>(data_register::PF_ALERT_MASK_B); }
            void setPermanentFailMaskB(const permanent_fail_b_t& failures) const { _parent.writeSubcommand(data_register::PF_ALERT_MASK_B, failures); }
            permanent_fail_c_t getPermanentFailMaskC() const { return _parent.readSubcommand<permanent_fail_c_t>(data_register::PF_ALERT_MASK_C); }
            void setPermanentFailMaskC(const permanent_fail_c_t& failures) const { _parent.writeSubcommand(data_register::PF_ALERT_MASK_C, failures); }
            permanent_fail_d_t getPermanentFailMaskD() const { return _parent.readSubcommand<permanent_fail_d_t>(data_register::PF_ALERT_MASK_D); }
            void setPermanentFailMaskD(const permanent_fail_d_t& failures) const { _parent.writeSubcommand(data_register::PF_ALERT_MASK_D, failures); }

        protected:
            Alarm(bq76942& parent)
                : _parent(parent)
            {
            }
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
            permanent_fail_a_t getEnabledA() const { return _parent.readSubcommand<permanent_fail_a_t>(data_register::ENABLED_PF_A); }
            void setEnabledA(const permanent_fail_a_t& failures) const { _parent.writeSubcommand(data_register::ENABLED_PF_A, failures); }
            permanent_fail_b_t getEnabledB() const { return _parent.readSubcommand<permanent_fail_b_t>(data_register::ENABLED_PF_B); }
            void setEnabledB(const permanent_fail_b_t& failures) const { _parent.writeSubcommand(data_register::ENABLED_PF_B, failures); }
            permanent_fail_c_t getEnabledC() const { return _parent.readSubcommand<permanent_fail_c_t>(data_register::ENABLED_PF_C); }
            void setEnabledC(const permanent_fail_c_t& failures) const { _parent.writeSubcommand(data_register::ENABLED_PF_C, failures); }
            permanent_fail_d_t getEnabledD() const { return _parent.readSubcommand<permanent_fail_d_t>(data_register::ENABLED_PF_D); }
            void setEnabledD(const permanent_fail_d_t& failures) const { _parent.writeSubcommand(data_register::ENABLED_PF_D, failures); }

        protected:
            PermanentFailure(bq76942& parent)
                : _parent(parent)
            {
            }
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
            fet_options_t getOptions() const { return _parent.readSubcommand<fet_options_t>(data_register::FET_OPTIONS); }
            void setOptions(const fet_options_t& options) const { _parent.writeSubcommand(data_register::FET_OPTIONS, options); }
            fet_charge_pump_control_t getChargePumpControl() const { return _parent.readSubcommand<fet_charge_pump_control_t>(data_register::CHG_PUMP_CONTROL); }
            void setChargePumpControl(const fet_charge_pump_control_t& control) const { _parent.writeSubcommand(data_register::CHG_PUMP_CONTROL, control); }
            int16_t getPrechargeStartVoltage() const { return _parent.readSubcommand<int16_t>(data_register::PRECHARGE_START_VOLTAGE); }
            void setPrechargeStartVoltage(const int16_t& voltage) const { _parent.writeSubcommandClamped<int16_t>(data_register::PRECHARGE_START_VOLTAGE, voltage, 0, std::numeric_limits<int16_t>::max()); }
            int16_t getPrechargeStopVoltage() const { return _parent.readSubcommand<int16_t>(data_register::PRECHARGE_STOP_VOLTAGE); }
            void setPrechargeStopVoltage(const int16_t& voltage) const { _parent.writeSubcommandClamped<int16_t>(data_register::PRECHARGE_STOP_VOLTAGE, voltage, 0, std::numeric_limits<int16_t>::max()); }
            std::chrono::milliseconds readPredischargeTimeout() const { return std::chrono::milliseconds(_parent.readSubcommand<uint16_t>(data_register::PREDISCHARGE_TIMEOUT) * 10); }
            void setPredischargeTimeout(const std::chrono::milliseconds& timeout) const
            {
                uint8_t roundedTimeout = static_cast<uint8_t>(std::round(timeout.count() / 10.0));
                _parent.writeSubcommand(data_register::PREDISCHARGE_TIMEOUT, roundedTimeout);
            }
            uint16_t getPredischargeStopDelta() const { return static_cast<uint16_t>(_parent.readSubcommand<uint8_t>(data_register::PREDISCHARGE_STOP_DELTA)) * 10; }
            void setPredischargeStopDelta(uint16_t voltage) const { _parent.writeSubcommand(data_register::PREDISCHARGE_STOP_DELTA, static_cast<uint8_t>(voltage / 10)); }

        protected:
            Fet(bq76942& parent)
                : _parent(parent)
            {
            }
            Fet(Fet&) = delete;
            Fet& operator=(Fet&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        // Settings:Current Thresholds
        uint32_t getDsgCurrentThreshold() const;
        void setDsgCurrentThreshold(const uint32_t& threshold) const;
        uint32_t getChgCurrentThreshold() const;
        void setChgCurrentThreshold(const uint32_t& threshold) const;

        // Settings:Cell Open-Wire
        std::chrono::seconds readCellOpenWireCheckTime() const { return std::chrono::seconds(_parent.readSubcommand<uint16_t>(data_register::CHECK_TIME)); }
        void setCellOpenWireCheckTime(const std::chrono::seconds& time) const { _parent.writeSubcommand(data_register::CHECK_TIME, static_cast<uint8_t>(time.count())); }

        // Settings:Interconnect Resistances
        int16_t getCellInterconnectResistance(const uint8_t& cell);
        void setCellInterconnectResistance(const uint8_t& cell, const int16_t& resistance);

        // Settings:Manufacturing
        manufacturing_status_init_t getManufacturingStatusInit() const { return _parent.readSubcommand<manufacturing_status_init_t>(data_register::MFG_STATUS_INIT); }
        void setManufacturingStatusInit(const manufacturing_status_init_t& status) const { _parent.writeSubcommand(data_register::MFG_STATUS_INIT, status); }

        // Settings:Cell Balancing Config
        class CellBalancing final
        {
        public:
            balancing_configuration_t getConfig() const { return _parent.readSubcommand<balancing_configuration_t>(data_register::BALANCING_CONFIGURATION); }
            void setConfig(const balancing_configuration_t& config) const { _parent.writeSubcommand(data_register::BALANCING_CONFIGURATION, config); }
            int8_t getMinCellTemp() const { return _parent.readSubcommand<int8_t>(data_register::MIN_CELL_TEMP); }
            void setMinCellTemp(const int8_t& temp) const { _parent.writeSubcommand(data_register::MIN_CELL_TEMP, temp); }
            int8_t getMaxInternalTemp() const { return _parent.readSubcommand<int8_t>(data_register::MAX_INTERNAL_TEMP); }
            void setMaxInternalTemp(const int8_t& temp) const { _parent.writeSubcommand(data_register::MAX_INTERNAL_TEMP, temp); }
            std::chrono::seconds readInterval() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::CELL_BALANCE_INTERVAL)); }
            void setInterval(const std::chrono::seconds& interval) const { _parent.writeSubcommandClamped<uint8_t>(data_register::CELL_BALANCE_INTERVAL, static_cast<uint8_t>(interval.count()), 1, 255); }
            uint8_t getMaxCells() const { return _parent.readSubcommand<uint8_t>(data_register::CELL_BALANCE_MAX_CELLS); }
            void setMaxCells(const uint8_t& cells) const { _parent.writeSubcommandClamped<uint8_t>(data_register::CELL_BALANCE_MAX_CELLS, cells, 0, 16); }
            int16_t getMinCellVCharge() const { return _parent.readSubcommand<int16_t>(data_register::CELL_BALANCE_MIN_CELL_V_CHARGE); }
            void setMinCellVCharge(const int16_t& voltage) const { _parent.writeSubcommandClamped<int16_t>(data_register::CELL_BALANCE_MIN_CELL_V_CHARGE, voltage, 0, 5000); }
            uint8_t getMinCellDeltaCharge() const { return _parent.readSubcommand<uint8_t>(data_register::CELL_BALANCE_MIN_DELTA_CHARGE); }
            void setMinCellDeltaCharge(const uint8_t& voltage) const { _parent.writeSubcommand(data_register::CELL_BALANCE_MIN_DELTA_CHARGE, voltage); }
            uint8_t getStopDeltaCharge() const { return _parent.readSubcommand<uint8_t>(data_register::CELL_BALANCE_STOP_DELTA_CHARGE); }
            void setStopDeltaCharge(const uint8_t& voltage) const { _parent.writeSubcommand(data_register::CELL_BALANCE_STOP_DELTA_CHARGE, voltage); }
            int16_t getMinCellVRelax() const { return _parent.readSubcommand<int16_t>(data_register::CELL_BALANCE_MIN_CELL_V_RELAX); }
            void setMinCellVRelax(const int16_t& voltage) const { _parent.writeSubcommandClamped<int16_t>(data_register::CELL_BALANCE_MIN_CELL_V_RELAX, voltage, 0, 5000); }
            uint8_t getMinCellDeltaRelax() const { return _parent.readSubcommand<uint8_t>(data_register::CELL_BALANCE_MIN_DELTA_RELAX); }
            void setMinCellDeltaRelax(const uint8_t& voltage) const { _parent.writeSubcommand(data_register::CELL_BALANCE_MIN_DELTA_RELAX, voltage); }
            uint8_t getStopDeltaRelax() const { return _parent.readSubcommand<uint8_t>(data_register::CELL_BALANCE_STOP_DELTA_RELAX); }
            void setStopDeltaRelax(const uint8_t& voltage) const { _parent.writeSubcommand(data_register::CELL_BALANCE_STOP_DELTA_RELAX, voltage); }

        protected:
            CellBalancing(bq76942& parent)
                : _parent(parent)
            {
            }
            CellBalancing(CellBalancing&) = delete;
            CellBalancing& operator=(CellBalancing&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        const Fuse fuse;
        const Configuration configuration;
        const Protection protection;
        const Alarm alarm;
        const PermanentFailure permanentFailure;
        const Fet fet;
        const CellBalancing cellBalancing;

    protected:
        Settings(bq76942& parent)
            : fuse(parent),
              configuration(parent),
              protection(parent),
              alarm(parent),
              permanentFailure(parent),
              fet(parent),
              cellBalancing(parent),
              _parent(parent)
        {
        }

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
            int16_t getCellVoltage() const { return _parent.readSubcommand<int16_t>(data_register::SHUTDOWN_CELL_VOLTAGE); }
            void setCellVoltage(const int16_t& voltage) const { _parent.writeSubcommandClamped<int16_t>(data_register::SHUTDOWN_CELL_VOLTAGE, voltage, 0, std::numeric_limits<int16_t>::max()); }
            int32_t getStackVoltage() const { return static_cast<int32_t>(_parent.readSubcommand<int16_t>(data_register::SHUTDOWN_STACK_VOLTAGE)) * 10; }
            void setStackVoltage(const int32_t& voltage) const { _parent.writeSubcommand(data_register::SHUTDOWN_STACK_VOLTAGE, static_cast<int16_t>(voltage / 10)); }

            std::chrono::seconds getLowVDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::LOW_V_SHUTDOWN_DELAY)); }
            void setLowVDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommandClamped<uint8_t>(data_register::LOW_V_SHUTDOWN_DELAY, static_cast<uint8_t>(delay.count()), 0, 63); };
            uint8_t getTemperature() const { return _parent.readSubcommand<uint8_t>(data_register::SHUTDOWN_TEMPERATURE); }
            void setTemperature(const uint8_t& temperature) const { _parent.writeSubcommandClamped<uint8_t>(data_register::SHUTDOWN_TEMPERATURE, temperature, 0, 255); }
            std::chrono::seconds getTemperatureDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::SHUTDOWN_TEMPERATURE_DELAY)); }
            void setTemperatureDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommandClamped<uint8_t>(data_register::SHUTDOWN_TEMPERATURE_DELAY, static_cast<uint8_t>(delay.count()), 0, 254); }
            std::chrono::milliseconds getFetOffDelay() const { return std::chrono::milliseconds(_parent.readSubcommand<uint8_t>(data_register::FET_OFF_DELAY) * 250); }
            void setFetOffDelay(const std::chrono::milliseconds& delay) const
            {
                const uint8_t roundedDelay = static_cast<uint8_t>(std::round(delay.count() / 250.0));
                _parent.writeSubcommandClamped<uint8_t>(data_register::FET_OFF_DELAY, roundedDelay, 0, 127);
            }
            std::chrono::milliseconds getCommandDelay() const { return std::chrono::milliseconds(_parent.readSubcommand<uint8_t>(data_register::SHUTDOWN_COMMAND_DELAY) * 250); }
            void setCommandDelay(const std::chrono::milliseconds& delay) const
            {
                const uint8_t roundedDelay = static_cast<uint8_t>(std::round(delay.count() / 250.0));
                _parent.writeSubcommandClamped<uint8_t>(data_register::SHUTDOWN_COMMAND_DELAY, roundedDelay, 0, 254);
            }
            std::chrono::minutes getAutoShutdownTime() const { return std::chrono::minutes(_parent.readSubcommand<uint8_t>(data_register::AUTO_SHUTDOWN_TIME)); }
            void setAutoShutdownTime(const std::chrono::minutes& time) const { _parent.writeSubcommandClamped<uint8_t>(data_register::AUTO_SHUTDOWN_TIME, static_cast<uint8_t>(time.count()), 0, 250); }
            std::chrono::seconds getRamFailShutdownTime() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::RAM_FAIL_SHUTDOWN_TIME)); }
            void setRamFailShutdownTime(const std::chrono::seconds& time) const { _parent.writeSubcommand<uint8_t>(data_register::RAM_FAIL_SHUTDOWN_TIME, static_cast<uint8_t>(time.count())); }

        protected:
            Shutdown(bq76942& parent)
                : _parent(parent)
            {
            }
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
            int16_t getCurrent() const { return _parent.readSubcommand<int16_t>(data_register::SLEEP_CURRENT); }
            void setCurrent(const int16_t& current) const { _parent.writeSubcommandClamped<int16_t>(data_register::SLEEP_CURRENT, current, 0, std::numeric_limits<int16_t>::max()); }
            std::chrono::seconds getVoltageReadDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::VOLTAGE_TIME)); }
            void setVoltageReadDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommandClamped<uint8_t>(data_register::VOLTAGE_TIME, static_cast<uint8_t>(delay.count()), 1, 255); }
            int16_t getWakeComparatorCurrent() const { return _parent.readSubcommand<int16_t>(data_register::WAKE_COMPARATOR_CURRENT); }
            void setWakeComparatorCurrent(const int16_t& current) const { _parent.writeSubcommandClamped<int16_t>(data_register::WAKE_COMPARATOR_CURRENT, current, 500, std::numeric_limits<int16_t>::max()); }
            std::chrono::seconds getHysteresisTime() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::SLEEP_HYSTERESIS_TIME)); }
            void setHysteresisTime(const std::chrono::seconds& time) const { _parent.writeSubcommand(data_register::SLEEP_HYSTERESIS_TIME, static_cast<uint8_t>(time.count())); }
            int32_t getChargerVoltageThreshold() const { return static_cast<int32_t>(_parent.readSubcommand<int16_t>(data_register::SLEEP_CHARGER_VOLTAGE_THRESHOLD)) * 10; }
            void setChargerVoltageThreshold(const int32_t& voltage) const { _parent.writeSubcommandClamped<int16_t>(data_register::SLEEP_CHARGER_VOLTAGE_THRESHOLD, static_cast<int16_t>(voltage / 10), 0, std::numeric_limits<int16_t>::max()); }
            int32_t getChargerPackTosDelta() const { return static_cast<int32_t>(_parent.readSubcommand<int16_t>(data_register::SLEEP_CHARGER_PACK_TOS_DELTA)) * 10; }
            void setChargerPackTosDelta(const int32_t& voltage) const { _parent.writeSubcommandClamped<int16_t>(data_register::SLEEP_CHARGER_PACK_TOS_DELTA, static_cast<int16_t>(voltage / 10), 10, 8500); }

        protected:
            Sleep(bq76942& parent)
                : _parent(parent)
            {
            }
            Sleep(Sleep&) = delete;
            Sleep& operator=(Sleep&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        const Shutdown shutdown;
        const Sleep sleep;

    protected:
        Power(bq76942& parent)
            : shutdown(parent),
              sleep(parent),
              _parent(parent)
        {
        }
        Power(Power&) = delete;
        Power& operator=(Power&) = delete;

    private:
        friend class bq76942;
        bq76942& _parent;
    };

    // System Data:Integrity
    uint16_t getConfigRamSignature() { return readSubcommand<uint16_t>(data_register::CONFIG_RAM_SIGNATURE); }
    void setConfigRamSignature(const uint16_t& signature) { writeSubcommandClamped<uint16_t>(data_register::CONFIG_RAM_SIGNATURE, signature, 0, 0x7FFF); }

    class Protections final
    {
    public:
        // Protections:CUV
        class CellUnderVoltage final
        {
        public:
            float getThreshold() const { return _parent.readSubcommand<uint8_t>(data_register::CUV_THRESHOLD) * 50.6; }
            void setThreshold(const float& threshold) const
            {
                const uint8_t roundedThreshold = static_cast<uint8_t>(std::round(threshold / 50.6));
                _parent.writeSubcommandClamped<uint8_t>(data_register::CUV_THRESHOLD, roundedThreshold, 20, 90);
            }
            std::chrono::microseconds getActivationDelay() const { return std::chrono::microseconds(_parent.readSubcommand<uint16_t>(data_register::CUV_DELAY) * 3300L); }
            void setActivationDelay(const std::chrono::microseconds& delay) const
            {
                const uint16_t roundedDelay = static_cast<uint16_t>(std::round(delay.count() / 3300.0));
                _parent.writeSubcommandClamped<uint16_t>(data_register::CUV_DELAY, roundedDelay, 1, 2047);
            }
            float getRecoveryHysteresis() const { return _parent.readSubcommand<uint8_t>(data_register::CUV_RECOVERY_HYSTERESIS) * 50.6; }
            void setRecoveryHysteresis(const float& hysteresis) const
            {
                const uint8_t roundedHysteresis = static_cast<uint8_t>(std::round(hysteresis / 50.6));
                _parent.writeSubcommandClamped<uint8_t>(data_register::CUV_RECOVERY_HYSTERESIS, roundedHysteresis, 2, 20);
            }

        protected:
            CellUnderVoltage(bq76942& parent)
                : _parent(parent)
            {
            }
            CellUnderVoltage(CellUnderVoltage&) = delete;
            CellUnderVoltage& operator=(CellUnderVoltage&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };
        // Protections:[COV, COVL]
        class CellOverVoltage final
        {
        public:
            float getThreshold() const { return _parent.readSubcommand<uint8_t>(data_register::COV_THRESHOLD) * 50.6; }
            void setThreshold(const float& threshold) const
            {
                const uint8_t roundedThreshold = static_cast<uint8_t>(std::round(threshold / 50.6));
                _parent.writeSubcommandClamped<uint8_t>(data_register::COV_THRESHOLD, roundedThreshold, 20, 110);
            }
            std::chrono::microseconds getActivationDelay() const { return std::chrono::microseconds(_parent.readSubcommand<uint16_t>(data_register::COV_DELAY) * 3300L); }
            void setActivationDelay(const std::chrono::microseconds& delay) const
            {
                const uint16_t roundedDelay = static_cast<uint16_t>(std::round(delay.count() / 3300.0));
                _parent.writeSubcommandClamped<uint16_t>(data_register::COV_DELAY, roundedDelay, 1, 2047);
            }
            float getRecoveryHysteresis() const { return _parent.readSubcommand<uint8_t>(data_register::COV_RECOVERY_HYSTERESIS) * 50.6; }
            void setRecoveryHysteresis(const float& hysteresis) const
            {
                const uint8_t roundedHysteresis = static_cast<uint8_t>(std::round(hysteresis / 50.6));
                _parent.writeSubcommandClamped<uint8_t>(data_register::COV_RECOVERY_HYSTERESIS, roundedHysteresis, 2, 20);
            }

            uint8_t getLatchLimit() const { return _parent.readSubcommand<uint8_t>(data_register::COVL_LATCH_LIMIT); }
            void setLatchLimit(const uint8_t& limit) const { _parent.writeSubcommand(data_register::COVL_LATCH_LIMIT, limit); }
            std::chrono::seconds getLatchCounterDecDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::COVL_COUNTER_DEC_DELAY)); }
            void setLatchCounterDecDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::COVL_COUNTER_DEC_DELAY, static_cast<uint8_t>(delay.count())); }
            std::chrono::seconds getLatchRecoveryTime() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::COVL_RECOVERY_TIME)); }
            void setLatchRecoveryTime(const std::chrono::seconds& time) const { _parent.writeSubcommand(data_register::COVL_RECOVERY_TIME, static_cast<uint8_t>(time.count())); }

        protected:
            CellOverVoltage(bq76942& parent)
                : _parent(parent)
            {
            }
            CellOverVoltage(CellOverVoltage&) = delete;
            CellOverVoltage& operator=(CellOverVoltage&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        class OverCurrentCharge final
        {
        public:
            uint16_t getThresholdVoltage() const { return static_cast<uint16_t>(_parent.readSubcommand<uint8_t>(data_register::OCC_THRESHOLD)) * 2; }
            void setThresholdVoltage(const uint16_t& voltage) const { _parent.writeSubcommandClamped<uint8_t>(data_register::OCC_THRESHOLD, static_cast<uint8_t>(voltage / 2), 2, 62); }
            std::chrono::microseconds getActivationDelay() const { return std::chrono::microseconds((_parent.readSubcommand<uint8_t>(data_register::OCC_DELAY) * 3300L) + 6600L); };
            void setActivationDelay(const std::chrono::microseconds& delay) const
            {
                const uint8_t roundedDelay = static_cast<uint8_t>(std::round((delay.count() - 6600L) / 3300.0));
                _parent.writeSubcommandClamped<uint8_t>(data_register::OCC_DELAY, roundedDelay, 1, 127);
            }
            int16_t getRecoveryThreshold() const { return _parent.readSubcommand<int16_t>(data_register::OCC_RECOVERY_THRESHOLD); }
            void setRecoveryThreshold(const int16_t& threshold) const { _parent.writeSubcommand(data_register::OCC_RECOVERY_THRESHOLD, threshold); }
            int32_t getRecoveryPackStackDelta() const { return static_cast<int32_t>(_parent.readSubcommand<int16_t>(data_register::OCC_PACK_TOS_DELTA)) * 10; }
            void setRecoveryPackStackDelta(const int32_t& delta) const { _parent.writeSubcommandClamped<int16_t>(data_register::OCC_PACK_TOS_DELTA, static_cast<int32_t>(delta / 10), 10, 8500); }

        protected:
            OverCurrentCharge(bq76942& parent)
                : _parent(parent)
            {
            }
            OverCurrentCharge(OverCurrentCharge&) = delete;
            OverCurrentCharge& operator=(OverCurrentCharge&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        class OverCurrentDischarge final
        {
        public:
            // note: no need to upgrade the type here, since the max of 100*2 = 200 is still within the uint8_t range
            uint8_t getTier1Threshold() const { return _parent.readSubcommand<uint8_t>(data_register::OCD1_THRESHOLD) * 2; }
            void setTier1Threshold(const uint8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::OCD1_THRESHOLD, threshold / 2, 2, 100); }
            std::chrono::microseconds getTier1Delay() const { return std::chrono::microseconds((_parent.readSubcommand<uint8_t>(data_register::OCD1_DELAY) * 3300L) + 6600L); }
            void setTier1Delay(const std::chrono::microseconds& delay) const
            {
                const uint8_t roundedDelay = static_cast<uint8_t>(std::round((delay.count() - 6600L) / 3300.0));
                _parent.writeSubcommandClamped<uint8_t>(data_register::OCD1_DELAY, roundedDelay, 1, 127);
            }
            uint8_t getTier2Threshold() const { return _parent.readSubcommand<uint8_t>(data_register::OCD2_THRESHOLD) * 2; }
            void setTier2Threshold(const uint8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::OCD2_THRESHOLD, threshold / 2, 2, 100); }
            std::chrono::microseconds getTier2Delay() const { return std::chrono::microseconds((_parent.readSubcommand<uint8_t>(data_register::OCD2_DELAY) * 3300L) + 6600L); }
            void setTier2Delay(const std::chrono::microseconds& delay) const
            {
                const uint8_t roundedDelay = static_cast<uint8_t>(std::round((delay.count() - 6600L) / 3300.0));
                _parent.writeSubcommandClamped<uint8_t>(data_register::OCD2_DELAY, roundedDelay, 1, 127);
            }

            float getTier3Threshold() const;
            void setTier3Threshold(const float& threshold) const;
            std::chrono::seconds getTier3Delay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::OCD3_DELAY)); };
            void setTier3Delay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::OCD3_DELAY, static_cast<uint8_t>(delay.count())); }

            int16_t getRecoveryThreshold() const { return _parent.readSubcommand<int16_t>(data_register::OCD_RECOVERY_THRESHOLD); }
            void setRecoveryThreshold(const int16_t& threshold) const { _parent.writeSubcommand(data_register::OCD_RECOVERY_THRESHOLD, threshold); }

            uint8_t getLatchLimit() const { return _parent.readSubcommand<uint8_t>(data_register::OCDL_LATCH_LIMIT); }
            void setLatchLimit(const uint8_t& limit) const { _parent.writeSubcommand(data_register::OCDL_LATCH_LIMIT, limit); }
            std::chrono::seconds getLatchCounterDecDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::OCDL_COUNTER_DEC_DELAY)); }
            void setLatchCounterDecDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::OCDL_COUNTER_DEC_DELAY, static_cast<uint8_t>(delay.count())); }
            std::chrono::seconds getLatchRecoveryTime() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::OCDL_RECOVERY_TIME)); }
            void setLatchRecoveryTime(const std::chrono::seconds& time) const { _parent.writeSubcommand(data_register::OCDL_RECOVERY_TIME, static_cast<uint8_t>(time.count())); }
            int16_t getLatchRecoveryThreshold() const { return _parent.readSubcommand<int16_t>(data_register::OCDL_RECOVERY_THRESHOLD); }
            void setLatchRecoveryThreshold(const int16_t& threshold) const { _parent.writeSubcommand(data_register::OCDL_RECOVERY_THRESHOLD, threshold); }

        protected:
            OverCurrentDischarge(bq76942& parent)
                : _parent(parent)
            {
            }
            OverCurrentDischarge(OverCurrentDischarge&) = delete;
            OverCurrentDischarge& operator=(OverCurrentDischarge&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        class ShortCircuit final
        {
        public:
            short_circuit_discharge_threshold getThreshold() const { return _parent.readSubcommand<short_circuit_discharge_threshold>(data_register::SCD_THRESHOLD); }
            void setThreshold(const short_circuit_discharge_threshold& threshold) const { _parent.writeSubcommand(data_register::SCD_THRESHOLD, threshold); }
            std::chrono::microseconds getActivationDelay() const { return std::chrono::microseconds((_parent.readSubcommand<uint8_t>(data_register::SCD_DELAY) - 1) * 15); }
            void setActivationDelay(const std::chrono::microseconds& delay) const
            {
                const uint8_t roundedDelay = static_cast<uint8_t>(std::round(delay.count() / 15.0) + 1);
                _parent.writeSubcommandClamped<uint8_t>(data_register::SCD_DELAY, roundedDelay, 1, 31);
            }
            std::chrono::seconds getRecoveryTime() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::SCD_RECOVERY_TIME)); }
            void setRecoveryTime(const std::chrono::seconds& time) const { _parent.writeSubcommand(data_register::SCD_RECOVERY_TIME, static_cast<uint8_t>(time.count())); }

            uint8_t getLatchLimit() const { return _parent.readSubcommand<uint8_t>(data_register::SCDL_LATCH_LIMIT); }
            void setLatchLimit(const uint8_t& limit) const { _parent.writeSubcommand(data_register::SCDL_LATCH_LIMIT, limit); }
            std::chrono::seconds getLatchCounterDecDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::SCDL_COUNTER_DEC_DELAY)); }
            void setLatchCounterDecDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::SCDL_COUNTER_DEC_DELAY, static_cast<uint8_t>(delay.count())); }
            std::chrono::seconds getLatchRecoveryTime() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::SCDL_RECOVERY_TIME)); }
            void setLatchRecoveryTime(const std::chrono::seconds& time) const { _parent.writeSubcommand(data_register::SCDL_RECOVERY_TIME, static_cast<uint8_t>(time.count())); }
            int16_t getLatchRecoveryThreshold() const { return _parent.readSubcommand<int16_t>(data_register::SCDL_RECOVERY_THRESHOLD); }
            void setLatchRecoveryThreshold(const int16_t& threshold) const { _parent.writeSubcommand(data_register::SCDL_RECOVERY_THRESHOLD, threshold); }

        protected:
            ShortCircuit(bq76942& parent)
                : _parent(parent)
            {
            }
            ShortCircuit(ShortCircuit&) = delete;
            ShortCircuit& operator=(ShortCircuit&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        class OverTemperature final
        {
        public:
            int8_t getChargeThreshold() const { return _parent.readSubcommand<int8_t>(data_register::OTC_THRESHOLD); }
            void setChargeThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::OTC_THRESHOLD, threshold, -40, 120); }
            std::chrono::seconds getChargeDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::OTC_DELAY)); }
            void setChargeDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::OTC_DELAY, static_cast<uint8_t>(delay.count())); }
            int8_t getChargeRecoveryThreshold() const { return _parent.readSubcommand<int8_t>(data_register::OTC_RECOVERY); }
            void setChargeRecoveryThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::OTC_RECOVERY, threshold, -40, 120); }

            int8_t getDischargeThreshold() const { return _parent.readSubcommand<int8_t>(data_register::OTD_THRESHOLD); }
            void setDischargeThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::OTD_THRESHOLD, threshold, -40, 120); }
            std::chrono::seconds getDischargeDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::OTD_DELAY)); }
            void setDischargeDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::OTD_DELAY, static_cast<uint8_t>(delay.count())); }
            int8_t getDischargeRecoveryThreshold() const { return _parent.readSubcommand<int8_t>(data_register::OTD_RECOVERY); }
            void setDischargeRecoveryThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::OTD_RECOVERY, threshold, -40, 120); }

            int8_t getFetThreshold() const { return _parent.readSubcommand<int8_t>(data_register::OTF_THRESHOLD); }
            void setFetThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::OTF_THRESHOLD, threshold, 0, 150); }
            std::chrono::seconds getFetDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::OTF_DELAY)); }
            void setFetDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::OTF_DELAY, static_cast<uint8_t>(delay.count())); }
            int8_t getFetRecoveryThreshold() const { return _parent.readSubcommand<int8_t>(data_register::OTF_RECOVERY); }
            void setFetRecoveryThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::OTF_RECOVERY, threshold, 0, 150); }

            int8_t getInternalThreshold() const { return _parent.readSubcommand<int8_t>(data_register::OTINT_THRESHOLD); }
            void setInternalThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::OTINT_THRESHOLD, threshold, -40, 120); }
            std::chrono::seconds getInternalDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::OTINT_DELAY)); }
            void setInternalDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::OTINT_DELAY, static_cast<uint8_t>(delay.count())); }
            int8_t getInternalRecoveryThreshold() const { return _parent.readSubcommand<int8_t>(data_register::OTINT_RECOVERY); }
            void setInternalRecoveryThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::OTINT_RECOVERY, threshold, -40, 120); }

        protected:
            OverTemperature(bq76942& parent)
                : _parent(parent)
            {
            }
            OverTemperature(OverTemperature&) = delete;
            OverTemperature& operator=(OverTemperature&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        class UnderTemperature final
        {
        public:
            int8_t getChargeThreshold() const { return _parent.readSubcommand<int8_t>(data_register::UTC_THRESHOLD); }
            void setChargeThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::UTC_THRESHOLD, threshold, -40, 120); }
            std::chrono::seconds getChargeDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::UTC_DELAY)); }
            void setChargeDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::UTC_DELAY, static_cast<uint8_t>(delay.count())); }
            int8_t getChargeRecoveryThreshold() const { return _parent.readSubcommand<int8_t>(data_register::UTC_RECOVERY); }
            void setChargeRecoveryThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::UTC_RECOVERY, threshold, -40, 120); }

            int8_t getDischargeThreshold() const { return _parent.readSubcommand<int8_t>(data_register::UTD_THRESHOLD); }
            void setDischargeThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::UTD_THRESHOLD, threshold, -40, 120); }
            std::chrono::seconds getDischargeDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::UTD_DELAY)); }
            void setDischargeDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::UTD_DELAY, static_cast<uint8_t>(delay.count())); }
            int8_t getDischargeRecoveryThreshold() const { return _parent.readSubcommand<int8_t>(data_register::UTD_RECOVERY); }
            void setDischargeRecoveryThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::UTD_RECOVERY, threshold, -40, 120); }

            int8_t getInternalThreshold() const { return _parent.readSubcommand<int8_t>(data_register::UTINT_THRESHOLD); }
            void setInternalThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::UTINT_THRESHOLD, threshold, -40, 120); }
            std::chrono::seconds getInternalDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::UTINT_DELAY)); }
            void setInternalDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::UTINT_DELAY, static_cast<uint8_t>(delay.count())); }
            int8_t getInternalRecoveryThreshold() const { return _parent.readSubcommand<int8_t>(data_register::UTINT_RECOVERY); }
            void setInternalRecoveryThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::UTINT_RECOVERY, threshold, -40, 120); }

        protected:
            UnderTemperature(bq76942& parent)
                : _parent(parent)
            {
            }
            UnderTemperature(UnderTemperature&) = delete;
            UnderTemperature& operator=(UnderTemperature&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        std::chrono::seconds getRecoveryTime() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::PROTECTIONS_RECOVERY_TIME)); }
        void setRecoveryTime(const std::chrono::seconds& time) const { _parent.writeSubcommand(data_register::PROTECTIONS_RECOVERY_TIME, static_cast<uint8_t>(time.count())); }

        std::chrono::seconds getHwdTimeout() const { return std::chrono::seconds(_parent.readSubcommand<uint16_t>(data_register::HWD_DELAY)); }
        void setHwdTimeout(const std::chrono::seconds& timeout) const { _parent.writeSubcommand(data_register::HWD_DELAY, static_cast<uint16_t>(timeout.count())); }

        std::chrono::seconds getLoadDetectTime() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::LOAD_DETECT_ACTIVE_TIME)); }
        void setLoadDetectTime(const std::chrono::seconds& time) const { _parent.writeSubcommand(data_register::LOAD_DETECT_ACTIVE_TIME, static_cast<uint8_t>(time.count())); }
        std::chrono::seconds getLoadDetectRetryDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::LOAD_DETECT_RETRY_DELAY)); }
        void setLoadDetectRetryDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::LOAD_DETECT_RETRY_DELAY, static_cast<uint8_t>(delay.count())); }
        std::chrono::hours getLoadDetectTimeout() const { return std::chrono::hours(_parent.readSubcommand<uint16_t>(data_register::LOAD_DETECT_TIMEOUT)); }
        void setLoadDetectTimeout(const std::chrono::hours& timeout) const { _parent.writeSubcommand(data_register::LOAD_DETECT_TIMEOUT, static_cast<uint16_t>(timeout.count())); }

        int16_t getPrechargeTimeoutCurrentThreshold() const { return _parent.readSubcommand<int16_t>(data_register::PTO_CHARGE_THRESHOLD); }
        void setPrechargeTimeoutCurrentThreshold(const int16_t& threshold) const { _parent.writeSubcommandClamped<int16_t>(data_register::PTO_CHARGE_THRESHOLD, threshold, 0, 1000); }
        std::chrono::seconds getPrechargeTimeoutDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint16_t>(data_register::PTO_DELAY)); }
        void setPrechargeTimeoutDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::PTO_DELAY, static_cast<uint16_t>(delay.count())); }
        float getPrechargeResetCharge() const;
        void setPrechargeResetCharge(const float& charge) const;

        const CellUnderVoltage underVoltage;
        const CellOverVoltage overVoltage;
        const OverCurrentCharge overCurrentCharge;
        const OverCurrentDischarge overCurrentDischarge;
        const ShortCircuit shortCircuit;
        const OverTemperature overTemp;
        const UnderTemperature underTemp;

    protected:
        Protections(bq76942& parent)
            : underVoltage(parent),
              overVoltage(parent),
              overCurrentCharge(parent),
              overCurrentDischarge(parent),
              shortCircuit(parent),
              overTemp(parent),
              underTemp(parent),
              _parent(parent)
        {
        }
        Protections(Protections&) = delete;
        Protections& operator=(Protections&) = delete;

    private:
        friend class bq76942;
        bq76942& _parent;
    };

    class PermanentFail final
    {
    public:
        int16_t getCopperDepositionThreshold() const { return _parent.readSubcommand<int16_t>(data_register::CU_DEP_THRESHOLD); }
        void setCopperDepositionThreshold(const int16_t& threshold) const { _parent.writeSubcommandClamped<int16_t>(data_register::CU_DEP_THRESHOLD, threshold, 0, std::numeric_limits<int16_t>::max()); }
        std::chrono::seconds getCopperDepositionDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::CU_DEP_DELAY)); }
        void setCopperDepositionDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::CU_DEP_DELAY, static_cast<uint8_t>(delay.count())); }

        int16_t getUnderVoltageThreshold() const { return _parent.readSubcommand<int16_t>(data_register::SUV_THRESHOLD); }
        void setUnderVoltageThreshold(const int16_t& threshold) const { _parent.writeSubcommandClamped<int16_t>(data_register::SUV_THRESHOLD, threshold, 0, std::numeric_limits<int16_t>::max()); }
        std::chrono::seconds getUnderVoltageDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::SUV_DELAY)); }
        void setUnderVoltageDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::SUV_DELAY, static_cast<uint8_t>(delay.count())); }

        int16_t getOverVoltageThreshold() const { return _parent.readSubcommand<int16_t>(data_register::SOV_THRESHOLD); }
        void setOverVoltageThreshold(const int16_t& threshold) const { _parent.writeSubcommandClamped<int16_t>(data_register::SOV_THRESHOLD, threshold, 0, std::numeric_limits<int16_t>::max()); }
        std::chrono::seconds getOverVoltageDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::SOV_DELAY)); }
        void setOverVoltageDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::SOV_DELAY, static_cast<uint8_t>(delay.count())); }

        int16_t getStackDeltaThreshold() const { return _parent.readSubcommand<int16_t>(data_register::TOS_THRESHOLD); }
        void setStackDeltaThreshold(const int16_t& threshold) const { _parent.writeSubcommandClamped<int16_t>(data_register::TOS_THRESHOLD, threshold, 0, std::numeric_limits<int16_t>::max()); }
        std::chrono::seconds getStackDeltaDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::TOS_DELAY)); }
        void setStackDeltaDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::TOS_DELAY, static_cast<uint8_t>(delay.count())); }

        float getChargeCurrentThreshold() const;
        void setChargeCurrentThreshold(const float& threshold) const;

        float getDischargeCurrentThreshold() const;
        void setDischargeCurrentThreshold(const float& threshold) const;

        int8_t getOverTempCellThreshold() const { return _parent.readSubcommand<int8_t>(data_register::SOT_THRESHOLD); }
        void setOverTempCellThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<int8_t>(data_register::SOT_THRESHOLD, threshold, -40, 120); }
        std::chrono::seconds getOverTempCellDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::SOT_DELAY)); }
        void setOverTempCellDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::SOT_DELAY, static_cast<uint8_t>(delay.count())); }

        int8_t getOverTempFetThreshold() const { return _parent.readSubcommand<int8_t>(data_register::SOTF_THRESHOLD); }
        void setOverTempFetThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<int8_t>(data_register::SOTF_THRESHOLD, threshold, 0, 150); }
        std::chrono::seconds getOverTempFetDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::SOTF_DELAY)); }
        void setOverTempFetDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::SOTF_DELAY, static_cast<uint8_t>(delay.count())); }

        class VoltageImbalanceRelaxed final
        {
        public:
            int16_t getCheckVoltage() const { return _parent.readSubcommand<int16_t>(data_register::VIMR_CHECK_VOLTAGE); }
            void setCheckVoltage(const int16_t& voltage) const { _parent.writeSubcommandClamped<int16_t>(data_register::VIMR_CHECK_VOLTAGE, voltage, 0, 5500); }
            int16_t getMaxRelaxCurrent() const { return _parent.readSubcommand<int16_t>(data_register::VIMR_MAX_RELAX_CURRENT); }
            void setMaxRelaxCurrent(const int16_t& current) const { _parent.writeSubcommandClamped<int16_t>(data_register::VIMR_MAX_RELAX_CURRENT, current, 10, std::numeric_limits<int16_t>::max()); }
            int16_t getThreshold() const { return _parent.readSubcommand<int16_t>(data_register::VIMR_THRESHOLD); }
            void setThreshold(const int16_t& threshold) const { _parent.writeSubcommandClamped<int16_t>(data_register::VIMR_THRESHOLD, threshold, 0, 5500); }
            std::chrono::seconds readActivationDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::VIMR_DELAY)); }
            void setActivationDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::VIMR_DELAY, static_cast<uint8_t>(delay.count())); }
            std::chrono::seconds readRelaxMinDuration() const { return std::chrono::seconds(_parent.readSubcommand<uint16_t>(data_register::VIMR_RELAX_MIN_DURATION)); }
            void setRelaxMinDuration(const std::chrono::seconds& duration) const { _parent.writeSubcommand(data_register::VIMR_RELAX_MIN_DURATION, static_cast<uint16_t>(duration.count())); }

        protected:
            VoltageImbalanceRelaxed(bq76942& parent)
                : _parent(parent)
            {
            }
            VoltageImbalanceRelaxed(VoltageImbalanceRelaxed&) = delete;
            VoltageImbalanceRelaxed& operator=(VoltageImbalanceRelaxed&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        class VoltageImbalanceActive final
        {
        public:
            int16_t getCheckVoltage() const { return _parent.readSubcommand<int16_t>(data_register::VIMA_CHECK_VOLTAGE); }
            void setCheckVoltage(const int16_t& voltage) const { _parent.writeSubcommandClamped<int16_t>(data_register::VIMA_CHECK_VOLTAGE, voltage, 0, 5500); }
            int16_t getMinActiveCurrent() const { return _parent.readSubcommand<int16_t>(data_register::VIMA_MIN_ACTIVE_CURRENT); }
            void setMinActiveCurrent(const int16_t& current) const { _parent.writeSubcommandClamped<int16_t>(data_register::VIMA_MIN_ACTIVE_CURRENT, current, 10, std::numeric_limits<int16_t>::max()); }
            int16_t getThreshold() const { return _parent.readSubcommand<int16_t>(data_register::VIMA_THRESHOLD); }
            void setThreshold(const int16_t& threshold) const { _parent.writeSubcommandClamped<int16_t>(data_register::VIMA_THRESHOLD, threshold, 0, 5500); }
            std::chrono::seconds readActivationDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::VIMA_DELAY)); }
            void setActivationDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::VIMA_DELAY, static_cast<uint8_t>(delay.count())); }

        protected:
            VoltageImbalanceActive(bq76942& parent)
                : _parent(parent)
            {
            }
            VoltageImbalanceActive(VoltageImbalanceActive&) = delete;
            VoltageImbalanceActive& operator=(VoltageImbalanceActive&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        class ChargeFetFail final
        {
        public:
            int16_t getThreshold() const { return _parent.readSubcommand<int16_t>(data_register::CFETF_OFF_THRESHOLD); }
            void setThreshold(const int16_t& threshold) const { _parent.writeSubcommandClamped<int16_t>(data_register::CFETF_OFF_THRESHOLD, threshold, 10, 5000); }
            std::chrono::seconds readActivationDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::CFETF_OFF_DELAY)); }
            void setActivationDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::CFETF_OFF_DELAY, static_cast<uint8_t>(delay.count())); }

        protected:
            ChargeFetFail(bq76942& parent)
                : _parent(parent)
            {
            }
            ChargeFetFail(ChargeFetFail&) = delete;
            ChargeFetFail& operator=(ChargeFetFail&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };
        class DischargeFetFail final
        {
        public:
            int16_t getThreshold() const { return _parent.readSubcommand<int16_t>(data_register::DFETF_OFF_THRESHOLD); }
            void setThreshold(const int16_t& threshold) const { _parent.writeSubcommandClamped<int16_t>(data_register::DFETF_OFF_THRESHOLD, threshold, -5000, -10); }
            std::chrono::seconds readActivationDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::DFETF_OFF_DELAY)); }
            void setActivationDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::DFETF_OFF_DELAY, static_cast<uint8_t>(delay.count())); }

        protected:
            DischargeFetFail(bq76942& parent)
                : _parent(parent)
            {
            }
            DischargeFetFail(DischargeFetFail&) = delete;
            DischargeFetFail& operator=(DischargeFetFail&) = delete;

        private:
            friend class bq76942;
            bq76942& _parent;
        };

        int16_t getVssfThreshold() const { return _parent.readSubcommand<int16_t>(data_register::VSSF_FAIL_THRESHOLD); }
        void setVssfThreshold(const int16_t& threshold) const { _parent.writeSubcommandClamped<int16_t>(data_register::VSSF_FAIL_THRESHOLD, threshold, 1, std::numeric_limits<int16_t>::max()); };
        std::chrono::seconds readVssfDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::VSSF_DELAY)); }
        void setVssfDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::VSSF_DELAY, static_cast<uint8_t>(delay.count())); }

        std::chrono::seconds read2LvlDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::PF_2LVL_DELAY)); }
        void set2LvlDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::PF_2LVL_DELAY, static_cast<uint8_t>(delay.count())); }
        std::chrono::seconds readLfofDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::LFOF_DELAY)); }
        void setLfofDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::LFOF_DELAY, static_cast<uint8_t>(delay.count())); }
        std::chrono::seconds readHardwareMuxfDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::HWMX_DELAY)); }
        void setHardwareMuxfDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::HWMX_DELAY, static_cast<uint8_t>(delay.count())); }

        const VoltageImbalanceRelaxed voltageImbalanceRelaxed;
        const VoltageImbalanceActive voltageImbalanceActive;
        const ChargeFetFail chargeFetFail;
        const DischargeFetFail dischargeFetFail;

    protected:
        PermanentFail(bq76942& parent)
            : voltageImbalanceRelaxed(parent),
              voltageImbalanceActive(parent),
              chargeFetFail(parent),
              dischargeFetFail(parent),
              _parent(parent)
        {
        }
        PermanentFail(PermanentFail&) = delete;
        PermanentFail& operator=(PermanentFail&) = delete;

    private:
        friend class bq76942;
        bq76942& _parent;
    };

    uint16_t getUnsealKey1() { return readSubcommand<uint16_t>(data_register::UNSEAL_KEY_STEP1); }
    uint16_t getUnsealKey2() { return readSubcommand<uint16_t>(data_register::UNSEAL_KEY_STEP2); }
    uint16_t getFullAccessKey1() { return readSubcommand<uint16_t>(data_register::FULL_ACCESS_KEY_STEP1); }
    uint16_t getFullAccessKey2() { return readSubcommand<uint16_t>(data_register::FULL_ACCESS_KEY_STEP2); }

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
        // can't check for size equality, since data register writes always return a 32 byte memory chunk
        if (dataBuf.size() < sizeof(T))
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

    /// @brief Write a subcommand, with optional data, and wait for it to complete
    /// @param registerAddr The subcommand address to write to
    /// @param data The data to write
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
    void _selectSubcommand(const uint16_t registerAddr, bool waitForCompletion = true);
    std::vector<uint8_t> _readSubcommand(const uint16_t registerAddr);
    void _writeSubcommand(const uint16_t registerAddr, const std::vector<uint8_t>& data);
    static uint8_t _calculateChecksum(uint16_t registerAddr, const std::vector<uint8_t>& data);

    void _queueTransmit(const uint8_t& data);
    void _queueTransmit(const std::vector<uint8_t>& data);

    /// @brief Temporary buffer for storing data so CRCs can be calculated
    std::vector<uint8_t> _dataBuf;

    std::vector<uint8_t> _transmitQueue;
    std::vector<uint8_t> _receiveQueue;
};