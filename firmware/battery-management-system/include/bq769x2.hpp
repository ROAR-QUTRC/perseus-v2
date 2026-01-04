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

    enum class security_state_option : uint8_t
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
    enum class adc_pullup_config : uint8_t
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
    enum class adc_measurement_type : uint8_t
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
    enum class user_amps_unit : uint8_t
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
        bool was_load_detect_on : 1 = 0;         // LD_ON
        bool has_load_detect_timed_out : 1 = 0;  // LD_TIMEOUT
        bool in_deepsleep : 1 = 0;               // DEEPSLEEP
        uint16_t _rsvd4 : 13 = 0;                // RSVD_0
    };
    struct protections_a_t
    {
        uint8_t _rsvd0 : 2 = 0;                // RSVD_0
        bool cell_undervoltage : 1 = 0;        // CUV
        bool cell_overvoltage : 1 = 1;         // COV
        bool overcurrent_charge : 1 = 0;       // OCC
        bool overcurrent_discharge_1 : 1 = 0;  // OCD1
        bool overcurrent_discharge_2 : 1 = 0;  // OCD2
        bool short_circuit_discharge : 1 = 1;  // SCD
    };
    struct protections_b_t
    {
        bool undertemp_charge : 1 = 0;     // UTC
        bool undertemp_discharge : 1 = 0;  // UTD
        bool internal_undertemp : 1 = 0;   // UTINT
        bool _rsvd3 : 1 = 0;               // RSVD_0
        bool overtemp_charge : 1 = 0;      // OTC
        bool overtemp_discharge : 1 = 0;   // OTD
        bool internal_overtemp : 1 = 0;    // OTINT
        bool fet_overtemp : 1 = 0;         // OTF
    };
    struct safety_alert_c_t
    {
        uint8_t _rsvd0 : 3 = 0;                  // RSVD_0
        bool precharge_timeout_suspend : 1;      // PTOS
        bool cell_overvoltage_latch : 1;         // COVL
        bool overcurrent_discharge_3 : 1;        // OCD3
        bool short_circuit_discharge_latch : 1;  // SCDL
    };
    struct protections_c_t
    {
        bool _rsvd0 : 1 = 0;               // RSVD_0
        bool host_watchdog_fault : 1 = 0;  // HWDF
        bool precharge_timeout : 1 = 0;    // PTO
        // NOTE: When used in Settings:Protection:Enabled Protections C,
        // this bit is only RSVD not RSVD_0
        bool _rsvd3 : 1 = 0;                         // RSVD_0
        bool cell_overvoltage_latch : 1 = 0;         // COVL
        bool overcurrent_discharge_latch : 1 = 0;    // OCDL
        bool short_circuit_discharge_latch : 1 = 0;  // SCDL
        bool overcurrent_discharge_3 : 1 = 0;        // OCD3
    };
    struct permanent_fail_a_t
    {
        bool safety_cell_undervoltage : 1;      // SUV
        bool safety_cell_overvoltage : 1;       // SOV
        bool safety_overcurrent_charge : 1;     // SOCC
        bool safety_overcurrent_discharge : 1;  // SOCD
        bool safety_overtemp : 1;               // SOT
        bool _rsvd5 : 1 = 0;                    // RSVD_0
        bool safety_overtemp_fet : 1;           // SOTF
        bool copper_deposition : 1;             // CUDEP
    };
    struct permanent_fail_b_t
    {
        bool charge_fet : 1;                     // CFETF
        bool discharge_fet : 1;                  // DFETF
        bool second_level_protector : 1;         // 2LVL
        bool voltage_imbalance_relax : 1;        // VIMR
        bool voltage_imbalance_active : 1;       // VIMA
        uint8_t _rsvd5 : 2 = 0;                  // RSVD_0
        bool short_circuit_discharge_latch : 1;  // SCDL
    };
    struct permanent_fail_alert_c_t
    {
        uint8_t _rsvd0 : 3 = 0;               // RSVD_0
        bool internal_LFO : 1;                // LFOF
        bool internal_voltage_reference : 1;  // VREF
        bool internal_vss_measurement : 1;    // VSSF
        bool hardware_mux : 1;                // HWMX
        bool _rsvd7 : 1 = 0;                  // RSVD_0
    };
    struct permanent_fail_c_t
    {
        bool otp_memory : 1;                  // OTPF
        bool data_ROM : 1;                    // DRMF
        bool instruction_ROM : 1;             // IRMF
        bool internal_LFO : 1;                // LFOF
        bool internal_voltage_reference : 1;  // VREF
        bool internal_vss_measurement : 1;    // VSSF
        bool hardware_mux : 1;                // HWMX
        bool commanded : 1;                   // CMDF
    };
    struct permanent_fail_d_t
    {
        // top of stack vs cell sum
        bool top_stack_vs_cell : 1;  // TOSF
        uint8_t _rsvd1 : 7 = 0;      // RSVD_0
    };
    struct battery_status_t
    {
        bool in_config_update_mode : 1;  // CFGUPDATE
        bool in_precharge_mode : 1;      // PCHG_MODE
        bool sleep_allowed : 1;          // SLEEP_EN
        // AKA: Power-On Reset
        // Whether or not a full reset has occurred since last CONFIG_UPDATE exit
        bool full_reset_occurred : 1;  // POR
        // whether or not the previous reset was due to the watchdog timer
        // NOTE: Independent of Host Watchdog settings
        bool was_watchdog_reset : 1;               // WD
        bool checking_cell_open_wire : 1;          // COW_CHK
        bool pending_otp_write : 1;                // OTPW
        bool otp_write_blocked : 1;                // OTPB
        security_state_option security_state : 2;  // SEC[1:0]
        bool fuse_active : 1;                      // FUSE
        bool safety_fault_active : 1;              // SS
        bool permanent_fail_active : 1;            // PF
        bool shutdown_pending : 1;                 // SD_CMD
        bool _rsvd14 : 1 = 0;                      // RSVD_0
        bool in_sleep : 1;                         // SLEEP
    };
    struct alarm_status_t
    {
        bool has_woken_from_sleep : 1 = 0;            // WAKE
        bool adc_scan_complete : 1 = 0;               // ADSCAN
        bool is_balancing_cells : 1 = 0;              // CB
        bool is_fuse_driven : 1 = 0;                  // FUSE
        bool stack_reached_shutdown_voltage : 1 = 0;  // SHUTV
        bool discharge_fet_off : 1 = 0;               // XDSG
        bool charge_fet_off : 1 = 0;                  // XCHG
        bool full_voltage_scan_complete : 1 = 0;      // FULLSCAN
        bool _rsvd8 : 1 = 0;                          // RSVD_0
        bool initialization_complete : 1 = 0;         // INITCOMP
        bool initialization_started : 1 = 0;          // INITSTART
        // anything in the alarm PF A, B, C, or D masks triggered
        bool alarm_PF_alert : 1 = 1;  // MSK_PFALERT
        // anything in the alarm SF A, B, or C masks triggered
        bool alarm_SF_alert : 1 = 1;        // MSK_SFALERT
        bool permanent_fail : 1 = 1;        // PF
        bool safety_status_a : 1 = 1;       // SSA
        bool safety_status_b_or_c : 1 = 1;  // SSBC
    };
    struct fet_status_t
    {
        bool charge_fet_on : 1;        // CHG_FET
        bool precharge_fet_on : 1;     // PCHG_FET
        bool discharge_fet_on : 1;     // DSG_FET
        bool predischarge_fet_on : 1;  // PDSG_FET
        bool dchg_asserted : 1;        // DCHG_PIN
        bool ddsg_asserted : 1;        // DDSG_PIN
        bool alert_asserted : 1;       // ALRT_PIN
        bool _rsvd7 : 1 = 0;           // RSVD_0
    };
    struct manufacturing_status_t
    {
        bool precharge_testing : 1;          // PCHG_TEST
        bool charge_testing : 1;             // CHG_TEST
        bool discharge_testing : 1;          // DSG_TEST
        bool _rsvd3 : 1 = 0;                 // RSVD_0
        bool autonomous_fets : 1;            // FET_EN
        bool predischarge_testing : 1;       // PDSG_TEST
        bool is_permanent_fail_enabled : 1;  // PF_EN
        bool is_otp_write_enabled : 1;       // OTPW_EN
        uint8_t _rsvd8 : 8 = 0;              // RSVD_0
    };
    struct fet_control_t
    {
        bool force_discharge_off : 1;     // DSG_OFF
        bool force_predischarge_off : 1;  // PDSG_OFF
        bool force_charge_off : 1;        // CHG_OFF
        bool force_precharge_off : 1;     // PCHG_OFF
        uint8_t _rsvd4 : 4 = 0;           // RSVD_0
    };
    struct regulator_control_t
    {
        bool reg_1_enable : 1;                // REG1_EN
        regulator_voltage reg_1_voltage : 3;  // REG1V_[2:0]
        bool reg_2_enable : 1;                // REG2_EN
        regulator_voltage reg_2_voltage : 3;  // REG2V_[2:0]
    };
    struct otp_write_result_t
    {
        struct result_register
        {
            // note: The following are NOT errors for the whole chip,
            // OTP programming just has stricter requirements
            bool over_voltage : 1;          // HV
            bool under_voltage : 1;         // LV
            bool over_temp : 1;             // HT
            bool data_write_fail : 1;       // NODATA
            bool signature_write_fail : 1;  // NOSIG
            bool otp_locked : 1;            // LOCK
            bool _rsvd6 : 1 = 0;            // RSVD_0
            bool otp_programming_ok : 1;    // OK
        };
        uint16_t fail_address;
    };
    struct firmware_version_t
    {
        union
        {
            uint16_t device_number;
            uint8_t device_number_bytes[2];
        };
        union
        {
            uint16_t firmware_version;
            uint8_t firmware_version_bytes[2];
        };
        union
        {
            uint16_t build_number;
            struct
            {
                uint8_t bcd_high_of_high_byte : 4;
                uint8_t bcd_low_of_high_byte : 4;
                uint8_t bcd_high_of_low_byte : 4;
                uint8_t bcd_low_of_low_byte : 4;
            };
        };
    };

    // --- SUBCOMMANDS ---
    struct security_key_pair_t
    {
        union
        {
            uint16_t step_1;
            uint8_t step_1_bytes[2];
        };
        union
        {
            uint16_t step_2;
            uint8_t step_2_bytes[2];
        };
    };
    union security_keys_t
    {
        struct
        {
            security_key_pair_t unseal;
            security_key_pair_t full_access;
        };
        uint64_t raw = 0x0000000000000000;
    };
    struct saved_pf_status_t
    {
        permanent_fail_a_t pf_status_a;
        permanent_fail_b_t pf_status_b;
        permanent_fail_c_t pf_status_c;
        permanent_fail_d_t pf_status_d;
    };
    struct da_status_1_t
    {
        int32_t cell_1_voltage_counts;
        int32_t cell_1_current_counts;
        int32_t cell_2_voltage_counts;
        int32_t cell_2_current_counts;
        int32_t cell_3_voltage_counts;
        int32_t cell_3_current_counts;
        int32_t cell_4_voltage_counts;
        int32_t cell_4_current_counts;
    };
    struct da_status_2_t
    {
        int32_t cell_5_voltage_counts;
        int32_t cell_5_current_counts;
        int32_t cell_6_voltage_counts;
        int32_t cell_6_current_counts;
        int32_t cell_7_voltage_counts;
        int32_t cell_7_current_counts;
        int32_t cell_8_voltage_counts;
        int32_t cell_8_current_counts;
    };
    struct da_status_3_t
    {
        int32_t cell_9_voltage_counts;
        int32_t cell_9_current_counts;
        int32_t cell_10_voltage_counts;
        int32_t cell_10_current_counts;
    };
    struct raw_da_status_5_t
    {
        int16_t vreg_18_adc_counts;
        int16_t vss_adc_counts;
        int16_t max_cell_voltage;
        int16_t min_cell_voltage;
        int16_t battery_voltage_sum;
        int16_t cell_temperature;
        int16_t fet_temperature;
        int16_t max_cell_temperature;
        int16_t min_cell_temperature;
        int16_t avg_cell_temperature;
        int16_t cc_3_current;
        int16_t cc_1_current;
        int32_t cc_2_counts;
        int32_t cc_3_counts;
    };
    struct raw_da_status_6_t
    {
        int32_t accumulated_charge;
        uint32_t accumulated_charge_fraction;
        uint32_t accumulated_charge_time;
        int32_t cfetoff_counts;
        int32_t dfetoff_counts;
        int32_t alert_counts;
        int32_t ts1_counts;
        int32_t ts2_counts;
    };
    struct da_status_7_t
    {
        int32_t ts3_counts;
        int32_t hdq_counts;
        int32_t dchg_counts;
        int32_t ddsg_counts;
    };
    union voltage_snapshot_t
    {
        struct
        {
            int16_t cell_1_voltage;
            int16_t cell_2_voltage;
            int16_t cell_3_voltage;
            int16_t cell_4_voltage;
            int16_t cell_5_voltage;
            int16_t cell_6_voltage;
            int16_t cell_7_voltage;
            int16_t cell_8_voltage;
            int16_t cell_9_voltage;
            int16_t cell_10_voltage;
        };
        int16_t voltages[10];
    };
    struct cb_status_2_t
    {
        uint32_t cell_1_balancing_time;
        uint32_t cell_2_balancing_time;
        uint32_t cell_3_balancing_time;
        uint32_t cell_4_balancing_time;
        uint32_t cell_5_balancing_time;
        uint32_t cell_6_balancing_time;
        uint32_t cell_7_balancing_time;
        uint32_t cell_8_balancing_time;
    };
    struct cb_status_3_t
    {
        uint32_t cell_9_balancing_time;
        uint32_t cell_10_balancing_time;
    };
    struct cal1_t
    {
        int16_t calibration_data_counter;
        int32_t cc_2_counts;
        int16_t pack_counts;
        int16_t top_of_stack_counts;
        int16_t ld_counts;
    };

    // --- DATA REGISTERS ---
    struct power_config_t
    {
        coulomb_conversion_speed wake_speed : 2 = coulomb_conversion_speed::T_24MS;  // WK_SPD_[1:0]
        // measurement loop speed during normal operation
        measurement_loop_speed loop_speed : 2 = measurement_loop_speed::FULL_SPEED;  // LOOP_SLOW_[1:0]
        // measurement loop speed during cell balancing
        measurement_loop_speed cell_balance_loop_speed : 2 = measurement_loop_speed::FULL_SPEED;  // CB_LOOP_SLOW_[1:0]
        // cleared: 3ms per conversion
        // set: 1.5ms per conversion (but lower accuracy)
        bool use_fast_adc : 1 = false;                        // FASTADC
        bool enable_overtemp_shutdown : 1 = true;             // OTSD
        bool enable_sleep : 1 = true;                         // SLEEP
        bool enable_deep_sleep_LFO : 1 = false;               // DPSLP_LFO
        bool enable_deep_sleep_LDO : 1 = false;               // DPSLP_LDO
        bool enable_deep_sleep_charger_wake : 1 = true;       // DPSLP_PD
        bool disable_shutdown_ts2_wake : 1 = false;           // SHUT_TS2
        bool enable_deep_sleep_overtemp_shutdown : 1 = true;  // DPSLP_OT spellchecker:disable-line
        uint8_t _rsvd14 : 2 = 0;                              // RSVD_0
    };
    struct reg0_config_t
    {
        bool reg_0_enable : 1 = true;  // REG0_EN
        bool _rsvd1 : 1;               // RSVD
        uint8_t _rsvd2 : 6 = 0;        // RSVD_0
    };
    struct hwd_regulator_options_t
    {
        // Regulator off time in seconds before turning back on
        uint8_t toggle_time : 4;           // TOGGLE_TIME_[3:0]
        hwd_toggle_option hwd_action : 2;  // TOGGLE_OPT_[1:0]
        uint8_t _rsvd5 : 2 = 0;            // RSVD_0
    };
    struct spi_configuration_t
    {
        uint8_t _rsvd0 : 5 = 0;  // RSVD_0
        // use digital filters (recommend for high-freq operation)
        bool enable_filters : 1;  // FILT
        // clear: MISO uses REG18 voltage
        // set: MISO uses REG1 voltage
        bool miso_uses_reg_1 : 1;  // MISO_REG1
        bool _rsvd7 : 1 = 0;       // RSVD_0
    };
    struct cfetoff_pin_configuration_t
    {
        union
        {
            struct
            {
                cfetoff_pin_function function : 2;  // PINFXN[1:0]
                bool enable_pulldown : 1;           // OPT[0]
                // clear: Driving high drives to HI-Z (unavailable with is_active_low)
                // set: Driving high drives to selected regulator
                bool disable_high_Z_drive : 1;  // OPT[1]
                // NOTE: Should be cleared when is_active_low is set
                bool pullup_to_reg1 : 1;  // OPT[2]
                // clear: High uses REG18
                // set: High uses REG1
                bool drive_high_uses_reg1 : 1;  // OPT[3]
                bool _rsvd6 : 1;                // OPT[4]
                bool is_active_low : 1;         // OPT[5]
            } function;
            struct
            {
                adc_pin_function function : 2;              // PINFXN[1:0]
                adc_measurement_type measurement_type : 2;  // OPT[1:0]
                polynomial_selection polynomial : 2;        // OPT[3:2]
                adc_pullup_config pullup_config : 2;        // OPT[5:4]
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
                bool enable_pulldown : 1;           // OPT[0]
                // clear: Driving high drives to HI-Z (unavailable with is_active_low)
                // set: Driving high drives to selected regulator
                bool disable_high_Z_drive : 1;  // OPT[1]
                // NOTE: Should be cleared when is_active_low is set
                bool pullup_to_reg1 : 1;  // OPT[2]
                // clear: High uses REG18
                // set: High uses REG1
                bool drive_high_uses_reg1 : 1;  // OPT[3]
                // set: acts as BOTHOFF
                bool is_both_off : 1;  // OPT[4]
                // _rsvd6 clear: acts as DFETOFF
                bool is_active_low : 1;  // OPT[5]
            } function;
            struct
            {
                adc_pin_function function : 2;              // PINFXN[1:0]
                adc_measurement_type measurement_type : 2;  // OPT[1:0]
                polynomial_selection polynomial : 2;        // OPT[3:2]
                adc_pullup_config pullup_config : 2;        // OPT[5:4]
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
                bool enable_pulldown : 1;         // OPT[0]
                // clear: Driving high drives to HI-Z (unavailable with is_active_low)
                // set: Driving high drives to selected regulator
                bool disable_high_Z_drive : 1;  // OPT[1]
                // NOTE: Should be cleared when is_active_low is set
                bool pullup_to_reg1 : 1;  // OPT[2]
                // clear: High uses REG18
                // set: High uses REG1
                bool drive_high_uses_reg1 : 1;  // OPT[3]
                bool _rsvd6 : 1;                // OPT[4]
                bool is_active_low : 1;         // OPT[5]
            } function;
            struct
            {
                adc_pin_function function : 2;              // PINFXN[1:0]
                adc_measurement_type measurement_type : 2;  // OPT[1:0]
                polynomial_selection polynomial : 2;        // OPT[3:2]
                adc_pullup_config pullup_config : 2;        // OPT[5:4]
            } adc;
        };
    };
    struct ts_pin_configuration_t
    {
        adc_pin_function function : 2;              // PINFXN[1:0]
        adc_measurement_type measurement_type : 2;  // OPT[1:0]
        polynomial_selection polynomial : 2;        // OPT[3:2]
        adc_pullup_config pullup_config : 2;        // OPT[5:4]
    };
    struct hdq_pin_configuration_t
    {
        union
        {
            struct
            {
                hdq_pin_function function : 2;  // PINFXN[1:0]
                bool enable_pulldown : 1;       // OPT[0]
                // clear: Driving high drives to HI-Z (unavailable with is_active_low)
                // set: Driving high drives to selected regulator
                bool disable_high_Z_drive : 1;  // OPT[1]
                // NOTE: Should be cleared when is_active_low is set
                bool pullup_to_reg1 : 1;  // OPT[2]
                // clear: High uses REG18
                // set: High uses REG1
                bool drive_high_uses_reg1 : 1;  // OPT[3]
                bool _rsvd6 : 1;                // OPT[4]
                bool is_active_low : 1;         // OPT[5]
            } function;
            struct
            {
                adc_pin_function function : 2;              // PINFXN[1:0]
                adc_measurement_type measurement_type : 2;  // OPT[1:0]
                polynomial_selection polynomial : 2;        // OPT[3:2]
                adc_pullup_config pullup_config : 2;        // OPT[5:4]
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
                bool enable_pulldown : 1;        // OPT[0]
                // clear: Driving high drives to HI-Z (unavailable with is_active_low)
                // set: Driving high drives to selected regulator
                bool disable_high_Z_drive : 1;  // OPT[1]
                // NOTE: Should be cleared when is_active_low is set
                bool pullup_to_reg1 : 1;  // OPT[2]
                // clear: High uses REG18
                // set: High uses REG1
                bool drive_high_uses_reg1 : 1;  // OPT[3]
                bool _rsvd6 : 1;                // OPT[4]
                bool is_active_low : 1;         // OPT[5]
            } function;
            struct
            {
                adc_pin_function function : 2;              // PINFXN[1:0]
                adc_measurement_type measurement_type : 2;  // OPT[1:0]
                polynomial_selection polynomial : 2;        // OPT[3:2]
                adc_pullup_config pullup_config : 2;        // OPT[5:4]
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
                bool enable_pulldown : 1;        // OPT[0]
                // clear: Driving high drives to HI-Z (unavailable with is_active_low)
                // set: Driving high drives to selected regulator
                bool disable_high_Z_drive : 1;  // OPT[1]
                // NOTE: Should be cleared when is_active_low is set
                bool pullup_to_reg1 : 1;  // OPT[2]
                // clear: High uses REG18
                // set: High uses REG1
                bool drive_high_uses_reg1 : 1;  // OPT[3]
                bool _rsvd6 : 1 = 0;            // OPT[4]
                bool is_active_low : 1;         // OPT[5]
            } function;
            struct
            {
                adc_pin_function function : 2;              // PINFXN[1:0]
                adc_measurement_type measurement_type : 2;  // OPT[1:0]
                polynomial_selection polynomial : 2;        // OPT[3:2]
                adc_pullup_config pullup_config : 2;        // OPT[5:4]
            } adc;
        };
    };
    struct da_configuration_t
    {
        user_amps_unit user_amps : 2 = user_amps_unit::MILLIAMP;  // USER_AMPS_[1:0]
        // clear: User volts is mV
        // set: User volts is cV (10mV)
        bool user_volts_is_centivolts : 1 = 0;          // USER_VOLTS_CV
        bool use_internal_as_cell_temperature : 1 = 0;  // TINT_EN
        bool use_internal_as_fet_temperature : 1 = 0;   // TINT_FETT
        uint8_t _rsvd5 : 3 = 0;                         // RSVD_0
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
        bool permanent_fail_stops_fets : 1 = 1;  // PF_FETS
        // if set, a PF will turn off the regulators
        bool permanent_fail_stops_regulators : 1 = 0;  // PF_REGS
        // if set, will enter deepsleep after writing OTP (if applicable) in a PF event
        bool permanent_fail_causes_deep_sleep : 1 = 0;  // PF_DPSLP
        // if set, will blow the fuse after a PF event
        bool permanent_fail_blows_fuse : 1 = 0;  // PF_FUSE
        // if set, will write PF status to the OTP
        bool permanent_fail_writes_otp : 1 = 0;  // PF_OTP
        bool _rsvd6 : 1 = 0;                     // RSVD_0
        // if set, will use PACK voltage instead of TOS (Top Of Stack)
        bool fuse_voltage_uses_pack_voltage : 1 = 0;  // PACK_FUSE
        // a FET Permanent Failure will ignore the min blow fuse voltage if this is set
        bool fet_fault_ignores_fuse_voltage : 1 = 0;     // FETF_FUSE
        bool use_overcurrent_charge_recovery : 1 = 0;    // OCDL_CURR_RECOVERY
        bool use_short_circuit_charge_recovery : 1 = 0;  // SCDL_CURR_RECOVERY
        uint8_t _rsvd11 : 5 = 0;                         // RSVD_0
    };
    struct chg_fet_protections_a_t
    {
        uint8_t _rsvd0 : 3 = 0;            // RSVD_0
        bool cell_overvoltage : 1;         // COV
        bool overcurrent_charge : 1;       // OCC
        uint8_t _rsvd5 : 2 = 0;            // RSVD_0
        bool short_circuit_discharge : 1;  // SCD
    };
    struct chg_fet_protections_b_t
    {
        bool undertemp_charge : 1;    // UTC
        bool _rsvd1 : 1 = 0;          // RSVD_0
        bool internal_undertemp : 1;  // UTINT
        bool _rsvd3 : 1 = 0;          // RSVD_0
        bool overtemp_charge : 1;     // OTC
        bool _rsvd5 : 1 = 0;          // RSVD_0
        bool internal_overtemp : 1;   // OTINT
        bool fet_overtemp : 1;        // OTF
    };
    struct chg_fet_protections_c_t
    {
        bool _rsvd0 : 1 = 0;                     // RSVD_0
        bool host_watchdog_fault : 1;            // HWDF
        bool precharge_timeout : 1;              // PTO
        bool _rsvd3 : 1 = 0;                     // RSVD_0
        bool cell_overvoltage_latch : 1;         // COVL
        bool _rsvd5 : 1 = 0;                     // RSVD_0
        bool short_circuit_discharge_latch : 1;  // SCDL
        bool _rsvd7 : 1 = 0;                     // RSVD_0
    };
    struct dsg_fet_protections_a_t
    {
        uint8_t _rsvd0 : 2 = 0;            // RSVD_0
        bool cell_undervoltage : 1;        // CUV
        uint8_t _rsvd3 : 2 = 0;            // RSVD_0
        bool overcurrent_discharge_1 : 1;  // OCD1
        bool overcurrent_discharge_2 : 1;  // OCD2
        bool short_circuit_discharge : 1;  // SCD
    };
    struct dsg_fet_protections_b_t
    {
        bool _rsvd0 : 1 = 0;           // RSVD_0
        bool undertemp_discharge : 1;  // UTD
        bool internal_undertemp : 1;   // UTINT
        uint8_t _rsvd3 : 2 = 0;        // RSVD_0
        bool overtemp_discharge : 1;   // OTD
        bool internal_overtemp : 1;    // OTINT
        bool fet_overtemp : 1;         // OTF
    };
    struct dsg_fet_protections_c_t
    {
        bool _rsvd0 : 1 = 0;                     // RSVD_0
        bool host_watchdog_fault : 1;            // HWDF
        uint8_t _rsvd2 : 3 = 0;                  // RSVD_0
        bool overcurrent_discharge_latch : 1;    // OCDL
        bool short_circuit_discharge_latch : 1;  // SCDL
        bool overcurrent_discharge_3 : 1;        // OCD3
    };
    struct alarm_sf_alert_mask_c_t
    {
        bool _rsvd0 : 1 = 0;                     // RSVD_0
        bool _rsvd1_set : 1 = 1;                 // RSVD_1
        bool precharge_timeout : 1;              // PTO
        bool _rsvd3 : 1 = 0;                     // RSVD_0
        bool cell_overvoltage_latch : 1;         // COVL
        bool overcurrent_discharge_latch : 1;    // OCDL
        bool short_circuit_discharge_latch : 1;  // SCDL
        bool overcurrent_discharge_3 : 1;        // OCD3
    };
    struct fet_options_t
    {
        bool enable_body_diode_protection : 1 = 1;    // SFET (Series FET)
        bool allow_charge_in_sleep : 1 = 0;           // SLEEPCHG
        bool allow_host_fet_control : 1 = 1;          // HOST_FET_EN
        bool enable_fet_control : 1 = 1;              // FET_CTRL_EN
        bool enable_predischarge : 1 = 0;             // PDSG_EN
        bool host_control_defaults_fets_off : 1 = 0;  // FET_INIT_OFF
        uint8_t _rsvd6 : 2 = 0;                       // RSVD_0
    };
    struct fet_charge_pump_control_t
    {
        // NOTE: If cleared, enable_fet_control should likely also be cleared
        bool enable_charge_pump : 1;  // CP_EN
        // clear: Charge pump uses 11V overdrive
        // set: Charge pump uses 5.5V overdrive
        bool use_low_overdrive : 1;  // LVEN
        // NOTE: Normally only used when allow_charge_in_sleep is cleared
        bool enable_sleep_dsg_source_follower : 1;  // SFMODE_SLEEP
        uint8_t : 5;                                // RSVD_0
    };
    struct manufacturing_status_init_t
    {
        uint8_t _rsvd0 : 4 = 0;              // RSVD_0
        bool autonomous_fets : 1;            // FET_EN
        bool _rsvd6 : 1 = 0;                 // RSVD_0
        bool is_permanent_fail_enabled : 1;  // PF_EN
        bool is_otp_write_enabled : 1;       // OTPW_EN
        uint8_t _rsvd8 : 8 = 0;              // RSVD_0
    };
    struct balancing_configuration_t
    {
        bool allow_charging_cell_balancing : 1 = 0;     // CB_CHG
        bool allow_relaxed_cell_balancing : 1 = 0;      // CB_RELAX
        bool allow_balancing_in_sleep : 1 = 0;          // CB_SLEEP
        bool balancing_exits_sleep : 1 = 0;             // CB_NOSLEEP
        bool ignore_host_controlled_balancing : 1 = 0;  // CB_NO_CMD
        uint8_t _rsvd5 : 3 = 0;                         // RSVD_0
    };
    struct security_settings_t
    {
        bool default_to_sealed : 1;  // SEAL
        bool lock_config : 1;        // LOCK_CFG
        bool permanent_seal : 1;     // PERM_SEAL
        uint8_t _rsvd3 : 5 = 0;      // RSVD_0
    };
#pragma pack(pop)

    typedef std::array<uint8_t, 32> manufacturer_data_t;

    struct da_status_5_t
    {
        int16_t vreg_18_adc_counts;
        int16_t vss_adc_counts;
        int16_t max_cell_voltage;
        int16_t min_cell_voltage;
        int32_t battery_voltage_sum;
        float cell_temperature;
        float fet_temperature;
        float max_cell_temperature;
        float min_cell_temperature;
        float avg_cell_temperature;
        float cc_3_current;
        float cc_1_current;
        int32_t cc_2_counts;
        int32_t cc_3_counts;
    };
    struct da_status_6_t
    {
        float accumulated_charge;
        std::chrono::seconds accumulated_charge_time;
        int32_t cfetoff_counts;
        int32_t dfetoff_counts;
        int32_t alert_counts;
        int32_t ts1_counts;
        int32_t ts2_counts;
    };

    constexpr static auto SUBCOMMAND_TIMEOUT = std::chrono::milliseconds(20);
    constexpr static int MAX_RETRIES = 3;

    bq76942(uint8_t address = 0x08);

    // --- DIRECT COMMANDS ---
    control_status_t get_control_status() { return read_direct<control_status_t>(direct_command::CONTROL_STATUS); }
    protections_a_t get_safety_alert_a() { return read_direct<protections_a_t>(direct_command::SAFETY_ALERT_A); }
    protections_a_t get_safety_status_a() { return read_direct<protections_a_t>(direct_command::SAFETY_STATUS_A); }
    protections_b_t get_safety_alert_b() { return read_direct<protections_b_t>(direct_command::SAFETY_ALERT_B); }
    protections_b_t get_safety_status_b() { return read_direct<protections_b_t>(direct_command::SAFETY_STATUS_B); }
    safety_alert_c_t get_safety_alert_c() { return read_direct<safety_alert_c_t>(direct_command::SAFETY_ALERT_C); }
    protections_c_t get_safety_status_c() { return read_direct<protections_c_t>(direct_command::SAFETY_STATUS_C); }
    permanent_fail_a_t get_permanent_fail_alert_a() { return read_direct<permanent_fail_a_t>(direct_command::PF_ALERT_A); }
    permanent_fail_a_t get_permanent_fail_status_a() { return read_direct<permanent_fail_a_t>(direct_command::PF_STATUS_A); }
    permanent_fail_b_t get_permanent_fail_alert_b() { return read_direct<permanent_fail_b_t>(direct_command::PF_ALERT_B); }
    permanent_fail_b_t get_permanent_fail_status_b() { return read_direct<permanent_fail_b_t>(direct_command::PF_STATUS_B); }
    permanent_fail_alert_c_t get_permanent_fail_alert_c() { return read_direct<permanent_fail_alert_c_t>(direct_command::PF_ALERT_C); }
    permanent_fail_c_t get_permanent_fail_status_c() { return read_direct<permanent_fail_c_t>(direct_command::PF_STATUS_C); }
    permanent_fail_d_t get_permanent_fail_alert_d() { return read_direct<permanent_fail_d_t>(direct_command::PF_ALERT_D); }
    permanent_fail_d_t get_permanent_fail_status_d() { return read_direct<permanent_fail_d_t>(direct_command::PF_STATUS_D); }
    battery_status_t get_battery_status() { return read_direct<battery_status_t>(direct_command::BATTERY_STATUS); }
    /// @brief Read the voltage of one of the cells
    /// @return The voltage of the cell in mV
    int16_t get_cell_voltage(const uint8_t cell);
    /// @brief Read the voltage at the top of the battery stack
    /// @return The voltage of the stack in mV
    int32_t get_stack_voltage();
    int32_t get_pack_voltage();
    int32_t get_ld_voltage();
    float get_CC2_current();
    alarm_status_t get_alarm_status() { return read_direct<alarm_status_t>(direct_command::ALARM_STATUS); }
    void clear_alarm_status(const alarm_status_t& alarm_status) { write_direct(direct_command::ALARM_STATUS, alarm_status); }
    alarm_status_t get_alarm_raw_status() { return read_direct<alarm_status_t>(direct_command::ALARM_RAW_STATUS); }
    alarm_status_t get_alarm_enable() { return read_direct<alarm_status_t>(direct_command::ALARM_ENABLE); }
    void set_alarm_enable(const alarm_status_t& alarm_enable) { write_direct(direct_command::ALARM_ENABLE, alarm_enable); }
    float get_temperature(const temperature_sensor& sensor) { return raw_temp_to_celsius(read_direct<int16_t>(static_cast<direct_command>(sensor))); }
    fet_status_t get_fet_status() { return read_direct<fet_status_t>(direct_command::FET_STATUS); }

    // --- COMMAND ONLY SUBCOMMANDS ---
    void enter_deep_sleep();
    void exit_deep_sleep() { write_subcommand(cmd_only_subcommand::EXIT_DEEPSLEEP); }
    // WARNING: THIS WILL POWER DOWN THE BMS COMPLETELY!
    // It can only be woken up by either power cycling or pulling down the TS2 pin!
    /// @brief Put the BMS into SHUTDOWN mode
    void shutdown();
    void reset() { write_subcommand(cmd_only_subcommand::RESET); }
    void predischarge_test() { write_subcommand(cmd_only_subcommand::PDSG_TEST); }
    void toggle_fuse() { write_subcommand(cmd_only_subcommand::FUSE_TOGGLE); }
    void precharge_test() { write_subcommand(cmd_only_subcommand::PCHG_TEST); }
    void charge_test() { write_subcommand(cmd_only_subcommand::CHG_TEST); }
    void discharge_test() { write_subcommand(cmd_only_subcommand::DSG_TEST); }
    void toggle_fet_test_mode() { write_subcommand(cmd_only_subcommand::FET_ENABLE); }
    void toggle_permanent_fail_enabled() { write_subcommand(cmd_only_subcommand::PF_ENABLE); }
    void seal() { write_subcommand(cmd_only_subcommand::SEAL); }
    void reset_charge_counter() { write_subcommand(cmd_only_subcommand::RESET_PASSQ); }
    void reset_passQ() { reset_charge_counter(); }
    void recover_precharge_timeout() { write_subcommand(cmd_only_subcommand::PTO_RECOVER); }
    void enter_config_update_mode();
    void exit_config_update_mode();
    void disable_discharge_fets() { write_subcommand(cmd_only_subcommand::DSG_PDSG_OFF); }
    void disable_charge_fets() { write_subcommand(cmd_only_subcommand::CHG_PCHG_OFF); }
    void disable_all_fets() { write_subcommand(cmd_only_subcommand::ALL_FETS_OFF); }
    void enable_all_fets() { write_subcommand(cmd_only_subcommand::ALL_FETS_ON); }
    void enable_sleep() { write_subcommand(cmd_only_subcommand::SLEEP_ENABLE); }
    void disable_sleep() { write_subcommand(cmd_only_subcommand::SLEEP_DISABLE); }
    void set_sleep_enabled(const bool enabled) { enabled ? enable_sleep() : disable_sleep(); }
    void recover_discharge_overcurrent_latch() { write_subcommand(cmd_only_subcommand::OCDL_RECOVER); }
    void recover_short_circuit_discharge_latch() { write_subcommand(cmd_only_subcommand::SCDL_RECOVER); }
    void restart_load_detect() { write_subcommand(cmd_only_subcommand::LOAD_DETECT_RESTART); }
    void force_load_detect_on() { write_subcommand(cmd_only_subcommand::LOAD_DETECT_ON); }
    void force_load_detect_off() { write_subcommand(cmd_only_subcommand::LOAD_DETECT_OFF); }
    void set_GPO(const gpo_state& state) { write_subcommand(static_cast<cmd_only_subcommand>(state)); }
    void set_GPO(const gpo& pin, const bool state);
    void force_permanent_fail();
    // Comm mode commands NOT IMPLEMENTED as this driver only supports I2C with CRC.
    // However, to support new modes would only require implementing the read/write_direct functions for the new mode,
    // since subcommands use those under the hood.
    // void swap_comm_mode();
    // void swap_to_I2C();
    // void swap_to_SPI();
    // void swap_to_HDQ();

    // --- SUBCOMMANDS ---
    uint16_t get_device_number() { return read_subcommand<uint16_t>(subcommand::DEVICE_NUMBER); };
    firmware_version_t get_firmware_version();
    uint16_t get_hardware_version() { return read_subcommand<uint16_t>(subcommand::HW_VERSION); };
    uint16_t get_IROM_signature() { return read_subcommand<uint16_t>(subcommand::IROM_SIG); };
    uint16_t get_static_config_signature() { return read_subcommand<uint16_t>(subcommand::STATIC_CFG_SIG); };
    // Intended for TI internal use
    // uint16_t get_prev_mac_write() { return read_subcommand<uint16_t>(subcommand::PREV_MAC_WRITE); };
    uint16_t get_DROM_signature() { return read_subcommand<uint16_t>(subcommand::DROM_SIG); };
    security_keys_t get_security_keys();
    void set_security_keys(const security_keys_t& keys);
    saved_pf_status_t get_saved_PF_status() { return read_subcommand<saved_pf_status_t>(subcommand::SAVED_PF_STATUS); };
    manufacturing_status_t get_manufacturing_status() { return read_subcommand<manufacturing_status_t>(subcommand::MANUFACTURING_STATUS); };
    manufacturer_data_t get_manufacturer_data();
    void set_manufacturer_data(const manufacturer_data_t& data);
    da_status_1_t get_DA_status_1() { return read_subcommand<da_status_1_t>(subcommand::DA_STATUS_1); };
    da_status_2_t get_DA_status_2() { return read_subcommand<da_status_2_t>(subcommand::DA_STATUS_2); };
    da_status_3_t get_DA_status_3() { return read_subcommand<da_status_3_t>(subcommand::DA_STATUS_3); };
    raw_da_status_5_t get_raw_DA_status_5() { return read_subcommand<raw_da_status_5_t>(subcommand::DA_STATUS_5); };
    da_status_5_t get_DA_status_5();
    raw_da_status_6_t get_raw_DA_status_6() { return read_subcommand<raw_da_status_6_t>(subcommand::DA_STATUS_6); };
    da_status_6_t get_DA_status_6();
    da_status_7_t get_DA_status_7() { return read_subcommand<da_status_7_t>(subcommand::DA_STATUS_7); };
    voltage_snapshot_t get_cell_under_volt_snapshot() { return read_subcommand<voltage_snapshot_t>(subcommand::CUV_SNAPSHOT); };
    voltage_snapshot_t get_cell_over_volt_snapshot() { return read_subcommand<voltage_snapshot_t>(subcommand::COV_SNAPSHOT); };
    selected_cells_t get_CB_active_cells() { return read_subcommand<selected_cells_t>(subcommand::CB_ACTIVE_CELLS); };
    void set_CB_active_cells(const selected_cells_t& cells) { write_subcommand(subcommand::CB_ACTIVE_CELLS, cells); }
    std::chrono::seconds read_cell_balancing_time() { return std::chrono::seconds(read_subcommand<uint16_t>(subcommand::CB_STATUS_1)); };
    cb_status_2_t get_CB_status_2() { return read_subcommand<cb_status_2_t>(subcommand::CB_STATUS_2); };
    cb_status_3_t get_CB_status_3() { return read_subcommand<cb_status_3_t>(subcommand::CB_STATUS_3); };
    void set_fet_control(const fet_control_t& control) { write_subcommand(subcommand::FET_CONTROL, control); }
    void set_regulator_control(const regulator_control_t& control) { write_subcommand(subcommand::REG12_CONTROL, control); }
    otp_write_result_t get_otp_write_check_result() { return read_subcommand<otp_write_result_t>(subcommand::OTP_WR_CHECK); }
    otp_write_result_t get_otp_write_result() { return read_subcommand<otp_write_result_t>(subcommand::OTP_WRITE); }
    cal1_t get_cal_1() { return read_subcommand<cal1_t>(subcommand::READ_CAL1); }
    uint16_t calibrate_cell_under_volt_threshold() { return read_subcommand<uint16_t>(subcommand::CAL_CUV); }
    uint16_t calibrate_cell_over_volt_threshold() { return read_subcommand<uint16_t>(subcommand::CAL_COV); }

    // --- DATA REGISTERS ---
    class Calibration final
    {
    public:
        // Calibration:Voltage
        class Voltage final
        {
        public:
            int16_t get_cell_gain(const uint8_t& cell) const;
            void set_cell_gain(const uint8_t& cell, const int16_t& gain) const;
            uint16_t get_pack_gain() const { return _parent.read_subcommand<uint16_t>(data_register::PACK_GAIN); }
            void set_pack_gain(const uint16_t& gain) const { _parent.write_subcommand(data_register::PACK_GAIN, gain); }
            uint16_t get_stack_gain() const { return _parent.read_subcommand<uint16_t>(data_register::TOS_GAIN); }
            void set_stack_gain(const uint16_t& gain) const { _parent.write_subcommand(data_register::TOS_GAIN, gain); }
            uint16_t get_ld_gain() const { return _parent.read_subcommand<uint16_t>(data_register::LD_GAIN); }
            void set_ld_gain(const uint16_t& gain) const { _parent.write_subcommand(data_register::LD_GAIN, gain); }
            int16_t get_adc_gain() const { return _parent.read_subcommand<int16_t>(data_register::ADC_GAIN); }
            void set_adc_gain(const int16_t& gain) const { _parent.write_subcommand(data_register::ADC_GAIN, gain); }

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
            void set_sense_resistor_value(const float& value) const;
            float get_CC_gain() const { return _parent.read_subcommand<float>(data_register::CC_GAIN); }
            void set_CC_gain(const float& gain) const { _parent.write_subcommand(data_register::CC_GAIN, gain); }
            float get_capacity_gain() const { return _parent.read_subcommand<float>(data_register::CAPACITY_GAIN); }
            void set_capacity_gain(const float& gain) const { _parent.write_subcommand(data_register::CAPACITY_GAIN, gain); }

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
        int16_t get_vcell_offset() const { return _parent.read_subcommand<int16_t>(data_register::VCELL_OFFSET); }
        void set_vcell_offset(const int16_t& offset) const { _parent.write_subcommand(data_register::VCELL_OFFSET, offset); }

        // Calibration:V Divider Offset
        int16_t get_V_divider_offset() const { return _parent.read_subcommand<int16_t>(data_register::VDIV_OFFSET); }
        void set_V_divider_offset(const int16_t& offset) const { _parent.write_subcommand(data_register::VDIV_OFFSET, offset); }

        // Calibration:Current Offset
        class CurrentOffset final
        {
        public:
            uint16_t get_coulomb_counter_offset_samples() const { return _parent.read_subcommand<uint16_t>(data_register::COULOMB_COUNTER_OFFSET_SAMPLES); }
            void set_coulomb_counter_offset_samples(const uint16_t& samples) const { _parent.write_subcommand(data_register::COULOMB_COUNTER_OFFSET_SAMPLES, samples); }
            int16_t get_board_offset_current() const { return _parent.read_subcommand<int16_t>(data_register::BOARD_OFFSET); }
            void set_board_offset_current(const int16_t& offset) const { _parent.write_subcommand(data_register::BOARD_OFFSET, offset); }

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
            int8_t get_internal_offset() const { return _parent.read_subcommand<int8_t>(data_register::INTERNAL_TEMP_OFFSET); }
            void set_internal_offset(const int8_t& offset) const { _parent.write_subcommand(data_register::INTERNAL_TEMP_OFFSET, offset); }
            int8_t get_cfetoff_offset() const { return _parent.read_subcommand<int8_t>(data_register::CFETOFF_TEMP_OFFSET); }
            void set_cfetoff_offset(const int8_t& offset) const { _parent.write_subcommand(data_register::CFETOFF_TEMP_OFFSET, offset); }
            int8_t get_dfetoff_offset() const { return _parent.read_subcommand<int8_t>(data_register::DFETOFF_TEMP_OFFSET); }
            void set_dfetoff_offset(const int8_t& offset) const { _parent.write_subcommand(data_register::DFETOFF_TEMP_OFFSET, offset); }
            int8_t get_alert_offset() const { return _parent.read_subcommand<int8_t>(data_register::ALERT_TEMP_OFFSET); }
            void set_alert_offset(const int8_t& offset) const { _parent.write_subcommand(data_register::ALERT_TEMP_OFFSET, offset); }
            int8_t get_ts1_offset() const { return _parent.read_subcommand<int8_t>(data_register::TS1_TEMP_OFFSET); }
            void set_ts1_offset(const int8_t& offset) const { _parent.write_subcommand(data_register::TS1_TEMP_OFFSET, offset); }
            int8_t get_ts2_offset() const { return _parent.read_subcommand<int8_t>(data_register::TS2_TEMP_OFFSET); }
            void set_ts2_offset(const int8_t& offset) const { _parent.write_subcommand(data_register::TS2_TEMP_OFFSET, offset); }
            int8_t get_ts3_offset() const { return _parent.read_subcommand<int8_t>(data_register::TS3_TEMP_OFFSET); }
            void set_ts3_offset(const int8_t& offset) const { _parent.write_subcommand(data_register::TS3_TEMP_OFFSET, offset); }
            int8_t get_hdq_offset() const { return _parent.read_subcommand<int8_t>(data_register::HDQ_TEMP_OFFSET); }
            void set_hdq_offset(const int8_t& offset) const { _parent.write_subcommand(data_register::HDQ_TEMP_OFFSET, offset); }
            int8_t get_dchg_offset() const { return _parent.read_subcommand<int8_t>(data_register::DCHG_TEMP_OFFSET); }
            void set_dchg_offset(const int8_t& offset) const { _parent.write_subcommand(data_register::DCHG_TEMP_OFFSET, offset); }
            int8_t get_ddsg_offset() const { return _parent.read_subcommand<int8_t>(data_register::DDSG_TEMP_OFFSET); }
            void set_ddsg_offset(const int8_t& offset) const { _parent.write_subcommand(data_register::DDSG_TEMP_OFFSET, offset); }

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
            int16_t get_gain() const { return _parent.read_subcommand<int16_t>(data_register::INT_GAIN); }
            void set_gain(const int16_t& gain) const { _parent.write_subcommand(data_register::INT_GAIN, gain); }
            int16_t get_base_offset() const { return _parent.read_subcommand<int16_t>(data_register::INT_BASE_OFFSET); }
            void set_base_offset(const int16_t& offset) const { _parent.write_subcommand(data_register::INT_BASE_OFFSET, offset); }
            int16_t get_maximum_AD() const { return _parent.read_subcommand<int16_t>(data_register::INT_MAXIMUM_AD); }
            void set_maximum_AD(const int16_t& ad) const { _parent.write_subcommand(data_register::INT_MAXIMUM_AD, ad); }
            int16_t get_maximum_temp() const { return _parent.read_subcommand<int16_t>(data_register::INT_MAXIMUM_TEMP); }
            void set_maximum_temp(const int16_t& temp) const { _parent.write_subcommand(data_register::INT_MAXIMUM_TEMP, temp); }

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
            int16_t get_coeff_A1() const { return _parent.read_subcommand<int16_t>(data_register::T18K_COEFF_A1); }
            void set_coeff_A1(const int16_t& coeff) const { _parent.write_subcommand(data_register::T18K_COEFF_A1, coeff); }
            int16_t get_coeff_A2() const { return _parent.read_subcommand<int16_t>(data_register::T18K_COEFF_A2); }
            void set_coeff_A2(const int16_t& coeff) const { _parent.write_subcommand(data_register::T18K_COEFF_A2, coeff); }
            int16_t get_coeff_A3() const { return _parent.read_subcommand<int16_t>(data_register::T18K_COEFF_A3); }
            void set_coeff_A3(const int16_t& coeff) const { _parent.write_subcommand(data_register::T18K_COEFF_A3, coeff); }
            int16_t get_coeff_A4() const { return _parent.read_subcommand<int16_t>(data_register::T18K_COEFF_A4); }
            void set_coeff_A4(const int16_t& coeff) const { _parent.write_subcommand(data_register::T18K_COEFF_A4, coeff); }
            int16_t get_coeff_A5() const { return _parent.read_subcommand<int16_t>(data_register::T18K_COEFF_A5); }
            void set_coeff_A5(const int16_t& coeff) const { _parent.write_subcommand(data_register::T18K_COEFF_A5, coeff); }
            int16_t get_coeff_B1() const { return _parent.read_subcommand<int16_t>(data_register::T18K_COEFF_B1); }
            void set_coeff_B1(const int16_t& coeff) const { _parent.write_subcommand(data_register::T18K_COEFF_B1, coeff); }
            int16_t get_coeff_B2() const { return _parent.read_subcommand<int16_t>(data_register::T18K_COEFF_B2); }
            void set_coeff_B2(const int16_t& coeff) const { _parent.write_subcommand(data_register::T18K_COEFF_B2, coeff); }
            int16_t get_coeff_B3() const { return _parent.read_subcommand<int16_t>(data_register::T18K_COEFF_B3); }
            void set_coeff_B3(const int16_t& coeff) const { _parent.write_subcommand(data_register::T18K_COEFF_B3, coeff); }
            int16_t get_coeff_B4() const { return _parent.read_subcommand<int16_t>(data_register::T18K_COEFF_B4); }
            void set_coeff_B4(const int16_t& coeff) const { _parent.write_subcommand(data_register::T18K_COEFF_B4, coeff); }
            int16_t get_adc0() const { return _parent.read_subcommand<int16_t>(data_register::T18K_ADC0); }
            void set_adc0(const int16_t& adc) const { _parent.write_subcommand(data_register::T18K_ADC0, adc); }

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
            int16_t get_coeff_A1() const { return _parent.read_subcommand<int16_t>(data_register::T180K_COEFF_A1); }
            void set_coeff_A1(const int16_t& coeff) const { _parent.write_subcommand(data_register::T180K_COEFF_A1, coeff); }
            int16_t get_coeff_A2() const { return _parent.read_subcommand<int16_t>(data_register::T180K_COEFF_A2); }
            void set_coeff_A2(const int16_t& coeff) const { _parent.write_subcommand(data_register::T180K_COEFF_A2, coeff); }
            int16_t get_coeff_A3() const { return _parent.read_subcommand<int16_t>(data_register::T180K_COEFF_A3); }
            void set_coeff_A3(const int16_t& coeff) const { _parent.write_subcommand(data_register::T180K_COEFF_A3, coeff); }
            int16_t get_coeff_A4() const { return _parent.read_subcommand<int16_t>(data_register::T180K_COEFF_A4); }
            void set_coeff_A4(const int16_t& coeff) const { _parent.write_subcommand(data_register::T180K_COEFF_A4, coeff); }
            int16_t get_coeff_A5() const { return _parent.read_subcommand<int16_t>(data_register::T180K_COEFF_A5); }
            void set_coeff_A5(const int16_t& coeff) const { _parent.write_subcommand(data_register::T180K_COEFF_A5, coeff); }
            int16_t get_coeff_B1() const { return _parent.read_subcommand<int16_t>(data_register::T180K_COEFF_B1); }
            void set_coeff_B1(const int16_t& coeff) const { _parent.write_subcommand(data_register::T180K_COEFF_B1, coeff); }
            int16_t get_coeff_B2() const { return _parent.read_subcommand<int16_t>(data_register::T180K_COEFF_B2); }
            void set_coeff_B2(const int16_t& coeff) const { _parent.write_subcommand(data_register::T180K_COEFF_B2, coeff); }
            int16_t get_coeff_B3() const { return _parent.read_subcommand<int16_t>(data_register::T180K_COEFF_B3); }
            void set_coeff_B3(const int16_t& coeff) const { _parent.write_subcommand(data_register::T180K_COEFF_B3, coeff); }
            int16_t get_coeff_B4() const { return _parent.read_subcommand<int16_t>(data_register::T180K_COEFF_B4); }
            void set_coeff_B4(const int16_t& coeff) const { _parent.write_subcommand(data_register::T180K_COEFF_B4, coeff); }
            int16_t get_adc0() const { return _parent.read_subcommand<int16_t>(data_register::T180K_ADC0); }
            void set_adc0(const int16_t& adc) const { _parent.write_subcommand(data_register::T180K_ADC0, adc); }

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
            int16_t get_coeff_A1() const { return _parent.read_subcommand<int16_t>(data_register::CUSTOM_COEFF_A1); }
            void set_coeff_A1(const int16_t& coeff) const { _parent.write_subcommand(data_register::CUSTOM_COEFF_A1, coeff); }
            int16_t get_coeff_A2() const { return _parent.read_subcommand<int16_t>(data_register::CUSTOM_COEFF_A2); }
            void set_coeff_A2(const int16_t& coeff) const { _parent.write_subcommand(data_register::CUSTOM_COEFF_A2, coeff); }
            int16_t get_coeff_A3() const { return _parent.read_subcommand<int16_t>(data_register::CUSTOM_COEFF_A3); }
            void set_coeff_A3(const int16_t& coeff) const { _parent.write_subcommand(data_register::CUSTOM_COEFF_A3, coeff); }
            int16_t get_coeff_A4() const { return _parent.read_subcommand<int16_t>(data_register::CUSTOM_COEFF_A4); }
            void set_coeff_A4(const int16_t& coeff) const { _parent.write_subcommand(data_register::CUSTOM_COEFF_A4, coeff); }
            int16_t get_coeff_A5() const { return _parent.read_subcommand<int16_t>(data_register::CUSTOM_COEFF_A5); }
            void set_coeff_A5(const int16_t& coeff) const { _parent.write_subcommand(data_register::CUSTOM_COEFF_A5, coeff); }
            int16_t get_coeff_B1() const { return _parent.read_subcommand<int16_t>(data_register::CUSTOM_COEFF_B1); }
            void set_coeff_B1(const int16_t& coeff) const { _parent.write_subcommand(data_register::CUSTOM_COEFF_B1, coeff); }
            int16_t get_coeff_B2() const { return _parent.read_subcommand<int16_t>(data_register::CUSTOM_COEFF_B2); }
            void set_coeff_B2(const int16_t& coeff) const { _parent.write_subcommand(data_register::CUSTOM_COEFF_B2, coeff); }
            int16_t get_coeff_B3() const { return _parent.read_subcommand<int16_t>(data_register::CUSTOM_COEFF_B3); }
            void set_coeff_B3(const int16_t& coeff) const { _parent.write_subcommand(data_register::CUSTOM_COEFF_B3, coeff); }
            int16_t get_coeff_B4() const { return _parent.read_subcommand<int16_t>(data_register::CUSTOM_COEFF_B4); }
            void set_coeff_B4(const int16_t& coeff) const { _parent.write_subcommand(data_register::CUSTOM_COEFF_B4, coeff); }
            int16_t get_rc0() const { return _parent.read_subcommand<int16_t>(data_register::CUSTOM_RC0); }
            void set_rc0(const int16_t& rc) const { _parent.write_subcommand(data_register::CUSTOM_RC0, rc); }
            int16_t get_adc0() const { return _parent.read_subcommand<int16_t>(data_register::CUSTOM_ADC0); }
            void set_adc0(const int16_t& adc) const { _parent.write_subcommand(data_register::CUSTOM_ADC0, adc); }

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
        int16_t get_coulomb_counter_deadband() const { return _parent.read_subcommand<int16_t>(data_register::COULOMB_COUNTER_DEADBAND); }
        void set_coulomb_counter_deadband(const int16_t& deadband) const { _parent.write_subcommand(data_register::COULOMB_COUNTER_DEADBAND, deadband); }

        // Calibration:CUV
        uint16_t get_cell_under_volt_threshold_override() const { return _parent.read_subcommand<uint16_t>(data_register::CUV_THRESHOLD_OVERRIDE); }
        void set_cell_under_volt_threshold_override(const uint16_t& threshold) const { _parent.write_subcommand(data_register::CUV_THRESHOLD_OVERRIDE, threshold); }

        // Calibration:COV
        uint16_t get_cell_over_volt_threshold_override() const { return _parent.read_subcommand<uint16_t>(data_register::COV_THRESHOLD_OVERRIDE); }
        void set_cell_over_volt_threshold_override(const uint16_t& threshold) const { _parent.write_subcommand(data_register::COV_THRESHOLD_OVERRIDE, threshold); }

        const Voltage voltage;
        const Current current;
        const CurrentOffset current_offset;
        const Temperature temperature;
        const InternalTempModel internal_temp_model;
        const T18KModel t_18K_model;
        const T180KModel t_180K_model;
        const CustomTemperatureModel custom_temperature_model;

    protected:
        Calibration(bq76942& parent)
            : voltage(parent),
              current(parent),
              current_offset(parent),
              temperature(parent),
              internal_temp_model(parent),
              t_18K_model(parent),
              t_180K_model(parent),
              custom_temperature_model(parent),
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
            int16_t get_min_blow_voltage() const { return _parent.read_subcommand<int16_t>(data_register::MIN_BLOW_FUSE_VOLTAGE); }
            void set_min_blow_voltage(const int16_t& voltage) const { _parent.write_subcommand(data_register::MIN_BLOW_FUSE_VOLTAGE, voltage); }
            std::chrono::seconds read_blow_timeout() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::FUSE_BLOW_TIMEOUT)); }
            void set_blow_timeout(const std::chrono::seconds& timeout) const { _parent.write_subcommand<uint8_t>(data_register::FUSE_BLOW_TIMEOUT, timeout.count()); }

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
            power_config_t get_power_config() const { return _parent.read_subcommand<power_config_t>(data_register::POWER_CONFIG); }
            void set_power_config(const power_config_t& config) const { _parent.write_subcommand(data_register::POWER_CONFIG, config); }
            regulator_control_t get_reg_12_config() const { return _parent.read_subcommand<regulator_control_t>(data_register::REG12_CONFIG); }
            void set_reg_12_config(const regulator_control_t& config) const { _parent.write_subcommand(data_register::REG12_CONFIG, config); }
            reg0_config_t get_reg0_config() const { return _parent.read_subcommand<reg0_config_t>(data_register::REG0_CONFIG); }
            void set_reg_0_config(const reg0_config_t& config) const { _parent.write_subcommand(data_register::REG0_CONFIG, config); }
            hwd_regulator_options_t get_hwd_regulator_options() const { return _parent.read_subcommand<hwd_regulator_options_t>(data_register::HWD_REGULATOR_OPTIONS); }
            void set_hwd_regulator_options(const hwd_regulator_options_t& options) const { _parent.write_subcommand(data_register::HWD_REGULATOR_OPTIONS, options); }
            comm_type get_comm_type() const { return static_cast<comm_type>(_parent.read_subcommand<uint8_t>(data_register::COMM_TYPE)); }
            // note: This will only apply on reset OR SWAP_COMM_MODE
            void set_comm_type(const comm_type& type) const { _parent.write_subcommand(data_register::COMM_TYPE, static_cast<uint8_t>(type)); }
            uint8_t get_i2c_address() const { return _parent.read_subcommand<uint8_t>(data_register::I2C_ADDRESS); }
            // note: This will only apply on reset OR SWAP_COMM_MODE
            void set_i2c_address(const uint8_t& address) const { _parent.write_subcommand(data_register::I2C_ADDRESS, address); }
            spi_configuration_t get_spi_configuration() const { return _parent.read_subcommand<spi_configuration_t>(data_register::SPI_CONFIGURATION); }
            void set_spi_configuration(const spi_configuration_t& config) const { _parent.write_subcommand(data_register::SPI_CONFIGURATION, config); }
            std::chrono::seconds read_comm_idle_time() const { return std::chrono::seconds(_parent.read_subcommand<uint16_t>(data_register::COMM_IDLE_TIME)); }
            void set_comm_idle_time(const std::chrono::seconds& time) const { _parent.write_subcommand(data_register::COMM_IDLE_TIME, time.count()); }
            cfetoff_pin_configuration_t get_cfetoff_pin_config() const { return _parent.read_subcommand<cfetoff_pin_configuration_t>(data_register::CFETOFF_PIN_CONFIG); }
            void set_cfetoff_pin_config(const cfetoff_pin_configuration_t& config) const { _parent.write_subcommand(data_register::CFETOFF_PIN_CONFIG, config); }
            dfetoff_pin_configuration_t get_dfetoff_pin_config() const { return _parent.read_subcommand<dfetoff_pin_configuration_t>(data_register::DFETOFF_PIN_CONFIG); }
            void set_dfetoff_pin_config(const dfetoff_pin_configuration_t& config) const { _parent.write_subcommand(data_register::DFETOFF_PIN_CONFIG, config); }
            alert_pin_configuration_t get_alert_pin_config() const { return _parent.read_subcommand<alert_pin_configuration_t>(data_register::ALERT_PIN_CONFIG); }
            void set_alert_pin_config(const alert_pin_configuration_t& config) const { _parent.write_subcommand(data_register::ALERT_PIN_CONFIG, config); }
            ts_pin_configuration_t get_ts1_pin_config() const { return _parent.read_subcommand<ts_pin_configuration_t>(data_register::TS1_CONFIG); }
            void set_ts1_pin_config(const ts_pin_configuration_t& config) const { _parent.write_subcommand(data_register::TS1_CONFIG, config); }
            ts_pin_configuration_t get_ts2_pin_config() const { return _parent.read_subcommand<ts_pin_configuration_t>(data_register::TS2_CONFIG); }
            void set_ts2_pin_config(const ts_pin_configuration_t& config) const { _parent.write_subcommand(data_register::TS2_CONFIG, config); }
            ts_pin_configuration_t get_ts3_pin_config() const { return _parent.read_subcommand<ts_pin_configuration_t>(data_register::TS3_CONFIG); }
            void set_ts3_pin_config(const ts_pin_configuration_t& config) const { _parent.write_subcommand(data_register::TS3_CONFIG, config); }
            hdq_pin_configuration_t get_hdq_pin_config() const { return _parent.read_subcommand<hdq_pin_configuration_t>(data_register::HDQ_PIN_CONFIG); }
            void set_hdq_pin_config(const hdq_pin_configuration_t& config) const { _parent.write_subcommand(data_register::HDQ_PIN_CONFIG, config); }
            dchg_pin_configuration_t get_dchg_pin_config() const { return _parent.read_subcommand<dchg_pin_configuration_t>(data_register::DCHG_PIN_CONFIG); }
            void set_dchg_pin_config(const dchg_pin_configuration_t& config) const { _parent.write_subcommand(data_register::DCHG_PIN_CONFIG, config); }
            ddsg_pin_configuration_t get_ddsg_pin_config() const { return _parent.read_subcommand<ddsg_pin_configuration_t>(data_register::DDSG_PIN_CONFIG); }
            void set_ddsg_pin_config(const ddsg_pin_configuration_t& config) const { _parent.write_subcommand(data_register::DDSG_PIN_CONFIG, config); }
            da_configuration_t get_DA_configuration() const { return _parent.read_subcommand<da_configuration_t>(data_register::DA_CONFIGURATION); }
            void set_DA_configuration(const da_configuration_t& config) const { _parent.write_subcommand(data_register::DA_CONFIGURATION, config); }
            selected_cells_t get_vcell_mode() const { return _parent.read_subcommand<selected_cells_t>(data_register::VCELL_MODE); }
            void set_vcell_mode(const selected_cells_t& mode) const { _parent.write_subcommand(data_register::VCELL_MODE, mode); }
            uint8_t get_CC3_samples() const { return _parent.read_subcommand<uint8_t>(data_register::CC3_SAMPLES); }
            void set_CC3_samples(const uint8_t& samples) const { _parent.write_subcommand(data_register::CC3_SAMPLES, samples); }

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
            protection_configuration_t get_config() const { return _parent.read_subcommand<protection_configuration_t>(data_register::PROTECTION_CONFIGURATION); }
            void set_config(const protection_configuration_t& config) const { _parent.write_subcommand(data_register::PROTECTION_CONFIGURATION, config); }
            protections_a_t get_enabled_A() const { return _parent.read_subcommand<protections_a_t>(data_register::ENABLED_PROTECTIONS_A); }
            void set_enabled_A(const protections_a_t& protections) const { _parent.write_subcommand(data_register::ENABLED_PROTECTIONS_A, protections); }
            protections_b_t get_enabled_B() const { return _parent.read_subcommand<protections_b_t>(data_register::ENABLED_PROTECTIONS_B); }
            void set_enabled_B(const protections_b_t& protections) const { _parent.write_subcommand(data_register::ENABLED_PROTECTIONS_B, protections); }
            protections_c_t get_enabled_C() const { return _parent.read_subcommand<protections_c_t>(data_register::ENABLED_PROTECTIONS_C); }
            void set_enabled_C(const protections_c_t& protections) const { _parent.write_subcommand(data_register::ENABLED_PROTECTIONS_C, protections); }
            chg_fet_protections_a_t get_chg_fet_A() const { return _parent.read_subcommand<chg_fet_protections_a_t>(data_register::CHG_FET_PROTECTIONS_A); }
            void set_chg_fet_A(const chg_fet_protections_a_t& protections) const { _parent.write_subcommand(data_register::CHG_FET_PROTECTIONS_A, protections); }
            chg_fet_protections_b_t get_chg_fet_B() const { return _parent.read_subcommand<chg_fet_protections_b_t>(data_register::CHG_FET_PROTECTIONS_B); }
            void set_chg_fet_B(const chg_fet_protections_b_t& protections) const { _parent.write_subcommand(data_register::CHG_FET_PROTECTIONS_B, protections); }
            chg_fet_protections_c_t get_chg_fet_C() const { return _parent.read_subcommand<chg_fet_protections_c_t>(data_register::CHG_FET_PROTECTIONS_C); }
            void set_chg_fet_C(const chg_fet_protections_c_t& protections) const { _parent.write_subcommand(data_register::CHG_FET_PROTECTIONS_C, protections); }
            dsg_fet_protections_a_t get_dsg_fet_A() const { return _parent.read_subcommand<dsg_fet_protections_a_t>(data_register::DSG_FET_PROTECTIONS_A); }
            void set_dsg_fet_A(const dsg_fet_protections_a_t& protections) const { _parent.write_subcommand(data_register::DSG_FET_PROTECTIONS_A, protections); }
            dsg_fet_protections_b_t get_dsg_fet_B() const { return _parent.read_subcommand<dsg_fet_protections_b_t>(data_register::DSG_FET_PROTECTIONS_B); }
            void set_dsg_fet_B(const dsg_fet_protections_b_t& protections) const { _parent.write_subcommand(data_register::DSG_FET_PROTECTIONS_B, protections); }
            dsg_fet_protections_c_t get_dsg_fet_C() const { return _parent.read_subcommand<dsg_fet_protections_c_t>(data_register::DSG_FET_PROTECTIONS_C); }
            void set_dsg_fet_C(const dsg_fet_protections_c_t& protections) const { _parent.write_subcommand(data_register::DSG_FET_PROTECTIONS_C, protections); }
            int16_t get_body_diode_threshold() const { return _parent.read_subcommand<int16_t>(data_register::BODY_DIODE_THRESHOLD); }
            void set_body_diode_threshold(const int16_t& threshold) const { _parent.write_subcommand_clamped<int16_t>(data_register::BODY_DIODE_THRESHOLD, threshold, 0, std::numeric_limits<int16_t>::max()); }

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
            alarm_status_t get_default_mask() const { return _parent.read_subcommand<alarm_status_t>(data_register::DEFAULT_ALARM_MASK); }
            void set_default_mask(const alarm_status_t& mask) const { _parent.write_subcommand(data_register::DEFAULT_ALARM_MASK, mask); }
            protections_a_t get_safety_alert_mask_A() const { return _parent.read_subcommand<protections_a_t>(data_register::SF_ALERT_MASK_A); }
            void set_safety_alert_mask_A(const protections_a_t& protections) const { _parent.write_subcommand(data_register::SF_ALERT_MASK_A, protections); }
            protections_b_t get_safety_alert_mask_B() const { return _parent.read_subcommand<protections_b_t>(data_register::SF_ALERT_MASK_B); }
            void set_safety_alert_mask_B(const protections_b_t& protections) const { _parent.write_subcommand(data_register::SF_ALERT_MASK_B, protections); }
            alarm_sf_alert_mask_c_t get_safety_alert_mask_C() const { return _parent.read_subcommand<alarm_sf_alert_mask_c_t>(data_register::SF_ALERT_MASK_C); }
            void set_safety_alert_mask_C(alarm_sf_alert_mask_c_t protections) const { _parent.write_subcommand(data_register::SF_ALERT_MASK_C, protections); }

            permanent_fail_a_t get_permanent_fail_mask_A() const { return _parent.read_subcommand<permanent_fail_a_t>(data_register::PF_ALERT_MASK_A); }
            void set_permanent_fail_mask_A(const permanent_fail_a_t& failures) const { _parent.write_subcommand(data_register::PF_ALERT_MASK_A, failures); }
            permanent_fail_b_t get_permanent_fail_mask_B() const { return _parent.read_subcommand<permanent_fail_b_t>(data_register::PF_ALERT_MASK_B); }
            void set_permanent_fail_mask_B(const permanent_fail_b_t& failures) const { _parent.write_subcommand(data_register::PF_ALERT_MASK_B, failures); }
            permanent_fail_c_t get_permanent_fail_mask_C() const { return _parent.read_subcommand<permanent_fail_c_t>(data_register::PF_ALERT_MASK_C); }
            void set_permanent_fail_mask_C(const permanent_fail_c_t& failures) const { _parent.write_subcommand(data_register::PF_ALERT_MASK_C, failures); }
            permanent_fail_d_t get_permanent_fail_mask_D() const { return _parent.read_subcommand<permanent_fail_d_t>(data_register::PF_ALERT_MASK_D); }
            void set_permanent_fail_mask_D(const permanent_fail_d_t& failures) const { _parent.write_subcommand(data_register::PF_ALERT_MASK_D, failures); }

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
            permanent_fail_a_t get_enabled_A() const { return _parent.read_subcommand<permanent_fail_a_t>(data_register::ENABLED_PF_A); }
            void set_enabled_A(const permanent_fail_a_t& failures) const { _parent.write_subcommand(data_register::ENABLED_PF_A, failures); }
            permanent_fail_b_t get_enabled_B() const { return _parent.read_subcommand<permanent_fail_b_t>(data_register::ENABLED_PF_B); }
            void set_enabled_B(const permanent_fail_b_t& failures) const { _parent.write_subcommand(data_register::ENABLED_PF_B, failures); }
            permanent_fail_c_t get_enabled_C() const { return _parent.read_subcommand<permanent_fail_c_t>(data_register::ENABLED_PF_C); }
            void set_enabled_C(const permanent_fail_c_t& failures) const { _parent.write_subcommand(data_register::ENABLED_PF_C, failures); }
            permanent_fail_d_t get_enabled_D() const { return _parent.read_subcommand<permanent_fail_d_t>(data_register::ENABLED_PF_D); }
            void set_enabled_D(const permanent_fail_d_t& failures) const { _parent.write_subcommand(data_register::ENABLED_PF_D, failures); }

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
            fet_options_t get_options() const { return _parent.read_subcommand<fet_options_t>(data_register::FET_OPTIONS); }
            void set_options(const fet_options_t& options) const { _parent.write_subcommand(data_register::FET_OPTIONS, options); }
            fet_charge_pump_control_t get_charge_pump_control() const { return _parent.read_subcommand<fet_charge_pump_control_t>(data_register::CHG_PUMP_CONTROL); }
            void set_charge_pump_control(const fet_charge_pump_control_t& control) const { _parent.write_subcommand(data_register::CHG_PUMP_CONTROL, control); }
            int16_t get_precharge_start_voltage() const { return _parent.read_subcommand<int16_t>(data_register::PRECHARGE_START_VOLTAGE); }
            void set_precharge_start_voltage(const int16_t& voltage) const { _parent.write_subcommand_clamped<int16_t>(data_register::PRECHARGE_START_VOLTAGE, voltage, 0, std::numeric_limits<int16_t>::max()); }
            int16_t get_precharge_stop_voltage() const { return _parent.read_subcommand<int16_t>(data_register::PRECHARGE_STOP_VOLTAGE); }
            void set_precharge_stop_voltage(const int16_t& voltage) const { _parent.write_subcommand_clamped<int16_t>(data_register::PRECHARGE_STOP_VOLTAGE, voltage, 0, std::numeric_limits<int16_t>::max()); }
            std::chrono::milliseconds read_predischarge_timeout() const { return std::chrono::milliseconds(_parent.read_subcommand<uint16_t>(data_register::PREDISCHARGE_TIMEOUT) * 10); }
            void set_predischarge_timeout(const std::chrono::milliseconds& timeout) const
            {
                uint8_t rounded_timeout = static_cast<uint8_t>(std::round(timeout.count() / 10.0));
                _parent.write_subcommand(data_register::PREDISCHARGE_TIMEOUT, rounded_timeout);
            }
            uint16_t get_predischarge_stop_delta() const { return static_cast<uint16_t>(_parent.read_subcommand<uint8_t>(data_register::PREDISCHARGE_STOP_DELTA)) * 10; }
            void set_predischarge_stop_delta(uint16_t voltage) const { _parent.write_subcommand(data_register::PREDISCHARGE_STOP_DELTA, static_cast<uint8_t>(voltage / 10)); }

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
        uint32_t get_dsg_current_threshold() const;
        void set_dsg_current_threshold(const uint32_t& threshold) const;
        uint32_t get_chg_current_threshold() const;
        void set_chg_current_threshold(const uint32_t& threshold) const;

        // Settings:Cell Open-Wire
        std::chrono::seconds read_cell_open_wire_check_time() const { return std::chrono::seconds(_parent.read_subcommand<uint16_t>(data_register::CHECK_TIME)); }
        void set_cell_open_wire_check_time(const std::chrono::seconds& time) const { _parent.write_subcommand(data_register::CHECK_TIME, static_cast<uint8_t>(time.count())); }

        // Settings:Interconnect Resistances
        int16_t get_cell_interconnect_resistance(const uint8_t& cell);
        void set_cell_interconnect_resistance(const uint8_t& cell, const int16_t& resistance);

        // Settings:Manufacturing
        manufacturing_status_init_t get_manufacturing_status_init() const { return _parent.read_subcommand<manufacturing_status_init_t>(data_register::MFG_STATUS_INIT); }
        void set_manufacturing_status_init(const manufacturing_status_init_t& status) const { _parent.write_subcommand(data_register::MFG_STATUS_INIT, status); }

        // Settings:Cell Balancing Config
        class CellBalancing final
        {
        public:
            balancing_configuration_t get_config() const { return _parent.read_subcommand<balancing_configuration_t>(data_register::BALANCING_CONFIGURATION); }
            void set_config(const balancing_configuration_t& config) const { _parent.write_subcommand(data_register::BALANCING_CONFIGURATION, config); }
            int8_t get_min_cell_temp() const { return _parent.read_subcommand<int8_t>(data_register::MIN_CELL_TEMP); }
            void set_min_cell_temp(const int8_t& temp) const { _parent.write_subcommand(data_register::MIN_CELL_TEMP, temp); }
            int8_t get_max_internal_temp() const { return _parent.read_subcommand<int8_t>(data_register::MAX_INTERNAL_TEMP); }
            void set_max_internal_temp(const int8_t& temp) const { _parent.write_subcommand(data_register::MAX_INTERNAL_TEMP, temp); }
            std::chrono::seconds read_interval() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::CELL_BALANCE_INTERVAL)); }
            void set_interval(const std::chrono::seconds& interval) const { _parent.write_subcommand_clamped<uint8_t>(data_register::CELL_BALANCE_INTERVAL, static_cast<uint8_t>(interval.count()), 1, 255); }
            uint8_t get_max_cells() const { return _parent.read_subcommand<uint8_t>(data_register::CELL_BALANCE_MAX_CELLS); }
            void set_max_cells(const uint8_t& cells) const { _parent.write_subcommand_clamped<uint8_t>(data_register::CELL_BALANCE_MAX_CELLS, cells, 0, 16); }
            int16_t get_min_cell_v_charge() const { return _parent.read_subcommand<int16_t>(data_register::CELL_BALANCE_MIN_CELL_V_CHARGE); }
            void set_min_cell_v_charge(const int16_t& voltage) const { _parent.write_subcommand_clamped<int16_t>(data_register::CELL_BALANCE_MIN_CELL_V_CHARGE, voltage, 0, 5000); }
            uint8_t get_min_cell_delta_charge() const { return _parent.read_subcommand<uint8_t>(data_register::CELL_BALANCE_MIN_DELTA_CHARGE); }
            void set_min_cell_delta_charge(const uint8_t& voltage) const { _parent.write_subcommand(data_register::CELL_BALANCE_MIN_DELTA_CHARGE, voltage); }
            uint8_t get_stop_delta_charge() const { return _parent.read_subcommand<uint8_t>(data_register::CELL_BALANCE_STOP_DELTA_CHARGE); }
            void set_stop_delta_charge(const uint8_t& voltage) const { _parent.write_subcommand(data_register::CELL_BALANCE_STOP_DELTA_CHARGE, voltage); }
            int16_t get_min_cell_v_relax() const { return _parent.read_subcommand<int16_t>(data_register::CELL_BALANCE_MIN_CELL_V_RELAX); }
            void set_min_cell_v_relax(const int16_t& voltage) const { _parent.write_subcommand_clamped<int16_t>(data_register::CELL_BALANCE_MIN_CELL_V_RELAX, voltage, 0, 5000); }
            uint8_t get_min_cell_delta_relax() const { return _parent.read_subcommand<uint8_t>(data_register::CELL_BALANCE_MIN_DELTA_RELAX); }
            void set_min_cell_delta_relax(const uint8_t& voltage) const { _parent.write_subcommand(data_register::CELL_BALANCE_MIN_DELTA_RELAX, voltage); }
            uint8_t get_stop_delta_relax() const { return _parent.read_subcommand<uint8_t>(data_register::CELL_BALANCE_STOP_DELTA_RELAX); }
            void set_stop_delta_relax(const uint8_t& voltage) const { _parent.write_subcommand(data_register::CELL_BALANCE_STOP_DELTA_RELAX, voltage); }

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
        const PermanentFailure permanent_failure;
        const Fet fet;
        const CellBalancing cell_balancing;

    protected:
        Settings(bq76942& parent)
            : fuse(parent),
              configuration(parent),
              protection(parent),
              alarm(parent),
              permanent_failure(parent),
              fet(parent),
              cell_balancing(parent),
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
            int16_t get_cell_voltage() const { return _parent.read_subcommand<int16_t>(data_register::SHUTDOWN_CELL_VOLTAGE); }
            void set_cell_voltage(const int16_t& voltage) const { _parent.write_subcommand_clamped<int16_t>(data_register::SHUTDOWN_CELL_VOLTAGE, voltage, 0, std::numeric_limits<int16_t>::max()); }
            int32_t get_stack_voltage() const { return static_cast<int32_t>(_parent.read_subcommand<int16_t>(data_register::SHUTDOWN_STACK_VOLTAGE)) * 10; }
            void set_stack_voltage(const int32_t& voltage) const { _parent.write_subcommand(data_register::SHUTDOWN_STACK_VOLTAGE, static_cast<int16_t>(voltage / 10)); }

            std::chrono::seconds get_low_v_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::LOW_V_SHUTDOWN_DELAY)); }
            void set_low_v_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand_clamped<uint8_t>(data_register::LOW_V_SHUTDOWN_DELAY, static_cast<uint8_t>(delay.count()), 0, 63); };
            uint8_t get_temperature() const { return _parent.read_subcommand<uint8_t>(data_register::SHUTDOWN_TEMPERATURE); }
            void set_temperature(const uint8_t& temperature) const { _parent.write_subcommand_clamped<uint8_t>(data_register::SHUTDOWN_TEMPERATURE, temperature, 0, 255); }
            std::chrono::seconds get_temperature_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::SHUTDOWN_TEMPERATURE_DELAY)); }
            void set_temperature_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand_clamped<uint8_t>(data_register::SHUTDOWN_TEMPERATURE_DELAY, static_cast<uint8_t>(delay.count()), 0, 254); }
            std::chrono::milliseconds get_fet_off_delay() const { return std::chrono::milliseconds(_parent.read_subcommand<uint8_t>(data_register::FET_OFF_DELAY) * 250); }
            void set_fet_off_delay(const std::chrono::milliseconds& delay) const
            {
                const uint8_t rounded_delay = static_cast<uint8_t>(std::round(delay.count() / 250.0));
                _parent.write_subcommand_clamped<uint8_t>(data_register::FET_OFF_DELAY, rounded_delay, 0, 127);
            }
            std::chrono::milliseconds get_command_delay() const { return std::chrono::milliseconds(_parent.read_subcommand<uint8_t>(data_register::SHUTDOWN_COMMAND_DELAY) * 250); }
            void set_command_delay(const std::chrono::milliseconds& delay) const
            {
                const uint8_t rounded_delay = static_cast<uint8_t>(std::round(delay.count() / 250.0));
                _parent.write_subcommand_clamped<uint8_t>(data_register::SHUTDOWN_COMMAND_DELAY, rounded_delay, 0, 254);
            }
            std::chrono::minutes get_auto_shutdown_time() const { return std::chrono::minutes(_parent.read_subcommand<uint8_t>(data_register::AUTO_SHUTDOWN_TIME)); }
            void set_auto_shutdown_time(const std::chrono::minutes& time) const { _parent.write_subcommand_clamped<uint8_t>(data_register::AUTO_SHUTDOWN_TIME, static_cast<uint8_t>(time.count()), 0, 250); }
            std::chrono::seconds get_ram_fail_shutdown_time() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::RAM_FAIL_SHUTDOWN_TIME)); }
            void set_ram_fail_shutdown_time(const std::chrono::seconds& time) const { _parent.write_subcommand<uint8_t>(data_register::RAM_FAIL_SHUTDOWN_TIME, static_cast<uint8_t>(time.count())); }

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
            int16_t get_current() const { return _parent.read_subcommand<int16_t>(data_register::SLEEP_CURRENT); }
            void set_current(const int16_t& current) const { _parent.write_subcommand_clamped<int16_t>(data_register::SLEEP_CURRENT, current, 0, std::numeric_limits<int16_t>::max()); }
            std::chrono::seconds get_voltage_read_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::VOLTAGE_TIME)); }
            void set_voltage_read_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand_clamped<uint8_t>(data_register::VOLTAGE_TIME, static_cast<uint8_t>(delay.count()), 1, 255); }
            int16_t get_wake_comparator_current() const { return _parent.read_subcommand<int16_t>(data_register::WAKE_COMPARATOR_CURRENT); }
            void set_wake_comparator_current(const int16_t& current) const { _parent.write_subcommand_clamped<int16_t>(data_register::WAKE_COMPARATOR_CURRENT, current, 500, std::numeric_limits<int16_t>::max()); }
            std::chrono::seconds get_hysteresis_time() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::SLEEP_HYSTERESIS_TIME)); }
            void set_hysteresis_time(const std::chrono::seconds& time) const { _parent.write_subcommand(data_register::SLEEP_HYSTERESIS_TIME, static_cast<uint8_t>(time.count())); }
            int32_t get_charger_voltage_threshold() const { return static_cast<int32_t>(_parent.read_subcommand<int16_t>(data_register::SLEEP_CHARGER_VOLTAGE_THRESHOLD)) * 10; }
            void set_charger_voltage_threshold(const int32_t& voltage) const { _parent.write_subcommand_clamped<int16_t>(data_register::SLEEP_CHARGER_VOLTAGE_THRESHOLD, static_cast<int16_t>(voltage / 10), 0, std::numeric_limits<int16_t>::max()); }
            int32_t get_charger_pack_tos_delta() const { return static_cast<int32_t>(_parent.read_subcommand<int16_t>(data_register::SLEEP_CHARGER_PACK_TOS_DELTA)) * 10; }
            void set_charger_pack_tos_delta(const int32_t& voltage) const { _parent.write_subcommand_clamped<int16_t>(data_register::SLEEP_CHARGER_PACK_TOS_DELTA, static_cast<int16_t>(voltage / 10), 10, 8500); }

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
    uint16_t get_config_ram_signature() { return read_subcommand<uint16_t>(data_register::CONFIG_RAM_SIGNATURE); }
    void set_config_ram_signature(const uint16_t& signature) { write_subcommand_clamped<uint16_t>(data_register::CONFIG_RAM_SIGNATURE, signature, 0, 0x7FFF); }

    class Protections final
    {
    public:
        // Protections:CUV
        class CellUnderVoltage final
        {
        public:
            float get_threshold() const { return _parent.read_subcommand<uint8_t>(data_register::CUV_THRESHOLD) * 50.6; }
            void set_threshold(const float& threshold) const
            {
                const uint8_t rounded_threshold = static_cast<uint8_t>(std::round(threshold / 50.6));
                _parent.write_subcommand_clamped<uint8_t>(data_register::CUV_THRESHOLD, rounded_threshold, 20, 90);
            }
            std::chrono::microseconds get_activation_delay() const { return std::chrono::microseconds(_parent.read_subcommand<uint16_t>(data_register::CUV_DELAY) * 3300L); }
            void set_activation_delay(const std::chrono::microseconds& delay) const
            {
                const uint16_t rounded_delay = static_cast<uint16_t>(std::round(delay.count() / 3300.0));
                _parent.write_subcommand_clamped<uint16_t>(data_register::CUV_DELAY, rounded_delay, 1, 2047);
            }
            float get_recovery_hysteresis() const { return _parent.read_subcommand<uint8_t>(data_register::CUV_RECOVERY_HYSTERESIS) * 50.6; }
            void set_recovery_hysteresis(const float& hysteresis) const
            {
                const uint8_t rounded_hysteresis = static_cast<uint8_t>(std::round(hysteresis / 50.6));
                _parent.write_subcommand_clamped<uint8_t>(data_register::CUV_RECOVERY_HYSTERESIS, rounded_hysteresis, 2, 20);
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
            float get_threshold() const { return _parent.read_subcommand<uint8_t>(data_register::COV_THRESHOLD) * 50.6; }
            void set_threshold(const float& threshold) const
            {
                const uint8_t rounded_threshold = static_cast<uint8_t>(std::round(threshold / 50.6));
                _parent.write_subcommand_clamped<uint8_t>(data_register::COV_THRESHOLD, rounded_threshold, 20, 110);
            }
            std::chrono::microseconds get_activation_delay() const { return std::chrono::microseconds(_parent.read_subcommand<uint16_t>(data_register::COV_DELAY) * 3300L); }
            void set_activation_delay(const std::chrono::microseconds& delay) const
            {
                const uint16_t rounded_delay = static_cast<uint16_t>(std::round(delay.count() / 3300.0));
                _parent.write_subcommand_clamped<uint16_t>(data_register::COV_DELAY, rounded_delay, 1, 2047);
            }
            float get_recovery_hysteresis() const { return _parent.read_subcommand<uint8_t>(data_register::COV_RECOVERY_HYSTERESIS) * 50.6; }
            void set_recovery_hysteresis(const float& hysteresis) const
            {
                const uint8_t rounded_hysteresis = static_cast<uint8_t>(std::round(hysteresis / 50.6));
                _parent.write_subcommand_clamped<uint8_t>(data_register::COV_RECOVERY_HYSTERESIS, rounded_hysteresis, 2, 20);
            }

            uint8_t get_latch_limit() const { return _parent.read_subcommand<uint8_t>(data_register::COVL_LATCH_LIMIT); }
            void set_latch_limit(const uint8_t& limit) const { _parent.write_subcommand(data_register::COVL_LATCH_LIMIT, limit); }
            std::chrono::seconds get_latch_counter_dec_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::COVL_COUNTER_DEC_DELAY)); }
            void set_latch_counter_dec_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::COVL_COUNTER_DEC_DELAY, static_cast<uint8_t>(delay.count())); }
            std::chrono::seconds get_latch_recovery_time() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::COVL_RECOVERY_TIME)); }
            void set_latch_recovery_time(const std::chrono::seconds& time) const { _parent.write_subcommand(data_register::COVL_RECOVERY_TIME, static_cast<uint8_t>(time.count())); }

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
            uint16_t get_threshold_voltage() const { return static_cast<uint16_t>(_parent.read_subcommand<uint8_t>(data_register::OCC_THRESHOLD)) * 2; }
            void set_threshold_voltage(const uint16_t& voltage) const { _parent.write_subcommand_clamped<uint8_t>(data_register::OCC_THRESHOLD, static_cast<uint8_t>(voltage / 2), 2, 62); }
            std::chrono::microseconds get_activation_delay() const { return std::chrono::microseconds((_parent.read_subcommand<uint8_t>(data_register::OCC_DELAY) * 3300L) + 6600L); };
            void set_activation_delay(const std::chrono::microseconds& delay) const
            {
                const uint8_t rounded_delay = static_cast<uint8_t>(std::round((delay.count() - 6600L) / 3300.0));
                _parent.write_subcommand_clamped<uint8_t>(data_register::OCC_DELAY, rounded_delay, 1, 127);
            }
            int16_t get_recovery_threshold() const { return _parent.read_subcommand<int16_t>(data_register::OCC_RECOVERY_THRESHOLD); }
            void set_recovery_threshold(const int16_t& threshold) const { _parent.write_subcommand(data_register::OCC_RECOVERY_THRESHOLD, threshold); }
            int32_t get_recovery_pack_stack_delta() const { return static_cast<int32_t>(_parent.read_subcommand<int16_t>(data_register::OCC_PACK_TOS_DELTA)) * 10; }
            void set_recovery_pack_stack_delta(const int32_t& delta) const { _parent.write_subcommand_clamped<int16_t>(data_register::OCC_PACK_TOS_DELTA, static_cast<int32_t>(delta / 10), 10, 8500); }

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
            uint8_t get_tier_1_threshold() const { return _parent.read_subcommand<uint8_t>(data_register::OCD1_THRESHOLD) * 2; }
            void set_tier_1_threshold(const uint8_t& threshold) const { _parent.write_subcommand_clamped<uint8_t>(data_register::OCD1_THRESHOLD, threshold / 2, 2, 100); }
            std::chrono::microseconds get_tier_1_delay() const { return std::chrono::microseconds((_parent.read_subcommand<uint8_t>(data_register::OCD1_DELAY) * 3300L) + 6600L); }
            void set_tier_1_delay(const std::chrono::microseconds& delay) const
            {
                const uint8_t rounded_delay = static_cast<uint8_t>(std::round((delay.count() - 6600L) / 3300.0));
                _parent.write_subcommand_clamped<uint8_t>(data_register::OCD1_DELAY, rounded_delay, 1, 127);
            }
            uint8_t get_tier_2_threshold() const { return _parent.read_subcommand<uint8_t>(data_register::OCD2_THRESHOLD) * 2; }
            void set_tier_2_threshold(const uint8_t& threshold) const { _parent.write_subcommand_clamped<uint8_t>(data_register::OCD2_THRESHOLD, threshold / 2, 2, 100); }
            std::chrono::microseconds get_tier_2_delay() const { return std::chrono::microseconds((_parent.read_subcommand<uint8_t>(data_register::OCD2_DELAY) * 3300L) + 6600L); }
            void set_tier_2_delay(const std::chrono::microseconds& delay) const
            {
                const uint8_t rounded_delay = static_cast<uint8_t>(std::round((delay.count() - 6600L) / 3300.0));
                _parent.write_subcommand_clamped<uint8_t>(data_register::OCD2_DELAY, rounded_delay, 1, 127);
            }

            float get_tier_3_threshold() const;
            void set_tier_3_threshold(const float& threshold) const;
            std::chrono::seconds get_tier_3_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::OCD3_DELAY)); };
            void set_tier_3_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::OCD3_DELAY, static_cast<uint8_t>(delay.count())); }

            int16_t get_recovery_threshold() const { return _parent.read_subcommand<int16_t>(data_register::OCD_RECOVERY_THRESHOLD); }
            void set_recovery_threshold(const int16_t& threshold) const { _parent.write_subcommand(data_register::OCD_RECOVERY_THRESHOLD, threshold); }

            uint8_t get_latch_limit() const { return _parent.read_subcommand<uint8_t>(data_register::OCDL_LATCH_LIMIT); }
            void set_latch_limit(const uint8_t& limit) const { _parent.write_subcommand(data_register::OCDL_LATCH_LIMIT, limit); }
            std::chrono::seconds get_latch_counter_dec_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::OCDL_COUNTER_DEC_DELAY)); }
            void set_latch_counter_dec_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::OCDL_COUNTER_DEC_DELAY, static_cast<uint8_t>(delay.count())); }
            std::chrono::seconds get_latch_recovery_time() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::OCDL_RECOVERY_TIME)); }
            void set_latch_recovery_time(const std::chrono::seconds& time) const { _parent.write_subcommand(data_register::OCDL_RECOVERY_TIME, static_cast<uint8_t>(time.count())); }
            int16_t get_latch_recovery_threshold() const { return _parent.read_subcommand<int16_t>(data_register::OCDL_RECOVERY_THRESHOLD); }
            void set_latch_recovery_threshold(const int16_t& threshold) const { _parent.write_subcommand(data_register::OCDL_RECOVERY_THRESHOLD, threshold); }

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
            short_circuit_discharge_threshold get_threshold() const { return _parent.read_subcommand<short_circuit_discharge_threshold>(data_register::SCD_THRESHOLD); }
            void set_threshold(const short_circuit_discharge_threshold& threshold) const { _parent.write_subcommand(data_register::SCD_THRESHOLD, threshold); }
            std::chrono::microseconds get_activation_delay() const { return std::chrono::microseconds((_parent.read_subcommand<uint8_t>(data_register::SCD_DELAY) - 1) * 15); }
            void set_activation_delay(const std::chrono::microseconds& delay) const
            {
                const uint8_t rounded_delay = static_cast<uint8_t>(std::round(delay.count() / 15.0) + 1);
                _parent.write_subcommand_clamped<uint8_t>(data_register::SCD_DELAY, rounded_delay, 1, 31);
            }
            std::chrono::seconds get_recovery_time() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::SCD_RECOVERY_TIME)); }
            void set_recovery_time(const std::chrono::seconds& time) const { _parent.write_subcommand(data_register::SCD_RECOVERY_TIME, static_cast<uint8_t>(time.count())); }

            uint8_t get_latch_limit() const { return _parent.read_subcommand<uint8_t>(data_register::SCDL_LATCH_LIMIT); }
            void set_latch_limit(const uint8_t& limit) const { _parent.write_subcommand(data_register::SCDL_LATCH_LIMIT, limit); }
            std::chrono::seconds get_latch_counter_dec_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::SCDL_COUNTER_DEC_DELAY)); }
            void set_latch_counter_dec_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::SCDL_COUNTER_DEC_DELAY, static_cast<uint8_t>(delay.count())); }
            std::chrono::seconds get_latch_recovery_time() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::SCDL_RECOVERY_TIME)); }
            void set_latch_recovery_time(const std::chrono::seconds& time) const { _parent.write_subcommand(data_register::SCDL_RECOVERY_TIME, static_cast<uint8_t>(time.count())); }
            int16_t get_latch_recovery_threshold() const { return _parent.read_subcommand<int16_t>(data_register::SCDL_RECOVERY_THRESHOLD); }
            void set_latch_recovery_threshold(const int16_t& threshold) const { _parent.write_subcommand(data_register::SCDL_RECOVERY_THRESHOLD, threshold); }

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
            int8_t get_charge_threshold() const { return _parent.read_subcommand<int8_t>(data_register::OTC_THRESHOLD); }
            void set_charge_threshold(const int8_t& threshold) const { _parent.write_subcommand_clamped<uint8_t>(data_register::OTC_THRESHOLD, threshold, -40, 120); }
            std::chrono::seconds get_charge_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::OTC_DELAY)); }
            void set_charge_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::OTC_DELAY, static_cast<uint8_t>(delay.count())); }
            int8_t get_charge_recovery_threshold() const { return _parent.read_subcommand<int8_t>(data_register::OTC_RECOVERY); }
            void set_charge_recovery_threshold(const int8_t& threshold) const { _parent.write_subcommand_clamped<uint8_t>(data_register::OTC_RECOVERY, threshold, -40, 120); }

            int8_t get_discharge_threshold() const { return _parent.read_subcommand<int8_t>(data_register::OTD_THRESHOLD); }
            void set_discharge_threshold(const int8_t& threshold) const { _parent.write_subcommand_clamped<uint8_t>(data_register::OTD_THRESHOLD, threshold, -40, 120); }
            std::chrono::seconds get_discharge_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::OTD_DELAY)); }
            void set_discharge_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::OTD_DELAY, static_cast<uint8_t>(delay.count())); }
            int8_t get_discharge_recovery_threshold() const { return _parent.read_subcommand<int8_t>(data_register::OTD_RECOVERY); }
            void set_discharge_recovery_threshold(const int8_t& threshold) const { _parent.write_subcommand_clamped<uint8_t>(data_register::OTD_RECOVERY, threshold, -40, 120); }

            int8_t get_fet_threshold() const { return _parent.read_subcommand<int8_t>(data_register::OTF_THRESHOLD); }
            void set_fet_threshold(const int8_t& threshold) const { _parent.write_subcommand_clamped<uint8_t>(data_register::OTF_THRESHOLD, threshold, 0, 150); }
            std::chrono::seconds get_fet_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::OTF_DELAY)); }
            void set_fet_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::OTF_DELAY, static_cast<uint8_t>(delay.count())); }
            int8_t get_fet_recovery_threshold() const { return _parent.read_subcommand<int8_t>(data_register::OTF_RECOVERY); }
            void set_fet_recovery_threshold(const int8_t& threshold) const { _parent.write_subcommand_clamped<uint8_t>(data_register::OTF_RECOVERY, threshold, 0, 150); }

            int8_t get_internal_threshold() const { return _parent.read_subcommand<int8_t>(data_register::OTINT_THRESHOLD); }
            void set_internal_threshold(const int8_t& threshold) const { _parent.write_subcommand_clamped<uint8_t>(data_register::OTINT_THRESHOLD, threshold, -40, 120); }
            std::chrono::seconds get_internal_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::OTINT_DELAY)); }
            void set_internal_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::OTINT_DELAY, static_cast<uint8_t>(delay.count())); }
            int8_t get_internal_recovery_threshold() const { return _parent.read_subcommand<int8_t>(data_register::OTINT_RECOVERY); }
            void set_internal_recovery_threshold(const int8_t& threshold) const { _parent.write_subcommand_clamped<uint8_t>(data_register::OTINT_RECOVERY, threshold, -40, 120); }

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
            int8_t get_charge_threshold() const { return _parent.read_subcommand<int8_t>(data_register::UTC_THRESHOLD); }
            void set_charge_threshold(const int8_t& threshold) const { _parent.write_subcommand_clamped<uint8_t>(data_register::UTC_THRESHOLD, threshold, -40, 120); }
            std::chrono::seconds get_charge_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::UTC_DELAY)); }
            void set_charge_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::UTC_DELAY, static_cast<uint8_t>(delay.count())); }
            int8_t get_charge_recovery_threshold() const { return _parent.read_subcommand<int8_t>(data_register::UTC_RECOVERY); }
            void set_charge_recovery_threshold(const int8_t& threshold) const { _parent.write_subcommand_clamped<uint8_t>(data_register::UTC_RECOVERY, threshold, -40, 120); }

            int8_t get_discharge_threshold() const { return _parent.read_subcommand<int8_t>(data_register::UTD_THRESHOLD); }
            void set_discharge_threshold(const int8_t& threshold) const { _parent.write_subcommand_clamped<uint8_t>(data_register::UTD_THRESHOLD, threshold, -40, 120); }
            std::chrono::seconds get_discharge_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::UTD_DELAY)); }
            void set_discharge_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::UTD_DELAY, static_cast<uint8_t>(delay.count())); }
            int8_t get_discharge_recovery_threshold() const { return _parent.read_subcommand<int8_t>(data_register::UTD_RECOVERY); }
            void set_discharge_recovery_threshold(const int8_t& threshold) const { _parent.write_subcommand_clamped<uint8_t>(data_register::UTD_RECOVERY, threshold, -40, 120); }

            int8_t get_internal_threshold() const { return _parent.read_subcommand<int8_t>(data_register::UTINT_THRESHOLD); }
            void set_internal_threshold(const int8_t& threshold) const { _parent.write_subcommand_clamped<uint8_t>(data_register::UTINT_THRESHOLD, threshold, -40, 120); }
            std::chrono::seconds get_internal_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::UTINT_DELAY)); }
            void set_internal_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::UTINT_DELAY, static_cast<uint8_t>(delay.count())); }
            int8_t get_internal_recovery_threshold() const { return _parent.read_subcommand<int8_t>(data_register::UTINT_RECOVERY); }
            void set_internal_recovery_threshold(const int8_t& threshold) const { _parent.write_subcommand_clamped<uint8_t>(data_register::UTINT_RECOVERY, threshold, -40, 120); }

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

        std::chrono::seconds get_recovery_time() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::PROTECTIONS_RECOVERY_TIME)); }
        void set_recovery_time(const std::chrono::seconds& time) const { _parent.write_subcommand(data_register::PROTECTIONS_RECOVERY_TIME, static_cast<uint8_t>(time.count())); }

        std::chrono::seconds get_hwd_timeout() const { return std::chrono::seconds(_parent.read_subcommand<uint16_t>(data_register::HWD_DELAY)); }
        void set_hwd_timeout(const std::chrono::seconds& timeout) const { _parent.write_subcommand(data_register::HWD_DELAY, static_cast<uint16_t>(timeout.count())); }

        std::chrono::seconds get_load_detect_time() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::LOAD_DETECT_ACTIVE_TIME)); }
        void set_load_detect_time(const std::chrono::seconds& time) const { _parent.write_subcommand(data_register::LOAD_DETECT_ACTIVE_TIME, static_cast<uint8_t>(time.count())); }
        std::chrono::seconds get_load_detect_retry_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::LOAD_DETECT_RETRY_DELAY)); }
        void set_load_detect_retry_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::LOAD_DETECT_RETRY_DELAY, static_cast<uint8_t>(delay.count())); }
        std::chrono::hours get_load_detect_timeout() const { return std::chrono::hours(_parent.read_subcommand<uint16_t>(data_register::LOAD_DETECT_TIMEOUT)); }
        void set_load_detect_timeout(const std::chrono::hours& timeout) const { _parent.write_subcommand(data_register::LOAD_DETECT_TIMEOUT, static_cast<uint16_t>(timeout.count())); }

        int16_t get_precharge_timeout_current_threshold() const { return _parent.read_subcommand<int16_t>(data_register::PTO_CHARGE_THRESHOLD); }
        void set_precharge_timeout_current_threshold(const int16_t& threshold) const { _parent.write_subcommand_clamped<int16_t>(data_register::PTO_CHARGE_THRESHOLD, threshold, 0, 1000); }
        std::chrono::seconds get_precharge_timeout_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint16_t>(data_register::PTO_DELAY)); }
        void set_precharge_timeout_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::PTO_DELAY, static_cast<uint16_t>(delay.count())); }
        float get_precharge_reset_charge() const;
        void set_precharge_reset_charge(const float& charge) const;

        const CellUnderVoltage under_voltage;
        const CellOverVoltage over_voltage;
        const OverCurrentCharge over_current_charge;
        const OverCurrentDischarge over_current_discharge;
        const ShortCircuit short_circuit;
        const OverTemperature over_temp;
        const UnderTemperature under_temp;

    protected:
        Protections(bq76942& parent)
            : under_voltage(parent),
              over_voltage(parent),
              over_current_charge(parent),
              over_current_discharge(parent),
              short_circuit(parent),
              over_temp(parent),
              under_temp(parent),
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
        int16_t get_copper_deposition_threshold() const { return _parent.read_subcommand<int16_t>(data_register::CU_DEP_THRESHOLD); }
        void set_copper_deposition_threshold(const int16_t& threshold) const { _parent.write_subcommand_clamped<int16_t>(data_register::CU_DEP_THRESHOLD, threshold, 0, std::numeric_limits<int16_t>::max()); }
        std::chrono::seconds get_copper_deposition_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::CU_DEP_DELAY)); }
        void set_copper_deposition_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::CU_DEP_DELAY, static_cast<uint8_t>(delay.count())); }

        int16_t get_under_voltage_threshold() const { return _parent.read_subcommand<int16_t>(data_register::SUV_THRESHOLD); }
        void set_under_voltage_threshold(const int16_t& threshold) const { _parent.write_subcommand_clamped<int16_t>(data_register::SUV_THRESHOLD, threshold, 0, std::numeric_limits<int16_t>::max()); }
        std::chrono::seconds get_under_voltage_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::SUV_DELAY)); }
        void set_under_voltage_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::SUV_DELAY, static_cast<uint8_t>(delay.count())); }

        int16_t get_over_voltage_threshold() const { return _parent.read_subcommand<int16_t>(data_register::SOV_THRESHOLD); }
        void set_over_voltage_threshold(const int16_t& threshold) const { _parent.write_subcommand_clamped<int16_t>(data_register::SOV_THRESHOLD, threshold, 0, std::numeric_limits<int16_t>::max()); }
        std::chrono::seconds get_over_voltage_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::SOV_DELAY)); }
        void set_over_voltage_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::SOV_DELAY, static_cast<uint8_t>(delay.count())); }

        int16_t get_stack_delta_threshold() const { return _parent.read_subcommand<int16_t>(data_register::TOS_THRESHOLD); }
        void set_stack_delta_threshold(const int16_t& threshold) const { _parent.write_subcommand_clamped<int16_t>(data_register::TOS_THRESHOLD, threshold, 0, std::numeric_limits<int16_t>::max()); }
        std::chrono::seconds get_stack_delta_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::TOS_DELAY)); }
        void set_stack_delta_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::TOS_DELAY, static_cast<uint8_t>(delay.count())); }

        float get_charge_current_threshold() const;
        void set_charge_current_threshold(const float& threshold) const;

        float get_discharge_current_threshold() const;
        void set_discharge_current_threshold(const float& threshold) const;

        int8_t get_over_temp_cell_threshold() const { return _parent.read_subcommand<int8_t>(data_register::SOT_THRESHOLD); }
        void set_over_temp_cell_threshold(const int8_t& threshold) const { _parent.write_subcommand_clamped<int8_t>(data_register::SOT_THRESHOLD, threshold, -40, 120); }
        std::chrono::seconds get_over_temp_cell_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::SOT_DELAY)); }
        void set_over_temp_cell_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::SOT_DELAY, static_cast<uint8_t>(delay.count())); }

        int8_t get_over_temp_fet_threshold() const { return _parent.read_subcommand<int8_t>(data_register::SOTF_THRESHOLD); }
        void set_over_temp_fet_threshold(const int8_t& threshold) const { _parent.write_subcommand_clamped<int8_t>(data_register::SOTF_THRESHOLD, threshold, 0, 150); }
        std::chrono::seconds get_over_temp_fet_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::SOTF_DELAY)); }
        void set_over_temp_fet_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::SOTF_DELAY, static_cast<uint8_t>(delay.count())); }

        class VoltageImbalanceRelaxed final
        {
        public:
            int16_t get_check_voltage() const { return _parent.read_subcommand<int16_t>(data_register::VIMR_CHECK_VOLTAGE); }
            void set_check_voltage(const int16_t& voltage) const { _parent.write_subcommand_clamped<int16_t>(data_register::VIMR_CHECK_VOLTAGE, voltage, 0, 5500); }
            int16_t get_max_relax_current() const { return _parent.read_subcommand<int16_t>(data_register::VIMR_MAX_RELAX_CURRENT); }
            void set_max_relax_current(const int16_t& current) const { _parent.write_subcommand_clamped<int16_t>(data_register::VIMR_MAX_RELAX_CURRENT, current, 10, std::numeric_limits<int16_t>::max()); }
            int16_t get_threshold() const { return _parent.read_subcommand<int16_t>(data_register::VIMR_THRESHOLD); }
            void set_threshold(const int16_t& threshold) const { _parent.write_subcommand_clamped<int16_t>(data_register::VIMR_THRESHOLD, threshold, 0, 5500); }
            std::chrono::seconds read_activation_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::VIMR_DELAY)); }
            void set_activation_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::VIMR_DELAY, static_cast<uint8_t>(delay.count())); }
            std::chrono::seconds read_relax_min_duration() const { return std::chrono::seconds(_parent.read_subcommand<uint16_t>(data_register::VIMR_RELAX_MIN_DURATION)); }
            void set_relax_min_duration(const std::chrono::seconds& duration) const { _parent.write_subcommand(data_register::VIMR_RELAX_MIN_DURATION, static_cast<uint16_t>(duration.count())); }

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
            int16_t get_check_voltage() const { return _parent.read_subcommand<int16_t>(data_register::VIMA_CHECK_VOLTAGE); }
            void set_check_voltage(const int16_t& voltage) const { _parent.write_subcommand_clamped<int16_t>(data_register::VIMA_CHECK_VOLTAGE, voltage, 0, 5500); }
            int16_t get_min_active_current() const { return _parent.read_subcommand<int16_t>(data_register::VIMA_MIN_ACTIVE_CURRENT); }
            void set_min_active_current(const int16_t& current) const { _parent.write_subcommand_clamped<int16_t>(data_register::VIMA_MIN_ACTIVE_CURRENT, current, 10, std::numeric_limits<int16_t>::max()); }
            int16_t get_threshold() const { return _parent.read_subcommand<int16_t>(data_register::VIMA_THRESHOLD); }
            void set_threshold(const int16_t& threshold) const { _parent.write_subcommand_clamped<int16_t>(data_register::VIMA_THRESHOLD, threshold, 0, 5500); }
            std::chrono::seconds read_activation_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::VIMA_DELAY)); }
            void set_activation_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::VIMA_DELAY, static_cast<uint8_t>(delay.count())); }

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
            int16_t get_threshold() const { return _parent.read_subcommand<int16_t>(data_register::CFETF_OFF_THRESHOLD); }
            void set_threshold(const int16_t& threshold) const { _parent.write_subcommand_clamped<int16_t>(data_register::CFETF_OFF_THRESHOLD, threshold, 10, 5000); }
            std::chrono::seconds read_activation_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::CFETF_OFF_DELAY)); }
            void set_activation_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::CFETF_OFF_DELAY, static_cast<uint8_t>(delay.count())); }

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
            int16_t get_threshold() const { return _parent.read_subcommand<int16_t>(data_register::DFETF_OFF_THRESHOLD); }
            void set_threshold(const int16_t& threshold) const { _parent.write_subcommand_clamped<int16_t>(data_register::DFETF_OFF_THRESHOLD, threshold, -5000, -10); }
            std::chrono::seconds read_activation_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::DFETF_OFF_DELAY)); }
            void set_activation_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::DFETF_OFF_DELAY, static_cast<uint8_t>(delay.count())); }

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

        int16_t get_vssf_threshold() const { return _parent.read_subcommand<int16_t>(data_register::VSSF_FAIL_THRESHOLD); }
        void set_vssf_threshold(const int16_t& threshold) const { _parent.write_subcommand_clamped<int16_t>(data_register::VSSF_FAIL_THRESHOLD, threshold, 1, std::numeric_limits<int16_t>::max()); };
        std::chrono::seconds read_vssf_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::VSSF_DELAY)); }
        void set_vssf_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::VSSF_DELAY, static_cast<uint8_t>(delay.count())); }

        std::chrono::seconds read_2_lvl_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::PF_2LVL_DELAY)); }
        void set_2_lvl_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::PF_2LVL_DELAY, static_cast<uint8_t>(delay.count())); }
        std::chrono::seconds read_lfof_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::LFOF_DELAY)); }
        void set_lfof_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::LFOF_DELAY, static_cast<uint8_t>(delay.count())); }
        std::chrono::seconds read_hardware_muxf_delay() const { return std::chrono::seconds(_parent.read_subcommand<uint8_t>(data_register::HWMX_DELAY)); }
        void set_hardware_muxf_delay(const std::chrono::seconds& delay) const { _parent.write_subcommand(data_register::HWMX_DELAY, static_cast<uint8_t>(delay.count())); }

        const VoltageImbalanceRelaxed voltage_imbalance_relaxed;
        const VoltageImbalanceActive voltage_imbalance_active;
        const ChargeFetFail charge_fet_fail;
        const DischargeFetFail discharge_fet_fail;

    protected:
        PermanentFail(bq76942& parent)
            : voltage_imbalance_relaxed(parent),
              voltage_imbalance_active(parent),
              charge_fet_fail(parent),
              discharge_fet_fail(parent),
              _parent(parent)
        {
        }
        PermanentFail(PermanentFail&) = delete;
        PermanentFail& operator=(PermanentFail&) = delete;

    private:
        friend class bq76942;
        bq76942& _parent;
    };

    uint16_t get_unseal_key1() { return read_subcommand<uint16_t>(data_register::UNSEAL_KEY_STEP1); }
    uint16_t get_unseal_key2() { return read_subcommand<uint16_t>(data_register::UNSEAL_KEY_STEP2); }
    uint16_t get_full_access_key1() { return read_subcommand<uint16_t>(data_register::FULL_ACCESS_KEY_STEP1); }
    uint16_t get_full_access_key2() { return read_subcommand<uint16_t>(data_register::FULL_ACCESS_KEY_STEP2); }

    const Calibration calibration{*this};
    const Settings settings{*this};
    const Power power{*this};
    const Protections protections{*this};
    const PermanentFail permanent_fail{*this};

    std::vector<uint8_t> read_direct(const uint8_t register_addr, const size_t bytes);
    template <typename T>
    T read_direct(const uint8_t register_addr)
    {
        std::vector<uint8_t> data_buf = read_direct(register_addr, sizeof(T));
        if (data_buf.size() != sizeof(T))
        {
            throw std::runtime_error(std::format("Data size mismatch - got {} bytes, expected {} for type {}",
                                                 data_buf.size(), sizeof(T), demangle(typeid(T).name())));
        }
        // both the BMS and the ESP32 are little-endian so this works nicely
        // for all data types
        T data{};
        std::copy_n(data_buf.begin(), sizeof(T), reinterpret_cast<uint8_t*>(&data));
        return data;
    }
    std::vector<uint8_t> read_direct(const direct_command register_addr, const size_t bytes)
    {
        return read_direct(static_cast<uint8_t>(register_addr), bytes);
    }
    template <typename T>
    T read_direct(const direct_command register_addr)
    {
        return read_direct<T>(static_cast<uint8_t>(register_addr));
    }

    void write_direct(const uint8_t register_addr, const std::vector<uint8_t>& data);
    template <typename T>
    void write_direct(const uint8_t register_addr, const T& data)
    {
        std::vector<uint8_t> data_buf(sizeof(T));
        std::copy_n(reinterpret_cast<const uint8_t*>(&data), sizeof(T), data_buf.begin());
        write_direct(register_addr, data_buf);
    }
    void write_direct(const direct_command register_addr, const std::vector<uint8_t>& data)
    {
        write_direct(static_cast<uint8_t>(register_addr), data);
    }
    template <typename T>
    void write_direct(const direct_command register_addr, const T& data)
    {
        write_direct(static_cast<uint8_t>(register_addr), data);
    }

    std::vector<uint8_t> read_subcommand(const uint16_t register_addr);
    template <typename T>
    T read_subcommand(const uint16_t register_addr)
    {
        std::vector<uint8_t> data_buf = read_subcommand(register_addr);
        // can't check for size equality, since data register writes always return a 32 byte memory chunk
        if (data_buf.size() < sizeof(T))
        {
            throw std::runtime_error(std::format("Data size mismatch - got {} bytes, expected {} for type {}",
                                                 data_buf.size(), sizeof(T), demangle(typeid(T).name())));
        }
        T data{};
        std::copy_n(data_buf.begin(), sizeof(T), reinterpret_cast<uint8_t*>(&data));
        return data;
    }
    std::vector<uint8_t> read_subcommand(const data_register register_addr) { return read_subcommand(static_cast<uint16_t>(register_addr)); }
    template <typename T>
    T read_subcommand(const data_register register_addr)
    {
        return read_subcommand<T>(static_cast<uint16_t>(register_addr));
    }
    std::vector<uint8_t> read_subcommand(const subcommand register_addr) { return read_subcommand(static_cast<uint16_t>(register_addr)); }
    template <typename T>
    T read_subcommand(const subcommand register_addr)
    {
        return read_subcommand<T>(static_cast<uint16_t>(register_addr));
    }

    /// @brief Write a subcommand, with optional data, and wait for it to complete
    /// @param register_addr The subcommand address to write to
    /// @param data The data to write
    void write_subcommand(const uint16_t register_addr, const std::vector<uint8_t>& data = {});
    template <typename T>
    void write_subcommand(const uint16_t register_addr, const T& data)
    {
        std::vector<uint8_t> data_buf(sizeof(T));
        std::copy_n(reinterpret_cast<const uint8_t*>(&data), sizeof(T), data_buf.begin());
        write_subcommand(register_addr, data_buf);
    }
    void write_subcommand(const data_register register_addr, const std::vector<uint8_t>& data = {}) { write_subcommand(static_cast<uint16_t>(register_addr), data); }
    template <typename T>
    void write_subcommand(const data_register register_addr, const T& data)
    {
        write_subcommand(static_cast<uint16_t>(register_addr), data);
    }
    void write_subcommand(const subcommand register_addr, const std::vector<uint8_t>& data = {}) { write_subcommand(static_cast<uint16_t>(register_addr), data); }
    template <typename T>
    void write_subcommand(const subcommand register_addr, const T& data)
    {
        write_subcommand(static_cast<uint16_t>(register_addr), data);
    }
    void write_subcommand(const cmd_only_subcommand register_addr) { write_subcommand(static_cast<uint16_t>(register_addr)); }

    template <typename T>
    void write_subcommand_clamped(const data_register register_addr, const T& data, const T& minimum, const T& maximum)
    {
        static_assert(std::is_arithmetic<T>::value, "T must be an arithmetic type");
        if ((data < minimum) || (data > maximum))
        {
            throw std::out_of_range(std::format("Value {} is out of range for register {:#x} - must be between {} and {}",
                                                data, static_cast<uint16_t>(register_addr), minimum, maximum));
        }
        write_subcommand(register_addr, data);
    }

    static float raw_temp_to_celsius(int16_t raw) { return kelvin_to_celsius(raw / 10.0f); }
    static float kelvin_to_celsius(float kelvin) { return kelvin - 273.15f; }
    static float get_user_amps_multiplier(const da_configuration_t& config);
    static int16_t get_user_volts_multiplier(const da_configuration_t& config);

private:
    uint8_t _address;

    std::vector<uint8_t> _read_direct(const uint8_t register_addr, const size_t bytes);
    void _write_direct(const uint8_t register_addr, const std::vector<uint8_t>& data);
    void _select_subcommand(const uint16_t register_addr, bool wait_for_completion = true);
    std::vector<uint8_t> _read_subcommand(const uint16_t register_addr);
    void _write_subcommand(const uint16_t register_addr, const std::vector<uint8_t>& data);
    static uint8_t _calculate_checksum(uint16_t register_addr, const std::vector<uint8_t>& data);

    void _queue_transmit(const uint8_t& data);
    void _queue_transmit(const std::vector<uint8_t>& data);

    /// @brief Temporary buffer for storing data so CRCs can be calculated
    std::vector<uint8_t> _data_buf;

    std::vector<uint8_t> _transmit_queue;
    std::vector<uint8_t> _receive_queue;
};