#include "main.hpp"

#include <Arduino.h>

#include <chrono>
#include <hi_can_twai.hpp>
#include <thread>

#include "bq769x2.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

steady_clock::time_point lastPowerFlow;
bool hasHadLoad = false;

constexpr float ACTIVITY_CURRENT = 100;

void setupBms(bq76942& bq);
void printBmsStatus(bq76942& bq);

////////////////////////
// TESTING VARIABLES
////////////////////////

// This will toggle all of the relevant temp sensing features in the safety protections except for chip internal temp protection
#define tempSensing true

// This sets how long the BMS will stay powered on waiting for a load of >100mA to be applied
#define noLoadTimeout 400s

// This will set the battery cell overtemperature (TS1)
int8_t cellProtectionTemp = 40;
int8_t cellRecoveryTemp = cellProtectionTemp - 3;
std::chrono::seconds cellProtectionDelay = 2s;

// This will set the FET overtemperature (TS3)
int8_t fetProtectionTemp = 50;
int8_t fetRecoveryTemp = fetProtectionTemp - 3;
std::chrono::seconds fetProtectionDelay = 1s;

void setup()
{
    bsp::initI2C();
    std::this_thread::sleep_for(1s);

    bq76942 bq;
    setupBms(bq);

    Serial.begin(115200);
    auto& interface = hi_can::TwaiInterface::getInstance();
    lastPowerFlow = steady_clock::now();
}

void loop()
{
    using namespace hi_can;

    auto& interface = TwaiInterface::getInstance();
    try
    {
        // handle CAN bus I/O
    }
    catch (const std::exception& e)
    {
        printf(std::format("CAN bus error: {}\n", e.what()).c_str());
    }

    try
    {
        static bq76942 bq;
        printBmsStatus(bq);

        bq.toggleFuse();  // blinky light
        if (!bq.getManufacturingStatus().autonomousFets)
        {
            printf("Reconfiguring BMS (was it reset?)...\n");
            std::this_thread::sleep_for(1s);
            setupBms(bq);
        }

        float cc2Current = bq.getCC2Current();
        printf(std::format("CC2 current: {:07.1f} mA || ", cc2Current).c_str());
        if (abs(cc2Current) > ACTIVITY_CURRENT)
        {
            lastPowerFlow = steady_clock::now();
            hasHadLoad = true;
        }

        const bool unloadedTimeout = steady_clock::now() - lastPowerFlow > noLoadTimeout;
        // Commented this out to stop the BMS from shutting down after a load is removed
        const bool loadDropped = false;
        // const bool loadDropped = (steady_clock::now() - lastPowerFlow) > 1s && hasHadLoad;
        if (unloadedTimeout || loadDropped)
        {
            printf("Shutting down.\n");
            bq.shutdown();
            // let BMS timeout and shut down
            std::this_thread::sleep_for(10s);
            // should not reach this point - if we do, BMS is not shutting down due to a safety reason
        }
    }
    catch (const std::exception& e)
    {
        printf(std::format("BMS Error: {}\n", e.what()).c_str());
        std::this_thread::sleep_for(500ms);
        lastPowerFlow = steady_clock::now();  // delay shutdown
    }
    vTaskDelay(500);
}

void printBmsStatus(bq76942& bq)
{
    auto fetStatus = bq.getFetStatus();
    bool pchg = fetStatus.prechargeFetOn;
    bool chg = fetStatus.chargeFetOn;
    bool dsg = fetStatus.dischargeFetOn;
    bool pdsg = fetStatus.predischargeFetOn;

    auto status6 = bq.getDAStatus6();
    printf(std::format("PCHG: {:d}  CHG: {:d}  DSG: {:d}  PDSG: {:d} STACK_VTG: {:04} PACK_VTG: {:04d} LD_VTG: {:04d} TOTAL_MA: {:.1f} || ",
                       pchg, chg, dsg, pdsg,
                       bq.getStackVoltage(), bq.getPackVoltage(), bq.getLdVoltage(),
                       status6.accumulatedCharge)
               .c_str());

    auto status = bq.getBatteryStatus();
    bool configUpdate = status.inConfigUpdateMode;
    bool inPrechargeMode = status.inPrechargeMode;
    bool sleepAllowed = status.sleepAllowed;
    bool fullResetOccurred = status.fullResetOccurred;
    bool wasWatchdogReset = status.wasWatchdogReset;
    bool checkingCellOpenWire = status.checkingCellOpenWire;
    bool pendingOtpWrite = status.pendingOtpWrite;
    bool otpWriteBlocked = status.otpWriteBlocked;
    bq76942::security_state securityState = status.securityState;
    bool fuseActive = status.fuseActive;
    bool safetyFaultActive = status.safetyFaultActive;
    bool permanentFailActive = status.permanentFailActive;
    bool shutdownPending = status.shutdownPending;
    bool inSleep = status.inSleep;

    // printf(std::format("Config update: {} Precharge: {}, Sleep allowed: {}, Full reset: {}, Watchdog reset: {}, Cell open wire: {}, Pending OTP write: {}, OTP write blocked: {}\n", configUpdate, inPrechargeMode, sleepAllowed, fullResetOccurred, wasWatchdogReset, checkingCellOpenWire, pendingOtpWrite, otpWriteBlocked).c_str());
    // printf(std::format("Security state: {}, Fuse active: {}, Safety fault active: {}, Permanent fail active: {}, Shutdown pending: {}, In sleep: {}\n", static_cast<uint8_t>(securityState), fuseActive, safetyFaultActive, permanentFailActive, shutdownPending, inSleep).c_str());

    // Read safety alarm registers and print only if anything is flagged
    // Store full structs to avoid rvalue address errors
    auto statusA = bq.getSafetyStatusA();
    auto statusB = bq.getSafetyStatusB();
    auto statusC = bq.getSafetyStatusC();

    uint8_t alarmAStatus = *reinterpret_cast<uint8_t*>(&statusA);
    uint8_t alarmBStatus = *reinterpret_cast<uint8_t*>(&statusB);
    uint8_t alarmCStatus = *reinterpret_cast<uint8_t*>(&statusC);

    std::vector<AlarmRegister> safetyAlarms = {
        {"ALARM A", alarmAStatus},
        {"ALARM B", alarmBStatus},
        {"ALARM C", alarmCStatus},
    };

    auto pfA = bq.getPermanentFailStatusA();
    auto pfB = bq.getPermanentFailStatusB();
    auto pfC = bq.getPermanentFailStatusC();
    auto pfD = bq.getPermanentFailStatusD();

    uint8_t pfAStatus = *reinterpret_cast<uint8_t*>(&pfA);
    uint8_t pfBStatus = *reinterpret_cast<uint8_t*>(&pfB);
    uint8_t pfCStatus = *reinterpret_cast<uint8_t*>(&pfC);
    uint8_t pfDStatus = *reinterpret_cast<uint8_t*>(&pfD);

    std::vector<AlarmRegister> permanentFails = {
        {"PERM FAIL A", pfAStatus},
        {"PERM FAIL B", pfBStatus},
        {"PERM FAIL C", pfCStatus},
        {"PERM FAIL D", pfDStatus},
    };

    for (const auto& alarm : safetyAlarms)
    {
        if (alarm.value != 0)
        {
            printf(std::format("\n{}: {:08b}\n", alarm.name, alarm.value).c_str());
        }
    }

    for (const auto& pf : permanentFails)
    {
        if (pf.value != 0)
        {
            printf(std::format("\n{}: {:08b}\n", pf.name, pf.value).c_str());
        }
    }

    // Print cell voltages
    for (int i = 0; i < 10; i++)
    {
        if (i >= 5 && i <= 8)
        {
            continue;
        }
        printf(std::format("Cell {}: {}  ", i, bq.getCellVoltage(i)).c_str());
    }

    // Print the temperature for both TS1 and TS3
    printf(" || TS1(BAT): %.1f °C ", bq.rawTempToCelsius(bq.getThermistorTemp1()));
    printf("TS3(FET): %.1f °C", bq.rawTempToCelsius(bq.getThermistorTemp2()));

    printf("\n");
}

void setupBms(bq76942& bq)
{
    try
    {
        // can't reset here, as that would power cycle this MCU
        // bq.reset();
        // std::this_thread::sleep_for(100ms);

        bq.enterConfigUpdateMode();

        /*
        NOTE ON CELL CONNECTION:
        Rule of thumb is max .1V per series cell maximum difference
        before connecting them in parallel, so for safety
        using 0.4V per pack max difference.
        */

        bq76942::selected_cells_t connectedCells = {.raw = 0x021F};
        auto manufacturingStatus = bq.getManufacturingStatus();

        // NOTE: If not programmed, the default is probably good.

        // Subcommand config
        if (!manufacturingStatus.autonomousFets)
            bq.toggleFetTestMode();
        if (!manufacturingStatus.isPermanentFailEnabled)
            bq.togglePermanentFailEnabled();
        // TODO: is write to CB_SET_LVL needed?
        bq.setRegulatorControl(bq76942::regulator_control_t{
            .reg1Enable = true,
            .reg1Voltage = bq76942::regulator_voltage::VOLTAGE_5_0,
            .reg2Enable = true,
            .reg2Voltage = bq76942::regulator_voltage::VOLTAGE_5_0,
        });

        // Data register config

        // Calibration:Voltage
        // Not needed, calibrated at factory, good enough for our use case

        // Calibration:Current
        // Default is for 1mOhm resistor, like we have
        // bq.calibration.current.setSenseResistorValue(1);

        // Calibration:Vcell Offset
        // Calibration:V Divider Offset
        // TODO: May need to calibrate these later

        // Calibration:Current Offset
        // Don't have the tools to measure the values needed to calibrate this better than factory

        // Calibration:Temperature
        // TODO: Calibrate this when all thermistors installed

        // Calibration:Internal Temp Model
        // Calibration:18K Temperature Model
        // Calibration:180K Temperature Model
        // Calibration:Custom Temperature Model
        // Same as current offset, can't calibrate better than factory

        // Calibration:Current Deadband
        // As noted in TRM, shouldn't be changed from default

        // Calibration:CUV
        // Calibration:COV
        // Using factory calibration, no need for custom thresholds

        // Settings:Fuse
        bq.settings.fuse.setBlowTimeout(255s);

        // Settings:Configuration
        bq.settings.configuration.setPowerConfig(bq76942::power_config_t{
            .cellBalanceLoopSpeed = bq76942::measurement_loop_speed::HALF_SPEED,
        });
        bq.settings.configuration.setReg0Config({.reg0Enable = true});
        bq.settings.configuration.setHwdRegulatorOptions(bq76942::hwd_regulator_options_t{
            .toggleTime = 2,
            .hwdAction = bq76942::hwd_toggle_option::REGULATORS_OFF_THEN_ON,
        });

        // TODO: Correct this when thermistors installed
        bq.settings.configuration.setTs1PinConfig(bq76942::ts_pin_configuration_t{
            .function = bq76942::adc_pin_function::ADC_INPUT_OR_THERMISTOR,
            .measurementType = bq76942::measurement_type::CELL_THERMISTOR,
            .polynomial = bq76942::polynomial_selection::POLYNOMIAL_18K,
            .pullupConfig = bq76942::pullup_config::PULLUP_18K,
        });

        bq.settings.configuration.setTs3PinConfig(bq76942::ts_pin_configuration_t{
            .function = bq76942::adc_pin_function::ADC_INPUT_OR_THERMISTOR,
            .measurementType = bq76942::measurement_type::FET_THERMISTOR,
            .polynomial = bq76942::polynomial_selection::POLYNOMIAL_18K,
            .pullupConfig = bq76942::pullup_config::PULLUP_18K,
        });

        // bq.settings.configuration.setTs1PinConfig(bq76942::ts_pin_configuration_t{
        //     .function = bq76942::adc_pin_function::ADC_INPUT_OR_THERMISTOR,
        // });

        // bq.settings.configuration.setTs3PinConfig(bq76942::ts_pin_configuration_t{
        //     .function = bq76942::adc_pin_function::ADC_INPUT_OR_THERMISTOR,
        // });

        bq.settings.configuration.setCfetoffPinConfig(bq76942::cfetoff_pin_configuration_t{
            .function = bq76942::cfetoff_pin_function::SPI_CS_OR_UNUSED,
        });
        bq.settings.configuration.setDfetoffPinConfig(bq76942::dfetoff_pin_configuration_t{
            .function = bq76942::dfetoff_pin_function::UNUSED,
        });
        bq.settings.configuration.setDdsgPinConfig(bq76942::ddsg_pin_configuration_t{
            .function = bq76942::ddsg_pin_function::UNUSED,
        });
        bq.settings.configuration.setDchgPinConfig(bq76942::dchg_pin_configuration_t{
            .function = bq76942::dchg_pin_function::UNUSED,
        });

        bq.settings.configuration.setDAConfiguration(bq76942::da_configuration_t{
            .userAmps = bq76942::user_amps::CENTIAMP,
            // TODO: CHANGE WHEN THERMISTORS INSTALLED
            .useInternalAsCellTemperature = false,
            .useInternalAsFetTemperature = false,
        });
        bq.settings.configuration.setVcellMode(connectedCells);
        // CC3 samples default is good

        // Settings:Protection
        bq.settings.protection.setConfig({
            .permanentFailStopsFets = true,
            .permanentFailStopsRegulators = false,
            .permanentFailCausesDeepSleep = true,
            .permanentFailBlowsFuse = true,
            .fetFaultIgnoresFuseVoltage = true,
            .useOvercurrentChargeRecovery = true,
            .useShortCircuitChargeRecovery = true,
        });
        bq.settings.protection.setEnabledA({
            .cellUndervoltage = true,
            .cellOvervoltage = true,
            .overcurrentCharge = true,
            .overcurrentDischarge1 = true,
            .overcurrentDischarge2 = true,
            .shortCircuitDischarge = true,
        });
        bq.settings.protection.setEnabledB({
            .undertempCharge = tempSensing,
            .undertempDischarge = tempSensing,
            .internalUndertemp = true,
            .overtempCharge = tempSensing,
            .overtempDischarge = tempSensing,
            .internalOvertemp = true,
            .fetOvertemp = tempSensing,
        });
        bq.settings.protection.setEnabledC({
            .hostWatchdogFault = true,
            .prechargeTimeout = true,
            .cellOvervoltageLatch = true,
            .overcurrentDischargeLatch = true,
            .shortCircuitDischargeLatch = true,
            .overcurrentDischarge3 = true,
        });
        bq.settings.protection.setChgFetA({
            .cellOvervoltage = true,
            .overcurrentCharge = true,
            .shortCircuitDischarge = true,
        });
        bq.settings.protection.setChgFetB({
            .undertempCharge = tempSensing,
            .internalUndertemp = true,
            .overtempCharge = tempSensing,
            .internalOvertemp = true,
            .fetOvertemp = tempSensing,
        });
        bq.settings.protection.setChgFetC({
            .hostWatchdogFault = true,
            .prechargeTimeout = true,
            .cellOvervoltageLatch = true,
            .shortCircuitDischargeLatch = true,
        });
        bq.settings.protection.setDsgFetA({
            .cellUndervoltage = true,
            .overcurrentDischarge2 = true,
            .shortCircuitDischarge = true,
        });
        bq.settings.protection.setDsgFetB({
            .undertempDischarge = tempSensing,
            .internalUndertemp = true,
            .overtempDischarge = tempSensing,
            .internalOvertemp = true,
            .fetOvertemp = tempSensing,
        });
        bq.settings.protection.setDsgFetC({
            .hostWatchdogFault = true,
            .shortCircuitDischargeLatch = true,
            .overcurrentDischarge3 = true,
        });
        // default body diode setting is fine

        // Settings:Alarm
        bq.settings.alarm.setDefaultMask({
            .stackReachedShutdownVoltage = true,
        });
        // default alarm masks enable all safety alarms

        // enable all permanent fail alarm masks
        uint8_t alarmMask = 0xEF;
        bq.settings.alarm.setPermanentFailMaskA(*reinterpret_cast<const bq76942::permanent_fail_a_t*>(&alarmMask));
        alarmMask = 0x9F;
        bq.settings.alarm.setPermanentFailMaskB(*reinterpret_cast<const bq76942::permanent_fail_b_t*>(&alarmMask));
        alarmMask = 0x78;
        bq.settings.alarm.setPermanentFailMaskC(*reinterpret_cast<const bq76942::permanent_fail_c_t*>(&alarmMask));
        alarmMask = 0x01;
        bq.settings.alarm.setPermanentFailMaskD({.topStackVsCell = true});

        // Settings:Permanent Failure
        bq.settings.permanentFailure.setEnabledA({
            .safetyCellUndervoltage = true,
            .safetyCellOvervoltage = true,
            .safetyOvercurrentCharge = true,
            .safetyOvercurrentDischarge = true,
            .safetyOvertemp = tempSensing,
            .safetyOvertempFet = tempSensing,
            .copperDeposition = true,
        });
        bq.settings.permanentFailure.setEnabledB({
            .chargeFet = true,
            .dischargeFet = true,
            .secondLevelProtector = true,
            .voltageImbalanceRelax = true,
            .voltageImbalanceActive = true,
            .shortCircuitDischargeLatch = true,
        });
        bq.settings.permanentFailure.setEnabledC({
            .otpMemory = true,
            .dataROM = true,
            .instructionROM = true,
            .internalLFO = true,
            .internalVoltageReference = true,
            .internalVssMeasurement = true,
            .hardwareMux = true,
            .commanded = false,
        });
        bq.settings.permanentFailure.setEnabledD({
            .topStackVsCell = true,
        });
        bq.settings.fet.setOptions({
            .enablePredischarge = true,
        });
        // charge pump defaults are good (11V overdrive)
        // precharge disabled by default
        // predischarge is voltage delta based, timeout at 5s, 0ms is no timeout
        bq.settings.fet.setPredischargeTimeout(5000ms);
        bq.settings.fet.setPredischargeStopDelta(5000);

        // Settings:Current Thresholds
        bq.settings.setDsgCurrentThreshold(100);
        bq.settings.setChgCurrentThreshold(50);

        // Settings:Cell Open-Wire

        // Settings:Interconnect Resistances
        // we don't care, not measurable (integrated battery pack)

        // Settings:Manufacturing
        bq.settings.setManufacturingStatusInit({
            .autonomousFets = true,
            .isPermanentFailEnabled = true,
            .isOtpWriteEnabled = false,
        });

        // Settings:Cell Balancing Config
        bq.settings.cellBalancing.setConfig({
            .allowChargingCellBalancing = true,
            .allowRelaxedCellBalancing = true,
            .allowBalancingInSleep = false,
            .balancingExitsSleep = true,
            .ignoreHostControlledBalancing = true,
        });
        // default temp thresholds are sensible
        /*
        RthetaJA: 66.0degC/W
        Vcell: 4.2V
        Rinternal = 15 to 46 ohms
        Rbalance = 20 ohms
        Rtot = Rinternal + (2*Rbalance)
        I = Vcell/Rtot = 0.0304A to 0.0553A
        P = I^2 * Rinternal = 0.043W to 0.140W
        Tambient = 30C
        Tmax = 60C
        deltaT = Tmax - Tambient = 30C
        Pmax = deltaT / RthetaJA ~= 0.455W
        max cells = Pmax / P = 3.23 cells = 3 cells
        However, as noted here: https://www.ti.com/lit/an/spra953d/spra953d.pdf
        Rtheta is inaccurate, so for safety, use 1 cell until
        temperatures can be verified
        TODO: Verify heat dissipation capabilities
        */
        bq.settings.cellBalancing.setMaxCells(1);
        // default balance voltages are good for Li-Po cells

        // Power:Shutdown
        // all the not set values are good defaults
        bq.power.shutdown.setCellVoltage(3000);
        bq.power.shutdown.setStackVoltage(3000 * 6);
        bq.power.shutdown.setLowVDelay(10s);
        bq.power.shutdown.setCommandDelay(1s);
        bq.power.shutdown.setAutoShutdownTime(5min);

        // Power:Sleep
        bq.power.sleep.setChargerVoltageThreshold(4200 * 6);

        // System Data:Integrity

        // Protections:CUV
        // technically 3.2V would be recommended, but safety margin is nice
        bq.protections.underVoltage.setThreshold(3300);

        // Protections:COV
        bq.protections.overVoltage.setThreshold(4250);

        // Protections:COVL
        bq.protections.overVoltage.setLatchLimit(3);

        // Protections:OCC
        // Threshold voltage is voltage across sense resistor.
        // Batteries are (nominally... not actually) 4500mAh,
        // so just under 2C is good to trip
        bq.protections.overCurrentCharge.setThresholdVoltage(8);
        bq.protections.overCurrentCharge.setRecoveryPackStackDelta(1000);

        // OCD1 and 2 thresholds are in sense resistor mV,
        // so for 1mOhm resistor it's 1:1 with mA
        // Protections:OCD1
        bq.protections.overCurrentDischarge.setTier1Threshold(140);
        bq.protections.overCurrentDischarge.setTier1Delay(10000us);

        // Protections:OCD2
        bq.protections.overCurrentDischarge.setTier2Threshold(100);
        bq.protections.overCurrentDischarge.setTier1Delay(20000us);

        // Protections:SCD
        bq.protections.shortCircuit.setThreshold(bq76942::short_circuit_discharge_threshold::THRESHOLD_150MV);
        bq.protections.shortCircuit.setActivationDelay(30us);
        bq.protections.shortCircuit.setRecoveryTime(20s);

        // Protections:OCD3
        // in mA / userAmps, so centiamps?
        bq.protections.overCurrentDischarge.setTier3Threshold(-90000.0f);
        bq.protections.overCurrentDischarge.setTier3Delay(3s);

        // Protections:OCD

        // Protections:OCDL
        bq.protections.overCurrentDischarge.setLatchLimit(3);

        // Protections:SCDL
        bq.protections.shortCircuit.setLatchLimit(3);
        bq.protections.shortCircuit.setLatchCounterDecDelay(20s);
        bq.protections.shortCircuit.setLatchRecoveryTime(15s);

        // FOR REFERENCE THESE ARE THE PARAMETERS TO SET FOR OVERTEMP THRESHOLDS FOR BATTERY CELL (DISCHARGE) AND FET THRESHOLDS

        // int8_t getDischargeThreshold() const { return _parent.readSubcommand<int8_t>(data_register::OTD_THRESHOLD); }
        // void setDischargeThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::OTD_THRESHOLD, threshold, -40, 120); }
        // std::chrono::seconds getDischargeDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::OTD_DELAY)); }
        // void setDischargeDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::OTD_DELAY, static_cast<uint8_t>(delay.count())); }
        // int8_t getDischargeRecoveryThreshold() const { return _parent.readSubcommand<int8_t>(data_register::OTD_RECOVERY); }
        // void setDischargeRecoveryThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::OTD_RECOVERY, threshold, -40, 120); }

        // int8_t getFetThreshold() const { return _parent.readSubcommand<int8_t>(data_register::OTF_THRESHOLD); }
        // void setFetThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::OTF_THRESHOLD, threshold, 0, 150); }
        // std::chrono::seconds getFetDelay() const { return std::chrono::seconds(_parent.readSubcommand<uint8_t>(data_register::OTF_DELAY)); }
        // void setFetDelay(const std::chrono::seconds& delay) const { _parent.writeSubcommand(data_register::OTF_DELAY, static_cast<uint8_t>(delay.count())); }
        // int8_t getFetRecoveryThreshold() const { return _parent.readSubcommand<int8_t>(data_register::OTF_RECOVERY); }
        // void setFetRecoveryThreshold(const int8_t& threshold) const { _parent.writeSubcommandClamped<uint8_t>(data_register::OTF_RECOVERY, threshold, 0, 150); }

        // Protections:OTD (BATTERY OVERTEMP THRESHOLD)
        bq.protections.overTemp.setDischargeThreshold(cellProtectionTemp);
        // bq.protections.overTemp.setDischargeDelay(cellProtectionDelay);

        // Protections:OTD

        // Protections:OTF (FET OVERTEMP THRESHOLD)
        bq.protections.overTemp.setFetThreshold(fetProtectionTemp);
        bq.protections.overTemp.setFetRecoveryThreshold(fetRecoveryTemp);
        // bq.protections.overTemp.setFetDelay(fetProtectionDelay); // Standard is 2 secs

        // Protections:OTINT

        // Protections:UTC

        // Protections:UTD

        // Protections:UTINT

        // Protections:Recovery

        // Protections:HWD
        bq.protections.setHwdTimeout(10s);

        // Protections:Load Detect
        // disabled by default
        bq.protections.setLoadDetectTime(10s);
        bq.protections.setLoadDetectRetryDelay(30s);

        // Protections:PTO

        // Permanent Fail:CUDEP
        bq.permanentFail.setCopperDepositionThreshold(2000);

        // Permanent Fail:SUV
        bq.permanentFail.setUnderVoltageThreshold(2700);

        // Permanent Fail:SOV
        bq.permanentFail.setOverVoltageThreshold(4400);

        // Permanent Fail:TOS

        // Permanent Fail:SOCC
        bq.permanentFail.setChargeCurrentThreshold(10000);

        // Permanent Fail:SOCD
        bq.permanentFail.setDischargeCurrentThreshold(-5000);  // in userA which is centiamps so 50A

        // Permanent Fail:SOT

        // Permanent Fail:SOTF

        // Permanent Fail:VIMR

        // Permanent Fail:VIMA

        // Permanent Fail:CFETF

        // Permanent Fail:DFETF

        // Permanent Fail:VSSF

        // Permanent Fail:2LVL

        // Permanent Fail:LFOF

        // Permanent Fail:HWMX

        // Security:Settings

        // Security:Keys

        bq.enableAllFets();
        bq.disableChargeFets();
        bq.exitConfigUpdateMode();
        // bq.toggleFetTestMode();
        // bq.chargeTest();
        // bq.dischargeTest();
    }
    catch (std::exception& e)
    {
        printf(std::format("Error configuring BMS: {}\n", e.what()).c_str());
    }
}