#include "main.hpp"

#include <Arduino.h>

#include <chrono>
#include <hi_can_twai.hpp>
#include <thread>

#include "bq769x2.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

steady_clock::time_point last_power_flow;
bool has_had_load = false;

constexpr float ACTIVITY_CURRENT = 100;

void setup_bms(bq76942& bq);
void print_bms_status(bq76942& bq);

void setup()
{
    bsp::initI2C();
    std::this_thread::sleep_for(1s);

    bq76942 bq;
    setup_bms(bq);

    Serial.begin(115200);
    auto& interface = hi_can::TwaiInterface::get_instance();
    last_power_flow = steady_clock::now();
}

void loop()
{
    using namespace hi_can;

    auto& interface = TwaiInterface::get_instance();
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
        bq76942 bq;
        print_bms_status(bq);

        bq.toggle_fuse();  // blinky light
        if (!bq.get_manufacturing_status().autonomous_fets)
        {
            printf("Reconfiguring BMS (was it reset?)...\n");
            std::this_thread::sleep_for(1s);
            setup_bms(bq);
        }

        float cc2_current = bq.get_CC2_current();
        printf(std::format("CC2 current: {:07.1f} mA\n", cc2_current).c_str());
        if (abs(cc2_current) > ACTIVITY_CURRENT)
        {
            last_power_flow = steady_clock::now();
            has_had_load = true;
        }

        const bool unloaded_timeout = steady_clock::now() - last_power_flow > 30s;
        const bool load_dropped = (steady_clock::now() - last_power_flow) > 1s && has_had_load;
        if (unloaded_timeout || load_dropped)
        {
            printf("Shutting down\n");
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
        last_power_flow = steady_clock::now();  // delay shutdown
    }
    vTaskDelay(50);
}

void print_bms_status(bq76942& bq)
{
    auto fet_status = bq.get_fet_status();
    bool pchg = fet_status.precharge_fet_on;
    bool chg = fet_status.charge_fet_on;
    bool dsg = fet_status.discharge_fet_on;
    bool pdsg = fet_status.predischarge_fet_on;

    auto status6 = bq.get_DA_status_6();
    printf(std::format("PCHG {:d}  CHG {:d}  DSG {:d}  PDSG {:d} stack vtg {:04} pack {:04d} charge {:.1f}\n",
                       pchg, chg, dsg, pdsg,
                       bq.get_stack_voltage(), bq.get_pack_voltage(),
                       status6.accumulated_charge)
               .c_str());

    auto status = bq.get_battery_status();
    bool config_update = status.in_config_update_mode;
    bool in_precharge_mode = status.in_precharge_mode;
    bool sleep_allowed = status.sleep_allowed;
    bool full_reset_occurred = status.full_reset_occurred;
    bool was_watchdog_reset = status.was_watchdog_reset;
    bool checking_cell_open_wire = status.checking_cell_open_wire;
    bool pending_otp_write = status.pending_otp_write;
    bool otp_write_blocked = status.otp_write_blocked;
    bq76942::security_state_option security_state = status.security_state;
    bool fuse_active = status.fuse_active;
    bool safety_fault_active = status.safety_fault_active;
    bool permanent_fail_active = status.permanent_fail_active;
    bool shutdown_pending = status.shutdown_pending;
    bool in_sleep = status.in_sleep;

    printf(std::format("Config update: {} Precharge: {}, Sleep allowed: {}, Full reset: {}, Watchdog reset: {}, Cell open wire: {}, Pending OTP write: {}, OTP write blocked: {}\n", config_update, in_precharge_mode, sleep_allowed, full_reset_occurred, was_watchdog_reset, checking_cell_open_wire, pending_otp_write, otp_write_blocked).c_str());
    printf(std::format("Security state: {}, Fuse active: {}, Safety fault active: {}, Permanent fail active: {}, Shutdown pending: {}, In sleep: {}\n", static_cast<uint8_t>(security_state), fuse_active, safety_fault_active, permanent_fail_active, shutdown_pending, in_sleep).c_str());

    // print alarm regs
    auto alarm_A_status = bq.get_safety_status_a();
    auto alarm_B_status = bq.get_safety_status_b();
    auto alarm_C_status = bq.get_safety_status_c();
    printf(std::format("A: {:08b} B: {:08b} C: {:08b}\n",
                       *reinterpret_cast<uint8_t*>(&alarm_A_status),
                       *reinterpret_cast<uint8_t*>(&alarm_B_status),
                       *reinterpret_cast<uint8_t*>(&alarm_C_status))
               .c_str());

    auto pf_A_status = bq.get_permanent_fail_status_a();
    auto pf_B_status = bq.get_permanent_fail_status_b();
    auto pf_C_status = bq.get_permanent_fail_status_c();
    auto pf_D_status = bq.get_permanent_fail_status_d();
    printf(std::format("PF A: {:08b} B: {:08b} C: {:08b} D: {:08b}\n",
                       *reinterpret_cast<uint8_t*>(&pf_A_status),
                       *reinterpret_cast<uint8_t*>(&pf_B_status),
                       *reinterpret_cast<uint8_t*>(&pf_C_status),
                       *reinterpret_cast<uint8_t*>(&pf_D_status))
               .c_str());

    for (int i = 0; i < 10; i++)
    {
        printf(std::format("Cell {}: {}\n", i, bq.get_cell_voltage(i)).c_str());
    }
}

void setup_bms(bq76942& bq)
{
    try
    {
        // can't reset here, as that would power cycle this MCU
        // bq.reset();
        // std::this_thread::sleep_for(100ms);

        bq.enter_config_update_mode();

        /*
        NOTE ON CELL CONNECTION:
        Rule of thumb is max .1V per series cell maximum difference
        before connecting them in parallel, so for safety
        using 0.4V per pack max difference.
        */

        bq76942::selected_cells_t connected_cells = {.raw = 0x021F};
        auto manufacturing_status = bq.get_manufacturing_status();

        // NOTE: If not programmed, the default is probably good.

        // Subcommand config
        if (!manufacturing_status.autonomous_fets)
            bq.toggle_fet_test_mode();
        if (!manufacturing_status.is_permanent_fail_enabled)
            bq.toggle_permanent_fail_enabled();
        // TODO: is write to CB_SET_LVL needed?
        bq.set_regulator_control(bq76942::regulator_control_t{
            .reg_1_enable = true,
            .reg_1_voltage = bq76942::regulator_voltage::VOLTAGE_5_0,
            .reg_2_enable = true,
            .reg_2_voltage = bq76942::regulator_voltage::VOLTAGE_5_0,
        });

        // Data register config

        // Calibration:Voltage
        // Not needed, calibrated at factory, good enough for our use case

        // Calibration:Current
        // Default is for 1mOhm resistor, like we have
        // bq.calibration.current.set_sense_resistor_value(1);

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
        bq.settings.fuse.set_blow_timeout(255s);

        // Settings:Configuration
        bq.settings.configuration.set_power_config(bq76942::power_config_t{
            .cell_balance_loop_speed = bq76942::measurement_loop_speed::HALF_SPEED,
        });
        bq.settings.configuration.set_reg_0_config({.reg_0_enable = true});
        bq.settings.configuration.set_hwd_regulator_options(bq76942::hwd_regulator_options_t{
            .toggle_time = 2,
            .hwd_action = bq76942::hwd_toggle_option::REGULATORS_OFF_THEN_ON,
        });

        // TODO: Correct this when thermistors installed
        bq.settings.configuration.set_ts1_pin_config(bq76942::ts_pin_configuration_t{
            .function = bq76942::adc_pin_function::UNUSED,
        });
        bq.settings.configuration.set_cfetoff_pin_config(bq76942::cfetoff_pin_configuration_t{
            .function = bq76942::cfetoff_pin_function::SPI_CS_OR_UNUSED,
        });
        bq.settings.configuration.set_dfetoff_pin_config(bq76942::dfetoff_pin_configuration_t{
            .function = bq76942::dfetoff_pin_function::UNUSED,
        });
        bq.settings.configuration.set_ddsg_pin_config(bq76942::ddsg_pin_configuration_t{
            .function = bq76942::ddsg_pin_function::UNUSED,
        });
        bq.settings.configuration.set_dchg_pin_config(bq76942::dchg_pin_configuration_t{
            .function = bq76942::dchg_pin_function::UNUSED,
        });

        bq.settings.configuration.set_DA_configuration(bq76942::da_configuration_t{
            .user_amps = bq76942::user_amps_unit::CENTIAMP,
            // TODO: CHANGE WHEN THERMISTORS INSTALLED
            .use_internal_as_cell_temperature = true,
            .use_internal_as_fet_temperature = true,
        });
        bq.settings.configuration.set_vcell_mode(connected_cells);
        // CC3 samples default is good

        // Settings:Protection
        bq.settings.protection.set_config({
            .permanent_fail_stops_fets = true,
            .permanent_fail_stops_regulators = false,
            .permanent_fail_causes_deep_sleep = true,
            .permanent_fail_blows_fuse = true,
            .fet_fault_ignores_fuse_voltage = true,
            .use_overcurrent_charge_recovery = true,
            .use_short_circuit_charge_recovery = true,
        });
        bq.settings.protection.set_enabled_A({
            .cell_undervoltage = true,
            .cell_overvoltage = true,
            .overcurrent_charge = true,
            .overcurrent_discharge_1 = true,
            .overcurrent_discharge_2 = true,
            .short_circuit_discharge = true,
        });
        bq.settings.protection.set_enabled_B({
            .undertemp_charge = true,
            .undertemp_discharge = true,
            .internal_undertemp = true,
            .overtemp_charge = true,
            .overtemp_discharge = true,
            .internal_overtemp = true,
            .fet_overtemp = true,
        });
        bq.settings.protection.set_enabled_C({
            .host_watchdog_fault = true,
            .precharge_timeout = true,
            .cell_overvoltage_latch = true,
            .overcurrent_discharge_latch = true,
            .short_circuit_discharge_latch = true,
            .overcurrent_discharge_3 = true,
        });
        bq.settings.protection.set_chg_fet_A({
            .cell_overvoltage = true,
            .overcurrent_charge = true,
            .short_circuit_discharge = true,
        });
        bq.settings.protection.set_chg_fet_B({
            .undertemp_charge = true,
            .internal_undertemp = true,
            .overtemp_charge = true,
            .internal_overtemp = true,
            .fet_overtemp = true,
        });
        bq.settings.protection.set_chg_fet_C({
            .host_watchdog_fault = true,
            .precharge_timeout = true,
            .cell_overvoltage_latch = true,
            .short_circuit_discharge_latch = true,
        });
        bq.settings.protection.set_dsg_fet_A({
            .cell_undervoltage = true,
            .overcurrent_discharge_2 = true,
            .short_circuit_discharge = true,
        });
        bq.settings.protection.set_dsg_fet_B({
            .undertemp_discharge = true,
            .internal_undertemp = true,
            .overtemp_discharge = true,
            .internal_overtemp = true,
            .fet_overtemp = true,
        });
        bq.settings.protection.set_dsg_fet_C({
            .host_watchdog_fault = true,
            .short_circuit_discharge_latch = true,
            .overcurrent_discharge_3 = true,
        });
        // default body diode setting is fine

        // Settings:Alarm
        bq.settings.alarm.set_default_mask({
            .stack_reached_shutdown_voltage = true,
        });
        // default alarm masks enable all safety alarms

        // enable all permanent fail alarm masks
        uint8_t alarm_mask = 0xEF;
        bq.settings.alarm.set_permanent_fail_mask_A(*reinterpret_cast<const bq76942::permanent_fail_a_t*>(&alarm_mask));
        alarm_mask = 0x9F;
        bq.settings.alarm.set_permanent_fail_mask_B(*reinterpret_cast<const bq76942::permanent_fail_b_t*>(&alarm_mask));
        alarm_mask = 0x78;
        bq.settings.alarm.set_permanent_fail_mask_C(*reinterpret_cast<const bq76942::permanent_fail_c_t*>(&alarm_mask));
        alarm_mask = 0x01;
        bq.settings.alarm.set_permanent_fail_mask_D({.top_stack_vs_cell = true});

        // Settings:Permanent Failure
        bq.settings.permanent_failure.set_enabled_A({
            .safety_cell_undervoltage = true,
            .safety_cell_overvoltage = true,
            .safety_overcurrent_charge = true,
            .safety_overcurrent_discharge = true,
            .safety_overtemp = true,
            .safety_overtemp_fet = true,
            .copper_deposition = true,
        });
        bq.settings.permanent_failure.set_enabled_B({
            .charge_fet = true,
            .discharge_fet = true,
            .second_level_protector = true,
            .voltage_imbalance_relax = true,
            .voltage_imbalance_active = true,
            .short_circuit_discharge_latch = true,
        });
        bq.settings.permanent_failure.set_enabled_C({
            .otp_memory = true,
            .data_ROM = true,
            .instruction_ROM = true,
            .internal_LFO = true,
            .internal_voltage_reference = true,
            .internal_vss_measurement = true,
            .hardware_mux = true,
            .commanded = false,
        });
        bq.settings.permanent_failure.set_enabled_D({
            .top_stack_vs_cell = true,
        });
        bq.settings.fet.set_options({
            .enable_predischarge = true,
        });
        // charge pump defaults are good (11V overdrive)
        // precharge disabled by default
        // predischarge is voltage delta based only, no timeout
        bq.settings.fet.set_predischarge_timeout(0ms);
        bq.settings.fet.set_predischarge_stop_delta(5000);

        // Settings:Current Thresholds
        bq.settings.set_dsg_current_threshold(100);
        bq.settings.set_chg_current_threshold(50);

        // Settings:Cell Open-Wire

        // Settings:Interconnect Resistances
        // we don't care, not measurable (integrated battery pack)

        // Settings:Manufacturing
        bq.settings.set_manufacturing_status_init({
            .autonomous_fets = true,
            .is_permanent_fail_enabled = true,
            .is_otp_write_enabled = false,
        });

        // Settings:Cell Balancing Config
        bq.settings.cell_balancing.set_config({
            .allow_charging_cell_balancing = true,
            .allow_relaxed_cell_balancing = true,
            .allow_balancing_in_sleep = false,
            .balancing_exits_sleep = true,
            .ignore_host_controlled_balancing = true,
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
        bq.settings.cell_balancing.set_max_cells(1);
        // default balance voltages are good for Li-Po cells

        // Power:Shutdown
        // all the not set values are good defaults
        bq.power.shutdown.set_cell_voltage(3000);
        bq.power.shutdown.set_stack_voltage(3000 * 6);
        bq.power.shutdown.set_low_v_delay(10s);
        bq.power.shutdown.set_command_delay(1s);
        bq.power.shutdown.set_auto_shutdown_time(5min);

        // Power:Sleep
        bq.power.sleep.set_charger_voltage_threshold(4200 * 6);

        // System Data:Integrity

        // Protections:CUV
        // technically 3.2V would be recommended, but safety margin is nice
        bq.protections.under_voltage.set_threshold(3300);

        // Protections:COV
        bq.protections.over_voltage.set_threshold(4250);

        // Protections:COVL
        bq.protections.over_voltage.set_latch_limit(3);

        // Protections:OCC
        // Threshold voltage is voltage across sense resistor.
        // Batteries are (nominally... not actually) 4500mAh,
        // so just under 2C is good to trip
        bq.protections.over_current_charge.set_threshold_voltage(8);
        bq.protections.over_current_charge.set_recovery_pack_stack_delta(1000);

        // OCD1 and 2 thresholds are in sense resistor mV,
        // so for 1mOhm resistor it's 1:1 with mA
        // Protections:OCD1
        bq.protections.over_current_discharge.set_tier_1_threshold(140);
        bq.protections.over_current_discharge.set_tier_1_delay(10000us);

        // Protections:OCD2
        bq.protections.over_current_discharge.set_tier_2_threshold(100);
        bq.protections.over_current_discharge.set_tier_1_delay(20000us);

        // Protections:SCD
        bq.protections.short_circuit.set_threshold(bq76942::short_circuit_discharge_threshold::THRESHOLD_150MV);
        bq.protections.short_circuit.set_activation_delay(30us);
        bq.protections.short_circuit.set_recovery_time(20s);

        // Protections:OCD3
        // in mA
        bq.protections.over_current_discharge.set_tier_3_threshold(-90000.0f);
        bq.protections.over_current_discharge.set_tier_3_delay(5s);

        // Protections:OCD

        // Protections:OCDL
        bq.protections.over_current_discharge.set_latch_limit(3);

        // Protections:SCDL
        bq.protections.short_circuit.set_latch_limit(3);
        bq.protections.short_circuit.set_latch_counter_dec_delay(20s);
        bq.protections.short_circuit.set_latch_recovery_time(15s);

        // Protections:OTC

        // Protections:OTD

        // Protections:OTF

        // Protections:OTINT

        // Protections:UTC

        // Protections:UTD

        // Protections:UTINT

        // Protections:Recovery

        // Protections:HWD
        bq.protections.set_hwd_timeout(10s);

        // Protections:Load Detect
        // disabled by default
        bq.protections.set_load_detect_time(10s);
        bq.protections.set_load_detect_retry_delay(30s);

        // Protections:PTO

        // Permanent Fail:CUDEP
        bq.permanent_fail.set_copper_deposition_threshold(2000);

        // Permanent Fail:SUV
        bq.permanent_fail.set_under_voltage_threshold(2700);

        // Permanent Fail:SOV
        bq.permanent_fail.set_over_voltage_threshold(4400);

        // Permanent Fail:TOS

        // Permanent Fail:SOCC
        bq.permanent_fail.set_charge_current_threshold(10000);

        // Permanent Fail:SOCD
        bq.permanent_fail.set_discharge_current_threshold(-200000);

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

        bq.enable_all_fets();
        bq.disable_charge_fets();
        bq.exit_config_update_mode();
        // bq.toggle_fet_test_mode();
        // bq.charge_test();
        // bq.discharge_test();
    }
    catch (std::exception& e)
    {
        printf(std::format("Error configuring BMS: {}\n", e.what()).c_str());
    }
}