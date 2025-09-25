#![no_std]
#![no_main]

use core::marker::PhantomData;

use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::pwm::{ChannelAPin, ChannelBPin, Config, Pwm, PwmOutput, SetDutyCycle, Slice};
use embassy_rp::Peri;
use embassy_time::{Duration, Ticker};
use {defmt_rtt as _, panic_probe as _};

/// Fully on duty cycle
const FULLY_ON_DUTY_CYCLE: u8 = 100;
/// Max duty cycle for driving inverter phases
const MAX_INVERTER_DUTY_CYCLE: u8 = 15;
/// Min duty cycle for driving inverter phases
const MIN_INVERTER_DUTY_CYCLE: u8 = 0;

/// Lookup table for phase voltages when commuting a three phase inverter.
const THREE_PHASE_COMMUTATION_TABLE: [InverterOutput; 6] = [
    InverterOutput {
        phase_a: PhaseState::HighDutyCycle(MAX_INVERTER_DUTY_CYCLE),
        phase_b: PhaseState::Low,
        phase_c: PhaseState::HighImpedance,
    },
    InverterOutput {
        phase_a: PhaseState::HighDutyCycle(MAX_INVERTER_DUTY_CYCLE),
        phase_b: PhaseState::HighImpedance,
        phase_c: PhaseState::Low,
    },
    InverterOutput {
        phase_a: PhaseState::HighImpedance,
        phase_b: PhaseState::HighDutyCycle(MAX_INVERTER_DUTY_CYCLE),
        phase_c: PhaseState::Low,
    },
    InverterOutput {
        phase_a: PhaseState::Low,
        phase_b: PhaseState::HighDutyCycle(MAX_INVERTER_DUTY_CYCLE),
        phase_c: PhaseState::HighImpedance,
    },
    InverterOutput {
        phase_a: PhaseState::Low,
        phase_b: PhaseState::HighImpedance,
        phase_c: PhaseState::HighDutyCycle(MAX_INVERTER_DUTY_CYCLE),
    },
    InverterOutput {
        phase_a: PhaseState::HighImpedance,
        phase_b: PhaseState::Low,
        phase_c: PhaseState::HighDutyCycle(MAX_INVERTER_DUTY_CYCLE),
    },
];

/// Represents how a half bridge phase should be driven
#[derive(Copy, Clone)]
enum PhaseState {
    HighDutyCycle(u8),
    Low,
    HighImpedance,
}

/// Represents the control outputs for a three phase inverter
#[derive(Copy, Clone)]
struct InverterOutput {
    phase_a: PhaseState,
    phase_b: PhaseState,
    phase_c: PhaseState,
}

/// Represents a half bridge driven by a high side and low side enable pin
struct HalfBridge<'d, S>
where
    S: Slice,
{
    pwm: Pwm<'d>,
    divider: u8,
    top: u16,
    dead_time_percentage: u8,
    _slice: PhantomData<S>,
}

impl<'d, S> HalfBridge<'d, S>
where
    S: Slice,
{
    /// Instantates a new half bridge driver with two GPIO and a PWM slice
    fn new(
        slice: Peri<'d, S>,
        high_pin: Peri<'d, impl ChannelAPin<S>>,
        low_pin: Peri<'d, impl ChannelBPin<S>>,
        pwm_frequency_hz: u32,
        dead_time_ns: u32,
    ) -> Self {
        // Calculate in tick units
        let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();
        let divider = 16u8;
        let period = (clock_freq_hz / (pwm_frequency_hz * divider as u32) - 1) as u16 / 2;
        let dead_time_ticks =
            ((dead_time_ns as u64 * clock_freq_hz as u64) / divider as u64 / 1_000_000_000) as u16;
        let dead_time_percentage = ((dead_time_ticks as u32 * 100) / period as u32) as u8;

        // Configure default PWM settings
        let mut config = Config::default();
        config.invert_a = false;
        config.invert_b = false;
        config.phase_correct = true;
        config.enable = true;
        config.divider = divider.into();
        config.compare_a = 0;
        config.compare_b = 0;
        config.top = period;

        let pwm = Pwm::new_output_ab(slice, high_pin, low_pin, config);

        Self {
            pwm: pwm,
            divider: divider,
            top: period,
            dead_time_percentage: dead_time_percentage,
            _slice: PhantomData,
        }
    }

    /// Set the half bridge to PWM the high side gate to the specified duty cycle
    fn set_high(&mut self, percentage: u8) {
        let mut complementary_config = Config::default();
        complementary_config.invert_a = false;
        complementary_config.invert_b = true;
        complementary_config.phase_correct = true;
        complementary_config.enable = true;
        complementary_config.divider = self.divider.into();
        complementary_config.compare_a = 0;
        complementary_config.compare_b = 0;
        complementary_config.top = self.top;

        self.pwm.set_config(&complementary_config);

        let (high_pwm, low_pwm) = self.pwm.split_by_ref();

        let _ = high_pwm.unwrap().set_duty_cycle_percent(percentage);
        let _ = low_pwm
            .unwrap()
            .set_duty_cycle_percent(percentage + self.dead_time_percentage);
    }

    /// Set the half bridge to be driven low
    fn set_low(&mut self) {
        let mut config = Config::default();
        config.invert_a = false;
        config.invert_b = false;
        config.phase_correct = true;
        config.enable = true;
        config.divider = self.divider.into();
        config.compare_a = 0;
        config.compare_b = 0;
        config.top = self.top;

        self.pwm.set_config(&config);

        let (high_pwm, low_pwm) = self.pwm.split_by_ref();

        let _ = high_pwm.unwrap().set_duty_cycle_fully_off();
        let _ = low_pwm.unwrap().set_duty_cycle_fully_on();
    }

    /// Changes the half bridge to a high impedance output
    fn set_high_impedance(&mut self) {
        let mut config = Config::default();
        config.invert_a = false;
        config.invert_b = false;
        config.phase_correct = true;
        config.enable = true;
        config.divider = self.divider.into();
        config.compare_a = 0;
        config.compare_b = 0;
        config.top = self.top;

        self.pwm.set_config(&config);

        let (high_pwm, low_pwm) = self.pwm.split_by_ref();

        let _ = high_pwm.unwrap().set_duty_cycle_fully_off();
        let _ = low_pwm.unwrap().set_duty_cycle_fully_off();
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let _onboard_led = Output::new(p.PIN_25, Level::High);

    let embassy_rp::Peripherals {
        PIN_4,
        PIN_5,
        PIN_12,
        PIN_13,
        PIN_14,
        PIN_15,
        ..
    } = p;

    let half_bridge_a = HalfBridge::new(p.PWM_SLICE2, PIN_4, PIN_5, 25_000, 1000);
    let half_bridge_b = HalfBridge::new(p.PWM_SLICE6, PIN_12, PIN_13, 25_000, 1000);
    let half_bridge_c = HalfBridge::new(p.PWM_SLICE7, PIN_14, PIN_15, 25_000, 1000);

    let _ = spawner.spawn(bldc_driver_task(
        half_bridge_a,
        half_bridge_b,
        half_bridge_c,
    ));

    // Keep the on board LED in scope
    loop {
        embassy_time::Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn bldc_driver_task(
    mut half_bridge_a: HalfBridge<'static, embassy_rp::peripherals::PWM_SLICE2>,
    mut half_bridge_b: HalfBridge<'static, embassy_rp::peripherals::PWM_SLICE6>,
    mut half_bridge_c: HalfBridge<'static, embassy_rp::peripherals::PWM_SLICE7>,
) {
    let mut step: usize = 0;
    let mut ticker = Ticker::every(Duration::from_millis(25));

    loop {
        step = (step + 1) % THREE_PHASE_COMMUTATION_TABLE.len();

        let output = THREE_PHASE_COMMUTATION_TABLE[step];

        match output.phase_a {
            PhaseState::HighDutyCycle(percentage) => half_bridge_a.set_high(percentage),
            PhaseState::Low => half_bridge_a.set_low(),
            PhaseState::HighImpedance => half_bridge_a.set_high_impedance(),
        };

        match output.phase_b {
            PhaseState::HighDutyCycle(percentage) => half_bridge_b.set_high(percentage),
            PhaseState::Low => half_bridge_b.set_low(),
            PhaseState::HighImpedance => half_bridge_b.set_high_impedance(),
        };

        match output.phase_c {
            PhaseState::HighDutyCycle(percentage) => half_bridge_c.set_high(percentage),
            PhaseState::Low => half_bridge_c.set_low(),
            PhaseState::HighImpedance => half_bridge_c.set_high_impedance(),
        };
        ticker.next().await;
    }
}
