#![no_std]
#![no_main]

use core::marker::PhantomData;

use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::pwm::{ChannelAPin, ChannelBPin, Config, Pwm, PwmOutput, SetDutyCycle, Slice};
use embassy_rp::Peri;
use embassy_time::{Duration, Ticker};
use {defmt_rtt as _, panic_probe as _};

/// Max duty cycle for driving inverter phases
const MAX_DUTY_CYCLE: DutyCycle = Some(10);
/// Min duty cycle for driving inverter phases
const MIN_DUTY_CYCLE: DutyCycle = Some(0);

/// Lookup table for phase voltages when commuting a three phase inverter.
const THREE_PHASE_COMMUTATION_TABLE: [PhaseOutput; 6] = [
    PhaseOutput {
        phase_a: MAX_DUTY_CYCLE,
        phase_b: MIN_DUTY_CYCLE,
        phase_c: None,
    },
    PhaseOutput {
        phase_a: MAX_DUTY_CYCLE,
        phase_b: None,
        phase_c: MIN_DUTY_CYCLE,
    },
    PhaseOutput {
        phase_a: None,
        phase_b: MAX_DUTY_CYCLE,
        phase_c: MIN_DUTY_CYCLE,
    },
    PhaseOutput {
        phase_a: MIN_DUTY_CYCLE,
        phase_b: MAX_DUTY_CYCLE,
        phase_c: None,
    },
    PhaseOutput {
        phase_a: MIN_DUTY_CYCLE,
        phase_b: None,
        phase_c: MAX_DUTY_CYCLE,
    },
    PhaseOutput {
        phase_a: None,
        phase_b: MIN_DUTY_CYCLE,
        phase_c: MAX_DUTY_CYCLE,
    },
];

/// Represents a three state duty cycle
type DutyCycle = Option<u8>;

/// Represents the control outputs for a three phase inverter.
#[derive(Copy, Clone)]
struct PhaseOutput {
    phase_a: DutyCycle,
    phase_b: DutyCycle,
    phase_c: DutyCycle,
}

/// Represents a half bridge driven by a high side and low side enable pin
struct HalfBridge<'d, S>
where
    S: Slice,
{
    high_pwm: PwmOutput<'d>,
    low_pwm: PwmOutput<'d>,
    _slice: PhantomData<S>,
}

impl<'d, S> HalfBridge<'d, S>
where
    S: Slice,
{
    /// Instantates a new half bridge driver with two GPIO and a PWM slice
    fn new(
        slice: Peri<'d, S>,
        high: Peri<'d, impl ChannelAPin<S>>,
        low: Peri<'d, impl ChannelBPin<S>>,
    ) -> Self {
        let pwm_config = Config::default();
        let pwm = Pwm::new_output_ab(slice, high, low, pwm_config);
        let (high_pwm, low_pwm) = pwm.split();

        Self {
            high_pwm: high_pwm.unwrap(),
            low_pwm: low_pwm.unwrap(),
            _slice: PhantomData,
        }
    }

    /// Set the half bridge to PWM the high side gate to the specified duty cycle
    fn set_high(&mut self, percentage: u8) {
        let _ = self.low_pwm.set_duty_cycle_fully_off();
        let _ = self.high_pwm.set_duty_cycle_percent(percentage);
    }

    /// Set the half bridge to be driven fully off
    fn set_low(&mut self) {
        let _ = self.high_pwm.set_duty_cycle_fully_off();
        let _ = self.low_pwm.set_duty_cycle_fully_on();
    }

    /// Changes the half bridge to a high impedance output
    fn set_high_impedance(&mut self) {
        let _ = self.high_pwm.set_duty_cycle_fully_off();
        let _ = self.low_pwm.set_duty_cycle_fully_off();
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let _onboard_led = Output::new(p.PIN_25, Level::High);

    let embassy_rp::Peripherals {
        PIN_10,
        PIN_11,
        PIN_12,
        PIN_13,
        PIN_14,
        PIN_15,
        ..
    } = p;

    let half_bridge_a = HalfBridge::new(p.PWM_SLICE5, PIN_10, PIN_11);
    let half_bridge_b = HalfBridge::new(p.PWM_SLICE6, PIN_12, PIN_13);
    let half_bridge_c = HalfBridge::new(p.PWM_SLICE7, PIN_14, PIN_15);

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
    mut half_bridge_a: HalfBridge<'static, embassy_rp::peripherals::PWM_SLICE5>,
    mut half_bridge_b: HalfBridge<'static, embassy_rp::peripherals::PWM_SLICE6>,
    mut half_bridge_c: HalfBridge<'static, embassy_rp::peripherals::PWM_SLICE7>,
) {
    let mut step: usize = 0;
    let mut ticker = Ticker::every(Duration::from_millis(50));

    loop {
        ticker.next().await;
        step = (step + 1) % THREE_PHASE_COMMUTATION_TABLE.len();

        let output = THREE_PHASE_COMMUTATION_TABLE[step];

        match output.phase_a {
            Some(0) => half_bridge_a.set_low(),
            Some(percentage) => half_bridge_a.set_high(percentage),
            None => half_bridge_a.set_high_impedance(),
        };

        match output.phase_b {
            Some(0) => half_bridge_b.set_low(),
            Some(percentage) => half_bridge_b.set_high(percentage),
            None => half_bridge_b.set_high_impedance(),
        };

        match output.phase_c {
            Some(0) => half_bridge_c.set_low(),
            Some(percentage) => half_bridge_c.set_high(percentage),
            None => half_bridge_c.set_high_impedance(),
        };
    }
}
