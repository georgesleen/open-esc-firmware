#![no_std]
#![no_main]

use core::marker::PhantomData;

use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output, Pin};
use embassy_rp::Peri;
use embassy_time::{Duration, Ticker};
use {defmt_rtt as _, panic_probe as _};

/// Represents an optional duty cycle
type DutyCycle = Option<f32>;

/// Represents the control outputs for a three phase inverter.
#[derive(Copy, Clone)]
struct PhaseOutput {
    phase_a: DutyCycle,
    phase_b: DutyCycle,
    phase_c: DutyCycle,
}

/// Lookup table for phase voltages when commuting a three phase inverter.
const THREE_PHASE_COMMUTATION_TABLE: [PhaseOutput; 6] = [
    PhaseOutput {
        phase_a: Some(1.0),
        phase_b: Some(0.0),
        phase_c: None,
    },
    PhaseOutput {
        phase_a: Some(1.0),
        phase_b: None,
        phase_c: Some(0.0),
    },
    PhaseOutput {
        phase_a: None,
        phase_b: Some(1.0),
        phase_c: Some(0.0),
    },
    PhaseOutput {
        phase_a: Some(0.0),
        phase_b: Some(1.0),
        phase_c: None,
    },
    PhaseOutput {
        phase_a: Some(0.0),
        phase_b: None,
        phase_c: Some(1.0),
    },
    PhaseOutput {
        phase_a: None,
        phase_b: Some(0.0),
        phase_c: Some(1.0),
    },
];

struct DrivenHigh;
struct DrivenLow;
struct HighImpedance;

struct HalfBridge<'d, State> {
    high_pin: Output<'d>,
    low_pin: Output<'d>,
    _state: PhantomData<State>,
}

impl<'d> HalfBridge<'d, HighImpedance> {
    /// Instantates a new half bridge driver with two GPIO
    fn new(high_pin: Peri<'d, impl Pin>, low_pin: Peri<'d, impl Pin>) -> Self {
        Self {
            high_pin: Output::new(high_pin, Level::Low),
            low_pin: Output::new(low_pin, Level::Low),
            _state: PhantomData::<HighImpedance>,
        }
    }

    /// Changes the half bridge output to V+
    fn set_high(mut self) -> HalfBridge<'d, DrivenHigh> {
        self.high_pin.set_high();
        self.low_pin.set_low();
        HalfBridge {
            high_pin: self.high_pin,
            low_pin: self.low_pin,
            _state: PhantomData,
        }
    }

    /// Changes the half bridge output to V-
    fn set_low(mut self) -> HalfBridge<'d, DrivenLow> {
        self.high_pin.set_low();
        self.low_pin.set_high();
        HalfBridge {
            high_pin: self.high_pin,
            low_pin: self.low_pin,
            _state: PhantomData,
        }
    }
}

impl<'d> HalfBridge<'d, DrivenHigh> {
    fn set_high_impedance(mut self) -> HalfBridge<'d, HighImpedance> {
        self.high_pin.set_low();
        self.low_pin.set_low();
        HalfBridge {
            high_pin: self.high_pin,
            low_pin: self.low_pin,
            _state: PhantomData,
        }
    }
}

impl<'d> HalfBridge<'d, DrivenLow> {
    fn set_high_impedance(mut self) -> HalfBridge<'d, HighImpedance> {
        self.high_pin.set_low();
        self.low_pin.set_low();
        HalfBridge {
            high_pin: self.high_pin,
            low_pin: self.low_pin,
            _state: PhantomData,
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut onboard_led = Output::new(p.PIN_25, Level::High);

    onboard_led.set_high();

    let embassy_rp::Peripherals {
        PIN_10,
        PIN_11,
        PIN_12,
        PIN_13,
        PIN_14,
        PIN_15,
        ..
    } = p;

    let half_bridge_a = HalfBridge::<HighImpedance>::new(PIN_10, PIN_11);
    let half_bridge_b = HalfBridge::<HighImpedance>::new(PIN_12, PIN_13);
    let half_bridge_c = HalfBridge::<HighImpedance>::new(PIN_14, PIN_15);

    let _ = spawner.spawn(bldc_driver_task(
        half_bridge_a,
        half_bridge_b,
        half_bridge_c,
    ));
}

#[embassy_executor::task]
async fn bldc_driver_task(
    mut half_bridge_a: HalfBridge<'static, HighImpedance>,
    mut half_bridge_b: HalfBridge<'static, HighImpedance>,
    mut half_bridge_c: HalfBridge<'static, HighImpedance>,
) {
    let mut step: usize = 0;
    let mut ticker = Ticker::every(Duration::from_millis(1000));

    half_bridge_a.set_high();
    half_bridge_b.set_low();

    loop {
        ticker.next().await;
        step = step + 1 % THREE_PHASE_COMMUTATION_TABLE.len();

        let output = THREE_PHASE_COMMUTATION_TABLE[step];
    }
}
