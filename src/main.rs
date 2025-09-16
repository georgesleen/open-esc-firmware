#![no_std]
#![no_main]

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

struct HalfBridge<'d> {
    high_pin: Output<'d>,
    low_pin: Output<'d>,
}

impl<'d> HalfBridge<'d> {
    /// Instantates a new half bridge driver with two GPIO
    fn new(high_pin: Peri<'d, impl Pin>, low_pin: Peri<'d, impl Pin>) -> Self {
        Self {
            high_pin: Output::new(high_pin, Level::Low),
            low_pin: Output::new(low_pin, Level::Low),
        }
    }

    /// Changes the half bridge output to V+
    fn set_high(mut self) -> HalfBridge<'d> {
        self.high_pin.set_high();
        self.low_pin.set_low();
        HalfBridge {
            high_pin: self.high_pin,
            low_pin: self.low_pin,
        }
    }

    /// Changes the half bridge output to V-
    fn set_low(mut self) -> HalfBridge<'d> {
        self.high_pin.set_low();
        self.low_pin.set_high();
        HalfBridge {
            high_pin: self.high_pin,
            low_pin: self.low_pin,
        }
    }

    /// Changes the half bridge to a high impedance output
    fn set_high_impedance(mut self) -> HalfBridge<'d> {
        self.high_pin.set_low();
        self.low_pin.set_low();
        HalfBridge {
            high_pin: self.high_pin,
            low_pin: self.low_pin,
        }
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

    let half_bridge_a = HalfBridge::new(PIN_10, PIN_11);
    let half_bridge_b = HalfBridge::new(PIN_12, PIN_13);
    let half_bridge_c = HalfBridge::new(PIN_14, PIN_15);

    let _ = spawner.spawn(bldc_driver_task(
        half_bridge_a,
        half_bridge_b,
        half_bridge_c,
    ));

    loop {
        embassy_time::Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn bldc_driver_task(
    mut half_bridge_a: HalfBridge<'static>,
    mut half_bridge_b: HalfBridge<'static>,
    mut half_bridge_c: HalfBridge<'static>,
) {
    let mut step: usize = 0;
    let mut ticker = Ticker::every(Duration::from_millis(50));

    loop {
        ticker.next().await;
        step = (step + 1) % THREE_PHASE_COMMUTATION_TABLE.len();

        let output = THREE_PHASE_COMMUTATION_TABLE[step];

        half_bridge_a = match output.phase_a {
            Some(1.0) => half_bridge_a.set_high(),
            Some(0.0) => half_bridge_a.set_low(),
            None => half_bridge_a.set_high_impedance(),
            _ => half_bridge_a.set_high_impedance(),
        };

        half_bridge_b = match output.phase_b {
            Some(1.0) => half_bridge_b.set_high(),
            Some(0.0) => half_bridge_b.set_low(),
            None => half_bridge_b.set_high_impedance(),
            _ => half_bridge_b.set_high_impedance(),
        };

        half_bridge_c = match output.phase_c {
            Some(1.0) => half_bridge_c.set_high(),
            Some(0.0) => half_bridge_c.set_low(),
            None => half_bridge_c.set_high_impedance(),
            _ => half_bridge_c.set_high_impedance(),
        };
    }
}
