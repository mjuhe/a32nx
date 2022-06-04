use self::pressure_valve::{PressureValve, PressureValveSignal};
use crate::shared::PressurizationOverheadShared;

use uom::si::f64::*;
pub mod cabin_pressure_controller;
pub mod cabin_pressure_simulation;
pub mod pressure_valve;

pub trait OutflowValveActuator {
    fn target_valve_position(
        &self,
        press_overhead: &impl PressurizationOverheadShared,
        cabin_simulation: &impl CabinFlowProperties,
    ) -> Ratio;
}

pub trait CabinPressure {
    fn exterior_pressure(&self) -> Pressure;
    fn cabin_pressure(&self) -> Pressure;
}

pub trait CabinFlowProperties {
    fn cabin_flow_out(&self) -> MassRate;
    fn flow_coefficient(&self) -> f64;
    fn z_coefficient(&self) -> f64;
}

pub trait PressurizationConstants {
    const MAX_CLIMB_RATE: f64;
    const MAX_CLIMB_RATE_IN_DESCENT: f64;
    const MAX_DESCENT_RATE: f64;
    const MAX_ABORT_DESCENT_RATE: f64;
    const MAX_TAKEOFF_DELTA_P: f64;
    const MAX_CLIMB_DELTA_P: f64;
    const MAX_CLIMB_CABIN_ALTITUDE: f64;
    const MAX_SAFETY_DELTA_P: f64;
    const MIN_SAFETY_DELTA_P: f64;
    const TAKEOFF_RATE: f64;
    const DEPRESS_RATE: f64;
    const EXCESSIVE_ALT_WARNING: f64;
    const EXCESSIVE_RESIDUAL_PRESSURE_WARNING: f64;
    const LOW_DIFFERENTIAL_PRESSURE_WARNING: f64;
}
