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
    fn cabin_flow(&self) -> [MassRate; 2];
    fn flow_coefficient(&self) -> f64;
    fn z_coefficient(&self) -> f64;
}
