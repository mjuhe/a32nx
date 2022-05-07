use self::{
    cabin_pressure_simulation::CabinPressureSimulation,
    pressure_valve::{PressureValve, PressureValveSignal},
};
use crate::shared::PressurizationOverheadShared;

use uom::si::f64::*;
pub mod cabin_pressure_controller;
pub mod cabin_pressure_simulation;
pub mod pressure_valve;

pub trait OutflowValveActuator {
    fn target_valve_position(
        &self,
        press_overhead: &impl PressurizationOverheadShared,
        cabin_pressure_simulation: &CabinPressureSimulation,
    ) -> Ratio;
}

pub trait CabinPressure {
    fn exterior_pressure(&self) -> Pressure;
    fn cabin_pressure(&self) -> Pressure;
}
