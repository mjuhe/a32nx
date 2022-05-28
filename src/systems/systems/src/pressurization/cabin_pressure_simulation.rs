use crate::{
    air_conditioning::PackFlow,
    shared::{AverageExt, CabinPressurization},
    simulation::{
        InitContext, SimulationElement, SimulatorWriter, UpdateContext, VariableIdentifier, Write,
    },
};

use super::{CabinFlowProperties, CabinPressure};

use uom::si::{
    area::square_meter,
    f64::*,
    mass_density::kilogram_per_cubic_meter,
    mass_rate::kilogram_per_second,
    pressure::{hectopascal, pascal},
    ratio::{percent, ratio},
    thermodynamic_temperature::kelvin,
    velocity::{foot_per_minute, meter_per_second},
    volume::cubic_meter,
};

use bounded_vec_deque::BoundedVecDeque;

pub struct CabinPressureSimulation {
    cabin_vs_id: VariableIdentifier,
    cabin_delta_pressure_id: VariableIdentifier,

    initialized: bool,
    previous_exterior_pressure: BoundedVecDeque<Pressure>,
    exterior_pressure: Pressure,
    outflow_valve_open_amount: Ratio,
    safety_valve_open_amount: Ratio,
    z_coefficient: f64,
    flow_coefficient: f64,
    cabin_flow_in: MassRate,
    cabin_flow_out: MassRate,
    cabin_vs: Velocity,
    cabin_pressure: Pressure,
    cabin_air_density: MassDensity,
    cabin_previous_temperature: ThermodynamicTemperature,

    //Aircraft dependant constants
    cabin_volume: Volume,
    cabin_leakage_area: Area,
    outflow_valve_size: Area,
    safety_valve_size: Area,
}

impl CabinPressureSimulation {
    // Atmospheric constants
    const R: f64 = 287.058; // Specific gas constant for air - m2/s2/K
    const GAMMA: f64 = 1.4; // Rate of specific heats for air
    const G: f64 = 9.80665; // Gravity - m/s2

    pub fn new(
        context: &mut InitContext,
        cabin_volume: Volume,
        cabin_leakage_area: Area,
        outflow_valve_size: Area,
        safety_valve_size: Area,
    ) -> Self {
        Self {
            cabin_vs_id: context.get_identifier("PRESS_CABIN_VS".to_owned()),
            cabin_delta_pressure_id: context
                .get_identifier("PRESS_CABIN_DELTA_PRESSURE".to_owned()),

            initialized: false,
            previous_exterior_pressure: BoundedVecDeque::from_iter(
                vec![Pressure::new::<hectopascal>(1013.25); 20],
                20,
            ),
            exterior_pressure: Pressure::new::<hectopascal>(1013.25),
            outflow_valve_open_amount: Ratio::new::<percent>(100.),
            safety_valve_open_amount: Ratio::new::<percent>(0.),
            z_coefficient: 0.0011,
            flow_coefficient: 1.,
            cabin_flow_in: MassRate::new::<kilogram_per_second>(0.),
            cabin_flow_out: MassRate::new::<kilogram_per_second>(0.),
            cabin_vs: Velocity::new::<meter_per_second>(0.),
            cabin_pressure: Pressure::new::<hectopascal>(1013.25),
            cabin_air_density: MassDensity::new::<kilogram_per_cubic_meter>(1.225),
            cabin_previous_temperature: ThermodynamicTemperature::new::<kelvin>(297.15),

            cabin_volume,
            cabin_leakage_area,
            outflow_valve_size,
            safety_valve_size,
        }
    }

    pub fn update(
        &mut self,
        context: &UpdateContext,
        pressurization: &impl CabinPressurization,
        pack_flow: &impl PackFlow,
        lgciu_gear_compressed: bool,
        cabin_temperature: ThermodynamicTemperature,
    ) {
        if !self.initialized {
            self.cabin_pressure = self.initialize_cabin_pressure(context, lgciu_gear_compressed);
            self.initialized = true;
        }
        self.cabin_air_density = MassDensity::new::<kilogram_per_cubic_meter>(
            self.cabin_pressure.get::<pascal>()
                / (CabinPressureSimulation::R * cabin_temperature.get::<kelvin>()),
        );
        self.exterior_pressure = self.exterior_pressure_low_pass_filter(context);
        self.z_coefficient = self.calculate_z();
        self.flow_coefficient =
            self.calculate_flow_coefficient(pressurization.should_open_outflow_valve());
        self.outflow_valve_open_amount = pressurization.outflow_valve_open_amount();
        self.safety_valve_open_amount = pressurization.safety_valve_open_amount();
        self.cabin_flow_in = pack_flow.pack_flow();
        self.cabin_flow_out = self.calculate_cabin_flow_out();
        self.cabin_vs = self.calculate_cabin_vs(cabin_temperature);
        self.cabin_pressure = self.calculate_cabin_pressure(context, cabin_temperature);
        self.cabin_previous_temperature = cabin_temperature;
    }

    fn initialize_cabin_pressure(
        &mut self,
        context: &UpdateContext,
        lgciu_gear_compressed: bool,
    ) -> Pressure {
        if lgciu_gear_compressed {
            context.ambient_pressure()
        } else {
            // Formula to simulate pressure start state if starting in flight
            let ambient_pressure: f64 = context.ambient_pressure().get::<hectopascal>();
            Pressure::new::<hectopascal>(
                -0.0002 * ambient_pressure.powf(2.) + 0.5463 * ambient_pressure + 658.85,
            )
        }
    }

    fn exterior_pressure_low_pass_filter(&mut self, context: &UpdateContext) -> Pressure {
        self.previous_exterior_pressure.pop_front();
        self.previous_exterior_pressure
            .push_back(context.ambient_pressure());

        self.previous_exterior_pressure.iter().average()
    }

    fn calculate_z(&self) -> f64 {
        const Z_VALUE_FOR_RP_CLOSE_TO_1: f64 = 1e-5;
        const Z_VALUE_FOR_RP_UNDER_053: f64 = 0.256;

        let pressure_ratio = (self.exterior_pressure / self.cabin_pressure).get::<ratio>();

        // Margin to avoid singularity at delta P = 0
        let margin = 1e-5;
        if (pressure_ratio - 1.).abs() <= margin {
            Z_VALUE_FOR_RP_CLOSE_TO_1
        } else if pressure_ratio > 0.53 {
            (pressure_ratio.powf(2. / Self::GAMMA)
                - pressure_ratio.powf((Self::GAMMA + 1.) / Self::GAMMA))
            .abs()
        } else {
            Z_VALUE_FOR_RP_UNDER_053
        }
    }

    fn calculate_flow_coefficient(&self, should_open_outflow_valve: bool) -> f64 {
        let pressure_ratio = (self.exterior_pressure / self.cabin_pressure).get::<ratio>();

        // Empirical smooth formula to avoid singularity at at delta P = 0
        let mut margin: f64 = -1.205e-4
            * (self.cabin_volume.get::<cubic_meter>() / 400.)
            * self.exterior_pressure.get::<hectopascal>()
            + 0.124108 * (self.cabin_volume.get::<cubic_meter>() / 400.);
        let slope: f64 = -5000. * margin + 510.;
        if should_open_outflow_valve {
            margin = 0.1;
            if (pressure_ratio - 1.).abs() < margin {
                -5e2 * (self.cabin_volume.get::<cubic_meter>() / 400.) * pressure_ratio
                    + 5e2 * (self.cabin_volume.get::<cubic_meter>() / 400.)
            } else if (pressure_ratio - 1.) > 0. {
                -50. * (self.cabin_volume.get::<cubic_meter>() / 400.)
            } else {
                50. * (self.cabin_volume.get::<cubic_meter>() / 400.)
            }
        } else if (pressure_ratio - 1.).abs() < margin {
            -slope * pressure_ratio * (self.cabin_volume.get::<cubic_meter>() / 400.)
                + slope * (self.cabin_volume.get::<cubic_meter>() / 400.)
        } else if (pressure_ratio - 1.) > 0. {
            -1.
        } else {
            1.
        }
    }

    fn calculate_cabin_flow_out(&self) -> MassRate {
        let area_leakage = self.cabin_leakage_area
            + self.safety_valve_size * self.safety_valve_open_amount.get::<ratio>();
        let outflow_valve_area =
            self.outflow_valve_open_amount.get::<ratio>() * self.outflow_valve_size;
        MassRate::new::<kilogram_per_second>(
            self.flow_coefficient
                * (area_leakage.get::<square_meter>() + outflow_valve_area.get::<square_meter>())
                * self.base_airflow_calculation(),
        )
    }

    fn calculate_cabin_vs(&self, cabin_temperature: ThermodynamicTemperature) -> Velocity {
        let vertical_speed = (self.cabin_flow_out.get::<kilogram_per_second>()
            - self.cabin_flow_in.get::<kilogram_per_second>())
            / ((self.cabin_air_density.get::<kilogram_per_cubic_meter>()
                * Self::G
                * self.cabin_volume.get::<cubic_meter>())
                / (Self::R * cabin_temperature.get::<kelvin>()));
        Velocity::new::<meter_per_second>(vertical_speed)
    }

    fn calculate_cabin_pressure(
        &self,
        context: &UpdateContext,
        cabin_temperature: ThermodynamicTemperature,
    ) -> Pressure {
        // Convert cabin V/S to pressure/delta
        let pressure_difference_temperature = Pressure::new::<pascal>(
            self.cabin_air_density.get::<kilogram_per_cubic_meter>()
                * Self::R
                * (cabin_temperature.get::<kelvin>()
                    - self.cabin_previous_temperature.get::<kelvin>()),
        );
        self.cabin_pressure
            * (1.
                - 2.25577e-5_f64
                    * self.cabin_vs.get::<meter_per_second>()
                    * context.delta_as_secs_f64())
            .powf(5.2559)
            + pressure_difference_temperature
    }

    fn base_airflow_calculation(&self) -> f64 {
        ((2. * (Self::GAMMA / (Self::GAMMA - 1.))
            * self.cabin_air_density.get::<kilogram_per_cubic_meter>()
            * self.cabin_pressure.get::<pascal>()
            * self.z_coefficient)
            .abs())
        .sqrt()
    }

    pub fn cabin_vs(&self) -> Velocity {
        self.cabin_vs
    }

    pub fn cabin_delta_p(&self) -> Pressure {
        self.cabin_pressure - self.exterior_pressure
    }
}

impl CabinPressure for CabinPressureSimulation {
    fn exterior_pressure(&self) -> Pressure {
        self.exterior_pressure
    }

    fn cabin_pressure(&self) -> Pressure {
        self.cabin_pressure
    }
}

impl CabinFlowProperties for CabinPressureSimulation {
    fn cabin_flow(&self) -> [MassRate; 2] {
        let flow_out_minus_ofv = self.cabin_flow_out
            - MassRate::new::<kilogram_per_second>(
                self.outflow_valve_open_amount.get::<ratio>()
                    * self.outflow_valve_size.get::<square_meter>()
                    * self.flow_coefficient
                    * self.base_airflow_calculation(),
            );
        [self.cabin_flow_in, flow_out_minus_ofv]
    }

    fn flow_coefficient(&self) -> f64 {
        self.flow_coefficient
    }

    fn z_coefficient(&self) -> f64 {
        self.z_coefficient
    }
}

impl SimulationElement for CabinPressureSimulation {
    fn write(&self, writer: &mut SimulatorWriter) {
        writer.write(&self.cabin_vs_id, self.cabin_vs().get::<foot_per_minute>());
        writer.write(&self.cabin_delta_pressure_id, self.cabin_delta_p());
    }
}
