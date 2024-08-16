use systems::{
    air_conditioning::Air,
    pneumatic::{
        valve::{DefaultValve, PressureRegulatingValve, PurelyPneumaticValve},
        ControllablePneumaticValve, PneumaticContainer, PneumaticPipe, PneumaticValveSignal,
    },
    shared::{
        arinc429::SignStatus, ControllerSignal, ElectricalBusType, ElectricalBuses,
        InternationalStandardAtmosphere,
    },
    simulation::{
        InitContext, SimulationElement, SimulationElementVisitor, SimulatorWriter, UpdateContext,
        VariableIdentifier, Write,
    },
};
use uom::si::{
    area::square_attometer,
    f64::*,
    mass::kilogram,
    pressure::{bar, psi},
    ratio::ratio,
    thermodynamic_temperature::degree_celsius,
    volume::cubic_meter,
};

struct A380OxygenSystem {
    cabin_oxygen_system: CabinOxygenSystem,
    crew_oxygen_system: CrewOxygenSystem,
    oscu: OxygenSystemControlUnit,
}

impl A380OxygenSystem {
    // Reference: https://www.ncbi.nlm.nih.gov/pmc/articles/PMC8672270/

    const HUMAN_LUNG_TIDAL_EXERCISE_VOLUME_PER_SECOND_METER: f64 = 0.0021; // m3/s
    const OXYGEN_KG_PER_CUBIC_METER: f64 = 1.429; // kg/m3

    fn new(context: &mut InitContext) -> Self {
        Self {
            cabin_oxygen_system: CabinOxygenSystem {},
            crew_oxygen_system: CrewOxygenSystem::new(context),
            oscu: OxygenSystemControlUnit {},
        }
    }

    #[cfg(test)]
    fn new_with_pressure(context: &mut InitContext, cylinder_pressure: Pressure) -> Self {
        Self {
            cabin_oxygen_system: CabinOxygenSystem {},
            crew_oxygen_system: CrewOxygenSystem::new_with_pressure(context, cylinder_pressure),
            oscu: OxygenSystemControlUnit {},
        }
    }

    fn update(&mut self, context: &UpdateContext) {
        self.crew_oxygen_system.update(context);
    }

    #[cfg(test)]
    fn set_number_of_crew_consumers(&mut self, number_of_consumers: usize) {
        self.crew_oxygen_system.set_consumers(number_of_consumers);
    }
}

impl SimulationElement for A380OxygenSystem {
    fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T) {
        self.crew_oxygen_system.accept(visitor);

        visitor.visit(self);
    }
}

struct CabinOxygenSystem {}

struct CrewOxygenSystem {
    crew_oxygen_pressure_id: VariableIdentifier,

    cylinder: OxygenCylinder,
    pressure_reducer_transmitter: PressureReducerTransmitter,
    low_pressure_distribution_circuit: LowPressureDistributionCircuit,
    // This is the count of how many people are consuming air
    air_consumers: Option<usize>,
}

impl CrewOxygenSystem {
    const CREW_MASK_EFFICIENCY: f64 = 0.14;

    fn new(context: &mut InitContext) -> Self {
        Self {
            crew_oxygen_pressure_id: context.get_identifier("OXYGEN_CREW_PRESSURE".to_owned()),

            // TODO: Randomise pressure of oxygen
            cylinder: OxygenCylinder::new(
                Volume::new::<cubic_meter>(2.235),
                Pressure::new::<psi>(1800.),
            ),
            pressure_reducer_transmitter: PressureReducerTransmitter::new(),
            low_pressure_distribution_circuit: LowPressureDistributionCircuit::new(
                ElectricalBusType::DirectCurrentEssential,
            ),

            air_consumers: None,
        }
    }

    #[cfg(test)]
    fn new_with_pressure(context: &mut InitContext, pressure: Pressure) -> Self {
        Self {
            crew_oxygen_pressure_id: context.get_identifier("OXYGEN_CREW_PRESSURE".to_owned()),

            // TODO: Randomise pressure of oxygen
            cylinder: OxygenCylinder::new(Volume::new::<cubic_meter>(2.235), pressure),
            pressure_reducer_transmitter: PressureReducerTransmitter::new(),
            low_pressure_distribution_circuit: LowPressureDistributionCircuit::new(
                ElectricalBusType::DirectCurrentEssential,
            ),

            air_consumers: None,
        }
    }

    fn update(&mut self, context: &UpdateContext) {
        // When the cylinder is at ambient pressure consumers can't breath from it anymore
        if self.air_consumers.is_some() && self.cylinder.pressure() > context.ambient_pressure() {
            let air_consumed_per_person = Mass::new::<kilogram>(
                A380OxygenSystem::HUMAN_LUNG_TIDAL_EXERCISE_VOLUME_PER_SECOND_METER
                    * A380OxygenSystem::OXYGEN_KG_PER_CUBIC_METER
                    * context.delta_as_secs_f64(),
            );
            let total_air_consumed = self.air_consumers.unwrap() as f64
                * air_consumed_per_person
                * (1. / Self::CREW_MASK_EFFICIENCY);
            self.low_pressure_distribution_circuit.change_fluid_amount(
                -total_air_consumed,
                self.low_pressure_distribution_circuit.temperature(),
                self.low_pressure_distribution_circuit.pressure(),
            );
        }
        self.pressure_reducer_transmitter
            .update(context, &mut self.cylinder);
        self.low_pressure_distribution_circuit
            .update(context, &mut self.pressure_reducer_transmitter);
        // println!("Cylinder pressure: {}, pressure reducer transmitter: {} low pressure distribution circuit: {}", self.cylinder.pressure().get::<psi>(), self.pressure_reducer_transmitter.pressure().get::<psi>(), self.low_pressure_distribution_circuit.pressure().get::<psi>());
        // println!("Cylinder mass: {}, pressure reducer transmitter: {} low pressure distribution circuit: {}", self.cylinder.mass().get::<kilogram>(), self.pressure_reducer_transmitter.mass().get::<kilogram>(), self.low_pressure_distribution_circuit.mass().get::<kilogram>());
        println!(
            "{}, {}, {}, {}, {}, {}",
            self.cylinder.pressure().get::<psi>(),
            self.pressure_reducer_transmitter.pressure().get::<psi>(),
            self.low_pressure_distribution_circuit
                .pressure()
                .get::<psi>(),
            self.cylinder.mass().get::<kilogram>(),
            self.pressure_reducer_transmitter.mass().get::<kilogram>(),
            self.low_pressure_distribution_circuit
                .mass()
                .get::<kilogram>()
        );
    }

    fn set_consumers(&mut self, number_of_consumers: usize) {
        self.air_consumers = Some(number_of_consumers);
    }
}

impl SimulationElement for CrewOxygenSystem {
    fn write(&self, writer: &mut SimulatorWriter) {
        // TODO: Correct sign status
        writer.write_arinc429(
            &self.crew_oxygen_pressure_id,
            self.cylinder.pressure(),
            SignStatus::NormalOperation,
        );
    }

    fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T) {
        self.low_pressure_distribution_circuit.accept(visitor);

        visitor.visit(self);
    }
}

// REGUL LO PR warning message when low pressure distribution circuit is below 3.3 bars
// Write cylinder pressure
struct OxygenSystemControlUnit {}

struct LowPressureSupplyValveSignal {
    target_open_amount: Ratio,
}

impl PneumaticValveSignal for LowPressureSupplyValveSignal {
    fn new(target_open_amount: Ratio) -> Self {
        Self { target_open_amount }
    }

    fn target_open_amount(&self) -> Ratio {
        self.target_open_amount
    }
}

struct LowPressureSupplyValveController {
    should_close_valve: bool,
    // TODO: Overhead signal
}

impl LowPressureSupplyValveController {
    fn new() -> Self {
        Self {
            should_close_valve: false,
        }
    }

    fn update(&mut self) {
        self.should_close_valve = false;
    }
}

impl ControllerSignal<LowPressureSupplyValveSignal> for LowPressureSupplyValveController {
    fn signal(&self) -> Option<LowPressureSupplyValveSignal> {
        if self.should_close_valve {
            Some(LowPressureSupplyValveSignal::new(Ratio::default()))
        } else {
            Some(LowPressureSupplyValveSignal::new(Ratio::new::<ratio>(1.)))
        }
    }
}

/// The low pressure distribution circuit connects the high pressure oxygen from the bottles to the low pressure manifold and
/// eventually the masks. The low pressure supply valve sends its position to the OSCU, and is controlled by the overhead panel
/// directly. If the valve is not energised, it maintains its current position.
struct LowPressureDistributionCircuit {
    distribution_circuit: PneumaticPipe,
    low_pressure_supply_valve: DefaultValve,
    low_pressure_supply_valve_controller: LowPressureSupplyValveController,

    powered_by: ElectricalBusType,
    is_powered: bool,
}

impl LowPressureDistributionCircuit {
    fn new(powered_by: ElectricalBusType) -> Self {
        Self {
            distribution_circuit: PneumaticPipe::new(
                Volume::new::<cubic_meter>(0.5),
                InternationalStandardAtmosphere::ground_pressure(),
                ThermodynamicTemperature::new::<degree_celsius>(15.),
            ),
            low_pressure_supply_valve: DefaultValve::new_closed(),
            low_pressure_supply_valve_controller: LowPressureSupplyValveController::new(),

            powered_by,
            is_powered: false,
        }
    }

    fn update(
        &mut self,
        context: &UpdateContext,
        pressure_reducer_transmitter: &mut impl PneumaticContainer,
    ) {
        if self.is_powered {
            self.low_pressure_supply_valve_controller.update();
            self.low_pressure_supply_valve
                .update_open_amount(&self.low_pressure_supply_valve_controller);
        }
        self.low_pressure_supply_valve.update_move_fluid(
            context,
            pressure_reducer_transmitter,
            &mut self.distribution_circuit,
        );
    }
}

impl SimulationElement for LowPressureDistributionCircuit {
    fn receive_power(&mut self, buses: &impl ElectricalBuses) {
        self.is_powered = buses.is_powered(self.powered_by)
    }
}

impl PneumaticContainer for LowPressureDistributionCircuit {
    fn pressure(&self) -> Pressure {
        self.distribution_circuit.pressure()
    }

    fn volume(&self) -> Volume {
        self.distribution_circuit.volume()
    }

    fn temperature(&self) -> ThermodynamicTemperature {
        self.distribution_circuit.temperature()
    }

    fn mass(&self) -> Mass {
        self.distribution_circuit.mass()
    }

    fn change_fluid_amount(
        &mut self,
        fluid_amount: Mass,
        fluid_temperature: ThermodynamicTemperature,
        fluid_pressure: Pressure,
    ) {
        self.distribution_circuit.change_fluid_amount(
            fluid_amount,
            fluid_temperature,
            fluid_pressure,
        );
    }

    fn update_temperature(&mut self, temperature_change: TemperatureInterval) {
        self.distribution_circuit
            .update_temperature(temperature_change);
    }
}

/// The Reducer/Transmitter decreases the pressure of the oxygen from the cylinder to the operating pressure of the system
/// and sends the cylinder pressure signal to the OSCU
struct PressureReducerTransmitter {
    pressure_reducer: PressureRegulatingValve,
    outlet_pipe: PneumaticPipe,
}

impl PressureReducerTransmitter {
    const LOW_PRESSURE_CIRCUIT_PSI: f64 = 5.;
    fn new() -> Self {
        Self {
            pressure_reducer: PressureRegulatingValve::new(),
            outlet_pipe: PneumaticPipe::new(
                Volume::new::<cubic_meter>(0.5),
                InternationalStandardAtmosphere::ground_pressure(),
                ThermodynamicTemperature::new::<degree_celsius>(15.),
            ),
        }
    }

    fn update(&mut self, context: &UpdateContext, cylinder: &mut impl PneumaticContainer) {
        self.pressure_reducer.update_move_fluid(
            context,
            cylinder,
            &mut self.outlet_pipe,
            Pressure::new::<bar>(Self::LOW_PRESSURE_CIRCUIT_PSI),
        );
    }
}

impl PneumaticContainer for PressureReducerTransmitter {
    fn pressure(&self) -> Pressure {
        self.outlet_pipe.pressure()
    }

    fn volume(&self) -> Volume {
        self.outlet_pipe.volume()
    }

    fn temperature(&self) -> ThermodynamicTemperature {
        self.outlet_pipe.temperature()
    }

    fn mass(&self) -> Mass {
        self.outlet_pipe.mass()
    }

    fn change_fluid_amount(
        &mut self,
        fluid_amount: Mass,
        fluid_temperature: ThermodynamicTemperature,
        fluid_pressure: Pressure,
    ) {
        self.outlet_pipe
            .change_fluid_amount(fluid_amount, fluid_temperature, fluid_pressure);
    }

    fn update_temperature(&mut self, temperature_change: TemperatureInterval) {
        self.outlet_pipe.update_temperature(temperature_change);
    }
}

struct OxygenCylinder {
    cylinder: PneumaticPipe,
    // TODO: add failures
}

impl OxygenCylinder {
    fn new(volume: Volume, pressure: Pressure) -> Self {
        Self {
            cylinder: PneumaticPipe::new(
                volume,
                pressure,
                ThermodynamicTemperature::new::<degree_celsius>(15.),
            ),
        }
    }
}

impl PneumaticContainer for OxygenCylinder {
    fn pressure(&self) -> Pressure {
        self.cylinder.pressure()
    }

    fn volume(&self) -> Volume {
        self.cylinder.volume()
    }

    fn temperature(&self) -> ThermodynamicTemperature {
        self.cylinder.temperature()
    }

    fn mass(&self) -> Mass {
        self.cylinder.mass()
    }

    fn change_fluid_amount(
        &mut self,
        fluid_amount: Mass,
        fluid_temperature: ThermodynamicTemperature,
        fluid_pressure: Pressure,
    ) {
        self.cylinder
            .change_fluid_amount(fluid_amount, fluid_temperature, fluid_pressure);
    }

    fn update_temperature(&mut self, temperature_change: TemperatureInterval) {
        self.cylinder.update_temperature(temperature_change);
    }
}

#[cfg(test)]
mod a380_oxygen_tests {
    use std::time::Duration;

    use systems::{
        electrical::{test::TestElectricitySource, ElectricalBus, Electricity},
        engine::EngineFireOverheadPanel,
        shared::{arinc429::Arinc429Word, PotentialOrigin},
        simulation::{
            test::{ReadByName, SimulationTestBed, TestBed, WriteByName},
            Aircraft, InitContext, SimulationElement, SimulationElementVisitor, UpdateContext,
        },
    };

    use super::*;

    struct TestAircraft {
        a380_oxygen_system: A380OxygenSystem,

        powered_dc_source_ess: TestElectricitySource,
        dc_ess_bus: ElectricalBus,
    }

    impl TestAircraft {
        fn new(context: &mut InitContext, pressure: Pressure) -> Self {
            Self {
                a380_oxygen_system: A380OxygenSystem::new_with_pressure(context, pressure),

                powered_dc_source_ess: TestElectricitySource::powered(
                    context,
                    PotentialOrigin::EmergencyGenerator,
                ),

                dc_ess_bus: ElectricalBus::new(context, ElectricalBusType::DirectCurrentEssential),
            }
        }

        fn set_number_of_crew_consumers(&mut self, number_of_consumers: usize) {
            self.a380_oxygen_system
                .set_number_of_crew_consumers(number_of_consumers);
        }

        fn power_dc_ess_bus(&mut self) {
            self.powered_dc_source_ess.power();
        }

        fn unpower_dc_ess_bus(&mut self) {
            self.powered_dc_source_ess.unpower();
        }
    }
    impl Aircraft for TestAircraft {
        fn update_before_power_distribution(
            &mut self,
            _context: &UpdateContext,
            electricity: &mut Electricity,
        ) {
            electricity.supplied_by(&self.powered_dc_source_ess);
            electricity.flow(&self.powered_dc_source_ess, &self.dc_ess_bus);
        }

        fn update_after_power_distribution(&mut self, context: &UpdateContext) {
            self.a380_oxygen_system.update(context);
        }
    }
    impl SimulationElement for TestAircraft {
        fn accept<V: SimulationElementVisitor>(&mut self, visitor: &mut V) {
            self.a380_oxygen_system.accept(visitor);

            visitor.visit(self);
        }
    }

    struct OxygenTestBed {
        test_bed: SimulationTestBed<TestAircraft>,
    }
    impl OxygenTestBed {
        fn new(cylinder_pressure: Pressure) -> Self {
            let mut test_bed = Self {
                test_bed: SimulationTestBed::<TestAircraft>::new(|context| {
                    TestAircraft::new(context, cylinder_pressure)
                }),
            };
            test_bed.set_ambient_pressure(InternationalStandardAtmosphere::ground_pressure());
            test_bed
        }

        fn with(self) -> Self {
            self
        }

        fn and(self) -> Self {
            self
        }

        fn then(self) -> Self {
            self
        }

        fn and_run(mut self) -> Self {
            self.run();
            self
        }

        fn run_with_delta_of(mut self, delta: Duration) -> Self {
            self.run_with_delta(delta);
            self
        }

        fn run_with_no_delta(mut self) -> Self {
            self.run_with_delta(Duration::ZERO);
            self
        }

        fn iterate(mut self, iterations: usize) -> Self {
            for _ in 0..iterations {
                self.run();
            }
            self
        }

        fn iterate_with_delta(mut self, iterations: usize, delta: Duration) -> Self {
            for _ in 0..iterations {
                self.run_with_delta(delta);
            }
            self
        }

        fn with_crew_breating_into_mask(mut self, number_of_consumers: usize) -> Self {
            self.command(|a| a.set_number_of_crew_consumers(number_of_consumers));
            self
        }

        fn crew_oxygen_pressure(&mut self) -> Pressure {
            self.query(|a| a.a380_oxygen_system.crew_oxygen_system.cylinder.pressure())
        }

        fn crew_pressure_regulator_pressure(&mut self) -> Pressure {
            self.query(|a| {
                a.a380_oxygen_system
                    .crew_oxygen_system
                    .pressure_reducer_transmitter
                    .pressure()
            })
        }

        fn crew_oxygen_pressure_read(&mut self) -> Arinc429Word<Pressure> {
            self.read_arinc429_by_name("OXYGEN_CREW_PRESSURE")
        }
    }
    impl TestBed for OxygenTestBed {
        type Aircraft = TestAircraft;

        fn test_bed(&self) -> &SimulationTestBed<TestAircraft> {
            &self.test_bed
        }

        fn test_bed_mut(&mut self) -> &mut SimulationTestBed<TestAircraft> {
            &mut self.test_bed
        }
    }

    fn test_bed() -> OxygenTestBed {
        OxygenTestBed::new(Pressure::new::<psi>(1800.))
    }

    fn test_bed_with_initial_pressure(pressure: Pressure) -> OxygenTestBed {
        OxygenTestBed::new(pressure)
    }

    mod a380_oxygen_system_tests {
        use super::*;

        #[test]
        fn pressure_writes_to_arinc() {
            let mut test_bed = test_bed().iterate(5);

            assert!(
                (test_bed
                    .crew_oxygen_pressure_read()
                    .normal_value()
                    .unwrap()
                    .get::<psi>()
                    - test_bed.crew_oxygen_pressure().get::<psi>())
                    < f64::EPSILON
            );
        }

        #[test]
        fn pressure_does_not_go_down_in_steady_state() {
            let mut test_bed = test_bed().iterate(500);

            let initial_pressure = test_bed.crew_oxygen_pressure();

            test_bed = test_bed.iterate(500);

            assert!(
                (test_bed.crew_oxygen_pressure().get::<psi>() - initial_pressure.get::<psi>())
                    .abs()
                    < 1.
            );
        }

        #[test]
        fn pressure_goes_down_with_consumers() {
            let mut test_bed = test_bed()
                .iterate(100)
                .with_crew_breating_into_mask(4)
                .iterate(400);

            let initial_pressure = test_bed.crew_oxygen_pressure();

            test_bed = test_bed.iterate(500);

            assert!(
                (test_bed.crew_oxygen_pressure().get::<psi>() - initial_pressure.get::<psi>())
                    < -1.
            );
        }

        #[test]
        fn pressure_regulator_outputs_right_pressure() {
            let mut test_bed = test_bed()
                .iterate(100)
                .with_crew_breating_into_mask(4)
                .iterate(400);

            let initial_pressure = test_bed.crew_pressure_regulator_pressure();

            test_bed = test_bed.iterate(500);

            assert!(
                (test_bed.crew_pressure_regulator_pressure().get::<psi>()
                    - initial_pressure.get::<psi>())
                .abs()
                    < 1.
            );

            assert!((test_bed.crew_pressure_regulator_pressure().get::<bar>() - 5.).abs() < 0.1)
        }

        #[test]
        fn pressure_regulator_outputs_right_pressure_when_bottle_pressure_is_low() {
            let mut test_bed = test_bed_with_initial_pressure(Pressure::new::<psi>(400.))
                .iterate(100)
                .with_crew_breating_into_mask(4)
                .iterate(400);

            let initial_pressure = test_bed.crew_pressure_regulator_pressure();

            test_bed = test_bed.iterate(500);

            assert!(
                (test_bed.crew_pressure_regulator_pressure().get::<psi>()
                    - initial_pressure.get::<psi>())
                .abs()
                    < 2.
            );

            assert!((test_bed.crew_pressure_regulator_pressure().get::<bar>() - 5.).abs() < 0.1)
        }

        #[test]
        fn oxygen_bottle_runs_out_of_oxygen_per_references() {
            let mut test_bed = test_bed_with_initial_pressure(Pressure::new::<psi>(350.))
                .iterate(10)
                .with_crew_breating_into_mask(2)
                .iterate(1350);

            // A 350 psi bottle should supply 2 crew members for 15 minutes (+ 50% safety margin)
            println!(
                "{}, {}",
                test_bed.crew_oxygen_pressure().get::<psi>(),
                InternationalStandardAtmosphere::ground_pressure().get::<psi>()
            );
            assert!(
                (test_bed.crew_oxygen_pressure().get::<psi>()
                    - InternationalStandardAtmosphere::ground_pressure().get::<psi>())
                .abs()
                    < 1.
            );
        }

        #[test]
        fn oxygen_bottle_pressure_does_not_go_below_ambient() {
            let mut test_bed = test_bed_with_initial_pressure(Pressure::new::<psi>(350.))
                .iterate(10)
                .with_crew_breating_into_mask(2)
                .iterate(1800);

            // A 350 psi bottle should supply 2 crew members for 15 minutes (+ 50% safety margin)
            println!(
                "{}, {}",
                test_bed.crew_oxygen_pressure().get::<psi>(),
                InternationalStandardAtmosphere::ground_pressure().get::<psi>()
            );
            assert!(
                (test_bed.crew_oxygen_pressure().get::<psi>()
                    - InternationalStandardAtmosphere::ground_pressure().get::<psi>())
                .abs()
                    < 5.
            );
        }
    }
}
