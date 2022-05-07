use systems::{
    accept_iterable,
    air_conditioning::{
        cabin_air::CabinZone, AirConditioningSystem, DuctTemperature, PackFlow, ZoneType,
    },
    overhead::{AutoManFaultPushButton, NormalOnPushButton, SpringLoadedSwitch, ValueKnob},
    pressurization::{
        cabin_pressure_controller::CabinPressureController,
        cabin_pressure_simulation::CabinPressureSimulation,
        pressure_valve::{PressureValve, PressureValveSignal},
        CabinPressure,
    },
    shared::{
        random_number, Cabin, CabinTemperature, ControllerSignal, EngineBleedPushbutton,
        EngineCorrectedN1, EngineFirePushButtons, EngineStartState, GroundSpeed,
        LgciuWeightOnWheels, PneumaticBleed, PressurizationOverheadShared,
    },
    simulation::{
        InitContext, SimulationElement, SimulationElementVisitor, SimulatorWriter, UpdateContext,
        VariableIdentifier, Write,
    },
};

use std::time::Duration;
use uom::si::{
    area::square_meter, f64::*, mass_rate::kilogram_per_second, pressure::hectopascal,
    ratio::percent, thermodynamic_temperature::kelvin, velocity::knot, volume::cubic_meter,
};

pub(super) struct A320AirConditioning {
    a320_cabin: A320Cabin,
    a320_air_conditioning_system: AirConditioningSystem<3>,
    a320_pressurization_system: A320PressurizationSystem,
}

impl A320AirConditioning {
    pub fn new(context: &mut InitContext) -> Self {
        let cabin_zones: [ZoneType; 3] =
            [ZoneType::Cockpit, ZoneType::Cabin(1), ZoneType::Cabin(2)];

        Self {
            a320_cabin: A320Cabin::new(context),
            a320_air_conditioning_system: AirConditioningSystem::new(context, cabin_zones),
            a320_pressurization_system: A320PressurizationSystem::new(context),
        }
    }

    pub fn update(
        &mut self,
        context: &UpdateContext,
        adirs: &impl GroundSpeed,
        engines: [&impl EngineCorrectedN1; 2],
        engine_fire_push_buttons: &impl EngineFirePushButtons,
        pneumatic: &(impl PneumaticBleed + EngineStartState),
        pneumatic_overhead: &impl EngineBleedPushbutton,
        pressurization_overhead: &A320PressurizationOverheadPanel,
        lgciu: [&impl LgciuWeightOnWheels; 2],
    ) {
        self.a320_air_conditioning_system.update(
            context,
            adirs,
            engines,
            engine_fire_push_buttons,
            pneumatic,
            pneumatic_overhead,
            &self.a320_pressurization_system,
            pressurization_overhead,
            lgciu,
        );
        self.a320_cabin.update(
            context,
            &self.a320_air_conditioning_system,
            &self.a320_air_conditioning_system,
            &self.a320_pressurization_system,
        );
        self.a320_pressurization_system.update(
            context,
            pressurization_overhead,
            engines,
            lgciu,
            &self.a320_air_conditioning_system,
            &self.a320_cabin,
        );
    }
}

impl SimulationElement for A320AirConditioning {
    fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T) {
        self.a320_cabin.accept(visitor);
        self.a320_air_conditioning_system.accept(visitor);
        self.a320_pressurization_system.accept(visitor);

        visitor.visit(self);
    }
}

struct A320Cabin {
    cabin_zone: [CabinZone<2>; 3],
}

impl A320Cabin {
    // TODO: Improve volume according to specs
    const A320_CABIN_VOLUME_CUBIC_METER: f64 = 200.; // m3
    const A320_COCKPIT_VOLUME_CUBIC_METER: f64 = 10.; // m3
    const A320_CABIN_LEAKAGE_AREA: f64 = 0.0003; // m2
    const A320_OUTFLOW_VALVE_SIZE: f64 = 0.03; // m2
    const A320_SAFETY_VALVE_SIZE: f64 = 0.02; //m2

    fn new(context: &mut InitContext) -> Self {
        Self {
            cabin_zone: [
                CabinZone::new(
                    context,
                    ZoneType::Cockpit,
                    Volume::new::<cubic_meter>(Self::A320_COCKPIT_VOLUME_CUBIC_METER),
                    2,
                    None,
                ),
                CabinZone::new(
                    context,
                    ZoneType::Cabin(1),
                    Volume::new::<cubic_meter>(Self::A320_CABIN_VOLUME_CUBIC_METER / 2.),
                    0,
                    Some([(1, 6), (7, 13)]),
                ),
                CabinZone::new(
                    context,
                    ZoneType::Cabin(2),
                    Volume::new::<cubic_meter>(Self::A320_CABIN_VOLUME_CUBIC_METER / 2.),
                    0,
                    Some([(14, 21), (22, 29)]),
                ),
            ],
        }
    }

    fn update(
        &mut self,
        context: &UpdateContext,
        duct_temperature: &impl DuctTemperature,
        pack_flow: &impl PackFlow,
        pressurization: &impl Cabin,
    ) {
        let flow_rate_per_cubic_meter: MassRate = MassRate::new::<kilogram_per_second>(
            pack_flow.pack_flow().get::<kilogram_per_second>()
                / (Self::A320_CABIN_VOLUME_CUBIC_METER + Self::A320_COCKPIT_VOLUME_CUBIC_METER),
        );
        for zone in self.cabin_zone.iter_mut() {
            zone.update(
                context,
                duct_temperature,
                flow_rate_per_cubic_meter,
                pressurization,
            );
        }
    }
}

impl CabinTemperature for A320Cabin {
    fn cabin_temperature(&self) -> ThermodynamicTemperature {
        // Weighted average temperature of the cabin, to be used in the pressurization system
        let average_temp_kelvin = (self.cabin_zone[0].zone_air_temperature().get::<kelvin>()
            * Self::A320_COCKPIT_VOLUME_CUBIC_METER
            + self.cabin_zone[1].zone_air_temperature().get::<kelvin>()
                * Self::A320_CABIN_VOLUME_CUBIC_METER
                / 2.
            + self.cabin_zone[2].zone_air_temperature().get::<kelvin>()
                * Self::A320_CABIN_VOLUME_CUBIC_METER
                / 2.)
            / (Self::A320_COCKPIT_VOLUME_CUBIC_METER + Self::A320_CABIN_VOLUME_CUBIC_METER);

        ThermodynamicTemperature::new::<kelvin>(average_temp_kelvin)
    }
}

impl SimulationElement for A320Cabin {
    fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T) {
        accept_iterable!(self.cabin_zone, visitor);

        visitor.visit(self);
    }
}

struct A320PressurizationSystem {
    active_cpc_sys_id: VariableIdentifier,

    cpc: [CabinPressureController; 2],
    outflow_valve: [PressureValve; 1], // Array to prepare for more than 1 outflow valve in A380
    safety_valve: PressureValve,
    residual_pressure_controller: ResidualPressureController,
    active_system: usize,

    cabin_pressure_simulation: CabinPressureSimulation, // To be merged in air con cabin
}

impl A320PressurizationSystem {
    pub fn new(context: &mut InitContext) -> Self {
        let random = random_number();
        let mut active: usize = 1;
        if random % 2 == 0 {
            active = 2
        }

        Self {
            active_cpc_sys_id: context.get_identifier("PRESS_ACTIVE_CPC_SYS".to_owned()),

            cpc: [
                CabinPressureController::new(
                    context,
                    Volume::new::<cubic_meter>(
                        A320Cabin::A320_CABIN_VOLUME_CUBIC_METER
                            + A320Cabin::A320_COCKPIT_VOLUME_CUBIC_METER,
                    ),
                    Area::new::<square_meter>(A320Cabin::A320_OUTFLOW_VALVE_SIZE),
                ),
                CabinPressureController::new(
                    context,
                    Volume::new::<cubic_meter>(
                        A320Cabin::A320_CABIN_VOLUME_CUBIC_METER
                            + A320Cabin::A320_COCKPIT_VOLUME_CUBIC_METER,
                    ),
                    Area::new::<square_meter>(A320Cabin::A320_OUTFLOW_VALVE_SIZE),
                ),
            ],
            outflow_valve: [PressureValve::new_outflow_valve(); 1],
            safety_valve: PressureValve::new_safety_valve(),
            residual_pressure_controller: ResidualPressureController::new(),
            active_system: active,

            cabin_pressure_simulation: CabinPressureSimulation::new(
                context,
                Volume::new::<cubic_meter>(
                    A320Cabin::A320_CABIN_VOLUME_CUBIC_METER
                        + A320Cabin::A320_COCKPIT_VOLUME_CUBIC_METER,
                ),
                Area::new::<square_meter>(A320Cabin::A320_CABIN_LEAKAGE_AREA),
                Area::new::<square_meter>(A320Cabin::A320_OUTFLOW_VALVE_SIZE),
                Area::new::<square_meter>(A320Cabin::A320_SAFETY_VALVE_SIZE),
            ),
        }
    }

    pub fn update(
        &mut self,
        context: &UpdateContext,
        press_overhead: &A320PressurizationOverheadPanel,
        engines: [&impl EngineCorrectedN1; 2],
        lgciu: [&impl LgciuWeightOnWheels; 2],
        pack_flow: &impl PackFlow,
        cabin_temperature: &impl CabinTemperature,
    ) {
        let lgciu_gears_compressed = lgciu
            .iter()
            .all(|&a| a.left_and_right_gear_compressed(true));

        for controller in self.cpc.iter_mut() {
            controller.update(
                context,
                engines,
                lgciu_gears_compressed,
                press_overhead,
                &self.cabin_pressure_simulation,
                &self.outflow_valve[0],
                &self.safety_valve,
                cabin_temperature,
            );
        }

        self.residual_pressure_controller.update(
            context,
            engines,
            self.outflow_valve[0].open_amount(),
            press_overhead.is_in_man_mode(),
            lgciu_gears_compressed,
            self.cabin_pressure_simulation.cabin_delta_p(),
        );

        for ofv_valve in self.outflow_valve.iter_mut() {
            ofv_valve.calculate_outflow_valve_position(
                &self.cpc[self.active_system - 1],
                press_overhead,
                &self.cabin_pressure_simulation,
            )
        }

        if self.residual_pressure_controller.signal().is_some() {
            self.outflow_valve[0].update(context, &self.residual_pressure_controller);
        } else {
            self.outflow_valve[0].update(context, press_overhead);
        }

        self.safety_valve
            .update(context, &self.cpc[self.active_system - 1]);

        self.cabin_pressure_simulation.update(
            context,
            self.outflow_valve[0].open_amount(),
            self.safety_valve.open_amount(),
            pack_flow,
            lgciu_gears_compressed,
            self.cpc[self.active_system - 1].should_open_outflow_valve()
                && !press_overhead.is_in_man_mode(),
            cabin_temperature,
        );

        self.switch_active_system();
    }

    fn switch_active_system(&mut self) {
        if self
            .cpc
            .iter_mut()
            .any(|controller| controller.should_switch_cpc())
        {
            self.active_system = if self.active_system == 1 { 2 } else { 1 };
        }
        for controller in &mut self.cpc {
            if controller.should_switch_cpc() {
                controller.reset_cpc_switch()
            }
        }
        // self.cpc.iter_mut().filter(|controller| controller.should_switch_cpc()).for_each(|controller| {
        //     controller.reset_cpc_switch();
        // });
    }
}

impl Cabin for A320PressurizationSystem {
    fn altitude(&self) -> Length {
        self.cpc[self.active_system - 1].cabin_altitude()
    }

    fn pressure(&self) -> Pressure {
        self.cabin_pressure_simulation.cabin_pressure()
    }
}

impl SimulationElement for A320PressurizationSystem {
    fn write(&self, writer: &mut SimulatorWriter) {
        writer.write(&self.active_cpc_sys_id, self.active_system);
    }

    fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T) {
        accept_iterable!(self.cpc, visitor);
        self.cabin_pressure_simulation.accept(visitor);

        visitor.visit(self);
    }
}

pub struct A320PressurizationOverheadPanel {
    mode_sel: AutoManFaultPushButton,
    man_vs_ctl_switch: SpringLoadedSwitch,
    ldg_elev_knob: ValueKnob,
    ditching: NormalOnPushButton,
}

impl A320PressurizationOverheadPanel {
    pub fn new(context: &mut InitContext) -> Self {
        Self {
            mode_sel: AutoManFaultPushButton::new_auto(context, "PRESS_MODE_SEL"),
            man_vs_ctl_switch: SpringLoadedSwitch::new(context, "PRESS_MAN_VS_CTL"),
            ldg_elev_knob: ValueKnob::new_with_value(context, "PRESS_LDG_ELEV", -2000.),
            ditching: NormalOnPushButton::new_normal(context, "PRESS_DITCHING"),
        }
    }

    fn man_vs_switch_position(&self) -> usize {
        self.man_vs_ctl_switch.position()
    }
}

impl SimulationElement for A320PressurizationOverheadPanel {
    fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T) {
        self.mode_sel.accept(visitor);
        self.man_vs_ctl_switch.accept(visitor);
        self.ldg_elev_knob.accept(visitor);
        self.ditching.accept(visitor);

        visitor.visit(self);
    }
}

impl ControllerSignal<PressureValveSignal> for A320PressurizationOverheadPanel {
    fn signal(&self) -> Option<PressureValveSignal> {
        if !self.is_in_man_mode() {
            None
        } else {
            match self.man_vs_switch_position() {
                0 => Some(PressureValveSignal::Open),
                1 => Some(PressureValveSignal::Neutral),
                2 => Some(PressureValveSignal::Close),
                _ => panic!("Could not convert manual vertical speed switch position '{}' to pressure valve signal.", self.man_vs_switch_position()),
            }
        }
    }
}

impl PressurizationOverheadShared for A320PressurizationOverheadPanel {
    fn is_in_man_mode(&self) -> bool {
        !self.mode_sel.is_auto()
    }

    fn ditching_is_on(&self) -> bool {
        self.ditching.is_on()
    }

    fn ldg_elev_is_auto(&self) -> bool {
        let margin = 100.;
        (self.ldg_elev_knob.value() + 2000.).abs() < margin
    }

    fn ldg_elev_knob_value(&self) -> f64 {
        self.ldg_elev_knob.value()
    }
}

struct ResidualPressureController {
    timer: Duration,
}

impl ResidualPressureController {
    fn new() -> Self {
        Self {
            timer: Duration::from_secs(0),
        }
    }

    fn update(
        &mut self,
        context: &UpdateContext,
        engines: [&impl EngineCorrectedN1; 2],
        outflow_valve_open_amount: Ratio,
        is_in_man_mode: bool,
        lgciu_gears_compressed: bool,
        cabin_delta_p: Pressure,
    ) {
        if outflow_valve_open_amount < Ratio::new::<percent>(100.)
            && is_in_man_mode
            && lgciu_gears_compressed
            && (!(engines
                .iter()
                .any(|&x| x.corrected_n1() > Ratio::new::<percent>(15.)))
                || context.indicated_airspeed() < Velocity::new::<knot>(70.))
            && cabin_delta_p > Pressure::new::<hectopascal>(2.5)
        {
            self.timer += context.delta();
        } else {
            self.timer = Duration::from_secs(0);
        }
    }
}

impl ControllerSignal<PressureValveSignal> for ResidualPressureController {
    fn signal(&self) -> Option<PressureValveSignal> {
        if self.timer > Duration::from_secs(15) {
            Some(PressureValveSignal::Open)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use systems::{
        overhead::AutoOffFaultPushButton,
        pneumatic::{valve::DefaultValve, EngineModeSelector, EngineState},
        shared::PneumaticValve,
        simulation::{
            test::{ReadByName, SimulationTestBed, TestBed, WriteByName},
            Aircraft, SimulationElement, SimulationElementVisitor, UpdateContext,
        },
    };
    use uom::si::{
        length::foot,
        pressure::{hectopascal, psi},
        thermodynamic_temperature::degree_celsius,
        velocity::foot_per_minute,
    };

    struct TestAdirs {
        ground_speed: Velocity,
    }
    impl TestAdirs {
        fn new() -> Self {
            Self {
                ground_speed: Velocity::new::<knot>(0.),
            }
        }
    }
    impl GroundSpeed for TestAdirs {
        fn ground_speed(&self) -> Velocity {
            self.ground_speed
        }
    }

    struct TestEngine {
        corrected_n1: Ratio,
    }
    impl TestEngine {
        fn new(engine_corrected_n1: Ratio) -> Self {
            Self {
                corrected_n1: engine_corrected_n1,
            }
        }
        fn set_engine_n1(&mut self, n: Ratio) {
            self.corrected_n1 = n;
        }
    }
    impl EngineCorrectedN1 for TestEngine {
        fn corrected_n1(&self) -> Ratio {
            self.corrected_n1
        }
    }

    struct TestEngineFirePushButtons {
        is_released: [bool; 2],
    }
    impl TestEngineFirePushButtons {
        fn new() -> Self {
            Self {
                is_released: [false, false],
            }
        }
    }
    impl EngineFirePushButtons for TestEngineFirePushButtons {
        fn is_released(&self, engine_number: usize) -> bool {
            self.is_released[engine_number - 1]
        }
    }

    struct TestFadec {
        engine_1_state: EngineState,
        engine_2_state: EngineState,
        engine_mode_selector_position: EngineModeSelector,
    }
    impl TestFadec {
        fn new(_context: &mut InitContext) -> Self {
            Self {
                engine_1_state: EngineState::Off,
                engine_2_state: EngineState::Off,
                engine_mode_selector_position: EngineModeSelector::Norm,
            }
        }

        fn engine_state(&self, number: usize) -> EngineState {
            match number {
                1 => self.engine_1_state,
                2 => self.engine_2_state,
                _ => panic!("Invalid engine number"),
            }
        }

        fn engine_mode_selector(&self) -> EngineModeSelector {
            self.engine_mode_selector_position
        }
    }

    struct TestPneumatic {
        apu_bleed_air_valve: DefaultValve,
        cross_bleed_valve: DefaultValve,
        fadec: TestFadec,
    }
    impl TestPneumatic {
        fn new(context: &mut InitContext) -> Self {
            Self {
                apu_bleed_air_valve: DefaultValve::new_closed(),
                cross_bleed_valve: DefaultValve::new_closed(),
                fadec: TestFadec::new(context),
            }
        }
    }
    impl PneumaticBleed for TestPneumatic {
        fn apu_bleed_is_on(&self) -> bool {
            self.apu_bleed_air_valve.is_open()
        }
        fn engine_crossbleed_is_on(&self) -> bool {
            self.cross_bleed_valve.is_open()
        }
    }
    impl EngineStartState for TestPneumatic {
        fn left_engine_state(&self) -> EngineState {
            self.fadec.engine_state(1)
        }
        fn right_engine_state(&self) -> EngineState {
            self.fadec.engine_state(2)
        }
        fn engine_mode_selector(&self) -> EngineModeSelector {
            self.fadec.engine_mode_selector()
        }
    }

    struct TestPneumaticOverhead {
        engine_1_bleed: AutoOffFaultPushButton,
        engine_2_bleed: AutoOffFaultPushButton,
    }
    impl TestPneumaticOverhead {
        fn new(context: &mut InitContext) -> Self {
            Self {
                engine_1_bleed: AutoOffFaultPushButton::new_auto(context, "PNEU_ENG_1_BLEED"),
                engine_2_bleed: AutoOffFaultPushButton::new_auto(context, "PNEU_ENG_2_BLEED"),
            }
        }
    }
    impl EngineBleedPushbutton for TestPneumaticOverhead {
        fn left_engine_bleed_pushbutton_is_auto(&self) -> bool {
            self.engine_1_bleed.is_auto()
        }

        fn right_engine_bleed_pushbutton_is_auto(&self) -> bool {
            self.engine_2_bleed.is_auto()
        }
    }

    struct TestLgciu {
        compressed: bool,
    }
    impl TestLgciu {
        fn new(compressed: bool) -> Self {
            Self { compressed }
        }

        fn set_on_ground(&mut self, on_ground: bool) {
            self.compressed = on_ground;
        }
    }
    impl LgciuWeightOnWheels for TestLgciu {
        fn left_and_right_gear_compressed(&self, _treat_ext_pwr_as_ground: bool) -> bool {
            self.compressed
        }
        fn right_gear_compressed(&self, _treat_ext_pwr_as_ground: bool) -> bool {
            true
        }
        fn right_gear_extended(&self, _treat_ext_pwr_as_ground: bool) -> bool {
            false
        }
        fn left_gear_compressed(&self, _treat_ext_pwr_as_ground: bool) -> bool {
            true
        }
        fn left_gear_extended(&self, _treat_ext_pwr_as_ground: bool) -> bool {
            false
        }
        fn left_and_right_gear_extended(&self, _treat_ext_pwr_as_ground: bool) -> bool {
            false
        }
        fn nose_gear_compressed(&self, _treat_ext_pwr_as_ground: bool) -> bool {
            true
        }
        fn nose_gear_extended(&self, _treat_ext_pwr_as_ground: bool) -> bool {
            false
        }
    }

    struct TestAircraft {
        a320_cabin_air: A320AirConditioning,
        adirs: TestAdirs,
        engine_1: TestEngine,
        engine_2: TestEngine,
        engine_fire_push_buttons: TestEngineFirePushButtons,
        pneumatic: TestPneumatic,
        pneumatic_overhead: TestPneumaticOverhead,
        pressurization_overhead: A320PressurizationOverheadPanel,
        lgciu1: TestLgciu,
        lgciu2: TestLgciu,
    }

    impl TestAircraft {
        fn new(context: &mut InitContext) -> Self {
            let mut test_aircraft = Self {
                a320_cabin_air: A320AirConditioning::new(context),
                adirs: TestAdirs::new(),
                engine_1: TestEngine::new(Ratio::new::<percent>(0.)),
                engine_2: TestEngine::new(Ratio::new::<percent>(0.)),
                engine_fire_push_buttons: TestEngineFirePushButtons::new(),
                pneumatic: TestPneumatic::new(context),
                pneumatic_overhead: TestPneumaticOverhead::new(context),
                pressurization_overhead: A320PressurizationOverheadPanel::new(context),
                lgciu1: TestLgciu::new(false),
                lgciu2: TestLgciu::new(false),
            };
            test_aircraft
                .a320_cabin_air
                .a320_pressurization_system
                .active_system = 1;
            test_aircraft
        }

        fn set_on_ground(&mut self, on_ground: bool) {
            self.lgciu1.set_on_ground(on_ground);
            self.lgciu2.set_on_ground(on_ground);
        }

        fn set_engine_n1(&mut self, n: Ratio) {
            self.engine_1.set_engine_n1(n);
            self.engine_2.set_engine_n1(n);
        }
    }
    impl Aircraft for TestAircraft {
        fn update_after_power_distribution(&mut self, context: &UpdateContext) {
            self.a320_cabin_air.update(
                context,
                &self.adirs,
                [&self.engine_1, &self.engine_2],
                &self.engine_fire_push_buttons,
                &self.pneumatic,
                &self.pneumatic_overhead,
                &self.pressurization_overhead,
                [&self.lgciu1, &self.lgciu2],
            );
        }
    }
    impl SimulationElement for TestAircraft {
        fn accept<V: SimulationElementVisitor>(&mut self, visitor: &mut V) {
            self.a320_cabin_air.accept(visitor);
            self.pressurization_overhead.accept(visitor);

            visitor.visit(self);
        }
    }

    struct CabinAirTestBed {
        test_bed: SimulationTestBed<TestAircraft>,
        stored_pressure: Option<Pressure>,
        stored_ofv_open_amount: Option<Ratio>,
        stored_vertical_speed: Option<Velocity>,
    }
    impl CabinAirTestBed {
        fn new() -> Self {
            let mut test_bed = CabinAirTestBed {
                test_bed: SimulationTestBed::new(TestAircraft::new),
                stored_pressure: None,
                stored_ofv_open_amount: None,
                stored_vertical_speed: None,
            };
            test_bed.set_indicated_altitude(Length::new::<foot>(0.));
            test_bed.set_ambient_temperature(ThermodynamicTemperature::new::<degree_celsius>(24.));
            test_bed.command_measured_temperature(
                [ThermodynamicTemperature::new::<degree_celsius>(24.); 2],
            );
            test_bed.command_pack_flow_selector_position(1);
            test_bed.command_engine_n1(Ratio::new::<percent>(30.));

            test_bed
        }

        fn on_ground(mut self) -> Self {
            self.set_ambient_pressure(Pressure::new::<hectopascal>(1013.25));
            self.set_indicated_airspeed(Velocity::new::<knot>(0.));
            self.set_indicated_altitude(Length::new::<foot>(0.));
            self.set_vertical_speed(Velocity::new::<foot_per_minute>(0.));
            self.command_on_ground(true);
            self.command_sea_level_pressure(Pressure::new::<hectopascal>(1013.25));
            self.command_destination_qnh(Pressure::new::<hectopascal>(1013.25));
            self.run();
            self
        }

        fn and(self) -> Self {
            self
        }

        fn then(self) -> Self {
            self
        }

        fn run_and(mut self) -> Self {
            self.run();
            self
        }

        fn run_with_delta_of(mut self, delta: Duration) -> Self {
            self.run_with_delta(delta);
            self
        }

        fn and_run(mut self) -> Self {
            self.run();
            self
        }

        fn with(self) -> Self {
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

        fn memorize_cabin_pressure(mut self) -> Self {
            self.stored_pressure = Some(self.cabin_pressure());
            self
        }

        fn memorize_outflow_valve_open_amount(mut self) -> Self {
            self.stored_ofv_open_amount = Some(self.outflow_valve_open_amount());
            self
        }

        fn memorize_vertical_speed(mut self) -> Self {
            self.stored_vertical_speed = Some(self.cabin_vs());
            self
        }

        fn initial_pressure(&self) -> Pressure {
            self.stored_pressure.unwrap()
        }

        fn ambient_pressure_of(mut self, pressure: Pressure) -> Self {
            self.set_ambient_pressure(pressure);
            self
        }

        fn indicated_airspeed_of(mut self, velocity: Velocity) -> Self {
            self.set_indicated_airspeed(velocity);
            self
        }

        fn vertical_speed_of(mut self, velocity: Velocity) -> Self {
            self.set_vertical_speed(velocity);
            self
        }

        fn set_on_ground(mut self) -> Self {
            self.command_on_ground(true);
            self
        }

        fn set_takeoff_power(mut self) -> Self {
            self.command_engine_n1(Ratio::new::<percent>(95.));
            self
        }

        fn command_measured_temperature(&mut self, temp_array: [ThermodynamicTemperature; 2]) {
            for (temp, id) in temp_array.iter().zip(["CKPT", "FWD"].iter()) {
                let zone_measured_temp_id = format!("COND_{}_TEMP", &id);
                self.write_by_name(&zone_measured_temp_id, temp.get::<degree_celsius>());
            }
        }

        fn command_pack_flow_selector_position(&mut self, value: u8) {
            self.write_by_name("KNOB_OVHD_AIRCOND_PACKFLOW_Position", value);
        }

        fn command_sea_level_pressure(&mut self, value: Pressure) {
            self.write_by_name("SEA LEVEL PRESSURE", value.get::<hectopascal>());
        }

        fn command_destination_qnh(&mut self, value: Pressure) {
            self.write_by_name("DESTINATION_QNH", value);
        }

        fn command_ditching_pb_on(mut self) -> Self {
            self.write_by_name("OVHD_PRESS_DITCHING_PB_IS_ON", true);
            self
        }

        fn command_mode_sel_pb_auto(mut self) -> Self {
            self.write_by_name("OVHD_PRESS_MODE_SEL_PB_IS_AUTO", true);
            self
        }

        fn command_mode_sel_pb_man(mut self) -> Self {
            self.write_by_name("OVHD_PRESS_MODE_SEL_PB_IS_AUTO", false);
            self
        }

        fn command_ldg_elev_knob_value(mut self, value: f64) -> Self {
            self.write_by_name("OVHD_PRESS_LDG_ELEV_KNOB", value);
            self
        }

        fn command_packs_on_off(mut self, on_off: bool) -> Self {
            self.write_by_name("OVHD_COND_PACK_1_PB_IS_ON", on_off);
            self.write_by_name("OVHD_COND_PACK_2_PB_IS_ON", on_off);
            self
        }

        fn command_man_vs_switch_position(mut self, position: usize) -> Self {
            if position == 0 {
                self.write_by_name("OVHD_PRESS_MAN_VS_CTL_SWITCH", 0);
            } else if position == 2 {
                self.write_by_name("OVHD_PRESS_MAN_VS_CTL_SWITCH", 2);
            } else {
                self.write_by_name("OVHD_PRESS_MAN_VS_CTL_SWITCH", 1);
            }
            self
        }

        fn command_on_ground(&mut self, on_ground: bool) {
            self.command(|a| a.set_on_ground(on_ground));
        }

        fn command_engine_n1(&mut self, n1: Ratio) {
            self.command(|a| a.set_engine_n1(n1));
        }

        fn command_aircraft_climb(mut self, init_altitude: Length, final_altitude: Length) -> Self {
            const KPA_FT: f64 = 0.0205; //KPa/ft ASL
            const PRESSURE_CONSTANT: f64 = 911.47;

            self.set_vertical_speed(Velocity::new::<foot_per_minute>(1000.));
            self.set_ambient_pressure(Pressure::new::<hectopascal>(
                PRESSURE_CONSTANT - init_altitude.get::<foot>() * (KPA_FT),
            ));

            for i in ((init_altitude.get::<foot>() / 1000.) as u32)
                ..((final_altitude.get::<foot>() / 1000.) as u32)
            {
                self.set_ambient_pressure(Pressure::new::<hectopascal>(
                    PRESSURE_CONSTANT - (((i * 1000) as f64) * (KPA_FT)),
                ));
                for _ in 1..10 {
                    self.run();
                    self.run();
                    self.run();
                    self.run();
                    self.run();
                }
            }
            self.set_ambient_pressure(Pressure::new::<hectopascal>(
                PRESSURE_CONSTANT - final_altitude.get::<foot>() * (KPA_FT),
            ));
            self.set_indicated_altitude(final_altitude);
            self.run();
            self
        }

        fn initial_outflow_valve_open_amount(&self) -> Ratio {
            self.stored_ofv_open_amount.unwrap()
        }

        fn initial_cabin_vs(&self) -> Velocity {
            self.stored_vertical_speed.unwrap()
        }

        fn cabin_altitude(&self) -> Length {
            self.query(|a| a.a320_cabin_air.a320_pressurization_system.cpc[0].cabin_altitude())
        }

        fn cabin_pressure(&self) -> Pressure {
            self.query(|a| {
                a.a320_cabin_air
                    .a320_pressurization_system
                    .cabin_pressure_simulation
                    .cabin_pressure()
            })
        }

        fn cabin_vs(&self) -> Velocity {
            self.query(|a| {
                a.a320_cabin_air
                    .a320_pressurization_system
                    .cabin_pressure_simulation
                    .cabin_vs()
            })
        }

        fn cabin_delta_p(&self) -> Pressure {
            self.query(|a| {
                a.a320_cabin_air
                    .a320_pressurization_system
                    .cabin_pressure_simulation
                    .cabin_delta_p()
            })
        }

        fn active_system(&self) -> usize {
            self.query(|a| a.a320_cabin_air.a320_pressurization_system.active_system)
        }

        fn outflow_valve_open_amount(&self) -> Ratio {
            self.query(|a| {
                a.a320_cabin_air.a320_pressurization_system.outflow_valve[0].open_amount()
            })
        }

        fn safety_valve_open_amount(&self) -> Ratio {
            self.query(|a| {
                a.a320_cabin_air
                    .a320_pressurization_system
                    .safety_valve
                    .open_amount()
            })
        }

        fn landing_elevation(&self) -> Length {
            self.query(|a| a.a320_cabin_air.a320_pressurization_system.cpc[0].landing_elevation())
        }

        fn is_mode_sel_pb_auto(&mut self) -> bool {
            self.read_by_name("OVHD_PRESS_MODE_SEL_PB_IS_AUTO")
        }
    }
    impl TestBed for CabinAirTestBed {
        type Aircraft = TestAircraft;

        fn test_bed(&self) -> &SimulationTestBed<TestAircraft> {
            &self.test_bed
        }

        fn test_bed_mut(&mut self) -> &mut SimulationTestBed<TestAircraft> {
            &mut self.test_bed
        }
    }

    fn test_bed() -> CabinAirTestBed {
        CabinAirTestBed::new()
    }

    fn test_bed_in_cruise() -> CabinAirTestBed {
        let mut test_bed =
            test_bed().command_aircraft_climb(Length::new::<foot>(0.), Length::new::<foot>(30000.));
        test_bed.set_indicated_altitude(Length::new::<foot>(30000.));
        test_bed.set_ambient_pressure(Pressure::new::<hectopascal>(300.));
        test_bed.set_vertical_speed(Velocity::new::<foot_per_minute>(99.));
        test_bed.run_with_delta(Duration::from_secs(31));
        test_bed
    }

    fn test_bed_in_descent() -> CabinAirTestBed {
        let mut test_bed = test_bed_in_cruise();
        test_bed.set_vertical_speed(Velocity::new::<foot_per_minute>(-260.));
        test_bed.run_with_delta(Duration::from_secs(31));
        test_bed
    }

    #[test]
    fn conversion_from_pressure_to_altitude_works() {
        let test_bed = test_bed()
            .on_ground()
            .run_and()
            .command_packs_on_off(false)
            .ambient_pressure_of(Pressure::new::<hectopascal>(696.86)) // Equivalent to 10,000ft from tables
            .iterate(200);

        assert!(
            (test_bed.cabin_altitude() - Length::new::<foot>(10000.)).abs()
                < Length::new::<foot>(100.)
        );
    }

    #[test]
    fn positive_cabin_vs_reduces_cabin_pressure() {
        let test_bed = test_bed()
            .run_and()
            .memorize_cabin_pressure()
            .then()
            .command_aircraft_climb(Length::new::<foot>(0.), Length::new::<foot>(10000.));

        assert!(test_bed.initial_pressure() > test_bed.cabin_pressure());
    }

    #[test]
    fn seventy_seconds_after_landing_cpc_switches() {
        let mut test_bed = test_bed()
            .run_with_delta_of(Duration::from_secs_f64(31.))
            .iterate(5);

        assert_eq!(test_bed.active_system(), 1);

        test_bed = test_bed
            .vertical_speed_of(Velocity::new::<foot_per_minute>(-260.))
            .run_with_delta_of(Duration::from_secs_f64(31.))
            .and_run()
            // Descent
            .indicated_airspeed_of(Velocity::new::<knot>(99.))
            .set_on_ground()
            .ambient_pressure_of(Pressure::new::<hectopascal>(1013.))
            .and_run()
            // Ground
            .run_with_delta_of(Duration::from_secs_f64(69.))
            .and_run();

        assert_eq!(test_bed.active_system(), 1);

        test_bed = test_bed
            .run_with_delta_of(Duration::from_secs_f64(1.))
            .and_run();

        assert_eq!(test_bed.active_system(), 2);

        test_bed = test_bed
            .run_with_delta_of(Duration::from_secs_f64(10.))
            .and_run();

        assert_eq!(test_bed.active_system(), 2);
    }

    #[test]
    fn fifty_five_seconds_after_landing_outflow_valve_opens() {
        let mut test_bed = test_bed()
            .run_with_delta_of(Duration::from_secs_f64(31.))
            .iterate(5);

        assert!(test_bed.outflow_valve_open_amount() > Ratio::new::<percent>(0.));
        assert!(test_bed.outflow_valve_open_amount() < Ratio::new::<percent>(99.));

        test_bed = test_bed
            .vertical_speed_of(Velocity::new::<foot_per_minute>(-260.))
            .run_with_delta_of(Duration::from_secs_f64(31.))
            .and_run()
            // Descent
            .indicated_airspeed_of(Velocity::new::<knot>(99.))
            .set_on_ground()
            .ambient_pressure_of(Pressure::new::<hectopascal>(1013.))
            .and_run()
            // Ground
            .run_with_delta_of(Duration::from_secs_f64(53.))
            .and_run();

        assert!(test_bed.outflow_valve_open_amount() < Ratio::new::<percent>(99.));

        test_bed = test_bed
            .run_with_delta_of(Duration::from_secs_f64(3.))
            .and_run();

        assert!(test_bed.outflow_valve_open_amount() > Ratio::new::<percent>(99.));

        test_bed = test_bed
            .run_with_delta_of(Duration::from_secs_f64(10.))
            .and_run();

        assert!(test_bed.outflow_valve_open_amount() > Ratio::new::<percent>(99.));
    }

    #[test]
    fn outflow_valve_closes_when_ditching_pb_is_on() {
        let mut test_bed = test_bed()
            .iterate_with_delta(1, Duration::from_secs_f64(20.))
            .iterate(5);

        assert!(test_bed.outflow_valve_open_amount() < Ratio::new::<percent>(90.));
        assert!(test_bed.outflow_valve_open_amount() > Ratio::new::<percent>(1.));

        test_bed = test_bed
            .command_ditching_pb_on()
            .run_with_delta_of(Duration::from_secs_f64(10.))
            .and_run();

        assert!(test_bed.outflow_valve_open_amount() < Ratio::new::<percent>(1.));
    }

    #[test]
    fn fifty_five_seconds_after_landing_outflow_valve_doesnt_open_if_ditching_pb_is_on() {
        let mut test_bed = test_bed()
            .run_with_delta_of(Duration::from_secs_f64(31.))
            .iterate(5);

        assert!(test_bed.outflow_valve_open_amount() > Ratio::new::<percent>(0.));
        assert!(test_bed.outflow_valve_open_amount() < Ratio::new::<percent>(99.));

        test_bed = test_bed
            .vertical_speed_of(Velocity::new::<foot_per_minute>(-260.))
            .run_with_delta_of(Duration::from_secs_f64(31.))
            .and_run()
            // Descent
            .indicated_airspeed_of(Velocity::new::<knot>(99.))
            .set_on_ground()
            .ambient_pressure_of(Pressure::new::<hectopascal>(1013.))
            .and_run()
            // Ground
            .run_with_delta_of(Duration::from_secs_f64(53.))
            .and_run();

        assert!(test_bed.outflow_valve_open_amount() < Ratio::new::<percent>(99.));

        test_bed = test_bed
            .command_ditching_pb_on()
            .run_with_delta_of(Duration::from_secs_f64(10.))
            .and_run();

        assert!(!(test_bed.outflow_valve_open_amount() > Ratio::new::<percent>(99.)));
        assert!(test_bed.outflow_valve_open_amount() < Ratio::new::<percent>(1.));
    }

    #[test]
    fn fifty_five_seconds_after_landing_outflow_valve_doesnt_open_if_mode_sel_man() {
        let mut test_bed = test_bed()
            .run_with_delta_of(Duration::from_secs_f64(31.))
            .iterate(5);

        assert!(test_bed.outflow_valve_open_amount() > Ratio::new::<percent>(0.));
        assert!(test_bed.outflow_valve_open_amount() < Ratio::new::<percent>(99.));

        test_bed = test_bed
            .memorize_outflow_valve_open_amount()
            .command_mode_sel_pb_man()
            .vertical_speed_of(Velocity::new::<foot_per_minute>(-260.))
            .run_with_delta_of(Duration::from_secs_f64(31.))
            .and_run()
            // Descent
            .indicated_airspeed_of(Velocity::new::<knot>(99.))
            .set_on_ground()
            .ambient_pressure_of(Pressure::new::<hectopascal>(1013.))
            .and_run()
            // Ground
            .run_with_delta_of(Duration::from_secs_f64(53.))
            .and_run()
            .run_with_delta_of(Duration::from_secs_f64(10.))
            .and_run();

        assert_eq!(
            test_bed.outflow_valve_open_amount(),
            test_bed.initial_outflow_valve_open_amount()
        );
    }

    #[test]
    fn rpcu_opens_ofv_if_mode_sel_man() {
        let mut test_bed = test_bed()
            .run_with_delta_of(Duration::from_secs_f64(31.))
            .iterate(5);

        assert!(test_bed.outflow_valve_open_amount() > Ratio::new::<percent>(0.));
        assert!(test_bed.outflow_valve_open_amount() < Ratio::new::<percent>(99.));

        test_bed = test_bed
            .command_mode_sel_pb_man()
            .vertical_speed_of(Velocity::new::<foot_per_minute>(-260.))
            .run_with_delta_of(Duration::from_secs_f64(31.))
            .and_run()
            // Descent
            .indicated_airspeed_of(Velocity::new::<knot>(69.))
            .set_on_ground()
            .ambient_pressure_of(Pressure::new::<hectopascal>(1013.))
            .and_run()
            .run_with_delta_of(Duration::from_secs_f64(4.))
            .and_run()
            // Ground
            .iterate(200);

        assert_eq!(
            test_bed.outflow_valve_open_amount(),
            Ratio::new::<percent>(100.)
        );
    }

    #[test]
    fn cpc_man_mode_starts_in_auto() {
        let mut test_bed = test_bed();

        assert!(test_bed.is_mode_sel_pb_auto());
    }

    #[test]
    fn cpc_switches_if_man_mode_is_engaged_for_at_least_10_seconds() {
        let mut test_bed = test_bed();

        assert_eq!(test_bed.active_system(), 1);

        test_bed = test_bed
            .command_mode_sel_pb_man()
            .run_and()
            .run_with_delta_of(Duration::from_secs_f64(11.))
            .command_mode_sel_pb_auto()
            .iterate(2);

        assert_eq!(test_bed.active_system(), 2);
    }

    #[test]
    fn cpc_does_not_switch_if_man_mode_is_engaged_for_less_than_10_seconds() {
        let mut test_bed = test_bed();

        assert_eq!(test_bed.active_system(), 1);

        test_bed = test_bed
            .command_mode_sel_pb_man()
            .run_and()
            .run_with_delta_of(Duration::from_secs_f64(9.))
            .command_mode_sel_pb_auto()
            .iterate(2);

        assert_eq!(test_bed.active_system(), 1);
    }

    #[test]
    fn cpc_switching_timer_resets() {
        let mut test_bed = test_bed();

        assert_eq!(test_bed.active_system(), 1);

        test_bed = test_bed
            .command_mode_sel_pb_man()
            .run_and()
            .run_with_delta_of(Duration::from_secs_f64(9.))
            .command_mode_sel_pb_auto()
            .iterate(2);
        assert_eq!(test_bed.active_system(), 1);

        test_bed = test_bed
            .command_mode_sel_pb_man()
            .run_and()
            .run_with_delta_of(Duration::from_secs_f64(9.))
            .command_mode_sel_pb_auto()
            .iterate(2);
        assert_eq!(test_bed.active_system(), 1);
    }

    #[test]
    fn cpc_targets_manual_landing_elev_if_knob_not_in_initial_position() {
        let mut test_bed = test_bed();

        assert_eq!(test_bed.landing_elevation(), Length::new::<foot>(0.));

        test_bed = test_bed.command_ldg_elev_knob_value(1000.).and_run();

        assert_eq!(test_bed.landing_elevation(), Length::new::<foot>(1000.));
    }

    #[test]
    fn cpc_targets_auto_landing_elev_if_knob_returns_to_initial_position() {
        let mut test_bed = test_bed();

        assert_eq!(test_bed.landing_elevation(), Length::new::<foot>(0.));

        test_bed = test_bed.command_ldg_elev_knob_value(1000.).and_run();

        assert_eq!(test_bed.landing_elevation(), Length::new::<foot>(1000.));

        test_bed = test_bed.command_ldg_elev_knob_value(-2000.).and_run();

        assert_eq!(test_bed.landing_elevation(), Length::new::<foot>(0.));
    }

    #[test]
    fn aircraft_vs_starts_at_0() {
        let test_bed = test_bed().on_ground().iterate(20);

        assert!((test_bed.cabin_vs()).abs() < Velocity::new::<foot_per_minute>(1.));
    }

    #[test]
    fn outflow_valve_stays_open_on_ground() {
        let mut test_bed = test_bed().on_ground();

        assert_eq!(
            test_bed.outflow_valve_open_amount(),
            Ratio::new::<percent>(100.)
        );

        test_bed = test_bed.iterate(10);

        assert_eq!(
            test_bed.outflow_valve_open_amount(),
            Ratio::new::<percent>(100.)
        );
    }

    #[test]
    fn cabin_vs_changes_to_takeoff() {
        let test_bed = test_bed()
            .on_ground()
            .iterate(20)
            .set_takeoff_power()
            .iterate_with_delta(100, Duration::from_millis(10));

        assert!(
            (test_bed.cabin_vs() - Velocity::new::<foot_per_minute>(-400.)).abs()
                < Velocity::new::<foot_per_minute>(3.)
        );
    }

    #[test]
    fn cabin_delta_p_does_not_exceed_0_1_during_takeoff() {
        let test_bed = test_bed()
            .on_ground()
            .iterate(20)
            .set_takeoff_power()
            .iterate(20);

        assert!(
            (test_bed.cabin_delta_p() - Pressure::new::<psi>(0.1)).abs()
                < Pressure::new::<psi>(0.01)
        );
        assert!(test_bed.cabin_vs() < Velocity::new::<foot_per_minute>(10.));
    }

    #[test]
    fn cabin_vs_changes_to_climb() {
        let test_bed = test_bed()
            .vertical_speed_of(Velocity::new::<foot_per_minute>(1000.))
            .ambient_pressure_of(Pressure::new::<hectopascal>(900.))
            .iterate(10);

        assert!(test_bed.cabin_vs() > Velocity::new::<foot_per_minute>(0.));
    }

    #[test]
    fn cabin_vs_increases_with_altitude() {
        let test_bed = test_bed()
            .iterate(10)
            .with()
            .vertical_speed_of(Velocity::new::<foot_per_minute>(1000.))
            .then()
            .command_aircraft_climb(Length::new::<foot>(0.), Length::new::<foot>(10000.))
            .with()
            .ambient_pressure_of(Pressure::new::<hectopascal>(696.85))
            .iterate(10)
            .memorize_vertical_speed()
            .then()
            .command_aircraft_climb(Length::new::<foot>(10000.), Length::new::<foot>(30000.))
            .and()
            .ambient_pressure_of(Pressure::new::<hectopascal>(300.92))
            .iterate(10);

        assert!(test_bed.cabin_vs() > test_bed.initial_cabin_vs());
    }

    #[test]
    fn cabin_vs_changes_to_cruise() {
        let test_bed = test_bed_in_cruise().iterate(10);

        assert!(test_bed.cabin_vs().abs() < Velocity::new::<foot_per_minute>(10.));
    }

    #[test]
    fn cabin_vs_changes_to_descent() {
        let test_bed = test_bed_in_cruise()
            .vertical_speed_of(Velocity::new::<foot_per_minute>(-260.))
            .run_with_delta_of(Duration::from_secs(31))
            .iterate(10);

        assert!(test_bed.cabin_vs() > Velocity::new::<foot_per_minute>(-750.));
        assert!(test_bed.cabin_vs() < Velocity::new::<foot_per_minute>(0.));
    }

    #[test]
    fn cabin_vs_changes_to_ground() {
        let test_bed = test_bed_in_descent()
            .vertical_speed_of(Velocity::new::<knot>(99.))
            .then()
            .on_ground()
            .iterate(10);

        assert!(
            (test_bed.cabin_vs() - Velocity::new::<foot_per_minute>(500.))
                < Velocity::new::<foot_per_minute>(10.)
        );
    }

    #[test]
    fn cabin_delta_p_does_not_exceed_8_06_psi_in_climb() {
        let test_bed = test_bed()
            .and_run()
            .with()
            .vertical_speed_of(Velocity::new::<foot_per_minute>(1000.))
            .iterate(5)
            .then()
            .command_aircraft_climb(Length::new::<foot>(1000.), Length::new::<foot>(39000.))
            .with()
            .ambient_pressure_of(Pressure::new::<hectopascal>(196.41))
            .vertical_speed_of(Velocity::new::<foot_per_minute>(0.))
            .iterate(10);

        assert!(test_bed.cabin_delta_p() < Pressure::new::<psi>(8.06));
    }

    #[test]
    fn outflow_valve_closes_to_compensate_packs_off() {
        let test_bed = test_bed()
            .iterate(10)
            .memorize_outflow_valve_open_amount()
            .then()
            .command_packs_on_off(false)
            .iterate(10);

        assert!(
            (test_bed.initial_outflow_valve_open_amount() - test_bed.outflow_valve_open_amount())
                > Ratio::new::<percent>(5.)
        );
    }

    #[test]
    fn outflow_valve_does_not_move_when_man_mode_engaged() {
        let test_bed = test_bed()
            .iterate(10)
            .command_mode_sel_pb_man()
            .and_run()
            .memorize_outflow_valve_open_amount()
            .then()
            .command_aircraft_climb(Length::new::<foot>(7000.), Length::new::<foot>(14000.))
            .iterate(10);

        assert!(
            (test_bed.outflow_valve_open_amount() - test_bed.initial_outflow_valve_open_amount())
                .abs()
                < Ratio::new::<percent>(1.)
        );
    }

    #[test]
    fn outflow_valve_responds_to_man_inputs_when_in_man_mode() {
        let test_bed = test_bed()
            .iterate(10)
            .command_mode_sel_pb_man()
            .and_run()
            .memorize_outflow_valve_open_amount()
            .command_man_vs_switch_position(0)
            .iterate(10);

        assert!(
            test_bed.outflow_valve_open_amount() > test_bed.initial_outflow_valve_open_amount()
        );
    }

    #[test]
    fn outflow_valve_position_affects_cabin_vs_when_in_man_mode() {
        let test_bed = test_bed()
            .with()
            .ambient_pressure_of(Pressure::new::<hectopascal>(600.))
            .iterate(10)
            .command_mode_sel_pb_man()
            .and_run()
            .memorize_vertical_speed()
            .then()
            .command_man_vs_switch_position(0)
            .iterate(10);

        assert!(test_bed.cabin_vs() > test_bed.initial_cabin_vs());
    }

    #[test]
    fn pressure_builds_up_when_ofv_closed_and_packs_on() {
        let test_bed = test_bed()
            .iterate(10)
            .memorize_cabin_pressure()
            .command_mode_sel_pb_man()
            .command_man_vs_switch_position(2)
            .iterate(100)
            .command_packs_on_off(true)
            .iterate(10);

        assert!(test_bed.cabin_pressure() > test_bed.initial_pressure());
    }

    #[test]
    fn pressure_decreases_when_ofv_closed_and_packs_off() {
        let test_bed = test_bed()
            .with()
            .ambient_pressure_of(Pressure::new::<hectopascal>(465.67))
            .iterate(40)
            .memorize_cabin_pressure()
            .then()
            .command_ditching_pb_on()
            .command_packs_on_off(false)
            .iterate(10);

        assert!(test_bed.cabin_pressure() < test_bed.initial_pressure());
    }

    #[test]
    fn pressure_is_constant_when_ofv_closed_and_packs_off_with_no_delta_p() {
        let test_bed = test_bed()
            .with()
            .ambient_pressure_of(test_bed().cabin_pressure())
            .iterate(40)
            .memorize_cabin_pressure()
            .command_ditching_pb_on()
            .command_packs_on_off(false)
            .iterate(100);

        assert!(
            (test_bed.cabin_pressure() - test_bed.initial_pressure()) < Pressure::new::<psi>(0.1)
        );
    }

    #[test]
    fn pressure_never_goes_below_ambient_when_ofv_opens() {
        let test_bed = test_bed()
            .with()
            .vertical_speed_of(Velocity::new::<foot_per_minute>(1000.))
            .command_aircraft_climb(Length::new::<foot>(0.), Length::new::<foot>(20000.))
            .then()
            .ambient_pressure_of(Pressure::new::<hectopascal>(465.63))
            .vertical_speed_of(Velocity::new::<foot_per_minute>(0.))
            .iterate(10)
            .command_mode_sel_pb_man()
            .command_packs_on_off(false)
            .and_run()
            .command_man_vs_switch_position(0)
            .iterate(500);

        assert!(
            (test_bed.cabin_pressure() - Pressure::new::<hectopascal>(465.63)).abs()
                < Pressure::new::<hectopascal>(10.)
        );
    }

    #[test]
    fn safety_valve_stays_closed_when_delta_p_is_less_than_8_6_psi() {
        let test_bed = test_bed()
            // Equivalent to SL - 8.6 PSI
            .ambient_pressure_of(Pressure::new::<hectopascal>(421.))
            .and_run();

        assert_eq!(
            test_bed.safety_valve_open_amount(),
            Ratio::new::<percent>(0.)
        );
    }

    #[test]
    fn safety_valve_stays_closed_when_delta_p_is_less_than_minus_1_psi() {
        let test_bed = test_bed()
            // Equivalent to SL + 1 PSI
            .ambient_pressure_of(Pressure::new::<hectopascal>(1080.))
            .and_run();

        assert_eq!(
            test_bed.safety_valve_open_amount(),
            Ratio::new::<percent>(0.)
        );
    }

    #[test]
    fn safety_valve_opens_when_delta_p_above_8_6_psi() {
        let test_bed = test_bed()
            .command_mode_sel_pb_man()
            .and_run()
            .command_man_vs_switch_position(2)
            .command_packs_on_off(false)
            .iterate(100)
            // Equivalent to SL - 10 PSI
            .ambient_pressure_of(Pressure::new::<hectopascal>(323.))
            .iterate(20);

        assert!(test_bed.safety_valve_open_amount() > Ratio::new::<percent>(0.));
    }

    #[test]
    fn safety_valve_opens_when_delta_p_below_minus_1_psi() {
        let test_bed = test_bed()
            .command_mode_sel_pb_man()
            .and_run()
            .command_man_vs_switch_position(2)
            .command_packs_on_off(false)
            .iterate(100)
            // Equivalent to SL + 2 PSI
            .ambient_pressure_of(Pressure::new::<hectopascal>(1400.))
            .iterate(20);

        assert!(test_bed.safety_valve_open_amount() > Ratio::new::<percent>(0.));
    }

    #[test]
    fn safety_valve_closes_when_condition_is_not_met() {
        let mut test_bed = test_bed()
            .command_mode_sel_pb_man()
            .and_run()
            .command_man_vs_switch_position(2)
            .command_packs_on_off(false)
            .iterate(100)
            // Equivalent to SL + 2 PSI
            .ambient_pressure_of(Pressure::new::<hectopascal>(1400.))
            .iterate(20);

        assert!(test_bed.safety_valve_open_amount() > Ratio::new::<percent>(0.));

        test_bed = test_bed
            .ambient_pressure_of(Pressure::new::<hectopascal>(1013.))
            .iterate(20);

        assert_eq!(
            test_bed.safety_valve_open_amount(),
            Ratio::new::<percent>(0.)
        );
    }
}
