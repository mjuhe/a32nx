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
    },
    shared::{
        random_number, Cabin, ControllerSignal, EngineBleedPushbutton, EngineCorrectedN1,
        EngineFirePushButtons, EngineStartState, GroundSpeed, LgciuWeightOnWheels, PneumaticBleed,
        PressurizationOverheadShared,
    },
    simulation::{
        InitContext, SimulationElement, SimulationElementVisitor, SimulatorWriter, UpdateContext,
        VariableIdentifier, Write,
    },
};

use std::time::Duration;
use uom::si::{
    f64::*, length::foot, mass_rate::kilogram_per_second, pressure::hectopascal, ratio::percent,
    velocity::knot, volume::cubic_meter,
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
        self.a320_pressurization_system
            .update(context, pressurization_overhead, engines, lgciu)
    }
}

impl SimulationElement for A320AirConditioning {
    fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T) {
        self.a320_cabin.accept(visitor);
        self.a320_air_conditioning_system.accept(visitor);

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
                CabinPressureController::new(context),
                CabinPressureController::new(context),
            ],
            outflow_valve: [PressureValve::new_outflow_valve(); 1],
            safety_valve: PressureValve::new_safety_valve(),
            residual_pressure_controller: ResidualPressureController::new(),
            active_system: active,

            cabin_pressure_simulation: CabinPressureSimulation::new(context),
        }
    }

    pub fn update(
        &mut self,
        context: &UpdateContext,
        press_overhead: &A320PressurizationOverheadPanel,
        engines: [&impl EngineCorrectedN1; 2],
        lgciu: [&impl LgciuWeightOnWheels; 2],
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
                &self.cabin_pressure_simulation, // replace with cabin data
                &self.outflow_valve[0],
                &self.safety_valve,
            );
        }

        self.residual_pressure_controller.update(
            context,
            engines,
            self.outflow_valve[0].open_amount(),
            press_overhead.is_in_man_mode(),
            lgciu_gears_compressed,
            self.cabin_pressure_simulation.cabin_delta_p(), // replace with cabin data
        );

        self.outflow_valve[0].calculate_outflow_valve_position(
            &self.cpc[self.active_system - 1],
            press_overhead,
            &self.cabin_pressure_simulation,
        );

        if self.residual_pressure_controller.signal().is_some() {
            self.outflow_valve[0].update(context, &self.residual_pressure_controller);
        } else {
            self.outflow_valve[0].update(context, press_overhead);
        }

        self.safety_valve
            .update(context, &self.cpc[self.active_system - 1]);

        // move to air con
        self.cabin_pressure_simulation.update(
            context,
            self.outflow_valve[0].open_amount(),
            self.safety_valve.open_amount(),
            true, //packs_are_on
            lgciu_gears_compressed,
            self.cpc[self.active_system - 1].should_open_outflow_valve()
                && !press_overhead.is_in_man_mode(),
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
        self.cpc.iter_mut().for_each(|controller| {
            controller.reset_cpc_switch();
        });
    }
}

impl Cabin for A320PressurizationSystem {
    fn altitude(&self) -> Length {
        // self.cpc[self.active_system - 1].cabin_altitude()
        Length::new::<foot>(0.)
    }

    fn pressure(&self) -> Pressure {
        // self.cabin_pressure_simulation.cabin_pressure()
        Pressure::new::<hectopascal>(1013.)
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
