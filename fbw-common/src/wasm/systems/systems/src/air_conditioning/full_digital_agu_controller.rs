use crate::{
    pneumatic::{EngineState, PneumaticValveSignal},
    shared::{
        pid::PidController, ControllerSignal, ElectricalBusType, EngineBleedPushbutton,
        EngineCorrectedN1, EngineFirePushButtons, EngineStartState, PackFlowValveState,
        PneumaticBleed,
    },
    simulation::{SimulationElement, SimulationElementVisitor, UpdateContext},
};

use super::{
    acs_controller::Pack, AirConditioningOverheadShared, OperatingChannel, PackFlow,
    PackFlowControllers, PackFlowValveSignal, PressurizationOverheadShared,
};

use uom::si::{
    f64::*,
    mass_rate::kilogram_per_second,
    ratio::{percent, ratio},
};

#[derive(Debug)]
enum FdacFault {
    OneChannelFault,
    BothChannelsFault,
}

#[derive(Clone, Copy)]
enum FcvFault {
    PositionDisagree,
    //More to be added
}

pub struct FullDigitalAGUController<const ENGINES: usize> {
    active_channel: OperatingChannel,
    stand_by_channel: OperatingChannel,
    flow_control: FDACFlowControl<ENGINES>,
    // agu_control
    fault: Option<FdacFault>,
}

impl<const ENGINES: usize> FullDigitalAGUController<ENGINES> {
    pub fn new(fdac_id: usize, powered_by: Vec<ElectricalBusType>) -> Self {
        Self {
            active_channel: OperatingChannel::new(1, powered_by[0]),
            stand_by_channel: OperatingChannel::new(2, powered_by[1]),
            flow_control: FDACFlowControl::new(fdac_id),
            // agu_control
            fault: None,
        }
    }

    pub fn update(
        &mut self,
        context: &UpdateContext,
        acs_overhead: &impl AirConditioningOverheadShared,
        any_door_open: bool,
        engine_fire_push_buttons: &impl EngineFirePushButtons,
        engines: [&impl EngineCorrectedN1; ENGINES],
        pack_flow_demand: &impl PackFlow,
        pneumatic: &(impl EngineStartState + PackFlowValveState + PneumaticBleed),
        pneumatic_overhead: &impl EngineBleedPushbutton<ENGINES>,
        pressurization_overhead: &impl PressurizationOverheadShared,
    ) {
        self.fault_determination();

        if !matches!(self.fault, Some(FdacFault::BothChannelsFault))
            && !self.active_channel.has_fault()
        {
            self.flow_control.update(
                context,
                acs_overhead,
                any_door_open,
                engine_fire_push_buttons,
                engines,
                pack_flow_demand,
                pneumatic,
                pneumatic_overhead,
                pressurization_overhead,
            )
        }
    }

    fn fault_determination(&mut self) {
        self.fault = match self.active_channel.has_fault() {
            true => {
                if self.stand_by_channel.has_fault() {
                    Some(FdacFault::BothChannelsFault)
                } else {
                    self.switch_active_channel();
                    Some(FdacFault::OneChannelFault)
                }
            }
            false => {
                if self.stand_by_channel.has_fault() {
                    Some(FdacFault::OneChannelFault)
                } else {
                    None
                }
            }
        };
    }

    fn switch_active_channel(&mut self) {
        std::mem::swap(&mut self.stand_by_channel, &mut self.active_channel);
    }

    pub fn fcv_status_determination(&self, fcv_id: usize) -> bool {
        self.flow_control.fcv_has_fault(fcv_id)
    }

    pub fn individual_pack_flow(&self, pack_id: Pack) -> MassRate {
        self.flow_control.individual_pack_flow(pack_id)
    }
}

impl<const ENGINES: usize> PackFlow for FullDigitalAGUController<ENGINES> {
    fn pack_flow(&self) -> MassRate {
        self.flow_control.pack_flow()
    }
}

impl<const ENGINES: usize> PackFlowControllers for FullDigitalAGUController<ENGINES> {
    type PackFlowControllerSignal = PackFlowController<ENGINES>;

    fn pack_flow_controller(&self, fcv_id: usize) -> &Self::PackFlowControllerSignal {
        self.flow_control.pack_flow_controller(fcv_id)
    }
}

impl<const ENGINES: usize> SimulationElement for FullDigitalAGUController<ENGINES> {
    fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T) {
        self.active_channel.accept(visitor);
        self.stand_by_channel.accept(visitor);

        visitor.visit(self);
    }
}

/// Each FDAC controls two FCV, FDAC 1 controls the left side (engine 1 & 2)
/// and FDAC 2 controls the right side (engine 3 & 4)
struct FDACFlowControl<const ENGINES: usize> {
    fdac_id: usize, // 1 or 2
    flow_control_valves_controller: [PackFlowController<ENGINES>; 2],
    fcv_open_allowed: [bool; 2],
    should_open_fcv: [bool; 2],
}

impl<const ENGINES: usize> FDACFlowControl<ENGINES> {
    fn new(fdac_id: usize) -> Self {
        Self {
            fdac_id,
            flow_control_valves_controller: [PackFlowController::new(); 2],
            fcv_open_allowed: [false; 2],
            should_open_fcv: [false; 2],
        }
    }

    fn update(
        &mut self,
        context: &UpdateContext,
        acs_overhead: &impl AirConditioningOverheadShared,
        any_door_open: bool,
        engine_fire_push_buttons: &impl EngineFirePushButtons,
        engines: [&impl EngineCorrectedN1; ENGINES],
        pack_flow_demand: &impl PackFlow,
        pneumatic: &(impl EngineStartState + PackFlowValveState + PneumaticBleed),
        pneumatic_overhead: &impl EngineBleedPushbutton<ENGINES>,
        pressurization_overhead: &impl PressurizationOverheadShared,
    ) {
        let fcv_id = [1, 2].map(|i| i + ((self.fdac_id == 2) as usize * 2));

        self.fcv_open_allowed = fcv_id.map(|id| {
            self.fcv_open_allowed_determination(
                acs_overhead,
                any_door_open,
                engine_fire_push_buttons,
                id,
                pressurization_overhead,
                pneumatic,
            )
        });

        self.should_open_fcv = fcv_id
            .iter()
            .zip(self.fcv_open_allowed)
            .map(|(&fcv, fcv_open_allowed)| {
                fcv_open_allowed && self.can_move_fcv(engines, fcv, pneumatic, pneumatic_overhead)
            })
            .collect::<Vec<_>>()
            .try_into()
            .unwrap_or_else(|v: Vec<_>| {
                panic!(
                    "Expected a Vec of length {} but it was {}",
                    self.should_open_fcv.len(),
                    v.len()
                )
            });

        // At the moment we split the demand of the pack between the two FCV equally
        // TODO: Add provision in case one of the valves fails
        self.flow_control_valves_controller
            .iter_mut()
            .enumerate()
            .for_each(|(id, controller)| {
                controller.update(
                    context,
                    fcv_id[id],
                    self.fcv_open_allowed[id],
                    self.should_open_fcv[id],
                    pack_flow_demand.pack_flow_demand(self.fdac_id.into()) / 2.,
                    pneumatic,
                )
            });
    }

    fn fcv_open_allowed_determination(
        &mut self,
        acs_overhead: &impl AirConditioningOverheadShared,
        any_door_open: bool,
        engine_fire_push_buttons: &impl EngineFirePushButtons,
        fcv_id: usize,
        pressurization_overhead: &impl PressurizationOverheadShared,
        pneumatic: &(impl PneumaticBleed + EngineStartState),
    ) -> bool {
        let onside_engine_numbers = [1, 2].map(|i| i + ((self.fdac_id == 2) as usize * 2));
        // The pack flow valve closes when:
        // The associated PACK pbs is set to off
        !(!acs_overhead.pack_pushbuttons_state()[self.fdac_id - 1]
            // The FIRE pb of the associated engine is pressed
            || engine_fire_push_buttons.is_released(fcv_id)
            // One onside engine starts, and the crossbleed valves are closed
            || onside_engine_numbers
                .iter()
                .any(|engine| pneumatic.engine_state(*engine) == EngineState::Starting)
                && !pneumatic.engine_crossbleed_is_on()
            // Any engine starts, and the crossbleed valves are opened
            || (1..=4).any(|engine| pneumatic.engine_state(engine) == EngineState::Starting)
                && pneumatic.engine_crossbleed_is_on()
            // The DITCHING pb-sw is set to ON
            || pressurization_overhead.ditching_is_on())
            // On ground, one door (or more) is opened, and at least one engine is running
            && !(any_door_open
                && (1..=4).any(|engine| pneumatic.engine_state(engine) == EngineState::On))
        // && ! pack overheat
        // && mixer unit burst
    }

    fn can_move_fcv(
        &self,
        engines: [&impl EngineCorrectedN1; ENGINES],
        fcv_id: usize,
        pneumatic: &(impl PneumaticBleed + EngineStartState),
        pneumatic_overhead: &impl EngineBleedPushbutton<ENGINES>,
    ) -> bool {
        ((engines[fcv_id - 1].corrected_n1() >= Ratio::new::<percent>(15.)
            && pneumatic_overhead.engine_bleed_pushbuttons_are_auto()[fcv_id - 1])
            || (engines
                .iter()
                .any(|e| e.corrected_n1() >= Ratio::new::<percent>(15.))
                && pneumatic_overhead
                    .engine_bleed_pushbuttons_are_auto()
                    .iter()
                    .any(|pb| *pb)
                && pneumatic.engine_crossbleed_is_on()))
            || pneumatic.apu_bleed_is_on()
    }

    pub fn fcv_has_fault(&self, fcv_id: usize) -> bool {
        self.flow_control_valves_controller[fcv_id - 1 - ((self.fdac_id == 2) as usize * 2)]
            .fault()
            .is_some()
    }

    pub fn individual_pack_flow(&self, pack_id: Pack) -> MassRate {
        self.flow_control_valves_controller[pack_id.to_index()].pack_flow()
    }
}

impl<const ENGINES: usize> PackFlow for FDACFlowControl<ENGINES> {
    fn pack_flow(&self) -> MassRate {
        self.flow_control_valves_controller[0].pack_flow()
            + self.flow_control_valves_controller[1].pack_flow()
    }
}

impl<const ENGINES: usize> PackFlowControllers for FDACFlowControl<ENGINES> {
    type PackFlowControllerSignal = PackFlowController<ENGINES>;

    fn pack_flow_controller(&self, fcv_id: usize) -> &Self::PackFlowControllerSignal {
        &self.flow_control_valves_controller[fcv_id - 1 - ((self.fdac_id == 2) as usize * 2)]
    }
}

#[derive(Copy, Clone)]
pub struct PackFlowController<const ENGINES: usize> {
    should_open_fcv: bool,
    pack_flow: MassRate,
    pid: PidController,
    fault: Option<FcvFault>,
}

impl<const ENGINES: usize> PackFlowController<ENGINES> {
    fn new() -> Self {
        Self {
            should_open_fcv: false,
            pack_flow: MassRate::default(),
            pid: PidController::new(0.01, 0.4, 0., 0., 1., 0., 1.),
            fault: None,
        }
    }

    fn update(
        &mut self,
        context: &UpdateContext,
        fcv_id: usize,
        fcv_open_allowed: bool,
        should_open_fcv: bool,
        pack_flow_demand: MassRate,
        pneumatic: &impl PackFlowValveState,
    ) {
        self.fault_determination(fcv_id, fcv_open_allowed, pneumatic);

        self.should_open_fcv = should_open_fcv;
        self.pack_flow = pneumatic.pack_flow_valve_air_flow(fcv_id);

        if self.should_open_fcv {
            self.pid
                .change_setpoint(pack_flow_demand.get::<kilogram_per_second>());
            self.pid.next_control_output(
                self.pack_flow.get::<kilogram_per_second>(),
                Some(context.delta()),
            );
        }
    }

    fn fault_determination(
        &mut self,
        fcv_id: usize,
        fcv_open_allowed: bool,
        pneumatic: &impl PackFlowValveState,
    ) {
        self.fault = if fcv_open_allowed != pneumatic.pack_flow_valve_is_open(fcv_id) {
            Some(FcvFault::PositionDisagree)
        } else {
            None
        }
    }

    fn fault(&self) -> Option<FcvFault> {
        self.fault
    }
}

impl<const ENGINES: usize> PackFlow for PackFlowController<ENGINES> {
    fn pack_flow(&self) -> MassRate {
        self.pack_flow
    }
}

impl<const ENGINES: usize> ControllerSignal<PackFlowValveSignal> for PackFlowController<ENGINES> {
    fn signal(&self) -> Option<PackFlowValveSignal> {
        let target_open = Ratio::new::<ratio>(if self.should_open_fcv {
            self.pid.output()
        } else {
            0.
        });
        Some(PackFlowValveSignal::new(target_open))
    }
}
