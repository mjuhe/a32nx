import { TurnDirection } from '@flybywiresim/fbw-sdk';
import { HoldType } from '@fmgc/flightplanning/data/flightplan';
import { SegmentClass } from '@fmgc/flightplanning/segments/SegmentClass';
import { FlightPlanIndex } from '@fmgc/index';
import { MfdFmsFpln } from 'instruments/src/MFD/pages/FMS/F-PLN/MfdFmsFpln';
import { ContextMenuElement } from 'instruments/src/MFD/pages/common/ContextMenu';

export enum FplnRevisionsMenuType {
  Waypoint,
  PseudoWaypoint,
  Discontinuity,
  Runway,
  Departure,
  Arrival,
  TooSteepPath,
}

export function getRevisionsMenu(fpln: MfdFmsFpln, type: FplnRevisionsMenuType): ContextMenuElement[] {
  const legIndex = fpln.props.fmcService.master?.revisedWaypointIndex.get();
  const planIndex = fpln.props.fmcService.master?.revisedWaypointPlanIndex.get();
  const altnFlightPlan = fpln.props.fmcService.master?.revisedWaypointIsAltn.get();

  if (legIndex == null || planIndex == null || altnFlightPlan == null) {
    return [];
  }

  return [
    {
      name: 'FROM P.POS DIR TO',
      disabled:
        altnFlightPlan ||
        legIndex >= (fpln.loadedFlightPlan?.firstMissedApproachLegIndex ?? Infinity) ||
        planIndex === FlightPlanIndex.Temporary ||
        [FplnRevisionsMenuType.Discontinuity || FplnRevisionsMenuType.TooSteepPath].includes(type) ||
        !fpln.loadedFlightPlan?.legElementAt(legIndex).isXF(),
      onPressed: () => {
        const ppos = fpln.props.fmcService.master?.navigation.getPpos();
        if (ppos) {
          fpln.props.fmcService.master?.flightPlanService.directToLeg(
            ppos,
            SimVar.GetSimVarValue('GPS GROUND TRUE TRACK', 'degree'),
            legIndex,
            true,
            planIndex,
          );
          fpln.props.mfd.uiService.navigateTo(
            `fms/${fpln.props.mfd.uiService.activeUri.get().category}/f-pln-direct-to`,
          );
        }
      },
    },
    {
      name: 'INSERT NEXT WPT',
      disabled: false, // always enabled?
      onPressed: () => fpln.openInsertNextWptFromWindow(),
    },
    {
      name: 'DELETE *',
      disabled:
        [FplnRevisionsMenuType.Runway || FplnRevisionsMenuType.TooSteepPath].includes(type) ||
        planIndex === FlightPlanIndex.Temporary,
      onPressed: () => {
        fpln.props.fmcService.master?.flightPlanService.deleteElementAt(legIndex, false, planIndex, altnFlightPlan);
      },
    },
    {
      name: 'DEPARTURE',
      disabled: type !== FplnRevisionsMenuType.Departure && type !== FplnRevisionsMenuType.Runway,
      onPressed: () =>
        fpln.props.mfd.uiService.navigateTo(`fms/${fpln.props.mfd.uiService.activeUri.get().category}/f-pln-departure`),
    },
    {
      name: 'ARRIVAL',
      disabled: type !== FplnRevisionsMenuType.Arrival && type !== FplnRevisionsMenuType.Runway,
      onPressed: () =>
        fpln.props.mfd.uiService.navigateTo(`fms/${fpln.props.mfd.uiService.activeUri.get().category}/f-pln-arrival`),
    },
    {
      name: '(N/A) OFFSET',
      disabled: true,
      onPressed: () =>
        fpln.props.mfd.uiService.navigateTo(`fms/${fpln.props.mfd.uiService.activeUri.get().category}/f-pln-offset`),
    },
    {
      name: 'HOLD',
      disabled: [FplnRevisionsMenuType.Discontinuity || FplnRevisionsMenuType.TooSteepPath].includes(type),
      onPressed: async () => {
        const waypoint = fpln.props.fmcService.master?.flightPlanService.active.legElementAt(legIndex);
        if (waypoint && !waypoint.isHX()) {
          const alt = waypoint.definition.altitude1
            ? waypoint.definition.altitude1
            : SimVar.GetSimVarValue('INDICATED ALTITUDE', 'feet');

          const previousLeg = fpln.props.fmcService.master?.flightPlanService.active.maybeElementAt(legIndex - 1);

          let inboundMagneticCourse = 100;
          const prevTerm = previousLeg?.isDiscontinuity === false && previousLeg?.terminationWaypoint();
          const wptTerm = waypoint.terminationWaypoint();
          if (previousLeg && previousLeg.isDiscontinuity === false && previousLeg.isXF() && prevTerm && wptTerm) {
            inboundMagneticCourse = Avionics.Utils.computeGreatCircleHeading(prevTerm.location, wptTerm.location);
          }

          const defaultHold = {
            inboundMagneticCourse,
            turnDirection: TurnDirection.Right,
            time: alt <= 14000 ? 1 : 1.5,
            type: HoldType.Computed,
          };
          await fpln.props.fmcService.master?.flightPlanService.addOrEditManualHold(
            legIndex,
            Object.assign({}, defaultHold),
            undefined,
            defaultHold,
            planIndex,
            altnFlightPlan,
          );

          fpln.props.fmcService.master?.revisedWaypointIndex.set(legIndex + 1); // We just inserted a new HOLD leg
        }
        fpln.props.mfd.uiService.navigateTo(`fms/${fpln.props.mfd.uiService.activeUri.get().category}/f-pln-hold`);
      },
    },
    {
      name: 'AIRWAYS',
      disabled: [FplnRevisionsMenuType.Discontinuity || FplnRevisionsMenuType.TooSteepPath].includes(type),
      onPressed: () => {
        fpln.props.fmcService.master?.flightPlanService.startAirwayEntry(legIndex);
        fpln.props.mfd.uiService.navigateTo(`fms/${fpln.props.mfd.uiService.activeUri.get().category}/f-pln-airways`);
      },
    },
    {
      name:
        !altnFlightPlan &&
        ![FplnRevisionsMenuType.Discontinuity || FplnRevisionsMenuType.TooSteepPath].includes(type) &&
        fpln.loadedFlightPlan?.legElementAt(legIndex).definition.overfly
          ? 'DELETE OVERFLY *'
          : 'OVERFLY *',
      disabled:
        altnFlightPlan || [FplnRevisionsMenuType.Discontinuity || FplnRevisionsMenuType.TooSteepPath].includes(type),
      onPressed: () =>
        fpln.props.fmcService.master?.flightPlanService.toggleOverfly(legIndex, planIndex, altnFlightPlan),
    },
    {
      name: 'ENABLE ALTN *',
      disabled: false,
      onPressed: () => {
        fpln.props.fmcService.master?.flightPlanService.enableAltn(legIndex, planIndex);
        fpln.props.fmcService.master?.acInterface.updateOansAirports();
      },
    },
    {
      name: 'NEW DEST',
      disabled: false,
      onPressed: () => fpln.openNewDestWindow(),
    },
    {
      name: 'CONSTRAINTS',
      disabled:
        altnFlightPlan || [FplnRevisionsMenuType.Discontinuity || FplnRevisionsMenuType.TooSteepPath].includes(type),
      onPressed: () =>
        fpln.props.mfd.uiService.navigateTo(
          `fms/${fpln.props.mfd.uiService.activeUri.get().category}/f-pln-vert-rev/alt`,
        ),
    },
    {
      name: 'CMS',
      disabled:
        altnFlightPlan || [FplnRevisionsMenuType.Discontinuity || FplnRevisionsMenuType.TooSteepPath].includes(type),
      onPressed: () =>
        fpln.props.mfd.uiService.navigateTo(
          `fms/${fpln.props.mfd.uiService.activeUri.get().category}/f-pln-vert-rev/cms`,
        ),
    },
    {
      name: 'STEP ALTs',
      disabled:
        altnFlightPlan || [FplnRevisionsMenuType.Discontinuity || FplnRevisionsMenuType.TooSteepPath].includes(type),
      onPressed: () =>
        fpln.props.mfd.uiService.navigateTo(
          `fms/${fpln.props.mfd.uiService.activeUri.get().category}/f-pln-vert-rev/step-alts`,
        ),
    },
    {
      name: '(N/A) WIND',
      disabled: true,
      onPressed: () => {
        // Find out whether waypoint is CLB, CRZ or DES waypoint and direct to appropriate WIND sub-page
        if (fpln.loadedFlightPlan?.legElementAt(legIndex)?.segment?.class === SegmentClass.Arrival) {
          fpln.props.mfd.uiService.navigateTo(`fms/${fpln.props.mfd.uiService.activeUri.get().category}/wind/des`);
        } else if (fpln.loadedFlightPlan?.legElementAt(legIndex)?.segment?.class === SegmentClass.Enroute) {
          fpln.props.mfd.uiService.navigateTo(`fms/${fpln.props.mfd.uiService.activeUri.get().category}/wind/crz`);
        } else {
          fpln.props.mfd.uiService.navigateTo(`fms/${fpln.props.mfd.uiService.activeUri.get().category}/wind/clb`);
        }
      },
    },
  ];
}
