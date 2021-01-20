import { FlightPlanManager } from "../flightplanning/FlightPlanManager"
import {Leg, Geometry, TFLeg, Type1Transition, Transition} from "./Geometry";

const mod = (x: number, n: number) => x - Math.floor(x / n) * n;

/**
 * This class will guide the aircraft by predicting a flight path and
 * calculating the autopilot inputs to follow the predicted flight path.
 */
export class GuidanceManager {
    private lastTransition?: number;
    public flightPlanManager: FlightPlanManager;

    constructor(flightPlanManager: FlightPlanManager) {
        this.flightPlanManager = flightPlanManager;
    }

    getActiveLeg(): TFLeg | null {
        const activeIndex = this.flightPlanManager.getActiveWaypointIndex();
        const from = this.flightPlanManager.getWaypoint(activeIndex - 1);
        const to = this.flightPlanManager.getWaypoint(activeIndex);

        if (!from || !to) {
            return null;
        }

        return new TFLeg(from, to);
    }

    getNextLeg(): TFLeg | null {
        const activeIndex = this.flightPlanManager.getActiveWaypointIndex();
        const from = this.flightPlanManager.getWaypoint(activeIndex);
        const to = this.flightPlanManager.getWaypoint(activeIndex + 1);

        if (!from || !to) {
            return null;
        }

        return new TFLeg(from, to);
    }

    /**
     * The active leg path geometry, used for immediate autoflight.
     */
    getActiveLegPathGeometry(): Geometry | null {
        const activeLeg = this.getActiveLeg();
        const nextLeg = this.getNextLeg();

        if (!activeLeg) {
            return null;
        }

        const legs = new Map<number, Leg>([[1, activeLeg], [2, nextLeg]]);
        const transitions = new Map<number, Transition>();

        if (nextLeg) {
            const kts = Math.max(SimVar.GetSimVarValue('AIRSPEED TRUE', 'knots'), 150); // knots, i.e. nautical miles per hour

            let bankAngleLimit = 25;
            if (kts < 150) {
                bankAngleLimit = 15 + Math.min(kts / 150, 1) * (25 - 15);
            } else if (kts > 300) {
                bankAngleLimit = 25 - Math.min((kts - 300) / 150, 1) * (25 - 19);
            }

            // turn radius
            const xKr = (kts ** 2 / (9.81 * Math.tan(bankAngleLimit * Avionics.Utils.DEG2RAD))) / 6080.2;

            // turn direction
            const courseChange = mod(nextLeg.bearing - activeLeg.bearing + 180, 360) - 180;
            const cw = courseChange >= 0;

            transitions.set(2, new Type1Transition(
                activeLeg,
                nextLeg,
                xKr,
                cw,
            ));
        }

        return new Geometry(transitions, legs);
    }

    /**
     * The full leg path geometry, used for the ND and F-PLN page.
     */
    getMultipleLegGeometry(): Geometry | null {
        //return null;

        return this.getActiveLegPathGeometry();
    }
}
