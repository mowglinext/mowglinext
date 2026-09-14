// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

/**
 * High-level numeric states that mean "the robot is under its own power and
 * may be moving". status_nodes.cpp publishes the numeric state reliably every
 * tick, unlike the free-text state_name, which carries substates (PLANNING,
 * OBSTACLE_BACKOFF, DYNAMIC_OBSTACLE_CLEARED, AREA_UNREACHABLE) that an
 * allowlist of names kept missing.
 */
const MOVING_STATES = [
    2,  // AUTONOMOUS
    3,  // RECORDING
    4,  // MANUAL_MOWING
] as const;

/**
 * True when the mower is in a self-propelled state AND is not sitting on the
 * charger.
 *
 * The charging term is not cosmetic: while the mower is on the dock the BT can
 * legitimately still report state 2 (ManualChargeGuard holds the coverage loop
 * in place rather than tearing the session down), so state alone would animate
 * the dashboard as though the robot were driving around while it is in fact
 * parked on its contacts.
 *
 * Extracted from MowgliNextPage useMowerData so the precedence is pinned by a
 * test. The inline form used to be
 *
 *   stateNum === 2 || stateNum === 3 || stateNum === 4 && !isCharging
 *
 * where && binds tighter than ||, so the charging term only ever qualified
 * state 4 and states 2 and 3 read as "moving" while charging.
 */
export function deriveIsMoving(
    stateNum: number | undefined | null,
    isCharging: boolean | undefined | null,
): boolean {
    if (isCharging) {
        return false;
    }
    return MOVING_STATES.some((state) => state === stateNum);
}
