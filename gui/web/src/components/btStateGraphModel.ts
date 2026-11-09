export interface Node {
  key: string;
  /** i18n key (under the btStateGraph namespace) for the display label. */
  labelKey: string;
  x: number;
  y: number;
  group: 'cycle' | 'fault' | 'manual';
}

export const NODES: Node[] = [
  // main cycle (left to right)
  {key: 'IDLE_DOCKED',   labelKey: 'nodeDocked',   x: 60,  y: 70, group: 'cycle'},
  {key: 'UNDOCKING',     labelKey: 'nodeUndock',   x: 170, y: 70, group: 'cycle'},
  {key: 'TRANSIT',       labelKey: 'nodeTransit',  x: 280, y: 70, group: 'cycle'},
  {key: 'MOWING',        labelKey: 'nodeMow',      x: 390, y: 70, group: 'cycle'},
  {key: 'RETURNING_HOME', labelKey: 'nodeReturn',  x: 500, y: 70, group: 'cycle'},
  {key: 'DOCKING',       labelKey: 'nodeDock',     x: 610, y: 70, group: 'cycle'},
  {key: 'CHARGING',      labelKey: 'nodeCharge',   x: 720, y: 70, group: 'cycle'},

  // fault row (below)
  {key: 'OBSTACLE_BACKOFF',          labelKey: 'nodeObstacle', x: 170, y: 140, group: 'fault'},
  {key: 'RAIN_DETECTED_DOCKING',     labelKey: 'nodeRainDock', x: 280, y: 140, group: 'fault'},
  {key: 'LOW_BATTERY_DOCKING',       labelKey: 'nodeLowBatt',  x: 390, y: 140, group: 'fault'},
  {key: 'BOUNDARY_RECOVERY',         labelKey: 'nodeRecovery', x: 500, y: 140, group: 'fault'},
  {key: 'EMERGENCY',                 labelKey: 'nodeEstop',    x: 610, y: 140, group: 'fault'},
  {key: 'FAILURE',                   labelKey: 'nodeFailure',  x: 720, y: 140, group: 'fault'},

  // manual row (above)
  {key: 'MANUAL_MOWING',  labelKey: 'nodeManual',    x: 390, y: 20,  group: 'manual'},
  {key: 'RECORDING',      labelKey: 'nodeRecording', x: 280, y: 20,  group: 'manual'},
];

// Aliases — the high-level status can report finer states; collapse them
// to the diagram's canonical node for highlighting purposes. Keep this in
// sync with the state_name values in mowgli_behavior's BT XML files.
const STATE_ALIASES: Record<string, string> = {
  'IDLE': 'IDLE_DOCKED',
  'RESUMING_UNDOCKING': 'UNDOCKING',
  'RESUMING_AFTER_RAIN': 'TRANSIT',
  'CRITICAL_BATTERY_DOCKING': 'LOW_BATTERY_DOCKING',
  'CRITICAL_BATTERY_CHARGING': 'CHARGING',
  'MANUAL_CHARGING': 'CHARGING',
  'COVERAGE_FAILED_DOCKING': 'RETURNING_HOME',
  'PLANNING': 'MOWING',
  'DYNAMIC_OBSTACLE_CLEARED': 'OBSTACLE_BACKOFF',
  // #487: coverage refused because the robot's OWN pose is a lethal/keepout
  // cell. Same family as an obstacle wedge from the operator's point of view.
  'START_POSE_BLOCKED': 'OBSTACLE_BACKOFF',
  // #500: repeat-dig escalation — held against an object it cannot free
  // itself from. Obstacle family for the operator; the dashboard card carries
  // the recovery hint (lift clear then Play, or Home).
  'DIG_OBSTRUCTION': 'OBSTACLE_BACKOFF',
  'PREFLIGHT_CHECK': 'UNDOCKING',
  'CALIBRATING_HEADING': 'TRANSIT',
  'BOUNDARY_EMERGENCY_STOP': 'EMERGENCY',
  'MOWING_COMPLETE': 'RETURNING_HOME',
  'RECORDING_COMPLETE': 'IDLE_DOCKED',
  'RAIN_WAITING': 'RAIN_DETECTED_DOCKING',
  'RAIN_TIMEOUT': 'IDLE_DOCKED',
};

export function nodeFor(state: string | undefined): string | null {
  if (!state) return null;
  if (NODES.some(n => n.key === state)) return state;
  const alias = STATE_ALIASES[state];
  if (alias) return alias;
  // Terminal-failure states (NAV_TO_DOCK_FAILED, CHARGER_FAILED, …) collapse
  // onto the generic failure node so a fault is never rendered as "nothing".
  if (/(_FAILED|_UNREACHABLE)$/.test(state)) return 'FAILURE';
  return null;
}
