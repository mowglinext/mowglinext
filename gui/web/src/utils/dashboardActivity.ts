import type {HighLevelStatus} from '../types/ros';

// Presentation only: numeric AUTONOMOUS includes preparation, transit and
// docking. It is not evidence of cutting, nor a replacement for BT authority.
export function dashboardActivity(status: Partial<HighLevelStatus>) {
  if (status.state === 3) return 'recording';
  if (status.state === 4) return 'manual';
  if (status.state !== 2) return undefined;
  if ([
    'RETURNING_HOME', 'LOW_BATTERY_DOCKING', 'CRITICAL_BATTERY_DOCKING',
    'RAIN_DETECTED_DOCKING', 'COVERAGE_FAILED_DOCKING', 'MOWING_COMPLETE',
  ].includes(status.state_name ?? '')) return 'returning';
  if (status.state_name === 'TRANSIT' || status.sub_state_name === 'TRANSIT') return 'transit';
  if (status.state_name === 'MOWING') return 'mowing';
  // Unknown/new autonomous substates must not claim mowing.
  return 'autonomous';
}
