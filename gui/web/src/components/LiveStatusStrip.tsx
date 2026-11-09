import {hasEmergency} from '../utils/statusTelemetry';
import {useHighLevelStatus} from "../hooks/useHighLevelStatus.ts";
import {useEmergency} from "../hooks/useEmergency.ts";
import {useThemeMode} from "../theme/ThemeContext.tsx";

/**
 * Thin animated strip pinned above the page header.
 *
 * Off when the robot is idle/parked; a green-tinted gradient when the robot
 * is moving (mowing/transit/undocking/recovering); pulsing red when emergency
 * is latched. Visual mode adds a restrained sheen to the moving state.
 */
const MOTION_STATES = new Set([
  "MOWING", "TRANSIT", "UNDOCKING", "RETURNING_HOME", "MANUAL_MOWING",
  "RESUMING_AFTER_RAIN", "RESUMING_UNDOCKING", "BOUNDARY_RECOVERY",
  "LOW_BATTERY_DOCKING", "CRITICAL_BATTERY_DOCKING", "RAIN_DETECTED_DOCKING",
  "COVERAGE_FAILED_DOCKING", "SKIP_STRIP", "PREFLIGHT_CHECK",
  "CALIBRATING_HEADING", "RECORDING", "OBSTACLE_BACKOFF",
]);

interface LiveStatusStripProps {
  height?: number;
}

export function LiveStatusStrip({height = 2}: LiveStatusStripProps) {
  const {colors, displayMode} = useThemeMode();
  const {highLevelStatus} = useHighLevelStatus();
  const emergency = useEmergency();

  const state = highLevelStatus.state_name;
  const isEmergency = hasEmergency(emergency, highLevelStatus);
  const isMoving = state ? MOTION_STATES.has(state) : false;

  if (!isMoving && !isEmergency) return null;

  const color = isEmergency ? colors.danger : colors.accent;
  const background = isEmergency
    ? color
    : `linear-gradient(90deg, transparent 0%, ${color} 30%, ${color}dd 50%, ${color} 70%, transparent 100%)`;

  return (
    <div
      aria-hidden
      style={{
        height,
        width: '100%',
        background,
        backgroundSize: '200% 100%',
        animation: isEmergency
          ? 'liveStripPulse 1.2s ease-in-out infinite'
          : displayMode === 'visual' ? 'liveStripSheen 3.6s ease-in-out infinite' : 'none',
        flexShrink: 0,
      }}
    />
  );
}
