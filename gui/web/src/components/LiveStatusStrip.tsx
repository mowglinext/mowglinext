import {useHighLevelStatus} from "../hooks/useHighLevelStatus.ts";
import {useEmergency} from "../hooks/useEmergency.ts";
import {useThemeMode} from "../theme/ThemeContext.tsx";

/**
 * Thin animated strip pinned above the page header.
 *
 * Off when the robot is idle/parked; an animated green-tinted gradient when
 * the robot is moving (mowing/transit/undocking/recovering); solid red when
 * emergency is latched. The point is for the chrome itself to signal "robot
 * is alive" without the operator having to read the header chips.
 */
const MOTION_STATES = new Set([
  "MOWING", "TRANSIT", "UNDOCKING", "RETURNING_HOME", "MANUAL_MOWING",
  "RESUMING_AFTER_RAIN", "RESUMING_UNDOCKING", "BOUNDARY_RECOVERY",
  "LOW_BATTERY_DOCKING", "CRITICAL_BATTERY_DOCKING",
  "COVERAGE_FAILED_DOCKING", "SKIP_STRIP", "PREFLIGHT_CHECK",
  "CALIBRATING_HEADING", "RECORDING",
]);

const keyframes = `
@keyframes liveStripSheen {
  0% { background-position: 0% 50%; }
  100% { background-position: 200% 50%; }
}
@keyframes liveStripPulse {
  0%, 100% { opacity: 0.55; }
  50% { opacity: 1; }
}
`;

interface LiveStatusStripProps {
  height?: number;
}

export function LiveStatusStrip({height = 2}: LiveStatusStripProps) {
  const {colors} = useThemeMode();
  const {highLevelStatus} = useHighLevelStatus();
  const emergency = useEmergency();

  const state = highLevelStatus.state_name;
  const isEmergency = highLevelStatus.emergency ?? emergency.active_emergency ?? false;
  const isMoving = state ? MOTION_STATES.has(state) : false;

  if (!isMoving && !isEmergency) return null;

  const color = isEmergency ? colors.danger : colors.accent;
  const background = isEmergency
    ? color
    : `linear-gradient(90deg, transparent 0%, ${color} 30%, ${color}dd 50%, ${color} 70%, transparent 100%)`;

  return (
    <>
      <style>{keyframes}</style>
      <div
        aria-hidden
        style={{
          height,
          width: '100%',
          background,
          backgroundSize: '200% 100%',
          animation: isEmergency
            ? 'liveStripPulse 1.2s ease-in-out infinite'
            : 'liveStripSheen 2.4s linear infinite',
          flexShrink: 0,
        }}
      />
    </>
  );
}
