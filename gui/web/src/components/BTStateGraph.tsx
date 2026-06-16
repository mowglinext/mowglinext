import {useThemeMode} from "../theme/ThemeContext.tsx";

/**
 * Compact horizontal state-machine view of the high-level BT cycle.
 *
 * Highlights the current state with a pulsing ring and draws a faint trail
 * from the prior state. The graph isn't exhaustive (the BT has ~25 states);
 * it captures the steady-state mowing cycle plus the most common branches
 * (rain, low-battery, emergency, boundary recovery).
 */

interface BTStateGraphProps {
  current: string | undefined;
}

interface Node {
  key: string;
  label: string;
  x: number;
  y: number;
  group: 'cycle' | 'fault' | 'manual';
}

interface Edge {
  from: string;
  to: string;
  curve?: number;
}

const NODES: Node[] = [
  // main cycle (left to right)
  {key: 'IDLE_DOCKED',   label: 'Docked',     x: 60,  y: 70, group: 'cycle'},
  {key: 'UNDOCKING',     label: 'Undock',     x: 170, y: 70, group: 'cycle'},
  {key: 'TRANSIT',       label: 'Transit',    x: 280, y: 70, group: 'cycle'},
  {key: 'MOWING',        label: 'Mow',        x: 390, y: 70, group: 'cycle'},
  {key: 'RETURNING_HOME', label: 'Return',    x: 500, y: 70, group: 'cycle'},
  {key: 'DOCKING',       label: 'Dock',       x: 610, y: 70, group: 'cycle'},
  {key: 'CHARGING',      label: 'Charge',     x: 720, y: 70, group: 'cycle'},

  // fault row (below)
  {key: 'RAIN_DETECTED_DOCKING',     label: 'Rain → dock', x: 280, y: 140, group: 'fault'},
  {key: 'LOW_BATTERY_DOCKING',       label: 'Low batt',    x: 390, y: 140, group: 'fault'},
  {key: 'BOUNDARY_RECOVERY',         label: 'Recovery',    x: 500, y: 140, group: 'fault'},
  {key: 'EMERGENCY',                 label: 'E-stop',      x: 610, y: 140, group: 'fault'},

  // manual row (above)
  {key: 'MANUAL_MOWING',  label: 'Manual',    x: 390, y: 20,  group: 'manual'},
  {key: 'RECORDING',      label: 'Recording', x: 280, y: 20,  group: 'manual'},
];

const EDGES: Edge[] = [
  {from: 'IDLE_DOCKED', to: 'UNDOCKING'},
  {from: 'UNDOCKING', to: 'TRANSIT'},
  {from: 'TRANSIT', to: 'MOWING'},
  {from: 'MOWING', to: 'RETURNING_HOME'},
  {from: 'RETURNING_HOME', to: 'DOCKING'},
  {from: 'DOCKING', to: 'CHARGING'},
  {from: 'CHARGING', to: 'IDLE_DOCKED', curve: -1},
  {from: 'MOWING', to: 'RAIN_DETECTED_DOCKING'},
  {from: 'MOWING', to: 'LOW_BATTERY_DOCKING'},
  {from: 'MOWING', to: 'BOUNDARY_RECOVERY'},
  {from: 'MOWING', to: 'EMERGENCY'},
];

// Aliases — the high-level status can report finer states; collapse them
// to the diagram's canonical node for highlighting purposes.
const STATE_ALIASES: Record<string, string> = {
  'IDLE': 'IDLE_DOCKED',
  'RESUMING_UNDOCKING': 'UNDOCKING',
  'RESUMING_AFTER_RAIN': 'TRANSIT',
  'CRITICAL_BATTERY_DOCKING': 'LOW_BATTERY_DOCKING',
  'COVERAGE_FAILED_DOCKING': 'RETURNING_HOME',
  'SKIP_STRIP': 'MOWING',
  'PREFLIGHT_CHECK': 'UNDOCKING',
  'CALIBRATING_HEADING': 'TRANSIT',
  'BOUNDARY_EMERGENCY_STOP': 'EMERGENCY',
  'MOWING_COMPLETE': 'RETURNING_HOME',
  'RECORDING_COMPLETE': 'IDLE_DOCKED',
  'RAIN_WAITING': 'RAIN_DETECTED_DOCKING',
  'RAIN_TIMEOUT': 'IDLE_DOCKED',
};

function nodeFor(state: string | undefined): string | null {
  if (!state) return null;
  if (NODES.some(n => n.key === state)) return state;
  return STATE_ALIASES[state] ?? null;
}

const keyframes = `
@media (prefers-reduced-motion: no-preference) {
  @keyframes btStatePulse {
    0%, 100% { opacity: 0.55; transform: scale(1); }
    50% { opacity: 1; transform: scale(1.08); }
  }
}
`;

const NODE_W = 86;
const NODE_H = 28;

// Cycle order used to highlight the path the robot has already passed through.
const CYCLE_ORDER = ['IDLE_DOCKED', 'UNDOCKING', 'TRANSIT', 'MOWING', 'RETURNING_HOME', 'DOCKING', 'CHARGING'];

export function BTStateGraph({current}: BTStateGraphProps) {
  const {colors} = useThemeMode();
  const activeKey = nodeFor(current);
  const activeCycleIdx = activeKey ? CYCLE_ORDER.indexOf(activeKey) : -1;

  const groupColor = (group: Node['group']): string => {
    switch (group) {
      case 'cycle': return colors.accent;
      case 'manual': return colors.sky;
      case 'fault': return colors.amber;
    }
  };

  /** True if this cycle node sits before-or-at the active state. */
  const isPast = (key: string): boolean => {
    const idx = CYCLE_ORDER.indexOf(key);
    return idx >= 0 && idx <= activeCycleIdx;
  };
  /** True for cycle edges whose `to` node is before-or-at the active state. */
  const isEdgePast = (toKey: string, fromKey: string): boolean => {
    const fromCycle = CYCLE_ORDER.indexOf(fromKey);
    const toCycle = CYCLE_ORDER.indexOf(toKey);
    if (fromCycle < 0 || toCycle < 0) return false;
    return toCycle <= activeCycleIdx;
  };

  return (
    <div style={{
      position: 'relative',
      width: '100%',
      maxWidth: 800,
      background: colors.bgElevated,
      borderRadius: 12,
      padding: '12px 12px 16px',
      overflowX: 'auto',
      WebkitOverflowScrolling: 'touch',
    }}>
      <style>{keyframes}</style>
      <div style={{
        display: 'flex', alignItems: 'baseline', justifyContent: 'space-between',
        marginBottom: 10,
      }}>
        <div style={{
          fontSize: 11, color: colors.textMuted, letterSpacing: '0.08em',
          textTransform: 'uppercase' as const,
        }}>
          Behavior tree state
        </div>
        {current && (
          <div style={{
            display: 'inline-flex', alignItems: 'center', gap: 8,
            padding: '4px 10px 4px 8px',
            background: `${colors.accent}14`,
            border: `1px solid ${colors.accent}50`,
            borderRadius: 999,
          }}>
            <span style={{
              width: 8, height: 8, borderRadius: 4,
              background: colors.accent,
              boxShadow: `0 0 8px ${colors.accent}`,
              animation: 'btStatePulse 1.8s ease-in-out infinite',
            }}/>
            <span style={{
              fontSize: 11, fontWeight: 700, color: colors.accent,
              letterSpacing: '0.04em',
            }}>
              {current}
            </span>
          </div>
        )}
      </div>
      <svg viewBox="0 0 800 180" width="100%" height="auto" preserveAspectRatio="xMidYMid meet" style={{display: 'block', width: '100%'}}>
        {/* edges */}
        {EDGES.map((e, i) => {
          const from = NODES.find(n => n.key === e.from);
          const to = NODES.find(n => n.key === e.to);
          if (!from || !to) return null;
          const startX = from.x + NODE_W / 2;
          const startY = from.y + NODE_H / 2;
          const endX = to.x - NODE_W / 2;
          const endY = to.y + NODE_H / 2;
          const isActive = activeKey === to.key;

          // Curved back-edge (Charge → Docked) uses a cubic to dodge the row.
          if (e.curve === -1) {
            const cpY = -10;
            const startBX = from.x;
            const endBX = to.x;
            return (
              <path
                key={i}
                d={`M ${startBX} ${from.y} C ${startBX} ${cpY}, ${endBX} ${cpY}, ${endBX} ${to.y}`}
                fill="none"
                stroke={isActive ? colors.accent : colors.border}
                strokeWidth={isActive ? 2 : 1}
                strokeDasharray="3 3"
                opacity={isActive ? 0.8 : 0.5}
              />
            );
          }
          const past = isEdgePast(to.key, from.key);
          const lit = isActive || past;
          return (
            <line
              key={i}
              x1={startX} y1={startY}
              x2={endX} y2={endY}
              stroke={lit ? groupColor(to.group) : colors.border}
              strokeWidth={isActive ? 2.5 : past ? 1.6 : 1}
              opacity={isActive ? 1 : past ? 0.85 : 0.45}
              markerEnd={lit ? 'url(#bt-arrow-active)' : 'url(#bt-arrow)'}
            />
          );
        })}

        {/* arrow markers */}
        <defs>
          <marker id="bt-arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6" markerHeight="6" orient="auto">
            <path d="M 0 0 L 10 5 L 0 10 z" fill={colors.border}/>
          </marker>
          <marker id="bt-arrow-active" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
            <path d="M 0 0 L 10 5 L 0 10 z" fill={colors.accent}/>
          </marker>
        </defs>

        {/* nodes */}
        {NODES.map(n => {
          const isActive = activeKey === n.key;
          const past = isPast(n.key);
          const accent = groupColor(n.group);
          return (
            <g key={n.key} transform={`translate(${n.x - NODE_W / 2}, ${n.y})`}>
              {isActive && (
                <>
                  {/* outer halo -- gentle glow that doesn't compete with the
                      pulse ring */}
                  <rect
                    x={-10} y={-10}
                    width={NODE_W + 20} height={NODE_H + 20}
                    rx={14}
                    fill={accent}
                    opacity={0.18}
                    style={{filter: `blur(8px)`}}
                  />
                  <rect
                    x={-4} y={-4}
                    width={NODE_W + 8} height={NODE_H + 8}
                    rx={10}
                    fill="none"
                    stroke={accent}
                    strokeWidth={2}
                    style={{
                      transformOrigin: `${NODE_W / 2 + 4}px ${NODE_H / 2 + 4}px`,
                      animation: 'btStatePulse 1.8s ease-in-out infinite',
                    }}
                  />
                </>
              )}
              <rect
                width={NODE_W} height={NODE_H} rx={6}
                fill={isActive ? `${accent}28` : past ? `${accent}3d` : colors.bgCard}
                stroke={isActive ? accent : past ? `${accent}55` : colors.border}
                strokeWidth={isActive ? 1.5 : 1}
              />
              <text
                x={NODE_W / 2} y={NODE_H / 2 + 4}
                textAnchor="middle"
                fontFamily="inherit"
                fontSize={11}
                fontWeight={isActive ? 700 : past ? 600 : 500}
                fill={isActive ? accent : past ? colors.text : colors.textSecondary}
              >
                {n.label}
              </text>
            </g>
          );
        })}
      </svg>
      <div style={{
        display: 'flex', gap: 16, fontSize: 11, color: colors.textMuted,
        marginTop: 4, justifyContent: 'center', flexWrap: 'wrap',
      }}>
        <div style={{display: 'flex', alignItems: 'center', gap: 6}}>
          <span style={{width: 10, height: 3, background: colors.accent, borderRadius: 2}}/>
          Cycle
        </div>
        <div style={{display: 'flex', alignItems: 'center', gap: 6}}>
          <span style={{width: 10, height: 3, background: colors.sky, borderRadius: 2}}/>
          Operator
        </div>
        <div style={{display: 'flex', alignItems: 'center', gap: 6}}>
          <span style={{width: 10, height: 3, background: colors.amber, borderRadius: 2}}/>
          Fault / recovery
        </div>
      </div>
    </div>
  );
}
