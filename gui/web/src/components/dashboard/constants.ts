// Typography commitments -- Mowgli is premium tech-garden hardware. Satoshi
// for the warm sans body + UI, Instrument Serif kept as accent for hero
// moments, Space Grotesk for tabular telemetry numerals.
export const FONT = "'Satoshi', 'Inter', -apple-system, BlinkMacSystemFont, sans-serif";
export const DISPLAY_FONT = "'Instrument Serif', 'Iowan Old Style', Georgia, serif";
export const MONO_FONT = "'Space Grotesk', 'JetBrains Mono', ui-monospace, monospace";

// Every state_name listed here must match a string emitted by main_tree.xml
// (grep PublishHighLevelStatus). 'tone' drives the HeroCard palette:
//   info     = calm / resting / success-ish steady states
//   primary  = active motion (mowing, transit, undocking, recovery)
//   warning  = non-fatal anomaly that still needs operator awareness
//   success  = terminal success (completion, charging-done banners)
//   danger   = emergency / blocking failure
export const MOWER_STATES: Record<string, { label: string; tone: 'info' | 'primary' | 'warning' | 'success' | 'danger'; friendly: string }> = {
  // Idle / docked family
  IDLE_DOCKED:                { label: 'Docked',             tone: 'info',    friendly: 'Resting on dock' },
  IDLE:                       { label: 'Idle',               tone: 'info',    friendly: 'Ready when you are' },
  CHARGING:                   { label: 'Charging',           tone: 'success', friendly: 'Topping up the battery' },

  // Autonomous mowing cycle
  PREFLIGHT_CHECK:            { label: 'Preflight Check',    tone: 'info',    friendly: 'Running preflight checks' },
  UNDOCKING:                  { label: 'Undocking',          tone: 'primary', friendly: 'Leaving the dock' },
  CALIBRATING_HEADING:        { label: 'Calibrating Heading',tone: 'info',    friendly: 'Calibrating heading' },
  MOWING:                     { label: 'Mowing',             tone: 'primary', friendly: 'Mowing the lawn' },
  TRANSIT:                    { label: 'Transit',            tone: 'primary', friendly: 'Moving to next strip' },
  SKIP_STRIP:                 { label: 'Skipping Strip',     tone: 'info',    friendly: 'Skipping unreachable strip' },
  RETURNING_HOME:             { label: 'Returning',          tone: 'primary', friendly: 'Heading back to dock' },
  MOWING_COMPLETE:            { label: 'Mowing Complete',    tone: 'success', friendly: 'All areas mowed' },

  // Recording
  RECORDING:                  { label: 'Recording',          tone: 'primary', friendly: 'Recording area boundary' },
  RECORDING_COMPLETE:         { label: 'Recording Saved',    tone: 'success', friendly: 'Area boundary saved' },

  // Manual
  MANUAL_MOWING:              { label: 'Manual Mowing',      tone: 'primary', friendly: 'Manual mowing mode' },

  // Battery
  LOW_BATTERY_DOCKING:        { label: 'Low Battery',        tone: 'warning', friendly: 'Low battery — heading to dock' },
  CRITICAL_BATTERY_DOCKING:   { label: 'Critical Battery',   tone: 'warning', friendly: 'Critical battery — heading to dock' },
  CRITICAL_BATTERY_NAV_FAILED:{ label: 'Battery Nav Failed', tone: 'danger',  friendly: 'Critical battery and cannot navigate home' },

  // Rain
  RAIN_DETECTED_DOCKING:      { label: 'Rain Detected',      tone: 'warning', friendly: 'Rain detected — heading to dock' },
  RAIN_WAITING:               { label: 'Waiting for Dry',    tone: 'warning', friendly: 'Waiting for the rain to stop' },
  RAIN_TIMEOUT:               { label: 'Rain Timeout',       tone: 'warning', friendly: 'Gave up waiting for dry weather' },
  RESUMING_AFTER_RAIN:        { label: 'Resuming',           tone: 'primary', friendly: 'Resuming after rain delay' },

  // Recovery / transitions
  RESUMING_UNDOCKING:         { label: 'Resuming Undock',    tone: 'primary', friendly: 'Leaving the dock to resume' },
  BOUNDARY_RECOVERY:          { label: 'Boundary Recovery',  tone: 'warning', friendly: 'Recovering after boundary event' },

  // Failures / emergencies
  EMERGENCY:                  { label: 'Emergency Stop',     tone: 'danger',  friendly: 'Emergency stop engaged' },
  BOUNDARY_EMERGENCY_STOP:    { label: 'Boundary Alert',     tone: 'danger',  friendly: 'Stopped — outside boundary' },
  UNDOCK_FAILED:              { label: 'Undock Failed',      tone: 'warning', friendly: 'Undock failed — check dock' },
  CHARGER_FAILED:             { label: 'Charger Failed',     tone: 'warning', friendly: 'Charger not detected' },
  NAV_TO_DOCK_FAILED:         { label: 'Nav to Dock Failed', tone: 'danger',  friendly: 'Could not navigate to dock' },
  COVERAGE_FAILED_DOCKING:    { label: 'Coverage Failed',    tone: 'warning', friendly: 'Coverage failed — returning to dock' },
};

export const fmt = {
  v: (n: number | undefined) => n == null ? '--' : `${n.toFixed(2)} V`,
  a: (n: number | undefined) => n == null ? '--' : `${n.toFixed(2)} A`,
  c: (n: number | undefined) => n == null ? '--' : `${n.toFixed(1)} C`,
  pct: (n: number | undefined) => n == null ? '--' : `${Math.round(n)}%`,
  rpm: (n: number | undefined) => n == null ? '--' : `${Math.round(n)}`,
  mins: (n: number | undefined) => {
    if (n == null) return '--';
    const m = Math.round(n);
    if (m < 60) return `${m} min`;
    return `${Math.floor(m / 60)}h ${m % 60}m`;
  },
};

export const KEYFRAMES_CSS = `
@keyframes mn-pulse {
  0%, 100% { opacity: 1; transform: scale(1); }
  50% { opacity: 0.4; transform: scale(0.85); }
}
@keyframes mn-rise {
  from { opacity: 0; transform: translateY(8px); }
  to   { opacity: 1; transform: translateY(0); }
}
@media (prefers-reduced-motion: no-preference) {
  @keyframes mn-bounds-glow {
    0%, 100% { box-shadow: 0 0 0 0 rgba(255,107,107,0.7), inset 0 0 0 1px rgba(255,107,107,0.6); }
    50% { box-shadow: 0 0 0 6px rgba(255,107,107,0), inset 0 0 0 1px rgba(255,107,107,0.9); }
  }
  @keyframes mn-pulse-red {
    0%, 100% { box-shadow: 0 0 0 0 rgba(255,107,107,0); }
    50% { box-shadow: 0 0 0 8px rgba(255,107,107,0.15); }
  }
}
.mn-card-hover:hover { transform: translateY(-1px); }

/* Luminous edge accent on every glass card -- the visual signature of the
   tech-garden language. Single gradient stroke applied via mask-composite
   so only the top-left quadrant lights up. */
.mn-glass-card { position: relative; }
.mn-glass-card::before {
  content: '';
  position: absolute; inset: 0;
  padding: 1px;
  border-radius: inherit;
  background: linear-gradient(140deg, rgba(124,255,178,0.32) 0%, rgba(124,255,178,0) 32%, rgba(255,255,255,0) 62%, rgba(69,214,232,0.18) 100%);
  -webkit-mask: linear-gradient(#000 0 0) content-box, linear-gradient(#000 0 0);
          mask: linear-gradient(#000 0 0) content-box, linear-gradient(#000 0 0);
  -webkit-mask-composite: xor; mask-composite: exclude;
  pointer-events: none;
}

.mn-btn { transition: background .12s, border-color .12s, transform .08s; }
.mn-btn:hover { transform: translateY(-1px); }
.mn-btn:active { transform: translateY(0); }

/* Editorial display family for big numbers + hero headlines.
   .mn-display reads like a survey instrument readout. */
.mn-display {
  font-family: 'Instrument Serif', 'Iowan Old Style', Georgia, serif;
  font-weight: 400;
  letter-spacing: -0.015em;
}
.mn-display em { font-style: italic; }
.mn-num {
  font-family: 'Instrument Serif', 'Iowan Old Style', Georgia, serif;
  font-weight: 400;
  font-variant-numeric: tabular-nums;
  letter-spacing: -0.02em;
}

/* AntD upgrades -- pull the framework into the editorial system. */
.ant-statistic-title {
  font-family: 'Satoshi', 'Inter', sans-serif !important;
  font-size: 11px !important;
  letter-spacing: 0.06em;
  text-transform: uppercase;
  font-weight: 600 !important;
}
.ant-statistic-content-value,
.ant-statistic-content-prefix,
.ant-statistic-content-suffix {
  font-family: 'Instrument Serif', 'Iowan Old Style', Georgia, serif !important;
  font-variant-numeric: tabular-nums;
  letter-spacing: -0.015em;
}
.ant-statistic-content-suffix {
  font-size: 0.55em;
  margin-left: 4px;
  vertical-align: 0.25em;
  font-family: 'Space Grotesk', monospace !important;
  text-transform: lowercase;
}
.ant-card-head-title,
.ant-collapse-header-text {
  font-family: 'Satoshi', 'Inter', sans-serif;
  font-weight: 600;
  letter-spacing: -0.005em;
}
.ant-typography h1,
.ant-typography h2,
.ant-typography h3 {
  font-family: 'Instrument Serif', Georgia, serif !important;
  font-weight: 400 !important;
  letter-spacing: -0.015em !important;
}
.ant-tabs-tab-btn {
  font-family: 'Satoshi', 'Inter', sans-serif;
  font-weight: 500;
}
.ant-table-thead > tr > th {
  font-family: 'Satoshi', 'Inter', sans-serif !important;
  font-size: 11px !important;
  letter-spacing: 0.06em;
  text-transform: uppercase;
  font-weight: 600 !important;
}

/* AntD buttons -- ensure healthy horizontal padding (some places use
   small/icon buttons and the new font-weight + uppercase tracking can
   push text edge-to-edge if padding is too tight). */
.ant-btn {
  padding-inline: 16px !important;
  display: inline-flex !important;
  align-items: center;
  gap: 6px;
}
.ant-btn-sm { padding-inline: 12px !important; }
.ant-btn-lg { padding-inline: 22px !important; }
.ant-btn-icon-only { padding-inline: 0 !important; }

/* AntD primary button -- lime gradient + dark ink so it matches the
   concept's hero Play. Default buttons get a glass treatment. */
.ant-btn-primary {
  background: linear-gradient(135deg, #7CFFB2 0%, #45D688 55%, #2BAA66 100%) !important;
  border: none !important;
  color: #02110D !important;
  font-weight: 700 !important;
  box-shadow: 0 8px 24px -10px rgba(124, 255, 178, 0.55), inset 0 1px 0 rgba(255, 255, 255, 0.35) !important;
  transition: transform 0.12s ease, box-shadow 0.12s ease !important;
}
.ant-btn-primary:hover,
.ant-btn-primary:focus {
  background: linear-gradient(135deg, #A3FFCB 0%, #5EE3A0 55%, #45D688 100%) !important;
  box-shadow: 0 10px 28px -8px rgba(124, 255, 178, 0.7), inset 0 1px 0 rgba(255, 255, 255, 0.42) !important;
  transform: translateY(-1px);
  color: #02110D !important;
}
.ant-btn-primary:active { transform: translateY(0); }

.ant-btn-default {
  background: rgba(255, 255, 255, 0.045) !important;
  border: 1px solid rgba(236, 255, 244, 0.10) !important;
  color: #ECFFF4 !important;
  backdrop-filter: blur(12px);
  font-weight: 600 !important;
}
.ant-btn-default:hover,
.ant-btn-default:focus {
  background: rgba(255, 255, 255, 0.075) !important;
  border-color: rgba(124, 255, 178, 0.35) !important;
  color: #ECFFF4 !important;
}

.ant-btn-dangerous {
  background: rgba(255, 107, 122, 0.12) !important;
  border: 1px solid rgba(255, 107, 122, 0.4) !important;
  color: #FF6B7A !important;
  font-weight: 600 !important;
}
.ant-btn-dangerous:hover,
.ant-btn-dangerous:focus {
  background: rgba(255, 107, 122, 0.18) !important;
  border-color: rgba(255, 107, 122, 0.6) !important;
  color: #FF6B7A !important;
}

/* AntD Steps -- pull the stepper into the concept palette. */
.ant-steps-item-process .ant-steps-item-icon {
  background: linear-gradient(135deg, #7CFFB2, #2BAA66) !important;
  border-color: transparent !important;
  box-shadow: 0 8px 22px -8px rgba(124, 255, 178, 0.5);
}
.ant-steps-item-process .ant-steps-item-icon > .ant-steps-icon { color: #02110D !important; }
.ant-steps-item-finish .ant-steps-item-icon {
  background: rgba(124, 255, 178, 0.16) !important;
  border-color: rgba(124, 255, 178, 0.5) !important;
}
.ant-steps-item-finish .ant-steps-item-icon > .ant-steps-icon { color: #7CFFB2 !important; }
.ant-steps-item-wait .ant-steps-item-icon {
  background: rgba(255, 255, 255, 0.03) !important;
  border-color: rgba(236, 255, 244, 0.14) !important;
}
.ant-steps-item-title { color: #ECFFF4 !important; }
.ant-steps-item-process > .ant-steps-item-container > .ant-steps-item-content > .ant-steps-item-title {
  color: #7CFFB2 !important;
}
.ant-steps-item-tail::after { background: rgba(236, 255, 244, 0.10) !important; }
.ant-steps-item-finish > .ant-steps-item-container > .ant-steps-item-tail::after {
  background: linear-gradient(90deg, #7CFFB2, #45D688) !important;
}

/* AntD Switch -- lime when on. */
.ant-switch-checked {
  background: linear-gradient(135deg, #7CFFB2, #2BAA66) !important;
}
.ant-switch-checked:hover {
  background: linear-gradient(135deg, #A3FFCB, #45D688) !important;
}

/* AntD Tag glow tweaks */
.ant-tag {
  border-radius: 999px !important;
  padding: 2px 10px !important;
  font-weight: 600 !important;
  border-width: 1px !important;
}

/* Staggered page entrance: children with [data-stagger] animate in
   sequence on initial mount. Combine with --stagger-index from JS. */
.mn-stagger > [data-stagger] {
  opacity: 0;
  animation: mn-rise 0.55s cubic-bezier(0.2, 0.7, 0.2, 1) both;
  animation-delay: calc(var(--stagger-index, 0) * 60ms + 80ms);
}
`;
