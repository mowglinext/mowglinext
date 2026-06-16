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
  IDLE_DOCKED:                { label: 'À la base',          tone: 'info',    friendly: 'Au repos sur la base' },
  IDLE:                       { label: 'Au repos',           tone: 'info',    friendly: 'Prêt quand vous voulez' },
  CHARGING:                   { label: 'En charge',          tone: 'success', friendly: 'Recharge de la batterie' },

  // Autonomous mowing cycle
  PREFLIGHT_CHECK:            { label: 'Pré-vol',            tone: 'info',    friendly: 'Vérifications avant départ' },
  UNDOCKING:                  { label: 'Départ base',        tone: 'primary', friendly: 'Quitte la base' },
  CALIBRATING_HEADING:        { label: 'Calibration cap',    tone: 'info',    friendly: 'Calibration du cap' },
  MOWING:                     { label: 'Tonte',              tone: 'primary', friendly: 'Tonte en cours' },
  TRANSIT:                    { label: 'Transit',            tone: 'primary', friendly: 'Trajet vers la bande suivante' },
  SKIP_STRIP:                 { label: 'Bande ignorée',      tone: 'info',    friendly: 'Bande inaccessible ignorée' },
  RETURNING_HOME:             { label: 'Retour base',        tone: 'primary', friendly: 'Retour vers la base' },
  MOWING_COMPLETE:            { label: 'Tonte terminée',     tone: 'success', friendly: 'Toutes les zones tondues' },

  // Recording
  RECORDING:                  { label: 'Enregistrement',     tone: 'primary', friendly: 'Enregistrement de la limite' },
  RECORDING_COMPLETE:         { label: 'Enregistré',         tone: 'success', friendly: 'Limite de zone enregistrée' },

  // Manual
  MANUAL_MOWING:              { label: 'Tonte manuelle',     tone: 'primary', friendly: 'Mode tonte manuelle' },

  // Battery
  LOW_BATTERY_DOCKING:        { label: 'Batterie faible',    tone: 'warning', friendly: 'Batterie faible — retour à la base' },
  CRITICAL_BATTERY_DOCKING:   { label: 'Batterie critique',  tone: 'warning', friendly: 'Batterie critique — retour à la base' },
  CRITICAL_BATTERY_NAV_FAILED:{ label: 'Échec retour',       tone: 'danger',  friendly: 'Batterie critique et retour impossible' },

  // Rain
  RAIN_DETECTED_DOCKING:      { label: 'Pluie détectée',     tone: 'warning', friendly: 'Pluie détectée — retour à la base' },
  RAIN_WAITING:               { label: 'Attente du sec',     tone: 'warning', friendly: "Attend la fin de la pluie" },
  RAIN_TIMEOUT:               { label: 'Délai pluie',        tone: 'warning', friendly: "Abandon de l'attente du temps sec" },
  RESUMING_AFTER_RAIN:        { label: 'Reprise',            tone: 'primary', friendly: 'Reprise après la pluie' },

  // Recovery / transitions
  RESUMING_UNDOCKING:         { label: 'Reprise départ',     tone: 'primary', friendly: 'Quitte la base pour reprendre' },
  BOUNDARY_RECOVERY:          { label: 'Récup. limite',      tone: 'warning', friendly: 'Récupération après un écart de limite' },

  // Failures / emergencies
  EMERGENCY:                  { label: "Arrêt d'urgence",    tone: 'danger',  friendly: "Arrêt d'urgence activé" },
  BOUNDARY_EMERGENCY_STOP:    { label: 'Alerte limite',      tone: 'danger',  friendly: 'Arrêt — hors limites' },
  UNDOCK_FAILED:              { label: 'Échec départ',       tone: 'warning', friendly: 'Échec du départ — vérifier la base' },
  CHARGER_FAILED:             { label: 'Échec chargeur',     tone: 'warning', friendly: 'Chargeur non détecté' },
  NAV_TO_DOCK_FAILED:         { label: 'Échec navigation',   tone: 'danger',  friendly: 'Navigation vers la base impossible' },
  COVERAGE_FAILED_DOCKING:    { label: 'Échec couverture',   tone: 'warning', friendly: 'Échec couverture — retour à la base' },
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
  background: var(--grad-edge);
  -webkit-mask: linear-gradient(#000 0 0) content-box, linear-gradient(#000 0 0);
          mask: linear-gradient(#000 0 0) content-box, linear-gradient(#000 0 0);
  -webkit-mask-composite: xor; mask-composite: exclude;
  pointer-events: none;
}

/* ─── AntD Card → liquid glass ──────────────────────────────────────────
   Pull every stock AntD Card into the tech-garden glass language so pages
   built on AntD (Diagnostics, Settings, Onboarding) match the hand-built
   DashCard/GlassCard surfaces without per-page edits. Same recipe as
   DashCard: translucent dark fill, backdrop blur, soft border, luminous
   top-left edge. */
.ant-card {
  position: relative;
  background: rgba(11, 24, 20, 0.6) !important;
  backdrop-filter: blur(24px) saturate(140%);
  -webkit-backdrop-filter: blur(24px) saturate(140%);
  border: 1px solid var(--border-soft) !important;
  border-radius: var(--radius-md) !important;
  box-shadow: 0 24px 60px -20px rgba(0,0,0,0.5), 0 4px 16px -4px rgba(0,0,0,0.3), inset 0 1px 0 rgba(255,255,255,0.04) !important;
  overflow: hidden;
}
.ant-card::before {
  content: '';
  position: absolute; inset: 0;
  padding: 1px;
  border-radius: inherit;
  background: var(--grad-edge);
  -webkit-mask: linear-gradient(#000 0 0) content-box, linear-gradient(#000 0 0);
          mask: linear-gradient(#000 0 0) content-box, linear-gradient(#000 0 0);
  -webkit-mask-composite: xor; mask-composite: exclude;
  pointer-events: none;
  z-index: 0;
}
.ant-card-head { border-bottom: 1px solid var(--border-soft) !important; background: transparent !important; }
.ant-card-body, .ant-card-head { position: relative; z-index: 1; }
/* Nested cards (a card inside a card) shouldn't double up the blur+edge —
   flatten the inner one to a plain translucent tile. */
.ant-card .ant-card {
  backdrop-filter: none;
  -webkit-backdrop-filter: none;
  background: var(--bg-elevated) !important;
  box-shadow: none !important;
}
.ant-card .ant-card::before { display: none; }

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
  background: var(--grad-primary) !important;
  border: none !important;
  color: var(--bg-deep) !important;
  font-weight: 700 !important;
  box-shadow: 0 8px 24px -10px rgba(124, 255, 178, 0.55), inset 0 1px 0 rgba(255, 255, 255, 0.35) !important;
  transition: transform 0.12s ease, box-shadow 0.12s ease !important;
}
.ant-btn-primary:hover,
.ant-btn-primary:focus {
  background: linear-gradient(135deg, var(--lime-bright) 0%, var(--lime) 55%, var(--mint) 100%) !important;
  box-shadow: 0 10px 28px -8px rgba(124, 255, 178, 0.7), inset 0 1px 0 rgba(255, 255, 255, 0.42) !important;
  transform: translateY(-1px);
  color: var(--bg-deep) !important;
}
.ant-btn-primary:active { transform: translateY(0); }

.ant-btn-default {
  background: var(--bg-card) !important;
  border: 1px solid var(--border-soft) !important;
  color: var(--ink) !important;
  backdrop-filter: blur(12px);
  font-weight: 600 !important;
}
.ant-btn-default:hover,
.ant-btn-default:focus {
  background: var(--bg-elevated) !important;
  border-color: var(--border-glow) !important;
  color: var(--ink) !important;
}

.ant-btn-dangerous {
  background: rgba(255, 107, 122, 0.12) !important;
  border: 1px solid rgba(255, 107, 122, 0.4) !important;
  color: var(--rose) !important;
  font-weight: 600 !important;
}
.ant-btn-dangerous:hover,
.ant-btn-dangerous:focus {
  background: rgba(255, 107, 122, 0.18) !important;
  border-color: rgba(255, 107, 122, 0.6) !important;
  color: var(--rose) !important;
}

/* AntD Steps -- pull the stepper into the concept palette. */
.ant-steps-item-process .ant-steps-item-icon {
  background: linear-gradient(135deg, var(--lime), var(--emerald)) !important;
  border-color: transparent !important;
  box-shadow: 0 8px 22px -8px rgba(124, 255, 178, 0.5);
}
.ant-steps-item-process .ant-steps-item-icon > .ant-steps-icon { color: var(--bg-deep) !important; }
.ant-steps-item-finish .ant-steps-item-icon {
  background: rgba(124, 255, 178, 0.16) !important;
  border-color: rgba(124, 255, 178, 0.5) !important;
}
.ant-steps-item-finish .ant-steps-item-icon > .ant-steps-icon { color: var(--lime) !important; }
.ant-steps-item-wait .ant-steps-item-icon {
  background: rgba(255, 255, 255, 0.03) !important;
  border-color: var(--border-sharp) !important;
}
.ant-steps-item-title { color: var(--ink) !important; }
.ant-steps-item-process > .ant-steps-item-container > .ant-steps-item-content > .ant-steps-item-title {
  color: var(--lime) !important;
}
.ant-steps-item-tail::after { background: rgba(236, 255, 244, 0.10) !important; }
.ant-steps-item-finish > .ant-steps-item-container > .ant-steps-item-tail::after {
  background: linear-gradient(90deg, var(--lime), var(--mint)) !important;
}

/* AntD Switch -- lime when on. */
.ant-switch-checked {
  background: linear-gradient(135deg, var(--lime), var(--emerald)) !important;
}
.ant-switch-checked:hover {
  background: linear-gradient(135deg, var(--lime-bright), var(--mint)) !important;
}

/* AntD Tag -- pill shape + brand palette (override AntD preset swatches so
   success/error/warning/processing tags read lime/rose/amber/cyan). */
.ant-tag {
  border-radius: 999px !important;
  padding: 2px 10px !important;
  font-weight: 600 !important;
  border-width: 1px !important;
}
.ant-tag-success, .ant-tag-green {
  color: var(--lime) !important;
  background: rgba(124, 255, 178, 0.12) !important;
  border-color: rgba(124, 255, 178, 0.32) !important;
}
.ant-tag-error, .ant-tag-red {
  color: var(--rose) !important;
  background: rgba(255, 107, 122, 0.12) !important;
  border-color: rgba(255, 107, 122, 0.34) !important;
}
.ant-tag-warning, .ant-tag-orange, .ant-tag-gold {
  color: var(--amber) !important;
  background: rgba(243, 168, 92, 0.12) !important;
  border-color: rgba(243, 168, 92, 0.34) !important;
}
.ant-tag-processing, .ant-tag-blue, .ant-tag-cyan {
  color: var(--aurora-cyan) !important;
  background: rgba(69, 214, 232, 0.12) !important;
  border-color: rgba(69, 214, 232, 0.34) !important;
}
/* AntD Progress -- brand the default track + stroke. */
.ant-progress-bg { background: var(--grad-primary) !important; }
.ant-progress-inner { background: var(--border-soft) !important; }

/* Staggered page entrance: children with [data-stagger] animate in
   sequence on initial mount. Combine with --stagger-index from JS. */
.mn-stagger > [data-stagger] {
  opacity: 0;
  animation: mn-rise 0.55s cubic-bezier(0.2, 0.7, 0.2, 1) both;
  animation-delay: calc(var(--stagger-index, 0) * 60ms + 80ms);
}
`;
