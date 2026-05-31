import {useState} from "react";
import {useThemeMode} from "../theme/ThemeContext.tsx";

/**
 * Top-down schematic of the robot. Each part is hoverable; hovering surfaces
 * the part's current value in a side panel. The schematic is stylised
 * (proportions are not to scale) — its job is to map the diagnostics page
 * onto something the operator can spatially reason about.
 */

export interface AnatomyInputs {
  batteryPct: number;
  vBattery: number;
  motorTempC: number;
  escTempC: number;
  gpsLabel: string;
  gpsOk: boolean;
  imuYawDeg: number;
  imuOk: boolean;
  lidarOk: boolean;
  wheelLeftRpm: number;
  wheelRightRpm: number;
  bladeOn: boolean;
  rain: boolean;
  dockCharging: boolean;
}

type Part = 'gps' | 'imu' | 'lidar' | 'battery' | 'blade' | 'wheelL' | 'wheelR' | 'motor' | 'dock' | 'rain';

interface PartInfo {
  label: string;
  value: string;
  ok: boolean;
}

function partInfo(part: Part, inputs: AnatomyInputs): PartInfo {
  switch (part) {
    case 'gps':
      return {label: 'GPS antenna', value: inputs.gpsLabel, ok: inputs.gpsOk};
    case 'imu':
      return {label: 'IMU', value: inputs.imuOk ? `yaw ${inputs.imuYawDeg.toFixed(0)}°` : 'no data', ok: inputs.imuOk};
    case 'lidar':
      return {label: 'LiDAR', value: inputs.lidarOk ? 'streaming' : 'no scan', ok: inputs.lidarOk};
    case 'battery':
      return {
        label: 'Battery',
        value: `${inputs.batteryPct.toFixed(0)}% · ${inputs.vBattery.toFixed(1)} V`,
        ok: inputs.batteryPct > 20,
      };
    case 'blade':
      return {label: 'Blade', value: inputs.bladeOn ? 'spinning' : 'off', ok: !inputs.bladeOn};
    case 'wheelL':
      return {label: 'Left wheel', value: `${inputs.wheelLeftRpm.toFixed(0)} rpm`, ok: true};
    case 'wheelR':
      return {label: 'Right wheel', value: `${inputs.wheelRightRpm.toFixed(0)} rpm`, ok: true};
    case 'motor':
      return {
        label: 'Motors',
        value: `motor ${inputs.motorTempC.toFixed(0)}°C · ESC ${inputs.escTempC.toFixed(0)}°C`,
        ok: inputs.motorTempC < 55,
      };
    case 'dock':
      return {label: 'Dock', value: inputs.dockCharging ? 'charging' : 'off-dock', ok: true};
    case 'rain':
      return {label: 'Rain sensor', value: inputs.rain ? 'wet' : 'dry', ok: !inputs.rain};
  }
}

interface RobotAnatomyProps {
  inputs: AnatomyInputs;
}

export function RobotAnatomy({inputs}: RobotAnatomyProps) {
  const {colors} = useThemeMode();
  const [hover, setHover] = useState<Part | null>(null);
  const active: Part = hover ?? 'battery';
  const info = partInfo(active, inputs);

  const partColor = (part: Part): string => {
    const {ok} = partInfo(part, inputs);
    return ok ? colors.accent : colors.amber;
  };

  const fill = (part: Part) => hover === part ? `${partColor(part)}55` : `${partColor(part)}22`;
  const stroke = (part: Part) => partColor(part);
  const sw = (part: Part) => hover === part ? 2 : 1;

  const handleEnter = (p: Part) => () => setHover(p);
  const handleLeave = () => setHover(null);

  return (
    <div className="mn-anatomy" style={{
      display: 'grid', gridTemplateColumns: '1fr minmax(200px, 240px)', gap: 14,
      background: colors.bgElevated, borderRadius: 12, padding: 14,
    }}>
      <style>{`
        @media (max-width: 640px) {
          .mn-anatomy { grid-template-columns: 1fr !important; }
        }
      `}</style>
      <div>
        <div style={{
          fontSize: 11, color: colors.textMuted, letterSpacing: '0.08em',
          textTransform: 'uppercase' as const, marginBottom: 10,
        }}>
          Robot anatomy
        </div>
        <svg viewBox="0 0 320 270" width="100%" style={{display: 'block', maxHeight: 290}}>
          {/* chassis */}
          <rect
            x={70} y={60} width={180} height={150} rx={28}
            fill={colors.bgCard} stroke={colors.border} strokeWidth={1.5}
          />

          {/* dock pad -- drawn behind chassis */}
          <rect
            x={130} y={238} width={60} height={18} rx={4}
            fill={fill('dock')} stroke={stroke('dock')} strokeWidth={sw('dock')}
            onMouseEnter={handleEnter('dock')} onMouseLeave={handleLeave}
            style={{cursor: 'pointer'}}
          />
          <text x={160} y={250} textAnchor="middle" fontSize={9} fill={colors.text}>Dock</text>

          {/* GPS antenna (top-center) */}
          <g onMouseEnter={handleEnter('gps')} onMouseLeave={handleLeave} style={{cursor: 'pointer'}}>
            <circle cx={160} cy={50} r={11} fill={fill('gps')} stroke={stroke('gps')} strokeWidth={sw('gps')}/>
            <line x1={160} y1={61} x2={160} y2={72} stroke={stroke('gps')} strokeWidth={1.5}/>
            <text x={160} y={36} textAnchor="middle" fontSize={9} fill={colors.text}>GPS</text>
          </g>

          {/* LiDAR (front) */}
          <g onMouseEnter={handleEnter('lidar')} onMouseLeave={handleLeave} style={{cursor: 'pointer'}}>
            <circle cx={160} cy={86} r={9} fill={fill('lidar')} stroke={stroke('lidar')} strokeWidth={sw('lidar')}/>
            <circle cx={160} cy={86} r={4} fill={stroke('lidar')} opacity={0.6}/>
            <text x={186} y={89} fontSize={9} fill={colors.textDim}>LiDAR</text>
          </g>

          {/* IMU (center) */}
          <g onMouseEnter={handleEnter('imu')} onMouseLeave={handleLeave} style={{cursor: 'pointer'}}>
            <rect x={150} y={110} width={20} height={14} rx={3}
                  fill={fill('imu')} stroke={stroke('imu')} strokeWidth={sw('imu')}/>
            <text x={186} y={120} fontSize={9} fill={colors.textDim}>IMU</text>
          </g>

          {/* Battery (rear) */}
          <g onMouseEnter={handleEnter('battery')} onMouseLeave={handleLeave} style={{cursor: 'pointer'}}>
            <rect x={120} y={150} width={80} height={26} rx={4}
                  fill={fill('battery')} stroke={stroke('battery')} strokeWidth={sw('battery')}/>
            <text x={160} y={166} textAnchor="middle" fontSize={9} fontWeight={600} fill={colors.text}>
              {inputs.batteryPct.toFixed(0)}%
            </text>
          </g>

          {/* Blade (center, below IMU) */}
          <g onMouseEnter={handleEnter('blade')} onMouseLeave={handleLeave} style={{cursor: 'pointer'}}>
            <circle cx={160} cy={195} r={18}
                    fill={fill('blade')} stroke={stroke('blade')} strokeWidth={sw('blade')}/>
            <g transform="translate(160 195)">
              <line x1={-12} y1={0} x2={12} y2={0} stroke={stroke('blade')} strokeWidth={2}/>
              <line x1={0} y1={-12} x2={0} y2={12} stroke={stroke('blade')} strokeWidth={2}/>
            </g>
            <text x={160} y={224} textAnchor="middle" fontSize={9} fill={colors.text}>Blade</text>
          </g>

          {/* Wheels */}
          <g onMouseEnter={handleEnter('wheelL')} onMouseLeave={handleLeave} style={{cursor: 'pointer'}}>
            <rect x={50} y={140} width={22} height={36} rx={5}
                  fill={fill('wheelL')} stroke={stroke('wheelL')} strokeWidth={sw('wheelL')}/>
            <text x={61} y={132} textAnchor="middle" fontSize={9} fill={colors.textDim}>L</text>
          </g>
          <g onMouseEnter={handleEnter('wheelR')} onMouseLeave={handleLeave} style={{cursor: 'pointer'}}>
            <rect x={248} y={140} width={22} height={36} rx={5}
                  fill={fill('wheelR')} stroke={stroke('wheelR')} strokeWidth={sw('wheelR')}/>
            <text x={259} y={132} textAnchor="middle" fontSize={9} fill={colors.textDim}>R</text>
          </g>

          {/* Drive motors */}
          <g onMouseEnter={handleEnter('motor')} onMouseLeave={handleLeave} style={{cursor: 'pointer'}}>
            <circle cx={90} cy={120} r={7} fill={fill('motor')} stroke={stroke('motor')} strokeWidth={sw('motor')}/>
            <circle cx={230} cy={120} r={7} fill={fill('motor')} stroke={stroke('motor')} strokeWidth={sw('motor')}/>
          </g>

          {/* Rain sensor */}
          <g onMouseEnter={handleEnter('rain')} onMouseLeave={handleLeave} style={{cursor: 'pointer'}}>
            <path d="M 105 84 q 4 -6 8 0 q 4 -6 8 0 q 4 -6 8 0"
                  fill="none" stroke={stroke('rain')} strokeWidth={sw('rain') + 0.5} strokeLinecap="round"/>
            <text x={117} y={75} textAnchor="middle" fontSize={9} fill={colors.textDim}>Rain</text>
          </g>

          {/* heading arrow (front) */}
          <path d="M 152 70 L 160 60 L 168 70 Z" fill={colors.accent} opacity={0.5}/>
        </svg>
      </div>

      <div style={{
        display: 'flex', flexDirection: 'column',
        background: colors.bgCard, borderRadius: 10, padding: '12px 14px',
      }}>
        <div style={{fontSize: 10, color: colors.textMuted, letterSpacing: '0.06em', textTransform: 'uppercase'}}>
          {hover ? 'Inspecting' : 'Hover a part'}
        </div>
        <div className="mn-display" style={{fontSize: 24, color: colors.text, marginTop: 4, lineHeight: 1.1}}>
          {info.label}
        </div>
        <div className="mn-num" style={{
          fontSize: 20, color: info.ok ? colors.accent : colors.amber,
          marginTop: 8, lineHeight: 1,
        }}>
          {info.value}
        </div>
        <div style={{
          marginTop: 'auto', paddingTop: 10, fontSize: 11, color: colors.textMuted,
        }}>
          Green = healthy, amber = needs attention.
        </div>
      </div>
    </div>
  );
}
