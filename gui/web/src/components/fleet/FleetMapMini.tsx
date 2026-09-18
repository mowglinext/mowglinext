import {useThemeMode} from "../../theme/ThemeContext.tsx";
import type {FleetPhase} from "../../utils/fleet.ts";

export interface FleetMapDot {
    id: string;
    name: string;
    /** Unit-square position (0..1), y down. */
    x: number;
    y: number;
    phase: FleetPhase;
    self: boolean;
}

interface FleetMapMiniProps {
    dots: FleetMapDot[];
    height?: number;
    emptyLabel: string;
}

const PHASE_COLOR: Record<FleetPhase, string> = {
    offline: "rgba(236,255,244,0.28)",
    idle: "rgba(236,255,244,0.7)",
    charging: "#45D6E8",
    mowing: "#7CFFB2",
    returning: "#FFD166",
    recording: "#C792EA",
    manual: "#FFD166",
    emergency: "#FF6B7A",
};

/**
 * Fleet-scale position sketch: every robot with a GPS fix as a labelled dot
 * in one shared local frame (north up). Deliberately map-free — datums can
 * differ between robots, so nothing here claims to be geo-registered.
 */
export function FleetMapMini({dots, height = 220, emptyLabel}: FleetMapMiniProps) {
    const {colors} = useThemeMode();
    const w = 600;
    const h = height * (w / 600);
    const toX = (x: number) => x * w;
    const toY = (y: number) => y * h;
    return (
        <svg
            viewBox={`0 0 ${w} ${h}`}
            width="100%"
            height={height}
            role="img"
            aria-label="fleet positions"
            style={{display: "block", background: "rgba(0,0,0,0.12)", borderRadius: 12}}
        >
            {/* contour texture */}
            {[0.25, 0.5, 0.75].map(f => (
                <g key={f} stroke="rgba(236,255,244,0.06)" fill="none">
                    <line x1={0} y1={toY(f)} x2={w} y2={toY(f)}/>
                    <line x1={toX(f)} y1={0} x2={toX(f)} y2={h}/>
                </g>
            ))}
            <text x={w - 14} y={18} fontSize={11} textAnchor="end" fill="rgba(236,255,244,0.42)">N ↑</text>
            {dots.length === 0 && (
                <text x={w / 2} y={h / 2} fontSize={14} textAnchor="middle" fill="rgba(236,255,244,0.5)">
                    {emptyLabel}
                </text>
            )}
            {dots.map(d => {
                const cx = toX(d.x), cy = toY(d.y);
                const color = PHASE_COLOR[d.phase];
                return (
                    <g key={d.id} data-testid={`fleet-dot-${d.id}`}>
                        {d.phase === "mowing" && (
                            <circle cx={cx} cy={cy} r={16} fill={color} opacity={0.18}/>
                        )}
                        <circle
                            cx={cx} cy={cy} r={d.self ? 7 : 6}
                            fill={color}
                            stroke={d.self ? colors.primary : "rgba(0,0,0,0.35)"}
                            strokeWidth={d.self ? 2.5 : 1}
                        />
                        <text
                            x={cx} y={cy - 12} fontSize={12} textAnchor="middle"
                            fill="var(--ink, #ECFFF4)" fontWeight={d.self ? 700 : 500}
                        >
                            {d.name}
                        </text>
                    </g>
                );
            })}
        </svg>
    );
}
