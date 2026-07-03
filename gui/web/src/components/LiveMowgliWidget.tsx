import {useMemo} from "react";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {useMowingMap} from "../hooks/useMowingMap.ts";
import {useFusionOdom} from "../hooks/useFusionOdom.ts";
import {useDiagnosticsSnapshot} from "../hooks/useDiagnosticsSnapshot.ts";
import {DashCard} from "./dashboard/Card.tsx";

/**
 * Dashboard "lawn view".
 *
 * One clean canvas: quiet polygon, soft inner glow, a single focal robot
 * dot with a Gaussian-blur halo. Labels live in the header strip above the
 * canvas, not inside it -- the visualisation breathes, the typography
 * carries the data.
 */

interface Point2D {
    x: number;
    y: number;
}

interface AreaInfo {
    poly: Point2D[];
    coverage: number;
    index: number;
    name?: string;
    totalCells: number;
}

function bboxFromPolygons(polys: Point2D[][], pad = 0.8): {x0: number; y0: number; x1: number; y1: number} {
    if (polys.length === 0) return {x0: -10, y0: -10, x1: 10, y1: 10};
    let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
    polys.forEach(poly => poly.forEach(p => {
        if (p.x < x0) x0 = p.x;
        if (p.y < y0) y0 = p.y;
        if (p.x > x1) x1 = p.x;
        if (p.y > y1) y1 = p.y;
    }));
    return {x0: x0 - pad, y0: y0 - pad, x1: x1 + pad, y1: y1 + pad};
}

function prettifyAreaName(raw: string | undefined, index: number): string {
    if (!raw) return `Area ${index + 1}`;
    if (/^(recorded_)?area[_\s-]?\d+$/i.test(raw)) return `Area ${index + 1}`;
    return raw;
}

const keyframes = `
@keyframes mnLawnPulse {
  0%, 100% { transform: scale(1); opacity: 0.45; }
  50% { transform: scale(1.35); opacity: 0; }
}
@keyframes mnLawnBreath {
  0%, 100% { opacity: 0.32; }
  50% { opacity: 0.58; }
}
`;

interface LiveMowgliWidgetProps {
    compact?: boolean;
    moving?: boolean;
    activeAreaIndex?: number;
}

export function LiveMowgliWidget({compact, moving, activeAreaIndex}: LiveMowgliWidgetProps) {
    const {colors} = useThemeMode();
    const map = useMowingMap();
    const odom = useFusionOdom();
    const {snapshot} = useDiagnosticsSnapshot();

    const polygons: AreaInfo[] = useMemo(() => {
        const areas = map.working_area ?? [];
        const coverageList = snapshot?.coverage ?? [];
        return areas.flatMap((area, i) => {
            const ring = area.area?.points ?? [];
            if (ring.length < 3) return [];
            const poly: Point2D[] = ring.map(p => ({x: p.x ?? 0, y: p.y ?? 0}));
            const cov = coverageList.find(c => c.area_index === i);
            return [{
                poly,
                coverage: cov?.coverage_percent ?? 0,
                totalCells: cov?.total_cells ?? 0,
                index: i,
                name: area.name,
            }];
        });
    }, [map, snapshot]);

    const dock: Point2D | null = (map.dock_x != null && map.dock_y != null)
        ? {x: map.dock_x, y: map.dock_y} : null;

    const robot: Point2D | null = (() => {
        const p = odom?.pose?.pose?.position;
        if (!p || (p.x === 0 && p.y === 0)) return null;
        return {x: p.x, y: p.y};
    })();
    const ori = odom?.pose?.pose?.orientation;
    const robotYaw = ori
        ? Math.atan2(2 * (ori.w * ori.z + ori.x * ori.y), 1 - 2 * (ori.y * ori.y + ori.z * ori.z))
        : 0;

    const allPoints = [
        ...polygons.map(p => p.poly),
        ...(dock ? [[dock]] : []),
        ...(robot ? [[robot]] : []),
    ];
    const bbox = bboxFromPolygons(allPoints, 1.5);
    const w = bbox.x1 - bbox.x0;
    const h = bbox.y1 - bbox.y0;

    const svgW = compact ? 360 : 720;
    const svgH = compact ? 200 : 280;

    const scale = Math.min(svgW / w, svgH / h);
    const drawW = w * scale;
    const drawH = h * scale;
    const ox = (svgW - drawW) / 2;
    const oy = (svgH - drawH) / 2;
    const toX = (mx: number) => ox + (mx - bbox.x0) * scale;
    const toY = (my: number) => oy + (bbox.y1 - my) * scale;

    const hasData = polygons.length > 0 || dock != null || robot != null;
    const headlineArea = polygons.find(p => p.index === activeAreaIndex) ?? polygons[0];
    const avgCoverage = polygons.length > 0
        ? polygons.reduce((s, p) => s + (p.totalCells > 0 ? p.coverage : 0), 0) /
          Math.max(1, polygons.filter(p => p.totalCells > 0).length)
        : 0;

    const robotLabel = !robot
        ? 'No live pose'
        : moving
            ? 'Mowgli is moving'
            : 'Mowgli is parked';

    return (
        <DashCard padding={0} style={{overflow: 'hidden'}}>
            <style>{keyframes}</style>

            {/* Header strip -- everything labelled in plain language */}
            <div style={{
                display: 'flex', alignItems: 'baseline', justifyContent: 'space-between',
                gap: 16, padding: compact ? '14px 16px 10px' : '18px 22px 14px',
                borderBottom: `1px solid ${colors.borderSubtle}`,
            }}>
                <div style={{minWidth: 0, flex: 1}}>
                    <div style={{
                        fontSize: 11, color: colors.textMuted, letterSpacing: '0.08em',
                        textTransform: 'uppercase' as const, fontWeight: 600,
                    }}>
                        Lawn view
                    </div>
                    <div className="mn-display" style={{
                        fontSize: compact ? 22 : 28, color: colors.text,
                        marginTop: 4, lineHeight: 1.1,
                    }}>
                        {hasData
                            ? <>{robotLabel}{headlineArea ? <> · <em>{prettifyAreaName(headlineArea.name, headlineArea.index)}</em></> : null}</>
                            : 'No areas recorded yet'}
                    </div>
                </div>

                {polygons.length > 0 && (
                    <div style={{textAlign: 'right'}}>
                        <div style={{
                            fontSize: 11, color: colors.textMuted, letterSpacing: '0.08em',
                            textTransform: 'uppercase' as const, fontWeight: 600,
                        }}>
                            Coverage
                        </div>
                        <div className="mn-num" style={{
                            fontSize: compact ? 24 : 32, color: colors.accent,
                            marginTop: 4, lineHeight: 1,
                        }}>
                            {avgCoverage.toFixed(0)}<span style={{
                                fontSize: 12, fontFamily: "'Space Grotesk', monospace",
                                color: colors.textMuted, marginLeft: 2,
                            }}>%</span>
                        </div>
                    </div>
                )}
            </div>

            {/* Canvas */}
            <div style={{
                padding: compact ? '12px 12px 14px' : '16px 22px 22px',
                background: `radial-gradient(circle at 50% 40%, ${colors.accent}05 0%, transparent 65%)`,
            }}>
                <svg
                    viewBox={`0 0 ${svgW} ${svgH}`}
                    width="100%"
                    style={{display: 'block', maxHeight: svgH}}
                    preserveAspectRatio="xMidYMid meet"
                >
                    <defs>
                        <filter id="mnLawnGlow" x="-50%" y="-50%" width="200%" height="200%">
                            <feGaussianBlur stdDeviation={5} result="blur"/>
                            <feMerge>
                                <feMergeNode in="blur"/>
                                <feMergeNode in="SourceGraphic"/>
                            </feMerge>
                        </filter>
                        <radialGradient id="mnLawnFill" cx="50%" cy="50%" r="50%">
                            <stop offset="0%" stopColor={colors.accent} stopOpacity={0.15}/>
                            <stop offset="100%" stopColor={colors.accent} stopOpacity={0.06}/>
                        </radialGradient>
                    </defs>

                    {/* polygons -- quiet, no patterns, no in-canvas labels */}
                    {polygons.map(({poly, index}) => {
                        const isActive = index === activeAreaIndex;
                        const path = poly.map((p, i) => `${i === 0 ? 'M' : 'L'} ${toX(p.x)} ${toY(p.y)}`).join(' ') + ' Z';
                        return (
                            <g key={index}>
                                <path d={path}
                                      fill="url(#mnLawnFill)"
                                      stroke={isActive ? colors.accent : `${colors.accent}88`}
                                      strokeWidth={isActive ? 2 : 1.5}
                                      strokeLinejoin="round"/>
                            </g>
                        );
                    })}

                    {/* dock -- minimal house glyph */}
                    {dock && (
                        <g transform={`translate(${toX(dock.x)} ${toY(dock.y)})`}>
                            <circle r={5} fill={colors.bgCard} stroke={colors.amber} strokeWidth={1.5}/>
                            <circle r={2} fill={colors.amber}/>
                        </g>
                    )}

                    {/* robot -- the focal point */}
                    {robot && (
                        <g transform={`translate(${toX(robot.x)} ${toY(robot.y)})`}>
                            {/* soft halo via Gaussian blur */}
                            <circle r={moving ? 9 : 7}
                                    fill={colors.accent}
                                    filter="url(#mnLawnGlow)"
                                    style={{
                                        transformOrigin: 'center',
                                        animation: moving
                                            ? 'mnLawnPulse 1.8s ease-out infinite'
                                            : 'mnLawnBreath 3.6s ease-in-out infinite',
                                    }}/>
                            {/* heading wedge -- ENU yaw, canvas flips Y so
                                up == north. rotate(90 - yawDeg) maps
                                heading=0 (east) -> +90 -> right and
                                heading=90 (north) -> 0 -> up. */}
                            <g transform={`rotate(${90 - (robotYaw * 180) / Math.PI})`}>
                                <path d="M 0 -18 L -4.5 -6 L 4.5 -6 Z"
                                      fill={colors.accent} opacity={0.85}/>
                            </g>
                            {/* core dot -- crisp on top of the glow */}
                            <circle r={7} fill={colors.bgCard}/>
                            <circle r={5} fill={colors.accent}/>
                        </g>
                    )}

                    {!hasData && (
                        <text x={svgW / 2} y={svgH / 2} textAnchor="middle"
                              fontSize={13} fill={colors.textMuted}>
                            Record an area on the Map to see your lawn here
                        </text>
                    )}
                </svg>

                {/* Footer chips -- discrete legend that only renders when relevant */}
                {hasData && (
                    <div style={{
                        display: 'flex', alignItems: 'center', justifyContent: 'center',
                        gap: 16, marginTop: compact ? 10 : 14,
                        fontSize: 11, color: colors.textMuted, fontWeight: 500,
                    }}>
                        {robot && (
                            <span style={{display: 'inline-flex', alignItems: 'center', gap: 6}}>
                                <span style={{
                                    width: 8, height: 8, borderRadius: 4,
                                    background: colors.accent,
                                    boxShadow: `0 0 6px ${colors.accent}`,
                                }}/>
                                Robot
                            </span>
                        )}
                        {dock && (
                            <span style={{display: 'inline-flex', alignItems: 'center', gap: 6}}>
                                <span style={{
                                    width: 8, height: 8, borderRadius: 4,
                                    border: `1.5px solid ${colors.amber}`,
                                }}/>
                                Dock
                            </span>
                        )}
                        {polygons.length > 0 && (
                            <span style={{display: 'inline-flex', alignItems: 'center', gap: 6}}>
                                <span style={{
                                    width: 14, height: 8, borderRadius: 2,
                                    background: `${colors.accent}1f`,
                                    border: `1px solid ${colors.accent}66`,
                                }}/>
                                {polygons.length === 1 ? 'Area' : `${polygons.length} areas`}
                            </span>
                        )}
                    </div>
                )}
            </div>
        </DashCard>
    );
}
