import {useMemo} from "react";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {useMowingMap} from "../hooks/useMowingMap.ts";
import {useFusionOdom} from "../hooks/useFusionOdom.ts";
import {useDiagnosticsSnapshot} from "../hooks/useDiagnosticsSnapshot.ts";
import {DashCard} from "./dashboard/Card.tsx";

/**
 * Top-down "live lawn" widget for the dashboard.
 *
 * Renders the recorded working areas as polygons, the dock as a pad, and the
 * robot's live pose as a pulsing dot with a heading arrow. Per-area coverage
 * is shown as a translucent fill inside each polygon (proportional to
 * coverage_percent from the diagnostics snapshot).
 *
 * This is the dashboard companion to the dedicated Map page -- compact,
 * read-only, and meant to make the dashboard feel alive without leaving home.
 */

interface Point2D {
    x: number;
    y: number;
}

function bboxFromPolygons(polys: Point2D[][], pad = 0.5): {x0: number; y0: number; x1: number; y1: number} {
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

const keyframes = `
@keyframes liveDotPulse {
  0% { transform: scale(1); opacity: 0.6; }
  100% { transform: scale(2.4); opacity: 0; }
}
@keyframes liveLawnShimmer {
  0%, 100% { opacity: 0.05; }
  50% { opacity: 0.13; }
}
@keyframes liveDotIdle {
  0%, 100% { transform: scale(1); opacity: 0.35; }
  50% { transform: scale(1.25); opacity: 0.55; }
}
`;

function prettifyAreaName(raw: string | undefined, index: number): string {
  if (!raw) return `Area ${index + 1}`;
  if (/^(recorded_)?area[_\s-]?\d+$/i.test(raw)) return `Area ${index + 1}`;
  return raw;
}

interface LiveMowgliWidgetProps {
    /** Render in a more compact layout (e.g. for mobile). */
    compact?: boolean;
    /** Whether the robot is currently moving -- drives the pulse intensity. */
    moving?: boolean;
    /** Optional explicit area index that is currently being mowed. */
    activeAreaIndex?: number;
}

export function LiveMowgliWidget({compact, moving, activeAreaIndex}: LiveMowgliWidgetProps) {
    const {colors} = useThemeMode();
    const map = useMowingMap();
    const odom = useFusionOdom();
    const {snapshot} = useDiagnosticsSnapshot();

    const polygons: {poly: Point2D[]; coverage: number; index: number; name?: string}[] = useMemo(() => {
        const areas = map.working_area ?? [];
        const coverageList = snapshot?.coverage ?? [];
        return areas.flatMap((area, i) => {
            const ring = area.area?.points ?? [];
            if (ring.length < 3) return [];
            const poly: Point2D[] = ring.map(p => ({x: p.x ?? 0, y: p.y ?? 0}));
            const covInfo = coverageList.find(c => c.area_index === i);
            return [{poly, coverage: covInfo?.coverage_percent ?? 0, index: i, name: area.name}];
        });
    }, [map, snapshot]);

    const dock: Point2D | null = (map.dock_x != null && map.dock_y != null)
        ? {x: map.dock_x, y: map.dock_y} : null;
    const dockHeading = map.dock_heading ?? 0;

    // Robot pose from /odometry/filtered_map
    const robot: Point2D | null = (() => {
        const p = odom?.pose?.pose?.position;
        if (!p || (p.x === 0 && p.y === 0)) return null;
        return {x: p.x, y: p.y};
    })();
    const ori = odom?.pose?.pose?.orientation;
    const robotYaw = ori
        ? Math.atan2(2 * (ori.w * ori.z + ori.x * ori.y), 1 - 2 * (ori.y * ori.y + ori.z * ori.z))
        : 0;

    // Build a bbox that includes all polygons + dock + robot so everything stays in frame.
    const allPoints = [...polygons.map(p => p.poly), ...(dock ? [[dock]] : []), ...(robot ? [[robot]] : [])];
    const bbox = bboxFromPolygons(allPoints, 1.5);
    const w = bbox.x1 - bbox.x0;
    const h = bbox.y1 - bbox.y0;
    const aspect = w / h;

    const svgW = compact ? 320 : 520;
    const svgH = compact ? 180 : 220;
    // Fit bbox into svg while preserving aspect; flip Y because SVG Y grows downward
    // but our map is in metres-east/metres-north (Y up).
    const scaleByW = svgW / w;
    const scaleByH = svgH / h;
    const scale = Math.min(scaleByW, scaleByH);
    const drawW = w * scale;
    const drawH = h * scale;
    const ox = (svgW - drawW) / 2;
    const oy = (svgH - drawH) / 2;

    const toX = (mx: number) => ox + (mx - bbox.x0) * scale;
    const toY = (my: number) => oy + (bbox.y1 - my) * scale;

    const hasData = polygons.length > 0 || dock != null || robot != null;

    return (
        <DashCard padding={compact ? 14 : 18} style={{
            background: `linear-gradient(135deg, ${colors.bgCard}, ${colors.bgElevated})`,
            position: 'relative', overflow: 'hidden',
        }}>
            <style>{keyframes}</style>

            <div style={{
                display: 'flex', alignItems: 'baseline', justifyContent: 'space-between',
                marginBottom: 10,
            }}>
                <div>
                    <div style={{
                        fontSize: 11, color: colors.textMuted, letterSpacing: '0.08em',
                        textTransform: 'uppercase' as const,
                    }}>
                        Lawn view
                    </div>
                    <div style={{fontSize: 13, color: colors.textDim, marginTop: 2}}>
                        {hasData
                            ? `${polygons.length} area${polygons.length === 1 ? '' : 's'} · ${robot ? 'robot tracked' : 'no live pose'}`
                            : 'Record an area on the Map to see your lawn here'}
                    </div>
                </div>
                {robot && (
                    <div style={{
                        fontSize: 10, color: colors.textMuted, fontFamily: 'monospace',
                    }}>
                        ({robot.x.toFixed(1)}, {robot.y.toFixed(1)}) · {((robotYaw * 180) / Math.PI).toFixed(0)}°
                    </div>
                )}
            </div>

            <svg
                viewBox={`0 0 ${svgW} ${svgH}`}
                width="100%"
                style={{display: 'block', maxHeight: svgH, aspectRatio: aspect.toFixed(2)}}
            >
                {/* subtle grid */}
                <defs>
                    <pattern id="lawnGrid" width={20} height={20} patternUnits="userSpaceOnUse">
                        <path d="M 20 0 L 0 0 0 20" fill="none" stroke={colors.borderSubtle} strokeWidth={0.5}/>
                    </pattern>
                    <radialGradient id="lawnVignette" cx="50%" cy="50%" r="60%">
                        <stop offset="0%" stopColor={colors.accent} stopOpacity={0}/>
                        <stop offset="100%" stopColor={colors.accent} stopOpacity={0.05}/>
                    </radialGradient>
                </defs>
                <rect width={svgW} height={svgH} fill="url(#lawnGrid)" opacity={0.5}/>
                <rect width={svgW} height={svgH} fill="url(#lawnVignette)"
                      style={{animation: moving ? 'liveLawnShimmer 3.2s ease-in-out infinite' : 'none'}}/>

                {/* lawn stripe pattern applied to polygons */}
                <defs>
                    <pattern id="lawnStripes" width={6} height={6} patternUnits="userSpaceOnUse"
                             patternTransform="rotate(45)">
                        <rect width={6} height={6} fill={`${colors.accent}10`}/>
                        <line x1={0} y1={0} x2={0} y2={6} stroke={`${colors.accent}22`} strokeWidth={1}/>
                    </pattern>
                </defs>

                {/* polygons */}
                {polygons.map(({poly, coverage, index, name}) => {
                    const isActive = index === activeAreaIndex;
                    const path = poly.map((p, i) => `${i === 0 ? 'M' : 'L'} ${toX(p.x)} ${toY(p.y)}`).join(' ') + ' Z';
                    const cx = poly.reduce((s, p) => s + toX(p.x), 0) / poly.length;
                    const cy = poly.reduce((s, p) => s + toY(p.y), 0) / poly.length;
                    const displayName = prettifyAreaName(name, index);
                    return (
                        <g key={index}>
                            <path d={path}
                                  fill="url(#lawnStripes)"
                                  stroke={isActive ? colors.accent : `${colors.accent}aa`}
                                  strokeWidth={isActive ? 2 : 1.4}/>
                            {isActive && (
                                <path d={path}
                                      fill={`${colors.accent}14`}
                                      stroke="none"/>
                            )}
                            <text x={cx} y={cy - 4} textAnchor="middle"
                                  fontSize={10} fontWeight={700}
                                  letterSpacing={0.5}
                                  fill={colors.text}>
                                {displayName.toUpperCase()}
                            </text>
                            {coverage > 0 ? (
                                <text x={cx} y={cy + 10} textAnchor="middle"
                                      fontSize={11} fontWeight={700}
                                      fill={colors.accent}>
                                    {coverage.toFixed(0)}%
                                </text>
                            ) : (
                                <text x={cx} y={cy + 10} textAnchor="middle"
                                      fontSize={9}
                                      fill={colors.textMuted}>
                                    not mowed yet
                                </text>
                            )}
                        </g>
                    );
                })}

                {/* dock */}
                {dock && (
                    <g transform={`translate(${toX(dock.x)} ${toY(dock.y)}) rotate(${-(dockHeading * 180) / Math.PI})`}>
                        <rect x={-9} y={-5} width={18} height={10} rx={2}
                              fill={colors.amberSoft} stroke={colors.amber} strokeWidth={1}/>
                        <text x={0} y={-8} textAnchor="middle" fontSize={8} fill={colors.amber} fontWeight={700}>
                            DOCK
                        </text>
                    </g>
                )}

                {/* robot */}
                {robot && (
                    <g transform={`translate(${toX(robot.x)} ${toY(robot.y)})`}>
                        {/* outer halo -- pulses while moving, gentle breath while idle */}
                        <circle r={moving ? 8 : 10}
                                fill={colors.accent}
                                style={{
                                    animation: moving
                                        ? 'liveDotPulse 1.8s ease-out infinite'
                                        : 'liveDotIdle 3.2s ease-in-out infinite',
                                    transformOrigin: '0 0',
                                }}/>
                        {/* heading wedge */}
                        <g transform={`rotate(${-(robotYaw * 180) / Math.PI - 90})`}>
                            <path d="M 0 -16 L -5 -3 L 5 -3 Z"
                                  fill={colors.accent} opacity={0.65}/>
                        </g>
                        <circle r={6} fill={colors.bgCard} stroke={colors.accent} strokeWidth={1.5}/>
                        <circle r={3} fill={colors.accent}/>
                    </g>
                )}

                {/* empty state hint */}
                {!hasData && (
                    <text x={svgW / 2} y={svgH / 2} textAnchor="middle"
                          fontSize={12} fill={colors.textMuted}>
                        Waiting for map data...
                    </text>
                )}
            </svg>
        </DashCard>
    );
}
