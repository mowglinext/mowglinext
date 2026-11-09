import {ArtworkViewport} from "./ArtworkViewport";
import {artworkUrl} from "./artworkUrl";
import {usePaintedShell} from "./usePaintedShell";
import {PartSprite} from "./PartSprite";
import {sensorArtworkProjection} from "./sensorArtworkProjection";
import {SHELLS, shellProjectionFrame} from "./shellAssets";
import {memo, useState} from "react";
import type {RobotGeometry} from "../../hooks/useRobotDescription";
import type {MowerStyle} from "../../hooks/useMowerVisual";
import {sensorCorners, type RobotView, type SensorGeometry} from "../../utils/robotModel";

function ShellSprite({style, view, x, y, width, height, artwork, colour}: {
    style: MowerStyle; view: RobotView; x: number; y: number; width: number; height: number; artwork:"full"|"map"; colour?: string;
}) {
    const [loaded, setLoaded] = useState(false);
    const shell = SHELLS[style];
    const [sx, sy, sw, sh] = shellProjectionFrame(style, view);
    const sourceSize = shell.sizes?.[view] ?? shell.size;
    const painted = usePaintedShell(style, view, artwork, colour);
    const cropped = artwork === "map" || !!painted;
    return <>
        {!loaded && <rect x={x} y={y} width={width} height={height} rx={Math.min(width,height)*.18} fill="#343e40"/>}
        <ArtworkViewport data-layer="shell-art" x={x} y={y} width={width} height={height}
        data-paint-colour={colour || "original"} data-paint-ready={!!painted}
        crop={[sx,sy,sw,sh]}>
        <rect data-shell-bounds x={sx} y={sy} width={sw} height={sh} fill="none"/>
        <image onLoad={()=>setLoaded(true)} onError={()=>setLoaded(false)} href={painted ?? artworkUrl(artwork === "map" ? `map/${style}-${view}.webp` : shell.files?.[view] ?? `${style}.png`)}
            x={cropped ? sx : 0} y={cropped ? sy : 0}
            width={cropped ? sw : sourceSize[0]} height={cropped ? sh : sourceSize[1]}/>
    </ArtworkViewport></>;
}

/** Sensor visuals use real URDF sizes. Box faces are projected after all three
 * mounting rotations, so IMU roll/pitch are not lost in the side view. */
export function SensorGraphic({sensor, view, highlight = false, artwork = "full"}: {sensor: SensorGeometry; view: RobotView; highlight?: boolean; artwork?:"full"|"map"}) {
    const projection = sensorArtworkProjection(sensor,view);
    const points = sensorCorners(sensor,view);
    const xs=points.map(p=>p[0]),ys=points.map(p=>p[1]);
    return <g data-sensor={sensor.id} data-x={sensor.x} data-y={sensor.y} data-z={sensor.z}>
        <g transform={projection.transform}>
            {/* Two translucent strokes give a soft edge without SVG filters:
                WebKit can drop the entire sensor raster in the filter path. */}
            {highlight && <g data-sensor-highlight fill="none" stroke="#9af4d0" pointerEvents="none">
                {[{width:.004, opacity:.18}, {width:.0012, opacity:.85}].map(edge =>
                    sensor.id === "lidar" && projection.view === "top"
                        ? <ellipse key={edge.width} rx={projection.width/2} ry={projection.height/2}
                            strokeWidth={edge.width} opacity={edge.opacity}/>
                        : <rect key={edge.width} x={-projection.width/2} y={-projection.height/2}
                            width={projection.width} height={projection.height} rx={Math.min(.003,projection.height/4)}
                            strokeWidth={edge.width} opacity={edge.opacity}/>) }
            </g>}
            <PartSprite artwork={artwork} part={sensor.id} view={projection.view} width={projection.width} height={projection.height}
                fallback={<rect x={-projection.width/2} y={-projection.height/2} width={projection.width} height={projection.height}
                    rx={.003} fill={sensor.id === "imu" ? "#326655" : "#7c9187"}/>}/>
        </g>
        {/* Retain the projected extent for hit testing and inspection without
            changing the part's physical dimensions to match a minimum UI size. */}
        <rect data-sensor-bounds x={Math.min(...xs)} y={Math.min(...ys)} width={Math.max(...xs)-Math.min(...xs)}
            height={Math.max(...ys)-Math.min(...ys)} fill="none"/>
    </g>;
}

function Wheel({x, y, radius, width, view, caster = false, artwork}: {x: number; y: number; radius: number; width: number; view: RobotView; caster?: boolean; artwork:"full"|"map"}) {
    return <g transform={`translate(${x} ${y})${caster && view === "top" ? " rotate(180)" : ""}`}>
        <PartSprite artwork={artwork} part={caster ? "caster" : "drive-wheel"} view={view} width={view === "top" ? width : radius*2} height={radius*2}
            fallback={view === "top" ? <rect x={-width/2} y={-radius} width={width} height={radius*2} rx={width*.15} fill="#293332"/>
                : <circle r={radius} fill="#293332" stroke="#6b867a" strokeWidth={.002}/>}/>
    </g>;
}

// Pose updates move the map wrapper; unchanged geometry must not rebuild artwork.
/** Map mode omits layout details entirely, avoiding hidden underbody artwork downloads. */
export const LayeredMower = memo(function LayeredMower({robot: r, style, colour, transparent, view = "top", sensors = true, highlightSensors = false, layoutDetails = true, artwork = "full"}: {
    robot: RobotGeometry; style: MowerStyle; colour?: string; transparent: boolean; view?: RobotView; sensors?: boolean; highlightSensors?: boolean; layoutDetails?: boolean; artwork?:"full"|"map";
}) {
    const bodyZ = r.chassisCenterZ ?? r.baseHeight/2;
    const bladeX = r.bladeX ?? r.chassisCenterX;
    const wheels = view === "top" ? [-1,1].map(sign => <Wheel artwork={artwork} key={sign} view={view}
        x={sign*r.wheelTrack/2} y={-r.wheelXOffset} radius={r.wheelRadius} width={r.wheelWidth}/>)
        : <Wheel artwork={artwork} view={view} x={-r.wheelXOffset} y={-(r.wheelZ ?? 0)} radius={r.wheelRadius} width={r.wheelWidth}/>;
    // Layering is artwork metadata; physical wheel placement always comes from URDF.
    const wheelsOverShell = SHELLS[style].wheelsOverShell?.[view] === true;
    return <g data-mower-style={style} data-view={view} data-transparent={transparent}>
        {layoutDetails && <g data-layer="casters">{(view === "top" ? [-1,1] : [1]).map(sign => <Wheel artwork={artwork} key={sign} view={view} caster
            x={view === "top" ? sign*r.casterTrack/2 : -r.casterXOffset}
            y={view === "top" ? -r.casterXOffset : -(r.casterZ ?? (-r.wheelRadius+r.casterRadius))}
            radius={r.casterRadius} width={r.casterWidth ?? r.casterRadius}/>)}</g>}
        {!wheelsOverShell && <g data-layer="wheels">{wheels}</g>}
        {layoutDetails && <g data-layer="blade" opacity={transparent ? 1 : 0}>
            <g transform={`translate(${view === "top" ? -(r.bladeY ?? 0) : -bladeX} ${view === "top" ? -bladeX : -(r.bladeZ ?? 0)})`}>
                <PartSprite artwork={artwork} part="blade" view={view} width={r.bladeRadius*2}
                    height={view === "top" ? r.bladeRadius*2 : (r.bladeThickness ?? .01)}
                    fallback={view === "top" ? <circle r={r.bladeRadius} fill="#526c60" stroke="#9ed9bd" strokeWidth={.002}/>
                        : <rect x={-r.bladeRadius} y={-.005} width={r.bladeRadius*2} height={.01} fill="#9ed9bd"/>}/>
            </g>
        </g>}
        {layoutDetails && sensors && (r.sensors ?? []).filter(s=>s.id === "imu").map(s=><SensorGraphic artwork={artwork} key={s.id} sensor={s} view={view} highlight={highlightSensors}/>)}
        <g data-layer="shell" opacity={transparent ? .24 : 1}>
            <ShellSprite colour={colour} artwork={artwork} key={style} style={style} view={view}
                x={view === "top" ? -r.baseWidth/2 : -r.chassisCenterX-r.baseLength/2}
                y={view === "top" ? -r.chassisCenterX-r.baseLength/2 : -bodyZ-r.baseHeight/2}
                width={view === "top" ? r.baseWidth : r.baseLength} height={view === "top" ? r.baseLength : r.baseHeight}/>
        </g>
        {wheelsOverShell && <g data-layer="wheels">{wheels}</g>}
        {sensors && (r.sensors ?? []).filter(s=>s.id !== "imu").map(s=><SensorGraphic artwork={artwork} key={s.id} sensor={s} view={view} highlight={highlightSensors}/>)}
    </g>;
});
