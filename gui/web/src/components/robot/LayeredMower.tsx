import {PartSprite} from "./PartSprite";
import {sensorArtworkProjection} from "./sensorArtworkProjection";
import {SHELLS} from "./shellAssets";
import {useId, useState} from "react";
import type {RobotGeometry} from "../../hooks/useRobotDescription";
import type {MowerStyle} from "../../hooks/useMowerVisual";
import {sensorCorners, type RobotView, type SensorGeometry} from "../../utils/robotModel";

function ShellSprite({style, view, x, y, width, height}: {
    style: MowerStyle; view: RobotView; x: number; y: number; width: number; height: number;
}) {
    const [loaded, setLoaded] = useState(false);
    const shell = SHELLS[style];
    const [sx, sy, sw, sh] = shell[view];
    return <>
        {!loaded && <rect x={x} y={y} width={width} height={height} rx={Math.min(width,height)*.18} fill="#343e40"/>}
        <svg data-layer="shell-art" x={x} y={y} width={width} height={height}
        viewBox={[sx,sy,sw,sh].join(" ")} preserveAspectRatio="none" overflow="hidden">
        <rect data-shell-bounds x={sx} y={sy} width={sw} height={sh} fill="none"/>
        <image onLoad={()=>setLoaded(true)} onError={()=>setLoaded(false)} href={`/assets/robots/layered/${style}.png`} width={shell.size[0]} height={shell.size[1]}
            transform={view === "top" && shell.topRotation ? `rotate(${shell.topRotation} ${sx+sw/2} ${sy+sh/2})` : undefined}/>
    </svg></>;
}

/** Sensor visuals use real URDF sizes. Box faces are projected after all three
 * mounting rotations, so IMU roll/pitch are not lost in the side view. */
export function SensorGraphic({sensor, view, highlight = false}: {sensor: SensorGeometry; view: RobotView; highlight?: boolean}) {
    const glowId = useId();
    const projection = sensorArtworkProjection(sensor,view);
    const points = sensorCorners(sensor,view);
    const xs=points.map(p=>p[0]),ys=points.map(p=>p[1]);
    return <g data-sensor={sensor.id} data-x={sensor.x} data-y={sensor.y} data-z={sensor.z}>
        {highlight && <defs><filter id={glowId} x="-25%" y="-25%" width="150%" height="150%" colorInterpolationFilters="sRGB">
            <feMorphology in="SourceAlpha" operator="dilate" radius=".0012" result="outline"/>
            <feGaussianBlur in="outline" stdDeviation=".0007" result="soft"/>
            <feFlood floodColor="#9af4d0" floodOpacity=".85"/>
            <feComposite in2="soft" operator="in"/>
            <feMerge><feMergeNode/><feMergeNode in="SourceGraphic"/></feMerge>
        </filter></defs>}
        <g transform={projection.transform} filter={highlight ? `url(#${glowId})` : undefined}>
            <PartSprite part={sensor.id} view={projection.view} width={projection.width} height={projection.height}
                fallback={<rect x={-projection.width/2} y={-projection.height/2} width={projection.width} height={projection.height}
                    rx={.003} fill={sensor.id === "imu" ? "#326655" : "#7c9187"}/>}/>
        </g>
        {/* Retain the projected extent for hit testing and inspection without
            changing the part's physical dimensions to match a minimum UI size. */}
        <rect data-sensor-bounds x={Math.min(...xs)} y={Math.min(...ys)} width={Math.max(...xs)-Math.min(...xs)}
            height={Math.max(...ys)-Math.min(...ys)} fill="none"/>
    </g>;
}

function Wheel({x, y, radius, width, view, caster = false}: {x: number; y: number; radius: number; width: number; view: RobotView; caster?: boolean}) {
    return <g transform={`translate(${x} ${y})${caster && view === "top" ? " rotate(180)" : ""}`}>
        <PartSprite part={caster ? "caster" : "drive-wheel"} view={view} width={view === "top" ? width : radius*2} height={radius*2}
            fallback={view === "top" ? <rect x={-width/2} y={-radius} width={width} height={radius*2} rx={width*.15} fill="#293332"/>
                : <circle r={radius} fill="#293332" stroke="#6b867a" strokeWidth={.002}/>}/>
    </g>;
}

export function LayeredMower({robot: r, style, transparent, view = "top", sensors = true, highlightSensors = false}: {
    robot: RobotGeometry; style: MowerStyle; transparent: boolean; view?: RobotView; sensors?: boolean; highlightSensors?: boolean;
}) {
    const id = useId();
    const rearX = r.chassisCenterX-r.baseLength*.40;
    const buttonW = Math.min(.095,r.baseWidth*.30), buttonD = Math.min(.04,r.baseLength*.08);
    const bodyZ = r.chassisCenterZ ?? r.baseHeight/2;
    const bladeX = r.bladeX ?? r.chassisCenterX;
    const wheels = view === "top" ? [-1,1].map(sign => <Wheel key={sign} view={view}
        x={sign*r.wheelTrack/2} y={-r.wheelXOffset} radius={r.wheelRadius} width={r.wheelWidth}/>)
        : <Wheel view={view} x={-r.wheelXOffset} y={-(r.wheelZ ?? 0)} radius={r.wheelRadius} width={r.wheelWidth}/>;
    return <g data-mower-style={style} data-view={view} data-transparent={transparent}>
        <defs><linearGradient id={id} x2="0" y2="1">
            <stop stopColor="#fc796d"/><stop offset="1" stopColor="#b52828"/>
        </linearGradient></defs>
        <g data-layer="casters">{(view === "top" ? [-1,1] : [1]).map(sign => <Wheel key={sign} view={view} caster
            x={view === "top" ? sign*r.casterTrack/2 : -r.casterXOffset}
            y={view === "top" ? -r.casterXOffset : -(r.casterZ ?? (-r.wheelRadius+r.casterRadius))}
            radius={r.casterRadius} width={r.casterWidth ?? r.casterRadius}/>)}</g>
        {view === "top" && <g data-layer="wheels">{wheels}</g>}
        <g data-layer="blade" opacity={transparent ? 1 : 0}>
            <g transform={`translate(${view === "top" ? -(r.bladeY ?? 0) : -bladeX} ${view === "top" ? -bladeX : -(r.bladeZ ?? 0)})`}>
                <PartSprite part="blade" view={view} width={r.bladeRadius*2}
                    height={view === "top" ? r.bladeRadius*2 : (r.bladeThickness ?? .01)}
                    fallback={view === "top" ? <circle r={r.bladeRadius} fill="#526c60" stroke="#9ed9bd" strokeWidth={.002}/>
                        : <rect x={-r.bladeRadius} y={-.005} width={r.bladeRadius*2} height={.01} fill="#9ed9bd"/>}/>
            </g>
        </g>
        {sensors && (r.sensors ?? []).filter(s=>s.id === "imu").map(s=><SensorGraphic key={s.id} sensor={s} view={view} highlight={highlightSensors}/>)}
        <g data-layer="shell" opacity={transparent ? .24 : 1}>
            <ShellSprite key={style} style={style} view={view}
                x={view === "top" ? -r.baseWidth/2 : -r.chassisCenterX-r.baseLength/2}
                y={view === "top" ? -r.chassisCenterX-r.baseLength/2 : -bodyZ-r.baseHeight/2}
                width={view === "top" ? r.baseWidth : r.baseLength} height={view === "top" ? r.baseLength : r.baseHeight}/>
        </g>
        <g data-layer="stop" data-robot-x={rearX}>
            <rect x={view === "top" ? -buttonW/2 : -rearX-buttonD/2}
                y={view === "top" ? -rearX-buttonD/2 : -bodyZ-r.baseHeight/2+.012}
                width={view === "top" ? buttonW : buttonD} height={view === "top" ? buttonD : .014}
                rx={.005} fill={`url(#${id})`} stroke="#872b2a" strokeWidth={.002}/>
        </g>
        {view === "side" && <g data-layer="wheels">{wheels}</g>}
        {sensors && (r.sensors ?? []).filter(s=>s.id !== "imu").map(s=><SensorGraphic key={s.id} sensor={s} view={view} highlight={highlightSensors}/>)}
    </g>;
}
