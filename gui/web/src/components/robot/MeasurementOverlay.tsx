import {useId} from "react";
import {useTranslation} from "react-i18next";
import {MEASUREMENTS, measurementLayout, type Measurement} from "./measurements";
import type {Bounds} from "../../utils/robotModel";

export function MeasurementOverlay({measurement: m, bounds}: {measurement: Measurement; bounds: Bounds}) {
    const id = useId();
    const {t} = useTranslation();
    const {unit, from, to} = measurementLayout(m, bounds);
    const labelX = (from[0]+to[0])/2 + (m.axis === "y" ? unit*1.5 : 0);
    const labelY = (from[1]+to[1])/2 + (m.axis === "x" ? unit*1.6 : 0);
    const zero = Math.hypot(to[0]-from[0], to[1]-from[1]) < 1e-8;
    return <g data-testid="measurement-overlay" data-measurement={m.key} pointerEvents="none"
        fontFamily="inherit" fontSize={unit*1.8} fill="#b5ffdb" stroke="#9ff4c8" strokeWidth={1.5}>
        <title>{t("settingsHardware."+MEASUREMENTS[m.key].label)} {m.value} m</title>
        <defs><marker id={id} viewBox="0 0 8 8" refX="4" refY="4" markerWidth="6" markerHeight="6" orient="auto-start-reverse">
            <path d="M 8 4 L 0 0 L 0 8 Z" fill="#9ff4c8" stroke="none"/>
        </marker></defs>
        {[m.from,m.to].map((p,i) => <g key={i}>
            <line x1={p[0]} y1={p[1]} x2={i ? to[0] : from[0]} y2={i ? to[1] : from[1]}
                strokeDasharray="3 3" vectorEffect="non-scaling-stroke" opacity=".65"/>
            <circle cx={p[0]} cy={p[1]} r={unit*.55} fill="#08291e" vectorEffect="non-scaling-stroke"/>
            {(!zero || i === 0) && <text x={p[0]-unit} y={p[1]} fontSize={unit*1.4} dominantBaseline="middle" stroke="#08291e" strokeWidth={unit*.2} paintOrder="stroke" textAnchor="end">
                {zero ? "A/B" : i ? "B" : "A"}
            </text>}
        </g>)}
        {!zero && <line x1={from[0]} y1={from[1]} x2={to[0]} y2={to[1]} vectorEffect="non-scaling-stroke"
            markerStart={"url(#"+id+")"} markerEnd={"url(#"+id+")"}/>}
        <text x={labelX} y={labelY} textAnchor="middle" dominantBaseline="middle"
            transform={m.axis === "y" ? "rotate(-90 "+labelX+" "+labelY+")" : undefined}
            stroke="#08291e" strokeWidth={unit*.35} paintOrder="stroke">
            {m.value.toFixed(MEASUREMENTS[m.key].precision)} m
        </text>
    </g>;
}
