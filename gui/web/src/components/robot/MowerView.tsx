import {DEFAULT_DOCK_STYLE} from "./dockAssets";
import {useTranslation} from "react-i18next";
import {useMowerVisual} from "../../hooks/useMowerVisual";
import type {RobotGeometry} from "../../hooks/useRobotDescription";
import type {RobotView} from "../../utils/robotModel";
import {resolveDockAppearance} from "../../constants/mowerAppearances";
import {DockGraphic} from "./DockGraphic";
import {dockedAssemblyBounds, previewDockOffsetY, setupDockOpacity, type DockVisualGeometry} from "./dockGeometry";
import {assemblyBounds} from "./assemblyBounds";
import {LayeredMower} from "./LayeredMower";
import {measurementLayout, type Measurement} from "./measurements";
import {MeasurementOverlay} from "./MeasurementOverlay";

export function MowerView({values = {}, robot, view, maxHeight = 300, highlightSensors = false, transparent, showDock = false, dockGeometry, measurement}: {values?: Record<string, unknown>; robot: RobotGeometry; view: RobotView; maxHeight?: number; highlightSensors?: boolean; transparent?: boolean; showDock?: boolean; dockGeometry?: DockVisualGeometry; measurement?: Measurement}) {
    const {t} = useTranslation();
    const [preference] = useMowerVisual(values);
    const dockStyle = resolveDockAppearance(preference.dockAppearance).layered ?? DEFAULT_DOCK_STYLE;
    const b = showDock ? dockedAssemblyBounds(robot,view,.05,dockStyle,dockGeometry) : assemblyBounds(robot,view,.05);
    const frame = measurement ? measurementLayout(measurement,b).bounds : b;
    return <figure style={{margin:0,minWidth:0,flex:"1 1 200px"}}>
        <figcaption style={{fontSize:12,opacity:.7,marginBottom:8}}>{t("mowerVisual."+view)}</figcaption>
        <svg role="img" aria-label={t("mowerVisual."+view)} data-testid={`mower-${view}`}
            viewBox={[frame.x,frame.y,frame.width,frame.height].join(" ")} width="100%" style={{display:"block",height:maxHeight}}>
            {showDock && <g opacity={setupDockOpacity(transparent ?? preference.transparent)} pointerEvents="none">
                <DockGraphic geometry={dockGeometry} view={view} style={dockStyle} offsetY={previewDockOffsetY(robot,view)}/>
            </g>}
            <LayeredMower robot={robot} {...preference} transparent={transparent ?? preference.transparent} view={view} highlightSensors={highlightSensors}/>
            <g stroke="#80ae9d" strokeWidth={.001} opacity={.65}>
                <path d="M -.012 0 H .012 M 0 -.012 V .012"/>
                {view === "side" && <path d={`M ${b.x} ${robot.wheelRadius-(robot.wheelZ ?? 0)} h ${b.width}`} strokeDasharray=".012 .012"/>}
            </g>
            {measurement && <MeasurementOverlay measurement={measurement} bounds={b}/>}
        </svg>
    </figure>;
}
