import {DockPreview} from "./DockPreview";
import {assemblyBounds} from "./assemblyBounds";
import {Card, Flex, Select, Switch, Typography} from "antd";
import {useTranslation} from "react-i18next";
import {useRobotDescription, type RobotGeometry} from "../../hooks/useRobotDescription";
import {MOWER_STYLES, useMowerVisual} from "../../hooks/useMowerVisual";
import {previewRobotGeometry, type RobotView} from "../../utils/robotModel";
import {LayeredMower} from "./LayeredMower";

export function MowerVisualControls({chooseStyle = false}: {chooseStyle?: boolean}) {
    const {t} = useTranslation();
    const [preference, setPreference] = useMowerVisual();
    return <Flex wrap gap={12} align="center" style={{marginBottom:12}}>
        {chooseStyle && <Select aria-label={t("mowerVisual.bodyStyle")} value={preference.style} style={{minWidth:150}}
            options={MOWER_STYLES.map(style => ({value:style,label:t("mowerVisual."+style)}))}
            onChange={style=>setPreference({style})}/>}
        <label style={{display:"inline-flex",gap:8,alignItems:"center"}}>
            <Switch size="small" checked={preference.transparent} onChange={transparent=>setPreference({transparent})}/>
            {t("mowerVisual.transparent")}
        </label>
        <Typography.Text type="secondary" style={{fontSize:11}}>{t("mowerVisual.browserPreference")}</Typography.Text>
    </Flex>;
}
export function MowerView({robot, view, maxHeight = 300, highlightSensors = false}: {robot: RobotGeometry; view: RobotView; maxHeight?: number; highlightSensors?: boolean}) {
    const {t} = useTranslation();
    const [preference] = useMowerVisual();
    const b = assemblyBounds(robot,view,.05);
    return <figure style={{margin:0,minWidth:0,flex:"1 1 200px"}}>
        <figcaption style={{fontSize:12,opacity:.7,marginBottom:8}}>{t("mowerVisual."+view)}</figcaption>
        <svg role="img" aria-label={t("mowerVisual."+view)} data-testid={`mower-${view}`}
            viewBox={[b.x,b.y,b.width,b.height].join(" ")} width="100%" style={{display:"block",height:maxHeight}}>
            <LayeredMower robot={robot} {...preference} view={view} highlightSensors={highlightSensors}/>
            <g stroke="#80ae9d" strokeWidth={.001} opacity={.65}>
                <path d="M -.012 0 H .012 M 0 -.012 V .012"/>
                {view === "side" && <path d={`M ${b.x} ${robot.wheelRadius} h ${b.width}`} strokeDasharray=".012 .012"/>}
            </g>
        </svg>
    </figure>;
}
export function MowerPreview({values, compact = false}: {values: Record<string, unknown>; compact?: boolean}) {
    const {t} = useTranslation();
    const live = useRobotDescription();
    const robot = previewRobotGeometry(live,values);
    return <Card size="small" title={t("mowerVisual.title")} style={{marginBottom:16}} data-testid="mower-preview">
        {!compact && <MowerVisualControls chooseStyle/>}
        {!live.fromUrdf ? <Typography.Text type="secondary">{t("mowerVisual.waiting")}</Typography.Text> : <>
            <Flex wrap gap={16}>
                <MowerView robot={robot} view="top" maxHeight={compact ? 210 : 300}/>
                {!compact && <MowerView robot={robot} view="side" maxHeight={300}/>}
            </Flex>
            <Typography.Text style={{display:"block",marginTop:12}}>
                {(robot.baseLength*100).toFixed(1)} × {(robot.baseWidth*100).toFixed(1)} × {(robot.baseHeight*100).toFixed(1)} cm
            </Typography.Text>
            <Typography.Text type="secondary" style={{fontSize:12}}>{t("mowerVisual.previewNote")}</Typography.Text>
        </>}
        {!compact && <DockPreview/>}
    </Card>;
}
