import {DEFAULT_DOCK_STYLE} from "./dockAssets";
import {SHELLS} from "./shellAssets";
import type {MeasurementEditorState} from "./useMeasurementEditor";
import {MeasurementDialog, MeasurementLegend} from "./MeasurementEditor";
import {measurementGeometry, MEASUREMENTS} from "./measurements";
import {resolveCasterPosition} from "../../utils/casterPosition";
import {DOCK_VISUAL_DEFAULTS} from "./dockGeometry";
import {resolveDockAppearance} from "../../constants/mowerAppearances";
import {dockVisualGeometry} from "./dockGeometry";
import {Button, Card, ColorPicker, Flex, Select, Switch, Typography} from "antd";
import {useTranslation} from "react-i18next";
import {useRobotDescription} from "../../hooks/useRobotDescription";
import {MOWER_STYLES, useMowerVisual, type MowerStyle} from "../../hooks/useMowerVisual";
import {previewRobotGeometry} from "../../utils/robotModel";
import {MowerView} from "./MowerView";
export {MowerView} from "./MowerView";

export function MowerVisualControls({values = {}, onStyleChange, onColourChange, transparent, onTransparencyChange}: {
    onColourChange?: (colour: string) => void; values?: Record<string, unknown>; onStyleChange?: (style: MowerStyle) => void; transparent?: boolean; onTransparencyChange?: (value: boolean) => void;
}) {
    const {t, i18n} = useTranslation();
    const [preference, setPreference] = useMowerVisual(values);
    return <Flex wrap gap={12} align="center" style={{marginBottom:12}}>
        {onStyleChange && <Select aria-label={t("mowerVisual.bodyStyle")} value={preference.style} style={{minWidth:150}}
            options={MOWER_STYLES.map(style => ({value:style,label:SHELLS[style].label[i18n.resolvedLanguage?.startsWith("fr") ? "fr" : "en"]}))}
            onChange={onStyleChange}/>}
        {onColourChange && SHELLS[preference.style].paint && <Flex wrap gap={8} align="center">
            <span>{t("mowerVisual.bodyColour")}</span>
            <ColorPicker value={preference.colour || "#74777a"} disabledAlpha
                onChange={value => onColourChange(value.toHexString())}
                presets={[{label:t("mowerVisual.bodyColour"),colors:[SHELLS[preference.style].paint!.suggestedColour,"#167bc1","#e97822","#3f8e79","#b83438","#dedbd3","#24282b"]}]}>
                <Button aria-label={t("mowerVisual.chooseColour")}>
                    <span aria-hidden style={{display:"inline-block",width:16,height:16,borderRadius:3,background:preference.colour || "#74777a",border:"1px solid currentColor"}}/>
                    {preference.colour || t("mowerVisual.originalColour")}
                </Button>
            </ColorPicker>
            <Button size="small" onClick={() => onColourChange(SHELLS[preference.style].paint!.suggestedColour)}>{t("mowerVisual.suggestedColour")}</Button>
            <Button size="small" disabled={!preference.colour} onClick={() => onColourChange("")}>{t("mowerVisual.originalColour")}</Button>
        </Flex>}
        <label style={{display:"inline-flex",gap:8,alignItems:"center"}}>
            <Switch size="small" checked={transparent ?? preference.transparent} onChange={value=>onTransparencyChange ? onTransparencyChange(value) : setPreference({transparent: value})}/>
            {t("mowerVisual.transparent")}
        </label>
        {!onTransparencyChange && <Typography.Text type="secondary" style={{fontSize:11}}>{t("mowerVisual.browserPreference")}</Typography.Text>}
    </Flex>;
}
export function MowerPreview({values, compact = false, editor, onStyleChange, onColourChange}: {values: Record<string, unknown>; onColourChange?: (colour: string) => void; compact?: boolean; editor?: MeasurementEditorState; onStyleChange?: (style: MowerStyle) => void}) {
    const {t} = useTranslation();
    const live = useRobotDescription();
    const robot = previewRobotGeometry(live,values);
    const [visual] = useMowerVisual(values);
    const dockStyle = resolveDockAppearance(visual.dockAppearance).layered ?? DEFAULT_DOCK_STYLE;
    const dockGeometry = dockVisualGeometry(values, robot.wheelXOffset, dockStyle, robot.chassisCenterX);
    const measurementValues = {...DOCK_VISUAL_DEFAULTS,...values,caster_x_offset:resolveCasterPosition(values)};
    const measurement = editor?.active && !editor.invalid ? measurementGeometry(editor.active,robot,measurementValues,dockStyle) : null;
    return <Card className={editor?.active ? "measurement-preview-active" : undefined} size="small" title={t("mowerVisual.title")} style={{marginBottom:16}} data-testid="mower-preview">
        {!compact && !editor?.active && <MowerVisualControls values={values} onStyleChange={onStyleChange} onColourChange={onColourChange}/>}
        {!live.fromUrdf ? <Typography.Text type="secondary">{t("mowerVisual.waiting")}</Typography.Text> : <>
            <Flex wrap gap={16}>
                <MowerView values={values} dockGeometry={dockGeometry} robot={robot} view="top" showDock={!compact} maxHeight={compact ? 210 : editor?.active ? 190 : 300} measurement={measurement?.view === "top" ? measurement : undefined} transparent={editor?.active?.startsWith("blade_") ? true : undefined}/>
                {!compact && <MowerView values={values} dockGeometry={dockGeometry} robot={robot} view="side" showDock maxHeight={editor?.active ? 190 : 300} measurement={measurement?.view === "side" ? measurement : undefined}/>}
            </Flex>
            {editor?.active && <div className="measurement-preview-summary">
                <Typography.Text strong>{t("settingsHardware."+MEASUREMENTS[editor.active].label)}</Typography.Text>
                <Button size="small" onClick={editor.dismiss}>{t("measurementEditor.close")}</Button>
            </div>}
            {measurement && <MeasurementLegend measurement={measurement}/>}
            {editor?.active && !measurement && <Typography.Text type="danger">{t("measurementEditor.invalid")}</Typography.Text>}
            {!editor?.active && <><Typography.Text style={{display:"block",marginTop:12}}>
                {(robot.baseLength*100).toFixed(1)} × {(robot.baseWidth*100).toFixed(1)} × {(robot.baseHeight*100).toFixed(1)} cm
            </Typography.Text>
            <Typography.Text type="secondary" style={{fontSize:12}}>{t("mowerVisual.previewNote")}</Typography.Text></>}
        </>}
        {editor?.session && <MeasurementDialog session={editor.session} live={live} values={values}
            onCancel={editor.close} onApply={editor.apply}/>}
    </Card>;
}
