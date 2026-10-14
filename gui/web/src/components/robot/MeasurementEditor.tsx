import {NumberWithAddon} from '../inputs/NumberWithAddon';
import {DEFAULT_DOCK_STYLE} from "./dockAssets";
import type {MeasurementEditorState} from "./useMeasurementEditor";
import {useEffect, useRef, useState} from "react";
import {Button, Modal, Typography} from "antd";
import type {InputNumberRef} from "rc-input-number";
import {useTranslation} from "react-i18next";
import type {RobotGeometry} from "../../hooks/useRobotDescription";
import {previewRobotGeometry} from "../../utils/robotModel";
import {resolveDockAppearance} from "../../constants/mowerAppearances";
import {useMowerVisual} from "../../hooks/useMowerVisual";
import {dockVisualGeometry} from "./dockGeometry";
import {MEASUREMENTS, measurementGeometry, validMeasurement, type Measurement, type MeasurementKey} from "./measurements";
import {MowerView} from "./MowerView";
import "./measurementEditor.css";

export function MeasurementLegend({measurement}: {measurement: Measurement}) {
    const {t} = useTranslation();
    return <div className="measurement-legend" data-testid="measurement-legend">
        <span><b>A</b> {t("measurementEditor."+measurement.fromLabel)}</span>
        <span><b>B</b> {t("measurementEditor."+measurement.toLabel)}</span>
    </div>;
}

export function MeasurementDialog({session, live, values, onCancel, onApply}: {
    session: NonNullable<MeasurementEditorState["session"]>; live: RobotGeometry; values: Record<string, unknown>;
    onCancel: () => void; onApply: (key: MeasurementKey, value: number) => void;
}) {
    const {t} = useTranslation();
    const [value, setValue] = useState<number | null>(session.value);
    const input = useRef<InputNumberRef>(null);
    const [viewport, setViewport] = useState(() => ({height: window.visualViewport?.height ?? window.innerHeight,
        top: window.visualViewport?.offsetTop ?? 0}));
    useEffect(() => {
        const vp = window.visualViewport;
        const resize = () => setViewport({height: vp?.height ?? window.innerHeight, top: vp?.offsetTop ?? 0});
        vp?.addEventListener("resize",resize); vp?.addEventListener("scroll",resize);
        return () => {vp?.removeEventListener("resize",resize); vp?.removeEventListener("scroll",resize);};
    }, []);
    const key = session.key, spec = MEASUREMENTS[key];
    const draft = {...values, [key]: value};
    const robot = previewRobotGeometry(live,draft);
    const [visual] = useMowerVisual(values);
    const dockStyle = resolveDockAppearance(visual.dockAppearance).layered ?? DEFAULT_DOCK_STYLE;
    const geometry = dockVisualGeometry(draft,robot.wheelXOffset,dockStyle,robot.chassisCenterX);
    const valid = validMeasurement(key,value);
    const measurement = measurementGeometry(key,robot,draft,dockStyle);
    const label = t("settingsHardware."+spec.label);
    const confirm = () => {if(validMeasurement(key,value)) onApply(key,value);};
    return <Modal open title={label} onCancel={onCancel} onOk={confirm} width="100%" className="measurement-dialog"
        rootClassName="measurement-dialog-root" destroyOnHidden focusTriggerAfterClose={false}
        styles={{wrapper:{height:viewport.height,top:viewport.top,bottom:"auto"},
            content:{maxHeight:viewport.height-8},body:{overflowY:"auto",minHeight:0}}}
        afterOpenChange={open => {if(open) input.current?.focus({preventScroll:true});}}
        footer={<div className="measurement-actions"><Button onClick={onCancel}>{t("measurementEditor.cancel")}</Button>
            <Button type="primary" disabled={!valid} onClick={confirm}>{t("measurementEditor.apply")}</Button></div>}>
        {live.fromUrdf ? <MowerView values={values} robot={robot} view={spec.view} showDock dockGeometry={geometry}
            measurement={measurement ?? undefined} transparent={key.startsWith("blade_") ? true : undefined}
            maxHeight={Math.max(115,Math.min(spec.view === "side" ? 180 : 240,viewport.height*.29))}/>
            : <Typography.Paragraph>{t("mowerVisual.waiting")}</Typography.Paragraph>}
        {measurement && live.fromUrdf && <MeasurementLegend measurement={measurement}/>}
        <label className="measurement-input-label" htmlFor="measurement-value">{label} (m)</label>
        <NumberWithAddon ref={input} id="measurement-value" aria-label={label+", m"} aria-invalid={!valid}
            aria-describedby="measurement-help" value={value} onChange={setValue} onInput={text => setValue(text.trim() && Number.isFinite(Number(text)) ? Number(text) : null)}
            aria-valuemin={spec.min} aria-valuemax={"max" in spec ? spec.max : undefined} step={spec.step} precision={spec.precision}
            inputMode="decimal" style={{width:"100%"}} size="large" after="m"
            onPressEnter={confirm}/>
        {!valid && <Typography.Text type="danger" role="status">{t("measurementEditor.invalid")}</Typography.Text>}
        <Typography.Paragraph id="measurement-help" className="measurement-help">{t("settingsHardware."+spec.label+"Tooltip")}</Typography.Paragraph>
        <Typography.Text type="secondary">{t("measurementEditor.draftNote")}</Typography.Text>
    </Modal>;
}
