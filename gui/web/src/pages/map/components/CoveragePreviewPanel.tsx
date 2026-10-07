import {Alert, Button, InputNumber, Select, Slider} from "antd";
import {useTranslation} from "react-i18next";
import {useThemeMode} from "../../../theme/ThemeContext.tsx";
import {RING_DIRECTION, type AngleMode, type DirectionChoice} from "../coveragePreview.ts";
import type {useCoveragePreview} from "../hooks/useCoveragePreview.ts";

type Preview = ReturnType<typeof useCoveragePreview>;

interface CoveragePreviewPanelProps {
    preview: Preview;
    /** Label for an area (named or "Area N"). */
    areaLabel: (index: number, name: string) => string;
    /** A paused mow exists: resuming it plans the area again with whatever is saved now. */
    resumeAvailable?: boolean;
}

/// Controls for the "mowing lines" overlay: pick an area, try a mow angle and a
/// perimeter direction, watch the planner's lines move, then save them for that
/// area (or as the robot-wide default). The lines are the real planner's
/// (preview_coverage); nothing here moves the robot.
export const CoveragePreviewPanel = ({preview, areaLabel, resumeAvailable = false}: CoveragePreviewPanelProps) => {
    const {colors} = useThemeMode();
    const {t} = useTranslation();
    const {
        areas, area, selectArea, hasAreaId, canEdit, choices, setAngleMode, setAngleDeg, setDirection,
        setStart, startAdjustable,
        robotWideAngle, robotWideDirection, shownAngle, dirty, differsFromRobotWide, reset, saveArea,
        saveRobotWide, saving, loading, error, result,
    } = preview;

    const label = {
        fontSize: 12, fontWeight: 600, color: colors.muted, textTransform: 'uppercase' as const,
        letterSpacing: '0.05em', marginTop: 10, marginBottom: 6, display: 'block',
    };
    const stat = {fontSize: 12, color: colors.textSecondary};

    if (areas.length === 0) {
        return <div style={{padding: 12, fontSize: 13, color: colors.textSecondary}}>{t('coveragePreview.noAreas')}</div>;
    }

    const directionName = (d: number) =>
        d === RING_DIRECTION.clockwise ? t('coveragePreview.directionClockwise')
            : d === RING_DIRECTION.counterClockwise ? t('coveragePreview.directionCounterClockwise')
                : t('coveragePreview.directionDefault');
    const robotWideAngleName = robotWideAngle < 0
        ? t('coveragePreview.angleAuto')
        : t('coveragePreview.angleDeg', {deg: Math.round(robotWideAngle)});

    const fixed = choices.angleMode === "fixed";
    const sliderValue = Math.round(fixed ? choices.angleDeg : shownAngle) % 180;
    const editable = canEdit && hasAreaId;
    const hasOwnLines = choices.angleMode !== "global" || choices.direction !== "global";
    // Swaths and perimeter rounds overlap slightly, so the planner's fraction can pass 1.
    const percent = Math.min(100, Math.round((result?.planned_fraction ?? 0) * 100));

    return (
        <div style={{padding: 12}}>
            <div style={{fontSize: 14, fontWeight: 600, color: colors.text}}>{t('coveragePreview.title')}</div>
            <div style={{fontSize: 12, color: colors.textSecondary, marginTop: 2}}>{t('coveragePreview.hint')}</div>

            {areas.length > 1 && (
                <>
                    <span style={label}>{t('coveragePreview.area')}</span>
                    <Select
                        size="small"
                        style={{width: '100%'}}
                        value={area?.id}
                        onChange={(id) => selectArea(id)}
                        options={areas.map((a, i) => ({value: a.id, label: areaLabel(i, a.getName())}))}
                    />
                </>
            )}

            <span style={label}>{t('coveragePreview.angle')}</span>
            <Select<AngleMode>
                size="small"
                style={{width: '100%'}}
                value={choices.angleMode}
                onChange={(mode) => setAngleMode(mode)}
                options={[
                    {value: "global", label: t('coveragePreview.angleModeGlobal', {value: robotWideAngleName})},
                    {value: "auto", label: t('coveragePreview.angleModeAuto')},
                    {value: "fixed", label: t('coveragePreview.angleModeFixed')},
                ]}
            />
            <div style={{display: 'flex', alignItems: 'center', gap: 8, marginTop: 4, opacity: fixed ? 1 : 0.45}}>
                <Slider
                    style={{flex: 1, margin: '4px 6px'}}
                    min={0}
                    max={179}
                    step={1}
                    value={sliderValue}
                    onChange={setAngleDeg}
                    tooltip={{formatter: (v) => t('coveragePreview.angleDeg', {deg: v})}}
                />
                <InputNumber
                    size="small"
                    style={{width: 64}}
                    min={0}
                    max={179}
                    step={1}
                    precision={0}
                    value={sliderValue}
                    onChange={(v) => v !== null && setAngleDeg(v)}
                />
            </div>
            {!fixed && (
                <div style={stat}>
                    {t('coveragePreview.angleAutoNow', {deg: Math.round(shownAngle)})}
                </div>
            )}

            <span style={label}>{t('coveragePreview.direction')}</span>
            <Select<DirectionChoice>
                size="small"
                style={{width: '100%'}}
                value={choices.direction}
                onChange={(d) => setDirection(d)}
                options={[
                    {value: "global", label: t('coveragePreview.directionGlobal', {value: directionName(robotWideDirection)})},
                    {value: RING_DIRECTION.planner, label: t('coveragePreview.directionDefault')},
                    {value: RING_DIRECTION.clockwise, label: t('coveragePreview.directionClockwise')},
                    {value: RING_DIRECTION.counterClockwise, label: t('coveragePreview.directionCounterClockwise')},
                ]}
            />

            <span style={label}>{t('coveragePreview.startPoint')}</span>
            <div style={stat}>
                {!startAdjustable
                    ? t('coveragePreview.startNeedsRings')
                    : choices.start ? t('coveragePreview.startCustom') : t('coveragePreview.startAutomatic')}
            </div>
            {startAdjustable && <div style={{...stat, marginTop: 2}}>{t('coveragePreview.startHint')}</div>}
            {startAdjustable && choices.start && (
                <Button size="small" type="link" style={{padding: 0}} onClick={() => setStart(null)}>
                    {t('coveragePreview.startReset')}
                </Button>
            )}

            <div style={{marginTop: 10, minHeight: 36}}>
                {loading && <div style={stat}>{t('coveragePreview.calculating')}</div>}
                {!loading && error && <Alert type="warning" showIcon message={error}/>}
                {!loading && !error && result?.success && (
                    <>
                        <div style={stat}>{t('coveragePreview.swaths', {count: result.swaths?.length ?? 0})}</div>
                        <div style={stat}>{t('coveragePreview.rings', {count: result.headland_passes ?? 0})}</div>
                        <div style={stat}>{t('coveragePreview.coverage', {percent})}</div>
                    </>
                )}
            </div>

            <div style={{...stat, marginTop: 8}}>
                {dirty
                    ? t('coveragePreview.unsaved')
                    : hasOwnLines ? t('coveragePreview.ownSettings') : t('coveragePreview.followsRobotWide')}
            </div>
            {!canEdit && <div style={{...stat, marginTop: 4}}>{t('coveragePreview.blockedMowing')}</div>}
            {canEdit && !hasAreaId && <div style={{...stat, marginTop: 4}}>{t('coveragePreview.blockedNoId')}</div>}
            {resumeAvailable && dirty && (
                <Alert style={{marginTop: 8}} type="warning" showIcon message={t('coveragePreview.resumeWarning')}/>
            )}
            <div style={{display: 'flex', gap: 8, marginTop: 8, flexWrap: 'wrap'}}>
                <Button size="small" type="primary" disabled={!dirty || !editable} loading={saving} onClick={() => void saveArea()}>
                    {t('coveragePreview.saveArea')}
                </Button>
                <Button size="small" disabled={!dirty || saving} onClick={reset}>
                    {t('coveragePreview.reset')}
                </Button>
            </div>
            <div style={{marginTop: 6}}>
                <Button size="small" type="link" style={{padding: 0}} disabled={!differsFromRobotWide || saving}
                    onClick={() => void saveRobotWide()}>
                    {t('coveragePreview.saveRobotWide')}
                </Button>
            </div>
        </div>
    );
};
