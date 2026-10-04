import {Button, Typography} from "antd";
import {useTranslation} from "react-i18next";
import {MOWER_MODELS} from "../constants/mowerModels.ts";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {boardLabel, panelLabel} from "./firmwareTargets.ts";

export interface FlashTargetSummaryProps {
    mowerModel?: string;
    boardType: string;
    panelType: string;
    /** True when either field was chosen by hand rather than derived from the model. */
    isManual: boolean;
    onChange: () => void;
}

/**
 * Read-only statement of what will be flashed — the board and panel the
 * onboarding mower model implies — with a single "Change" affordance for the
 * operator whose hardware differs from the stock build.
 */
export const FlashTargetSummary = ({mowerModel, boardType, panelType, isManual, onChange}: FlashTargetSummaryProps) => {
    const {t} = useTranslation();
    const {colors} = useThemeMode();
    const modelEntry = MOWER_MODELS.find((m) => m.value === mowerModel);
    const modelName = modelEntry ? t(modelEntry.label) : mowerModel;

    return (
        <div
            data-testid="flash-target-summary"
            style={{
                border: `1px solid ${colors.border}`,
                borderRadius: 8,
                padding: "10px 12px",
                marginBottom: 16,
                display: "flex",
                alignItems: "flex-start",
                justifyContent: "space-between",
                gap: 12,
            }}
        >
            <div>
                <Typography.Text strong>{t("flashBoard.targetTitle")}</Typography.Text>
                <div style={{marginTop: 4}}>
                    <Typography.Text>{boardLabel(boardType, mowerModel, t) ?? boardType}</Typography.Text>
                    <Typography.Text type="secondary"> · </Typography.Text>
                    <Typography.Text>{panelLabel(panelType) ?? panelType}</Typography.Text>
                </div>
                <Typography.Text type="secondary" style={{fontSize: 12}}>
                    {isManual
                        ? t("flashBoard.targetSetManually")
                        : modelName
                            ? t("flashBoard.targetFromModel", {model: modelName})
                            : t("flashBoard.targetFromSaved")}
                </Typography.Text>
            </div>
            <Button size="small" onClick={onChange}>{t("flashBoard.targetChange")}</Button>
        </div>
    );
};
