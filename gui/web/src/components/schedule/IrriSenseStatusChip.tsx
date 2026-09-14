import {Tooltip} from "antd";
import {useTranslation} from "react-i18next";
import dayjs from "dayjs";
import {useThemeMode} from "../../theme/ThemeContext.tsx";
import {useSoilStatus} from "../../hooks/useSoilStatus.ts";
import {soilVerdict, type SoilStatus} from "../../types/irrisense.ts";
import {FONT} from "../dashboard";

interface IrriSenseStatusChipProps {
    /** Injected for tests; the page lets the hook fetch it. */
    status?: SoilStatus | null;
}

/**
 * The one-line soil verdict shown next to the schedule list: wet (and whether
 * that suspends scheduled mows), dry, or unknown. Renders nothing while the
 * integration is switched off so the page looks exactly as before for
 * operators without IrriSense.
 */
export function IrriSenseStatusChip({status: injected}: IrriSenseStatusChipProps) {
    const {t} = useTranslation();
    const {colors} = useThemeMode();
    const polled = useSoilStatus({enabled: injected === undefined});
    const status = injected === undefined ? polled.status : injected;

    if (!status || !status.enabled) return null;

    const verdict = soilVerdict(status);
    const blocking = verdict === "wet" && status.gateScheduler;
    const label = blocking
        ? t("schedulePage.soilWetBlocking")
        : verdict === "wet"
            ? t("schedulePage.soilWet")
            : verdict === "dry"
                ? t("schedulePage.soilDry")
                : t("schedulePage.soilUnknown");
    const color = verdict === "wet" ? colors.sky : verdict === "dry" ? colors.accent : colors.textMuted;

    const details = [
        status.reason,
        status.fetchedAt ? t("schedulePage.soilFetchedAt", {value: dayjs(status.fetchedAt).format("YYYY-MM-DD HH:mm")}) : null,
        status.error,
    ].filter(Boolean).join(" · ");

    return (
        <Tooltip title={details || undefined}>
            <span
                role="status"
                data-testid="irrisense-status-chip"
                data-verdict={verdict}
                style={{
                    display: "inline-flex", alignItems: "center", gap: 6,
                    fontSize: 11, fontWeight: 700, letterSpacing: "0.04em",
                    color, background: `${color}18`, border: `1px solid ${color}66`,
                    borderRadius: 100, padding: "3px 10px", fontFamily: FONT,
                    textTransform: "uppercase",
                }}
            >
                <span aria-hidden="true" style={{width: 7, height: 7, borderRadius: "50%", background: color}}/>
                {label}
            </span>
        </Tooltip>
    );
}
