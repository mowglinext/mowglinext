import {motion, type Variants} from "framer-motion";
import {OctagonAlert} from "lucide-react";
import {useTranslation} from "react-i18next";
import {isKnownStopReason} from "../../utils/stopReason.ts";

interface StopReasonBannerProps {
    /** HighLevelStatus.stop_reason: a stable machine code, empty when there is nothing to report. */
    reason?: string;
    /** HighLevelStatus.stop_detail: a short English sentence with the numbers. */
    detail?: string;
    /**
     * True while a session is in progress (AUTONOMOUS / RECORDING /
     * MANUAL_MOWING, see isSessionActive). The tree records a reason when it
     * retires ONE area and keeps mowing the others, so the banner waits until
     * the mower has actually stopped (IDLE, or docked after a failure).
     */
    sessionActive?: boolean;
    variants?: Variants;
}

/**
 * Dashboard notice for WHY the mower stopped. The failure states the behavior
 * tree publishes (AREA_UNREACHABLE, NAV_TO_DOCK_FAILED, ...) are replaced by
 * IDLE within seconds, so a robot standing in the field used to say nothing.
 * The tree keeps the reason until the next command and this shows it until
 * then. Unknown codes (a newer robot than this GUI) still show their raw code
 * and detail rather than disappearing.
 */
export function StopReasonBanner({reason, detail, sessionActive = false, variants}: StopReasonBannerProps) {
    const {t} = useTranslation();
    if (!reason || sessionActive) return null;
    const known = isKnownStopReason(reason);
    return (
        <motion.div
            variants={variants}
            role="alert"
            data-testid="stop-reason-banner"
            style={{
                display: "flex", alignItems: "flex-start", gap: 12,
                padding: "12px 16px", marginBottom: 14, borderRadius: 14,
                background: "rgba(255,184,77,0.10)",
                border: "1px solid rgba(255,184,77,0.35)",
                boxShadow: "0 0 18px rgba(255,184,77,0.12)",
            }}
        >
            <OctagonAlert size={18} style={{color: "var(--amber, #FFB84D)", flexShrink: 0, marginTop: 1}}/>
            <div style={{minWidth: 0}}>
                <div style={{fontSize: 13, fontWeight: 600, color: "var(--ink, #ECFFF4)"}}>
                    {known ? t(`stopReason.${reason}.title`) : t("stopReason.genericTitle")}
                </div>
                {known && (
                    <div style={{fontSize: 12, color: "rgba(236,255,244,0.75)", marginTop: 2}}>
                        {t(`stopReason.${reason}.hint`)}
                    </div>
                )}
                {(detail || !known) && (
                    <div data-testid="stop-reason-detail"
                         style={{fontSize: 11, color: "rgba(236,255,244,0.55)", marginTop: 4}}>
                        {[known ? null : reason, detail].filter(Boolean).join(" · ")}
                    </div>
                )}
            </div>
        </motion.div>
    );
}
