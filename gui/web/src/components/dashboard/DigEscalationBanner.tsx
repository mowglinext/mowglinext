import {motion, type Variants} from "framer-motion";
import {AlertTriangle} from "lucide-react";
import {useTranslation} from "react-i18next";
import {useStatus} from "../../hooks/useStatus.ts";
import {useMowerAction} from "../MowerActions.tsx";
import {AsyncButton} from "../AsyncButton.tsx";
import type {Status} from "../../types/ros.ts";

interface DigEscalationBannerProps {
    /** Injected for tests; the dashboard lets the hook fetch it. */
    status?: Pick<Status, "dig_escalated" | "dig_escalated_distance_m" | "dig_escalated_required_distance_m">;
    variants?: Variants;
}

/**
 * Dashboard warning shown while hardware_bridge has latched a repeat-dig
 * escalation (mowgli_hardware/dig_escalation.hpp) — three-or-more digs in
 * the same spot, the robot judged unable to free itself, mowing halted. The
 * latch clears itself once the robot reaches the charger or once the fused
 * pose is 2x dig_escalate_radius_m from the spot; this banner is the operator
 * path in between (issue reported 2026-09-14): an operator who has already
 * moved the mower away from the obstruction by hand can clear it here
 * instead of driving the whole way home or carrying it a full metre. The Clear button stays disabled until the
 * chassis has actually moved dig_escalated_required_distance_m past the
 * spot the escalation latched at — ~/clear_dig_escalation enforces the same
 * distance server-side, so a stale/racy click still fails safely with the
 * backend's own reason surfaced via AsyncButton's error notification.
 */
export function DigEscalationBanner({status: injected, variants}: DigEscalationBannerProps) {
    const {t} = useTranslation();
    const polled = useStatus();
    const status = injected ?? polled;
    const mowerAction = useMowerAction();
    if (!status.dig_escalated) return null;

    const distance = status.dig_escalated_distance_m ?? 0;
    const required = status.dig_escalated_required_distance_m ?? 0.5;
    // Strict > — matches CanClearDigEscalation server-side exactly (see
    // dig_escalation.hpp), so a click never reaches the backend only to be
    // refused at a boundary the button claimed was already clear.
    const canClear = distance > required;
    const clearAction = mowerAction("clear_dig_escalation");

    return (
        <motion.div
            variants={variants}
            role="alert"
            data-testid="dig-escalation-banner"
            style={{
                display: "flex", alignItems: "center", gap: 12,
                padding: "12px 16px", marginBottom: 14, borderRadius: 14,
                background: "rgba(255,77,79,0.10)",
                border: "1px solid rgba(255,77,79,0.35)",
                boxShadow: "0 0 18px rgba(255,77,79,0.12)",
            }}
        >
            <AlertTriangle size={18} style={{color: "var(--danger, #FF4D4F)", flexShrink: 0}}/>
            <div style={{minWidth: 0, flex: 1}}>
                <div style={{fontSize: 13, fontWeight: 600, color: "var(--ink, #ECFFF4)"}}>
                    {t("mowgliNextPage.digEscalationTitle")}
                </div>
                <div style={{fontSize: 11, color: "rgba(236,255,244,0.6)"}}>
                    {canClear
                        ? t("mowgliNextPage.digEscalationReady", {distance: distance.toFixed(2)})
                        : t("mowgliNextPage.digEscalationBody", {
                            distance: distance.toFixed(2),
                            required: required.toFixed(2),
                        })}
                </div>
            </div>
            <AsyncButton
                danger
                size="small"
                disabled={!canClear}
                onAsyncClick={clearAction}
            >
                {t("mowgliNextPage.digEscalationClear")}
            </AsyncButton>
        </motion.div>
    );
}
