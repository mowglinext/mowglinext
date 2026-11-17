import {motion, type Variants} from "framer-motion";
import {AlertTriangle} from "lucide-react";
import {useTranslation} from "react-i18next";
import {useStatus} from "../../hooks/useStatus.ts";
import {useMowerAction} from "../MowerActions.tsx";
import {AsyncButton} from "../AsyncButton.tsx";
import type {Status} from "../../types/ros.ts";
import {useDigObstructionAck} from "../../hooks/useDigObstructionAck.ts";

interface DigEscalationBannerProps {
    /** Injected for tests; the dashboard lets the hook fetch it. */
    status?: Pick<Status, "dig_escalated" | "dig_escalated_distance_m" | "dig_escalated_required_distance_m">;
    variants?: Variants;
    /** Coordinate the entire acknowledgement → clear → START with the page. */
    runCommand?: (command: () => Promise<void>) => Promise<unknown>;
    commandPending?: boolean;
}

/**
 * Dashboard warning shown while hardware_bridge has latched a repeat-dig
 * escalation (mowgli_hardware/dig_escalation.hpp) — three-or-more digs in
 * the same spot, the robot judged unable to free itself, mowing halted. The
 * latch clears itself once the robot reaches the charger or once the fused
 * pose is 2x dig_escalate_radius_m from the spot; this banner is the operator
 * path in between: "Clear & resume" releases the hold and sends START.
 * Once the chassis has moved past dig_escalated_required_distance_m it is a
 * plain clear; closer to the spot the operator first confirms an
 * acknowledgement, and the bridge keeps the spot on probation so one more
 * dig there holds the robot again (field report 2026-10-16: on the spot only
 * HOME used to act). Failures surface via AsyncButton's error notification.
 */
export function DigEscalationBanner({status: injected, variants, runCommand, commandPending = false}: DigEscalationBannerProps) {
    const {t} = useTranslation();
    const polled = useStatus();
    const status = injected ?? polled;
    const mowerAction = useMowerAction();
    const ackStart = useDigObstructionAck();
    if (!status.dig_escalated) return null;

    const distance = status.dig_escalated_distance_m ?? 0;
    const required = status.dig_escalated_required_distance_m ?? 0.5;
    // Strict > — matches CanClearDigEscalation server-side exactly (see
    // dig_escalation.hpp): past it the release needs no acknowledgement.
    const canClear = distance > required;
    const clearAndResume = ackStart(mowerAction("high_level_control", {Command: 1}));

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
                disabled={commandPending}
                onAsyncClick={runCommand ? () => runCommand(clearAndResume) : clearAndResume}
            >
                {t("mowgliNextPage.digEscalationClear")}
            </AsyncButton>
        </motion.div>
    );
}
