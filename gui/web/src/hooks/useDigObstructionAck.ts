import {App} from "antd";
import {useTranslation} from "react-i18next";
import {useMowerAction} from "../components/MowerActions.tsx";
import {useStatus} from "./useStatus.ts";

export const DIG_OBSTRUCTION_STATE = "DIG_OBSTRUCTION";

type MowerCall = () => Promise<void>;

/**
 * Wraps a "start mowing" action so it also works while the robot is held by
 * a repeat-dig escalation (DIG_OBSTRUCTION). The behaviour tree refuses START
 * while /hardware_bridge/dig_escalated is latched, so the latch has to be
 * released first through ~/clear_dig_escalation:
 *
 * - moved past dig_escalated_required_distance_m: a plain clear, no prompt;
 * - still at the spot: the operator confirms an ACKNOWLEDGEMENT. The bridge
 *   then keeps the spot on probation — one more dig there holds the robot
 *   again (mowgli_hardware/dig_escalation.hpp, DigProbation).
 *
 * Outside an escalation the wrapped action runs unchanged.
 */
export const useDigObstructionAck = (stateName?: string) => {
    const {modal} = App.useApp();
    const {t} = useTranslation();
    const mowerAction = useMowerAction();
    const status = useStatus();

    return (start: MowerCall): MowerCall => async () => {
        const escalated = status.dig_escalated === true || stateName === DIG_OBSTRUCTION_STATE;
        if (!escalated) return start();

        const release = async () => {
            await mowerAction("clear_dig_escalation")();
            await start();
        };
        const distance = status.dig_escalated_distance_m ?? 0;
        const required = status.dig_escalated_required_distance_m ?? 0.5;
        // Strict > — the same boundary as CanClearDigEscalation server-side.
        if (distance > required) return release();

        return new Promise<void>((resolve, reject) => {
            modal.confirm({
                title: t("digObstructionAck.title"),
                content: t("digObstructionAck.body", {distance: distance.toFixed(2)}),
                okText: t("digObstructionAck.ok"),
                okType: "danger",
                cancelText: t("digObstructionAck.cancel"),
                onOk: async () => {
                    try {
                        await release();
                        resolve();
                    } catch (e) {
                        reject(e instanceof Error ? e : new Error(String(e)));
                    }
                },
                onCancel: () => resolve(),
            });
        });
    };
};
