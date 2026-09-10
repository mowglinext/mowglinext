import {motion, type Variants} from "framer-motion";
import {Droplets} from "lucide-react";
import {useTranslation} from "react-i18next";
import {useSoilStatus} from "../../hooks/useSoilStatus.ts";
import {soilVerdict, type SoilStatus} from "../../types/irrisense.ts";

interface SoilWetBannerProps {
    /** Injected for tests; the dashboard lets the hook fetch it. */
    status?: SoilStatus | null;
    variants?: Variants;
}

/**
 * Dashboard warning shown while IrriSense Cloud reports the garden as WET.
 * Renders nothing when the integration is off, the verdict is dry, or the
 * status is unknown (fail-open: an outage must not look like a warning).
 */
export function SoilWetBanner({status: injected, variants}: SoilWetBannerProps) {
    const {t} = useTranslation();
    const polled = useSoilStatus({enabled: injected === undefined});
    const status = injected === undefined ? polled.status : injected;
    if (!status || !status.enabled || soilVerdict(status) !== "wet") return null;
    const details = [
        status.reason,
        status.gateScheduler ? t("mowgliNextPage.soilWetBlocking") : null,
    ].filter(Boolean).join(" · ");
    return (
        <motion.div
            variants={variants}
            role="status"
            data-testid="soil-wet-banner"
            style={{
                display: "flex", alignItems: "center", gap: 12,
                padding: "12px 16px", marginBottom: 14, borderRadius: 14,
                background: "rgba(96,176,255,0.10)",
                border: "1px solid rgba(96,176,255,0.35)",
                boxShadow: "0 0 18px rgba(96,176,255,0.12)",
            }}
        >
            <Droplets size={18} style={{color: "var(--sky, #60B0FF)", flexShrink: 0}}/>
            <div style={{minWidth: 0}}>
                <div style={{fontSize: 13, fontWeight: 600, color: "var(--ink, #ECFFF4)"}}>
                    {t("mowgliNextPage.soilWetTitle")}
                </div>
                {details && (
                    <div style={{fontSize: 11, color: "rgba(236,255,244,0.6)"}}>{details}</div>
                )}
            </div>
        </motion.div>
    );
}
