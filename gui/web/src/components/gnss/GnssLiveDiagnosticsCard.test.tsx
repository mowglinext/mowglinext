import { App } from "antd";
import { render, screen } from "@testing-library/react";
import type { ComponentProps } from "react";
import { describe, expect, it } from "vitest";
import en from "../../i18n/locales/en.json";
import { gnssStatusSamples } from "../../test/mocks.tsx";
import { GnssLiveDiagnosticsCard } from "./GnssLiveDiagnosticsCard.tsx";

const renderCard = (
    sample: keyof typeof gnssStatusSamples,
    overrides?: Partial<ComponentProps<typeof GnssLiveDiagnosticsCard>>,
) => render(
    <App>
        <GnssLiveDiagnosticsCard
            gnssStatus={gnssStatusSamples[sample]}
            latitude={48.123456789}
            longitude={2.987654321}
            altitudeM={123.456}
            horizontalAccuracyM={0.012}
            {...overrides}
        />
    </App>,
);

describe("GnssLiveDiagnosticsCard", () => {
    it("renders the solved UM982 sample as the rich diagnostics card used by Diagnostic > Localisation", () => {
        renderCard("dual_antenna_um982_solved");

        expect(screen.getByText(en.diagnosticsPage.gpsGnssTitle)).toBeInTheDocument();
        expect(screen.getByText(en.diagnosticsPage.position)).toBeInTheDocument();
        expect(screen.getByText(en.diagnosticsPage.latitude)).toBeInTheDocument();
        expect(screen.getByText(en.diagnosticsPage.longitude)).toBeInTheDocument();
        expect(screen.getAllByText(en.gpsStatus.rtkFixed).length).toBeGreaterThan(0);
        expect(screen.getAllByText(en.gpsStatus.correctionStreamActive).length).toBeGreaterThan(0);
        expect(screen.getByText("Unicore UM982")).toBeInTheDocument();
        expect(screen.getByText("18 / 24")).toBeInTheDocument();
        expect(screen.getByText("21 / 24")).toBeInTheDocument();
        expect(screen.getByText("41.5 dB-Hz")).toBeInTheDocument();
        expect(screen.getByText("50.0 dB-Hz")).toBeInTheDocument();
        expect(screen.getByText(en.settingsGnssLiveStatus.msmStateValid)).toBeInTheDocument();
        expect(screen.getByText("GPS+GLO+GAL")).toBeInTheDocument();
        expect(screen.getByText("4095")).toBeInTheDocument();
        expect(screen.getByText(en.gpsStatus.baselineComputed)).toBeInTheDocument();
        expect(screen.getByText("184.32")).toBeInTheDocument();
        expect(screen.getByTestId("gnss-satellites-used-fill")).toHaveStyle({ width: "75.0%" });
        expect(screen.getByTestId("gnss-satellites-tracked-fill")).toHaveStyle({ width: "87.5%" });
        expect(screen.getByTestId("gnss-cn0-mean-fill")).toHaveStyle({ width: "92.2%" });
        expect(screen.getByTestId("gnss-cn0-max-fill")).toHaveStyle({ width: "100.0%" });
    });

    it("keeps unknown satellite and C/N0 states explicit without fake bars", () => {
        renderCard("satellite_cn0_unknown", {
            latitude: undefined,
            longitude: undefined,
            altitudeM: undefined,
            horizontalAccuracyM: undefined,
        });

        expect(screen.getByText(en.settingsGnssLiveStatus.satelliteSectionTitle)).toBeInTheDocument();
        expect(screen.getByText(en.settingsGnssLiveStatus.signalQualitySectionTitle)).toBeInTheDocument();
        expect(screen.getAllByText(en.diagnosticsPage.unknown).length).toBeGreaterThan(0);
        expect(screen.queryByTestId("gnss-satellites-used-fill")).not.toBeInTheDocument();
        expect(screen.queryByTestId("gnss-satellites-tracked-fill")).not.toBeInTheDocument();
        expect(screen.queryByTestId("gnss-cn0-mean-fill")).not.toBeInTheDocument();
        expect(screen.queryByTestId("gnss-cn0-max-fill")).not.toBeInTheDocument();
    });

    it.each([
        ["ntrip_startup_waiting", en.gpsStatus.correctionStreamWaiting],
        ["correction_stream_active", en.gpsStatus.correctionStreamActive],
        ["correction_stream_unavailable", en.gpsStatus.correctionStreamUnavailable],
        ["correction_stream_error", en.gpsStatus.correctionStreamError],
    ] as const)("renders correction stream sample %s with the right state label", (sample, expectedLabel) => {
        renderCard(sample);
        expect(screen.getAllByText(expectedLabel).length).toBeGreaterThan(0);
    });

    it("renders a valid MSM state explicitly", () => {
        renderCard("msm_summary_present");
        expect(screen.getByText(en.settingsGnssLiveStatus.msmStateValid)).toBeInTheDocument();
    });

    it("renders a seen-but-undecoded MSM state explicitly", () => {
        renderCard("msm_malformed_not_decoded");
        expect(screen.getByText(en.settingsGnssLiveStatus.msmStateSeenUndecoded)).toBeInTheDocument();
    });

    it("renders a not-seen MSM state explicitly", () => {
        renderCard("msm_summary_not_seen");
        expect(screen.getByText(en.settingsGnssLiveStatus.msmStateNotSeen)).toBeInTheDocument();
    });
});
