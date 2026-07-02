import {App} from "antd";
import {render, screen} from "@testing-library/react";
import {describe, expect, it} from "vitest";
import en from "../../i18n/locales/en.json";
import {gnssStatusSamples} from "../../test/mocks.tsx";
import {UniversalGnssLiveStatusCard} from "./UniversalGnssLiveStatusCard.tsx";

const renderCard = (sample: keyof typeof gnssStatusSamples) => render(
    <App>
        <UniversalGnssLiveStatusCard
            diagnostics={undefined}
            gnssStatus={gnssStatusSamples[sample]}
            selectedBaud={921600}
            selectedConfigBaud={921600}
            selectedProfile="rover_high_precision"
            selectedSignalProfile="all_signals"
            selectedReceiverFamily="auto"
        />
    </App>,
);

describe("UniversalGnssLiveStatusCard", () => {
    it("renders the solved UM982 baseline sample with canonical Universal GNSS labels", () => {
        renderCard("dual_antenna_um982_solved");

        expect(screen.getByText("Unicore UM982")).toBeInTheDocument();
        expect(screen.getByText(en.gpsStatus.rtkFixed)).toBeInTheDocument();
        expect(screen.getByText(en.gpsStatus.correctionStreamActive)).toBeInTheDocument();
        expect(screen.getByText(en.gpsStatus.baselineComputed)).toBeInTheDocument();
        expect(screen.getByText("184.32")).toBeInTheDocument();
        expect(screen.getByText("-0.85")).toBeInTheDocument();
        expect(screen.getByText("1.247")).toBeInTheDocument();
        expect(screen.getByText("GPS+GLO+GAL · MSM 7 · station 4095 · sat 18 · sig 28 · cell 52")).toBeInTheDocument();
    });

    it("keeps baseline-unavailable samples explicit instead of inventing solved values", () => {
        renderCard("baseline_unavailable_unknown");

        expect(screen.getByText("Unicore UM982")).toBeInTheDocument();
        expect(screen.getByText(en.gpsStatus.gpsFix)).toBeInTheDocument();
        expect(screen.queryByText("184.32")).not.toBeInTheDocument();
        expect(screen.getAllByText(en.settingsGnssLiveStatus.unknown).length).toBeGreaterThan(0);
    });

    it.each([
        ["ntrip_startup_waiting", en.gpsStatus.correctionStreamWaiting],
        ["correction_stream_active", en.gpsStatus.correctionStreamActive],
        ["correction_stream_unavailable", en.gpsStatus.correctionStreamUnavailable],
        ["correction_stream_error", en.gpsStatus.correctionStreamError],
    ] as const)("renders correction stream sample %s with the right state label", (sample, expectedLabel) => {
        renderCard(sample);
        expect(screen.getByText(expectedLabel)).toBeInTheDocument();
    });

    it("renders the Generic NMEA sample from the public rtk_mode projection", () => {
        renderCard("nmea_gga_fix_quality_float");

        expect(screen.getByText("Generic NMEA")).toBeInTheDocument();
        expect(screen.getByText(en.gpsStatus.rtkFloat)).toBeInTheDocument();
    });
});
