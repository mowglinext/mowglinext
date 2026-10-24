import {describe, expect, it} from "vitest";
import {render, screen} from "@testing-library/react";
import {StopReasonBanner} from "./StopReasonBanner.tsx";
import {KNOWN_STOP_REASONS} from "../../utils/stopReason.ts";
import {isSessionActive} from "../../utils/mowerMotion.ts";
import en from "../../i18n/locales/en.json";
import fr from "../../i18n/locales/fr.json";

describe("StopReasonBanner", () => {
    it("explains a known reason and shows the detail from the robot", () => {
        render(<StopReasonBanner reason="AREA_START_BLOCKED" detail="Area 3: robot at 0.54, -0.76"/>);
        const banner = screen.getByTestId("stop-reason-banner");
        expect(banner).toHaveTextContent(/own position is blocked/i);
        expect(banner).toHaveTextContent(/press Play/i);
        expect(screen.getByTestId("stop-reason-detail")).toHaveTextContent("Area 3: robot at 0.54, -0.76");
    });

    it("renders nothing when there is no reason", () => {
        const {rerender} = render(<StopReasonBanner reason="" detail="left over"/>);
        expect(screen.queryByTestId("stop-reason-banner")).not.toBeInTheDocument();
        rerender(<StopReasonBanner/>);
        expect(screen.queryByTestId("stop-reason-banner")).not.toBeInTheDocument();
    });

    it("is hidden while the mower is working and shown once it has stopped", () => {
        // The tree records AREA_START_BLOCKED when it retires ONE area and then
        // keeps mowing the others: "stopped, press Play" would be false there.
        const {rerender} = render(
            <StopReasonBanner reason="AREA_START_BLOCKED" detail="Area 3" sessionActive={isSessionActive(2)}/>);
        expect(screen.queryByTestId("stop-reason-banner")).not.toBeInTheDocument();

        // Session over: IDLE (state 1), docked or not.
        rerender(<StopReasonBanner reason="AREA_START_BLOCKED" detail="Area 3" sessionActive={isSessionActive(1)}/>);
        expect(screen.getByTestId("stop-reason-banner")).toHaveTextContent(/own position is blocked/i);
    });

    it("still shows a code this GUI does not know", () => {
        render(<StopReasonBanner reason="SOMETHING_NEW" detail="from a newer robot"/>);
        const banner = screen.getByTestId("stop-reason-banner");
        expect(banner).toHaveTextContent(/mower stopped/i);
        expect(screen.getByTestId("stop-reason-detail")).toHaveTextContent("SOMETHING_NEW · from a newer robot");
    });

    it("has a title and hint for every known code in every locale", () => {
        for (const locale of [en, fr] as Array<Record<string, any>>) {
            expect(typeof locale.stopReason.genericTitle).toBe("string");
            for (const code of KNOWN_STOP_REASONS) {
                expect(typeof locale.stopReason[code]?.title).toBe("string");
                expect(typeof locale.stopReason[code]?.hint).toBe("string");
            }
        }
    });
});
