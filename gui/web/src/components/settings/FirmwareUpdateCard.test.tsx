import { render, screen } from "@testing-library/react";
import { beforeEach, describe, expect, it, vi } from "vitest";
import { FirmwareUpdateCard } from "./FirmwareUpdateCard.tsx";
import type { AvailableFirmware } from "../../hooks/useAvailableFirmware.ts";

let available: { data: AvailableFirmware | null; error: string | null; loading: boolean; refresh: () => Promise<void> };
let isCharging = true;

vi.mock("../../hooks/useAvailableFirmware.ts", () => ({
    useAvailableFirmware: () => available,
}));
vi.mock("../../hooks/useHighLevelStatus.ts", () => ({
    useHighLevelStatus: () => ({ highLevelStatus: { is_charging: isCharging } }),
}));
vi.mock("../FlashBoardComponent.tsx", () => ({
    FlashBoardComponent: () => <div>flash-form</div>,
}));

const offer = (overrides: Partial<AvailableFirmware> = {}): AvailableFirmware => ({
    board: "BOARD_YARDFORCE500",
    panel: "PANEL_TYPE_YARDFORCE_500_CLASSIC",
    available: true,
    fw_version: "1.10.200",
    protocol_version: 7,
    release: "deployment-abc-1-1",
    own_release: true,
    ...overrides,
});

describe("FirmwareUpdateCard", () => {
    beforeEach(() => {
        available = { data: offer(), error: null, loading: false, refresh: vi.fn() };
        isCharging = true;
    });

    it("offers to flash a different firmware version from this installation", () => {
        render(<FirmwareUpdateCard firmwareVersion="1.10.192" state="compatible" advanced={false} />);
        expect(screen.getByText(/Firmware 1.10.200 \(protocol 7\) is available/)).toBeInTheDocument();
        expect(screen.getByRole("button", { name: "Flash firmware 1.10.200" })).toBeEnabled();
    });

    it("offers nothing when the board already runs this installation's firmware", () => {
        render(<FirmwareUpdateCard firmwareVersion="1.10.200" state="compatible" advanced={false} />);
        expect(screen.getByText(/runs the firmware of this installation/)).toBeInTheDocument();
        expect(screen.queryByRole("button", { name: /Flash firmware/ })).not.toBeInTheDocument();
    });

    it("only flashes on the charger while the running firmware still works", () => {
        isCharging = false;
        render(<FirmwareUpdateCard firmwareVersion="1.10.192" state="compatible" advanced={false} />);
        expect(screen.getByRole("button", { name: "Flash firmware 1.10.200" })).toBeDisabled();
    });

    it("lets an incompatible firmware be replaced anywhere (the stack cannot drive it anyway)", () => {
        isCharging = false;
        render(<FirmwareUpdateCard firmwareVersion="1.10.192" state="incompatible" advanced={false} />);
        expect(screen.getByRole("button", { name: "Flash firmware 1.10.200" })).toBeEnabled();
    });

    it("says when the firmware comes from the latest stable release instead", () => {
        available.data = offer({ own_release: false, release: "v1.4.0" });
        render(<FirmwareUpdateCard firmwareVersion="1.10.192" state="compatible" advanced={false} />);
        expect(screen.getByText(/taken from the latest stable release v1.4.0/)).toBeInTheDocument();
    });

    it("points to the onboarding when no board was ever selected", () => {
        available.data = offer({ board: "", available: false, fw_version: undefined });
        render(<FirmwareUpdateCard firmwareVersion="1.10.192" state="compatible" advanced={false} />);
        expect(screen.getByText(/No board has been selected yet/)).toBeInTheDocument();
        expect(screen.queryByRole("button", { name: /Flash firmware/ })).not.toBeInTheDocument();
    });
});
