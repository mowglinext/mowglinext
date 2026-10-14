import {App} from "antd";
import type {ComponentProps} from "react";
import { beforeEach, describe, expect, it, vi } from "vitest";
import { render, screen, waitFor } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { MapToolbarMobile } from "./MapToolbarMobile.tsx";
import en from "../../../i18n/locales/en.json";

vi.mock("../../../theme/ThemeContext.tsx", () => ({
  useThemeMode: () => ({
    colors: {
      glassBackground: "#111",
      glassBorder: "1px solid #222",
      glassShadow: "none",
      bgElevated: "#222",
      danger: "#f00",
      text: "#fff",
    },
  }),
}));

vi.mock("../../../components/AsyncButton.tsx", () => ({
  default: ({ children, onAsyncClick, ...props }: any) => (
    <button onClick={onAsyncClick} {...props}>
      {children}
    </button>
  ),
}));

vi.mock("./ShapePickerDropdown.tsx", () => ({
  ShapePickerDropdown: ({ children }: any) => children,
}));

describe("MapToolbarMobile", () => {
  const defaultProps = {
    editMap: false,
    hasUnsavedChanges: false,
    manualMode: false,
    useSatellite: true,
    historyIndex: 0,
    editHistoryLength: 1,
    mowingAreas: [],
    stateName: "IDLE",
    highLevelState: 1,
    emergency: false,
    onEditMap: vi.fn(),
    onSaveMap: vi.fn().mockResolvedValue(undefined),
    onUndo: vi.fn(),
    onRedo: vi.fn(),
    onToggleSatellite: vi.fn(),
    onManualMode: vi.fn().mockResolvedValue(undefined),
    onStopManualMode: vi.fn().mockResolvedValue(undefined),
    onBackupMap: vi.fn(),
    onRestoreMap: vi.fn(),
    onDownloadGeoJSON: vi.fn(),
    onUploadGeoJSON: vi.fn(),
    onImportOpenMower: vi.fn(),
    onResetMowingProgress: vi.fn(),
    onMowArea: vi.fn().mockResolvedValue(undefined),
    onStart: vi.fn().mockResolvedValue(undefined),
    onHome: vi.fn().mockResolvedValue(undefined),
    onEmergencyOn: vi.fn().mockResolvedValue(undefined),
    onEmergencyOff: vi.fn().mockResolvedValue(undefined),
  };

    it("links to Hardware instead of offering separate mower/dock image selectors", async () => {
        const user = userEvent.setup();
        render(<MapToolbarMobile {...defaultProps} />);
        await user.click(screen.getByLabelText(en.mapToolbarMobile.more));
        expect(screen.getByRole("link", {name: en.mowerVisual.hardwareLink})).toHaveAttribute("href", "#/settings?section=hardware");
        expect(screen.queryByText(en.mapToolbar.dockAppearance)).not.toBeInTheDocument();
        expect(screen.queryByText(en.mapToolbar.mowerAppearance)).not.toBeInTheDocument();
    });

  it("reports a rejected area start and clears its loading state", async () => {
    const user = userEvent.setup();
    const log = vi.spyOn(console, 'error').mockImplementation(() => undefined);
    const onMowArea = vi.fn().mockRejectedValueOnce(new Error('Cannot start this area'));
    const mowingAreas: ComponentProps<typeof MapToolbarMobile>['mowingAreas'] = [{
      key: 'area-7', label: 'Front lawn',
      feat: {type: 'Feature', geometry: {type: 'Polygon', coordinates: []}, properties: {}},
    }];
    render(<App><MapToolbarMobile {...defaultProps} mowingAreas={mowingAreas} onMowArea={onMowArea}/></App>);
    await user.click(screen.getByRole('button', {name: en.mapToolbarMobile.mowArea}));
    await user.click(screen.getByText('Front lawn'));
    expect(await screen.findByText('Cannot start this area')).toBeInTheDocument();
    expect(onMowArea).toHaveBeenCalledExactlyOnceWith('area-7');
    await waitFor(() => expect(screen.getByRole('button', {name: en.mapToolbarMobile.mowArea})).not.toHaveClass('ant-btn-loading'));
    log.mockRestore();
  });

  beforeEach(() => {
    vi.clearAllMocks();
  });

  it("offers the reset action in the mobile data menu while idle", async () => {
    const user = userEvent.setup();
    render(<MapToolbarMobile {...defaultProps} />);

    await user.click(screen.getByLabelText(en.mapToolbarMobile.more));
    await user.click(screen.getByText(en.resetMowingProgress.action));

    expect(defaultProps.onResetMowingProgress).toHaveBeenCalledOnce();
  });
});
