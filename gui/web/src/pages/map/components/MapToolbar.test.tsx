import {describe, it, expect, vi, beforeEach} from 'vitest';
import {render, screen} from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import {MapToolbar} from './MapToolbar.tsx';
import en from "../../../i18n/locales/en.json";

vi.mock('../../../components/AsyncButton.tsx', () => ({
    default: ({children, onAsyncClick, ...props}: any) => (
        <button onClick={onAsyncClick} {...props}>{children}</button>
    ),
}));
vi.mock('../../../components/AsyncDropDownButton.tsx', () => ({
    default: ({children, ...props}: any) => (
        <button {...props}>{children}</button>
    ),
}));

describe('MapToolbar', () => {
    const defaultProps = {
        manualMode: false,
        useSatellite: true,
        mowingAreas: [],
        stateName: 'IDLE',
        emergency: false,
        onEditMap: vi.fn(),
        onToggleSatellite: vi.fn(),
        onManualMode: vi.fn().mockResolvedValue(undefined),
        onStopManualMode: vi.fn().mockResolvedValue(undefined),
        onBackupMap: vi.fn(),
        onRestoreMap: vi.fn(),
        onDownloadGeoJSON: vi.fn(),
        onImportOpenMower: vi.fn(),
        onMowArea: vi.fn().mockResolvedValue(undefined),
        onStart: vi.fn().mockResolvedValue(undefined),
        onHome: vi.fn().mockResolvedValue(undefined),
        onEmergencyOn: vi.fn().mockResolvedValue(undefined),
        onEmergencyOff: vi.fn().mockResolvedValue(undefined),
    };

    beforeEach(() => {
        vi.clearAllMocks();
    });

    it('renders Edit Map button', () => {
        render(<MapToolbar {...defaultProps} />);
        expect(screen.getByText(en.mapToolbar.editMap)).toBeInTheDocument();
    });

    it('shows Start button when IDLE', () => {
        render(<MapToolbar {...defaultProps} stateName="IDLE" />);
        expect(screen.getByText(en.mapToolbar.start)).toBeInTheDocument();
    });

    it('shows Home button when not IDLE', () => {
        render(<MapToolbar {...defaultProps} stateName="MOWING" />);
        expect(screen.getByText(en.mapToolbar.home)).toBeInTheDocument();
    });

    it('shows Emergency On when no emergency', () => {
        render(<MapToolbar {...defaultProps} emergency={false} />);
        expect(screen.getByText(en.mapToolbar.emergencyOn)).toBeInTheDocument();
    });

    it('shows Emergency Off when emergency active', () => {
        render(<MapToolbar {...defaultProps} emergency={true} />);
        expect(screen.getByText(en.mapToolbar.emergencyOff)).toBeInTheDocument();
    });

    it('shows Mow area dropdown', () => {
        render(<MapToolbar {...defaultProps} />);
        expect(screen.getByText(en.mapToolbar.mowArea)).toBeInTheDocument();
    });

    it('always shows the More dropdown trigger', () => {
        render(<MapToolbar {...defaultProps} />);
        expect(screen.getByText(en.mapToolbar.more)).toBeInTheDocument();
    });

    it('shows Backup Map and Restore Map in More dropdown', async () => {
        const user = userEvent.setup();
        render(<MapToolbar {...defaultProps} />);
        await user.click(screen.getByText(en.mapToolbar.more));
        expect(screen.getByText(en.mapToolbar.backupMap)).toBeInTheDocument();
        expect(screen.getByText(en.mapToolbar.restoreMap)).toBeInTheDocument();
    });

    it('shows Download GeoJSON in More dropdown', async () => {
        const user = userEvent.setup();
        render(<MapToolbar {...defaultProps} />);
        await user.click(screen.getByText(en.mapToolbar.more));
        expect(screen.getByText(en.mapToolbar.downloadGeojson)).toBeInTheDocument();
    });

    it('calls onEditMap when Edit Map clicked', async () => {
        const user = userEvent.setup();
        render(<MapToolbar {...defaultProps} />);
        await user.click(screen.getByText(en.mapToolbar.editMap));
        expect(defaultProps.onEditMap).toHaveBeenCalled();
    });

    it('calls onBackupMap when Backup Map clicked in More dropdown', async () => {
        const user = userEvent.setup();
        render(<MapToolbar {...defaultProps} />);
        await user.click(screen.getByText(en.mapToolbar.more));
        await user.click(screen.getByText(en.mapToolbar.backupMap));
        expect(defaultProps.onBackupMap).toHaveBeenCalled();
    });

    it('calls onRestoreMap when Restore Map clicked in More dropdown', async () => {
        const user = userEvent.setup();
        render(<MapToolbar {...defaultProps} />);
        await user.click(screen.getByText(en.mapToolbar.more));
        await user.click(screen.getByText(en.mapToolbar.restoreMap));
        expect(defaultProps.onRestoreMap).toHaveBeenCalled();
    });

    it('calls onDownloadGeoJSON when Download GeoJSON clicked in More dropdown', async () => {
        const user = userEvent.setup();
        render(<MapToolbar {...defaultProps} />);
        await user.click(screen.getByText(en.mapToolbar.more));
        await user.click(screen.getByText(en.mapToolbar.downloadGeojson));
        expect(defaultProps.onDownloadGeoJSON).toHaveBeenCalled();
    });
});
