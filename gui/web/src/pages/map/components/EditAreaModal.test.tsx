import {describe, it, expect, vi} from 'vitest';
import {render, screen} from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import {EditAreaModal} from './EditAreaModal.tsx';
import {MowingAreaEdit} from '../utils/types.ts';
import i18n from "../../../i18n";
import en from "../../../i18n/locales/en.json";

describe('EditAreaModal', () => {
    const area = new MowingAreaEdit();
    area.name = 'Garden';
    area.mowing_order = 2;
    area.feature_type = 'workarea';
    area.orig_feature_type = 'workarea';

    const defaultProps = {
        open: true,
        area,
        onChange: vi.fn(),
        onSave: vi.fn(),
        onCancel: vi.fn(),
    };

    it('renders with area name in title', () => {
        render(<EditAreaModal {...defaultProps} />);
        expect(screen.getByText(i18n.t('mapEditArea.titleEditNamed', {name: 'Garden'}))).toBeInTheDocument();
    });

    it('shows generic title when no name', () => {
        const noNameArea = new MowingAreaEdit();
        render(<EditAreaModal {...defaultProps} area={noNameArea} />);
        expect(screen.getByText(en.mapEditArea.titleEdit)).toBeInTheDocument();
    });

    it('renders with area name in input for workarea', () => {
        render(<EditAreaModal {...defaultProps} />);
        expect(screen.getByDisplayValue('Garden')).toBeInTheDocument();
    });

    it('renders with mowing order for workarea', () => {
        render(<EditAreaModal {...defaultProps} />);
        expect(screen.getByDisplayValue('2')).toBeInTheDocument();
    });

    it('shows area type selector', () => {
        render(<EditAreaModal {...defaultProps} />);
        expect(screen.getByText(en.mapEditArea.areaTypeMowing)).toBeInTheDocument();
    });

    it('hides name and mowing order for navigation type', () => {
        const navArea = new MowingAreaEdit();
        navArea.feature_type = 'navigation';
        render(<EditAreaModal {...defaultProps} area={navArea} />);
        expect(screen.queryByDisplayValue('Garden')).not.toBeInTheDocument();
        expect(screen.queryByText(en.mapEditArea.mowingOrder)).not.toBeInTheDocument();
    });

    it('hides name and mowing order for obstacle type', () => {
        const obstArea = new MowingAreaEdit();
        obstArea.feature_type = 'obstacle';
        render(<EditAreaModal {...defaultProps} area={obstArea} />);
        expect(screen.queryByText(en.mapEditArea.areaName)).not.toBeInTheDocument();
        expect(screen.queryByText(en.mapEditArea.mowingOrder)).not.toBeInTheDocument();
    });

    it('offers the recorded-outline shrink only when converting INTO an obstacle', () => {
        const converting = new MowingAreaEdit();
        converting.feature_type = 'obstacle';
        converting.orig_feature_type = 'workarea';
        const {unmount} = render(<EditAreaModal {...defaultProps} area={converting} />);
        expect(screen.getByText(en.mapEditArea.shrinkRecorded)).toBeInTheDocument();
        unmount();

        // An obstacle that already is one must never be shrunk a second time.
        const existing = new MowingAreaEdit();
        existing.feature_type = 'obstacle';
        existing.orig_feature_type = 'obstacle';
        render(<EditAreaModal {...defaultProps} area={existing} />);
        expect(screen.queryByText(en.mapEditArea.shrinkRecorded)).not.toBeInTheDocument();
    });

    it('reports the shrink choice through onChange', async () => {
        const onChange = vi.fn();
        const user = userEvent.setup();
        const converting = new MowingAreaEdit();
        converting.feature_type = 'obstacle';
        converting.orig_feature_type = 'workarea';
        render(<EditAreaModal {...defaultProps} area={converting} onChange={onChange} />);
        await user.click(screen.getByText(en.mapEditArea.shrinkRecorded));
        expect(onChange).toHaveBeenCalledWith(expect.objectContaining({shrink_recorded: false}));
    });

    it('does not render when closed', () => {
        render(<EditAreaModal {...defaultProps} open={false} />);
        expect(screen.queryByText(i18n.t('mapEditArea.titleEditNamed', {name: 'Garden'}))).not.toBeInTheDocument();
    });

    it('calls onSave when Save clicked', async () => {
        const onSave = vi.fn();
        const user = userEvent.setup();
        render(<EditAreaModal {...defaultProps} onSave={onSave} />);
        await user.click(screen.getByText(en.mapEditArea.save));
        expect(onSave).toHaveBeenCalled();
    });

    it('calls onCancel when Cancel clicked', async () => {
        const onCancel = vi.fn();
        const user = userEvent.setup();
        render(<EditAreaModal {...defaultProps} onCancel={onCancel} />);
        await user.click(screen.getByText(en.mapEditArea.cancel));
        expect(onCancel).toHaveBeenCalled();
    });

    it('calls onChange when name is edited', async () => {
        const onChange = vi.fn();
        const user = userEvent.setup();
        render(<EditAreaModal {...defaultProps} onChange={onChange} />);
        const input = screen.getByDisplayValue('Garden');
        await user.clear(input);
        await user.type(input, 'Back Yard');
        expect(onChange).toHaveBeenCalled();
    });
});
