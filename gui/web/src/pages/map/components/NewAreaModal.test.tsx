import {ConfigProvider} from 'antd';
import {afterEach, beforeEach, describe, it, expect, vi, type MockInstance} from 'vitest';
import {cleanup, fireEvent, render, screen, waitFor} from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import {NewAreaModal} from './NewAreaModal.tsx';
import en from "../../../i18n/locales/en.json";

describe('NewAreaModal', () => {
    const defaultProps = {
        open: true,
        areaType: 'workarea' as const,
        areaName: '',
        onAreaTypeChange: vi.fn(),
        onAreaNameChange: vi.fn(),
        onSave: vi.fn(),
        onCancel: vi.fn(),
    };

    it('renders when open', () => {
        render(<NewAreaModal {...defaultProps} />);
        expect(screen.getByText(en.mapNewArea.title)).toBeInTheDocument();
    });

    it('does not render content when closed', () => {
        render(<NewAreaModal {...defaultProps} open={false} />);
        expect(screen.queryByText(en.mapNewArea.title)).not.toBeInTheDocument();
    });

    it('shows name input for workarea type', () => {
        render(<NewAreaModal {...defaultProps} areaType="workarea" />);
        expect(screen.getByPlaceholderText(en.mapNewArea.areaNamePlaceholder)).toBeInTheDocument();
    });

    it('hides name input for navigation type', () => {
        render(<NewAreaModal {...defaultProps} areaType="navigation" />);
        expect(screen.queryByPlaceholderText(en.mapNewArea.areaNamePlaceholder)).not.toBeInTheDocument();
    });

    it('hides name input for obstacle type', () => {
        render(<NewAreaModal {...defaultProps} areaType="obstacle" />);
        expect(screen.queryByPlaceholderText(en.mapNewArea.areaNamePlaceholder)).not.toBeInTheDocument();
    });

    it('calls onSave when Add area clicked', async () => {
        const onSave = vi.fn();
        const user = userEvent.setup();
        render(<NewAreaModal {...defaultProps} onSave={onSave} />);
        await user.click(screen.getByText(en.mapNewArea.addArea));
        expect(onSave).toHaveBeenCalled();
    });

    it('calls onCancel when Cancel clicked', async () => {
        const onCancel = vi.fn();
        const user = userEvent.setup();
        render(<NewAreaModal {...defaultProps} onCancel={onCancel} />);
        await user.click(screen.getByText(en.mapNewArea.cancel));
        expect(onCancel).toHaveBeenCalled();
    });

    it('shows area type selector', () => {
        render(<NewAreaModal {...defaultProps} />);
        expect(screen.getByText(en.mapNewArea.areaTypeWorking)).toBeInTheDocument();
    });
});

// Observe real warnings without suppressing their console output.
let warningSpy: MockInstance<typeof console.warn>;
let errorSpy: MockInstance<typeof console.warn>;
beforeEach(() => {
    warningSpy = vi.spyOn(console, 'warn');
    errorSpy = vi.spyOn(console, 'error');
});
afterEach(() => {
    cleanup();
    for (const spy of [warningSpy, errorSpy]) {
        expect(spy.mock.calls.flat().join(' ')).not.toMatch(/\[antd: Modal\].*destroyOnClose/);
    }
    vi.restoreAllMocks();
});

describe('NewAreaModal lifecycle', () => {
    it.each(['workarea', 'navigation', 'obstacle'] as const)('reopens controlled %s values after Save and Cancel without unmount callbacks', async areaType => {
        const onSave = vi.fn(), onCancel = vi.fn(), onAreaNameChange = vi.fn(), onAreaTypeChange = vi.fn();
        const props = {open: true, areaType, areaName: 'First', onSave, onCancel, onAreaNameChange, onAreaTypeChange};
        const view = render(<NewAreaModal {...props}/>, {wrapper: ({children}) => <ConfigProvider theme={{token: {motion: false}}}>{children}</ConfigProvider>});
        expect(screen.getByRole('combobox')).toBeInTheDocument();
        if (areaType === 'workarea') {
            const input = screen.getByDisplayValue('First');
            fireEvent.change(input, {target: {value: 'Edited'}});
            expect(onAreaNameChange).toHaveBeenCalledExactlyOnceWith('Edited');
            view.rerender(<NewAreaModal {...props} areaName="Edited"/>);
            fireEvent.keyDown(screen.getByDisplayValue('Edited'), {key: 'Enter', code: 'Enter', keyCode: 13});
            expect(onSave).toHaveBeenCalledOnce();
            onSave.mockClear();
        }
        fireEvent.click(screen.getByRole('button', {name: en.mapNewArea.addArea}));
        expect(onSave).toHaveBeenCalledOnce();
        view.rerender(<NewAreaModal {...props} open={false}/>);
        await waitFor(() => expect(document.querySelector('.ant-modal-body')).toBeNull());
        expect(onSave).toHaveBeenCalledOnce(); expect(onCancel).not.toHaveBeenCalled();
        view.rerender(<NewAreaModal {...props} areaName="Second"/>);
        if (areaType === 'workarea') expect(screen.getByDisplayValue('Second')).toBeInTheDocument();
        else expect(screen.queryByRole('textbox')).not.toBeInTheDocument();
        fireEvent.click(screen.getByRole('button', {name: en.mapNewArea.cancel}));
        view.rerender(<NewAreaModal {...props} open={false}/>);
        await waitFor(() => expect(document.querySelector('.ant-modal-body')).toBeNull());
        view.rerender(<NewAreaModal {...props} areaName="Third"/>);
        if (areaType === 'workarea') expect(screen.getByDisplayValue('Third')).toBeInTheDocument();
        expect(onSave).toHaveBeenCalledOnce(); expect(onCancel).toHaveBeenCalledOnce();
        expect(onAreaTypeChange).not.toHaveBeenCalled();
    });
});
