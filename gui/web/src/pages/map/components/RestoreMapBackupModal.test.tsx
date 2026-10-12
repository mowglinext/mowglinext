import {describe, it, expect, vi} from 'vitest';
import {render, screen, waitFor} from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import {App} from 'antd';
import {RestoreMapBackupModal} from './RestoreMapBackupModal.tsx';
import type {MapBackup} from '../hooks/useMapBackups.ts';
import en from '../../../i18n/locales/en.json';
import '../../../i18n';

const backup = (id: string, names: string[]): MapBackup => ({
    id, created_at: '2026-10-04T12:00:00.000Z', size_bytes: 2048,
    areas: names.length, obstacles: 3, ignore_lines: 1, area_names: names,
});

function renderModal(overrides: Partial<React.ComponentProps<typeof RestoreMapBackupModal>> = {}) {
    const props = {
        open: true,
        onClose: vi.fn(),
        list: vi.fn(() => Promise.resolve({backups: [backup('areas-A', ['Voor', 'Achter']), backup('areas-B', ['Voor'])], keep: 20})),
        restore: vi.fn(() => Promise.resolve()),
        onRestored: vi.fn(),
        ...overrides,
    };
    render(<App><RestoreMapBackupModal {...props}/></App>);
    return props;
}

describe('RestoreMapBackupModal', () => {
    it('lists the backups with what is in them', async () => {
        renderModal();
        expect(await screen.findByText('Voor, Achter')).toBeInTheDocument();
        expect(screen.getAllByText('3 obstacles')).toHaveLength(2);
        expect(screen.getAllByRole('button', {name: en.mapBackups.restore})).toHaveLength(2);
    });

    it('says so when there is nothing to restore yet', async () => {
        renderModal({list: vi.fn(() => Promise.resolve({backups: [], keep: 20}))});
        expect(await screen.findByText(en.mapBackups.empty)).toBeInTheDocument();
    });

    it('restores only after the operator confirms, then tells the page', async () => {
        const props = renderModal();
        const user = userEvent.setup();
        await user.click((await screen.findAllByRole('button', {name: en.mapBackups.restore}))[0]);

        expect(props.restore).not.toHaveBeenCalled(); // a confirmation comes first
        // the confirmation dialog's OK button is the last "Restore" in the document
        await waitFor(() => expect(screen.getAllByRole('button', {name: en.mapBackups.restore})).toHaveLength(3));
        const restoreButtons = screen.getAllByRole("button", {name: en.mapBackups.restore});
        await user.click(restoreButtons[restoreButtons.length - 1]);

        await waitFor(() => expect(props.restore).toHaveBeenCalledWith('areas-A'));
        await waitFor(() => expect(props.onRestored).toHaveBeenCalledOnce());
    });
});
