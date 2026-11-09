import {describe, expect, it, vi} from 'vitest';
import {act, render, screen, waitFor} from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import {AsyncPopconfirm} from './AsyncPopconfirm';

describe('promise-aware confirmation', () => {
    it('keeps the confirmation open and loading until the action settles', async () => {
        let finish: () => void = () => { throw new Error('Promise not initialized'); };
        const pending = new Promise<void>(resolve => { finish = resolve; });
        const onConfirm = vi.fn(() => pending);
        const user = userEvent.setup();
        render(<AsyncPopconfirm title="Confirm removal" okText="Remove" onConfirm={onConfirm}>
            <button>Review removal</button>
        </AsyncPopconfirm>);
        await user.click(screen.getByText('Review removal'));
        await user.click(screen.getByRole('button', {name: 'Remove'}));
        expect(onConfirm).toHaveBeenCalledOnce();
        expect(screen.getByRole('button', {name: /Remove/})).toHaveClass('ant-btn-loading');
        expect(screen.getByText('Confirm removal')).toBeInTheDocument();
        await act(async () => { finish(); await pending; });
        await waitFor(() => expect(screen.queryByRole('button', {name: 'Remove'})).not.toBeInTheDocument());
    });
});
