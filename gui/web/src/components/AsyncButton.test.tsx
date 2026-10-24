import {App} from 'antd';
import {fireEvent, render, screen, waitFor} from '@testing-library/react';
import {afterEach, describe, expect, it, vi} from 'vitest';
import AsyncButton from './AsyncButton';

afterEach(() => vi.restoreAllMocks());

describe('shared asynchronous action failure boundary', () => {
    it.each([new Error('Backend unavailable'), {error: {error: 'Backend unavailable'}}, {error: 'Backend unavailable'}])('reports request failure and releases loading for %j', failure => {
        vi.spyOn(console, 'error').mockImplementation(() => undefined);
        const action = vi.fn().mockRejectedValue(failure);
        render(<App><AsyncButton onAsyncClick={action}>Run action</AsyncButton></App>);
        fireEvent.click(screen.getByText('Run action'));
        return waitFor(() => {
            expect(screen.getByText('Backend unavailable')).toBeInTheDocument();
            expect(screen.getByRole('button', {name: /Run action/})).not.toHaveClass('ant-btn-loading');
            expect(action).toHaveBeenCalledOnce();
        });
    });

    it('does not turn a malformed error object into successful feedback', async () => {
        vi.spyOn(console, 'error').mockImplementation(() => undefined);
        render(<App><AsyncButton onAsyncClick={vi.fn().mockRejectedValue({error: {error: 42}})}>Run action</AsyncButton></App>);
        fireEvent.click(screen.getByText('Run action'));
        await waitFor(() => expect(document.querySelector('.ant-notification-notice-error')).not.toBeNull());
        expect(screen.getByRole('button', {name: /Run action/})).not.toHaveClass('ant-btn-loading');
        expect(document.querySelector('.ant-notification-notice-success')).toBeNull();
    });
});
