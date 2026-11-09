import {App} from 'antd';
import {render, screen, waitFor} from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import {afterEach, expect, it, vi} from 'vitest';
import AsyncDropDownButton from './AsyncDropDownButton';

afterEach(() => vi.restoreAllMocks());

it('passes the actual menu event once and surfaces a generated-client rejection', async () => {
    vi.spyOn(console, 'error').mockImplementation(() => undefined);
    const action = vi.fn().mockRejectedValue({error: {error: 'Operation refused'}});
    const user = userEvent.setup();
    render(<App><AsyncDropDownButton menu={{items: [{key: 'attempt', label: 'Try operation'}], onAsyncClick: action}}>Actions</AsyncDropDownButton></App>);
    await user.click(screen.getByText('Actions'));
    await user.click(screen.getByText('Try operation'));
    expect(await screen.findByText('Operation refused')).toBeInTheDocument();
    expect(action).toHaveBeenCalledOnce();
    expect(action).toHaveBeenCalledWith(expect.objectContaining({key: 'attempt'}));
    await waitFor(() => expect(document.querySelector('.ant-dropdown-button .ant-btn-loading')).toBeNull());
    expect(document.querySelector('.ant-notification-notice-success')).toBeNull();
});
