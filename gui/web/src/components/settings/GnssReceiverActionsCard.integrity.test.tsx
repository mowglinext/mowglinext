import {act, fireEvent, render, screen, waitFor} from '@testing-library/react';
import {beforeEach, expect, it, vi} from 'vitest';
import {GnssReceiverActionsCard} from './GnssReceiverActionsCard';

const mocks = vi.hoisted(() => ({
    api: {request: vi.fn<(args: {path: string; body?: unknown}) => Promise<{data?: unknown; error?: unknown; ok?: boolean; status?: number}>>()},
    notification: {success: vi.fn(), error: vi.fn(), warning: vi.fn()},
    confirm: vi.fn<(args: {onOk: () => Promise<void>}) => void>(),
}));
vi.mock('../../hooks/useApi', () => ({useApi: () => mocks.api}));
vi.mock('react-i18next', () => ({useTranslation: () => ({t: (key: string) => key})}));
vi.mock('antd', async importOriginal => {
    const actual = await importOriginal<typeof import('antd')>();
    return {...actual, App: {...actual.App, useApp: () => ({notification: mocks.notification, modal: {confirm: mocks.confirm}})}};
});
beforeEach(() => {vi.clearAllMocks(); mocks.api.request.mockResolvedValue({data: {success: true}});});
const click = (name: string) => fireEvent.click(screen.getByRole('button', {name: new RegExp(`settingsGnssReceiver.${name}`)}));

it.each(['plan', 'apply', 'factory-reset-apply', 'restart'] as const)('dispatches successful %s once with its required confirmation', async action => {
    const persist = vi.fn().mockResolvedValue(true);
    mocks.api.request.mockResolvedValue({data: {success: true, action: action === 'factory-reset-apply' ? 'factory_reset_apply' : action, ...(action === 'restart' ? {restart_attempted: true, restart_succeeded: true} : {})}});
    render(<GnssReceiverActionsCard onPersistBeforeAction={persist}/>);
    click({plan: 'actionPlan', apply: 'actionApply', 'factory-reset-apply': 'actionFactoryReset', restart: 'actionRestart'}[action]);
    if (action === 'apply' || action === 'factory-reset-apply') {
        expect(mocks.api.request).not.toHaveBeenCalled(); expect(persist).not.toHaveBeenCalled();
        await act(() => mocks.confirm.mock.calls[0][0].onOk());
    }
    await waitFor(() => expect(mocks.notification.success).toHaveBeenCalledTimes(1));
    expect(persist).toHaveBeenCalledTimes(1); expect(mocks.api.request).toHaveBeenCalledTimes(1);
    expect(mocks.api.request).toHaveBeenCalledWith(expect.objectContaining({path: `/settings/gnss/${action}`,
        body: action === 'apply' ? {confirm: true} : action === 'factory-reset-apply' ? {confirm_factory_reset: true} : action === 'restart' ? {} : undefined}));
});
it.each([{data: {}}, {data: undefined, error: {}}, {data: undefined, ok: false, status: 500},
    {data: {success: true, executions: [{}]}}, {data: {success: 'true'}}])('never announces success for invalid/HTTP error %j', async response => {
    mocks.api.request.mockResolvedValue(response); render(<GnssReceiverActionsCard/>); click('actionPlan');
    await waitFor(() => expect(mocks.notification.error).toHaveBeenCalledTimes(1));
    expect(mocks.notification.success).not.toHaveBeenCalled();
    expect(screen.getByText('settingsGnssReceiver.operationUnconfirmed')).toBeInTheDocument();
    expect(screen.queryByText('settingsGnssReceiver.tagSuccess')).not.toBeInTheDocument();
    expect(mocks.api.request).toHaveBeenCalledTimes(1);
});
it.each([false, undefined])('requires exactly confirmed prerequisite persistence (%s)', async confirmed => {
    const persist = vi.fn().mockResolvedValue(confirmed);
    render(<GnssReceiverActionsCard onPersistBeforeAction={persist}/>); click('actionPlan');
    await waitFor(() => expect(mocks.notification.error).toHaveBeenCalledTimes(1));
    expect(mocks.api.request).not.toHaveBeenCalled(); expect(mocks.notification.success).not.toHaveBeenCalled();
    expect(screen.getByText('settingsGnssReceiver.persistenceUnconfirmed')).toBeInTheDocument();
});
it('blocks dirty settings when no persistence prerequisite is provided', async () => {
    render(<GnssReceiverActionsCard isDirty/>); click('actionPlan');
    await waitFor(() => expect(mocks.notification.error).toHaveBeenCalled()); expect(mocks.api.request).not.toHaveBeenCalled();
});
it.each([false, true])('summary, tag and notification agree on partial failure %s', async partial => {
    mocks.api.request.mockResolvedValue({data: {success: false, partial_failure: partial, restart_attempted: true, restart_error: 'restore failed'}});
    render(<GnssReceiverActionsCard/>); click('actionPlan');
    await waitFor(() => expect(partial ? mocks.notification.warning : mocks.notification.error).toHaveBeenCalledTimes(1));
    expect(mocks.notification.success).not.toHaveBeenCalled();
    expect(screen.getByText(partial ? 'settingsGnssReceiver.tagPartialFailure' : 'settingsGnssReceiver.tagFailed')).toBeInTheDocument();
    expect(screen.getByText('settingsGnssReceiver.tagGpsRestartFailed')).toBeInTheDocument();
});
it('does not infer restart completion from an attempted flag', async () => {
    mocks.api.request.mockResolvedValue({data: {success: true, restart_attempted: true}});
    render(<GnssReceiverActionsCard/>); click('actionRestart');
    await waitFor(() => expect(mocks.notification.warning).toHaveBeenCalledTimes(1));
    expect(mocks.notification.success).not.toHaveBeenCalled(); expect(screen.queryByText('settingsGnssReceiver.tagGpsRestarted')).not.toBeInTheDocument();
});
it('serializes repeated confirmation callbacks while prerequisite saving is pending', async () => {
    let finish: (value: boolean) => void = () => {};
    const persist = vi.fn(() => new Promise<boolean>(resolve => {finish = resolve;}));
    render(<GnssReceiverActionsCard onPersistBeforeAction={persist}/>); click('actionApply');
    const confirm = mocks.confirm.mock.calls[0][0].onOk;
    let first = Promise.resolve(); act(() => {first = confirm();});
    await act(() => confirm()); click('actionRestart');
    expect(persist).toHaveBeenCalledTimes(1); expect(mocks.api.request).not.toHaveBeenCalled();
    await act(async () => {finish(true); await first;}); expect(mocks.api.request).toHaveBeenCalledTimes(1);
});
it('keeps the action locked during the HTTP request, without retrying', async () => {
    let finish: (response: {data: unknown}) => void = () => {};
    mocks.api.request.mockImplementation(() => new Promise(resolve => {finish = resolve;}));
    render(<GnssReceiverActionsCard/>); click('actionPlan'); click('actionRestart'); click('actionPlan');
    expect(mocks.api.request).toHaveBeenCalledTimes(1);
    await act(async () => {finish({data: {}}); await Promise.resolve();});
    expect(mocks.notification.error).toHaveBeenCalledTimes(1); expect(mocks.api.request).toHaveBeenCalledTimes(1);
});
it('unmount during prerequisite saving prevents dispatch', async () => {
    let finish: (value: boolean) => void = () => {};
    const persist = () => new Promise<boolean>(resolve => {finish = resolve;});
    const card = render(<GnssReceiverActionsCard onPersistBeforeAction={persist}/>); click('actionPlan'); card.unmount();
    await act(async () => {finish(true); await Promise.resolve();}); expect(mocks.api.request).not.toHaveBeenCalled(); expect(mocks.notification.success).not.toHaveBeenCalled();
});
it('late responses after unmount cannot notify or trigger another request', async () => {
    let finish: (response: {data: unknown}) => void = () => {};
    mocks.api.request.mockImplementation(() => new Promise(resolve => {finish = resolve;}));
    const card = render(<GnssReceiverActionsCard/>); click('actionPlan'); card.unmount();
    await act(async () => {finish({data: {success: true}}); await Promise.resolve();});
    expect(mocks.notification.success).not.toHaveBeenCalled(); expect(mocks.api.request).toHaveBeenCalledTimes(1);
});
it('a network rejection clears the previous result without claiming the operation did not occur', async () => {
    render(<GnssReceiverActionsCard/>); click('actionPlan'); await waitFor(() => expect(mocks.notification.success).toHaveBeenCalledTimes(1));
    mocks.api.request.mockRejectedValue(new Error('offline')); click('actionRestart');
    await waitFor(() => expect(mocks.notification.error).toHaveBeenCalledTimes(1));
    expect(screen.queryByText('settingsGnssReceiver.tagSuccess')).not.toBeInTheDocument();
    expect(screen.getByText('settingsGnssReceiver.operationUnconfirmed')).toBeInTheDocument();
});

it('manual baud recovery updates only the selected current baud and never auto-retries', async () => {
    mocks.api.request.mockResolvedValue({data: {success: true, manual_baud_required: true, execution_baud: 'auto', config_baud: '921600', runtime_baud: '460800'}});
    const change = vi.fn();
    render(<GnssReceiverActionsCard manualCurrentBaud="115200" onManualCurrentBaudChange={change}/>); click('actionPlan');
    await waitFor(() => expect(mocks.notification.error).toHaveBeenCalledTimes(1));
    fireEvent.mouseDown(screen.getByRole('combobox'));
    fireEvent.click(await screen.findByText('230400', {selector: '.ant-select-item-option-content'}));
    expect(change).toHaveBeenCalledWith('230400', expect.objectContaining({value: '230400'})); expect(mocks.api.request).toHaveBeenCalledTimes(1);
    expect(screen.getByText('settingsGnssReceiver.fieldRuntimeBaud')).toBeInTheDocument();
    expect(screen.getByText('settingsGnssReceiver.fieldConfigBaud')).toBeInTheDocument();
    expect(screen.queryByText('settingsGnssReceiver.fieldActiveVerifiedBaud')).not.toBeInTheDocument();
});

it.each([true, false])('the reset card accepts the exact Go action identifier and reports success=%s consistently', async success => {
    mocks.api.request.mockResolvedValue({data: {action: 'factory_reset_apply', success, partial_failure: !success,
        restart_attempted: true, ...(success ? {restart_succeeded: true} : {restart_error: 'restore failed'})}});
    render(<GnssReceiverActionsCard/>); click('actionFactoryReset');
    expect(mocks.api.request).not.toHaveBeenCalled();
    await act(() => mocks.confirm.mock.calls[0][0].onOk());
    expect(mocks.api.request).toHaveBeenCalledWith(expect.objectContaining({path: '/settings/gnss/factory-reset-apply', body: {confirm_factory_reset: true}}));
    expect(success ? mocks.notification.success : mocks.notification.warning).toHaveBeenCalledTimes(1);
    expect(success ? mocks.notification.warning : mocks.notification.success).not.toHaveBeenCalled();
    expect(mocks.notification.error).not.toHaveBeenCalled();
    expect(screen.getByText(success ? 'settingsGnssReceiver.tagSuccess' : 'settingsGnssReceiver.tagPartialFailure')).toBeInTheDocument();
    expect(screen.queryByText('settingsGnssReceiver.operationUnconfirmed')).not.toBeInTheDocument();
});
