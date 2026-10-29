import {act, fireEvent, render, screen, within} from '@testing-library/react';
import {beforeEach, describe, expect, it, vi} from 'vitest';
import {MemoryRouter} from 'react-router-dom';
import {ThemeProvider} from '../theme/ThemeContext';
import type {HighLevelStatus, Status} from '../types/ros';
import type {FusionOdom} from '../hooks/useFusionOdom';
import {MowgliNextPage} from './MowgliNextPage';

const mock = vi.hoisted(() => ({
  high: {} as HighLevelStatus, status: {} as Status, odom: {} as FusionOdom,
  settings: {} as Record<string, unknown>, session: false, mobile: false,
  raster: null as {width: number; height: number; resolution: number; originX: number; originY: number; dataUrl: string} | null,
  call: vi.fn(), error: vi.fn(), warning: vi.fn(), confirm: vi.fn<(options: {onOk: () => Promise<unknown>; onCancel?: () => void}) => void>(),
}));
vi.mock('antd', async importOriginal => ({...await importOriginal<typeof import('antd')>(),
  App: {useApp: () => ({notification: {error: mock.error}, message: {warning: mock.warning}, modal: {confirm: mock.confirm}})},
}));
vi.mock('../hooks/useApi', () => ({useApi: () => ({mowglinext: {callCreate: mock.call}})}));
vi.mock('../hooks/useHighLevelStatus', () => ({useHighLevelStatus: () => ({highLevelStatus: mock.high})}));
vi.mock('../hooks/useStatus', () => ({useStatus: () => mock.status}));
vi.mock('../hooks/useFusionOdom', () => ({useFusionOdom: () => mock.odom}));
vi.mock('../hooks/useSettings', () => ({useSettings: () => ({settings: mock.settings})}));
vi.mock('../hooks/useCoverageSession', () => ({useCoverageSession: () => ({session_active: mock.session})}));
vi.mock('../hooks/usePower', () => ({usePower: () => ({v_battery: 26})}));
vi.mock('../hooks/useEmergency', () => ({useEmergency: () => ({active_emergency: false})}));
vi.mock('../hooks/useGnssStatus', () => ({useGnssStatus: () => ({})}));
vi.mock('../hooks/useBladeTelemetry', () => ({useBladeTelemetry: () => ({rpm: NaN, motorTemp: NaN})}));
vi.mock('../hooks/useIsMobile', () => ({useIsMobile: () => mock.mobile}));
vi.mock('../hooks/useWeather', () => ({useWeather: () => null}));
vi.mock('../hooks/useHardwareBackend', () => ({useHardwareBackend: () => ({backend: 'mowgli', loading: false})}));
vi.mock('../hooks/useMavrosInfo', () => ({useMavrosInfo: () => ({})}));
vi.mock('../hooks/useMowingMap', () => ({useMowingMap: () => ({working_area_indices: [2], working_area: [
  {name: 'Orchard', area: {points: [{x: 0, y: 0}, {x: 10, y: 0}, {x: 10, y: 10}]}},
]})}));
vi.mock('../hooks/useDiagnosticsSnapshot', () => ({useDiagnosticsSnapshot: () => ({snapshot: {coverage: [
  {area_index: 0, total_cells: 80000, mowed_cells: 10000},
  {area_index: 2, total_cells: 40000, mowed_cells: 16000},
]}})}));
vi.mock('../hooks/useMowProgress', () => ({useMowProgress: () => null}));
vi.mock('../utils/mowProgress', () => ({rasterizeMowProgress: () => mock.raster}));
vi.mock('../components/dashboard/StatusMowerIllustration', () => ({StatusMowerIllustration: () => null}));
vi.mock('../components/dashboard/SoilWetBanner', () => ({SoilWetBanner: () => null}));
vi.mock('../components/BladeDirectionDisplay', () => ({BladeDirectionDisplay: () => null}));
const page = () => <MemoryRouter><ThemeProvider><MowgliNextPage/></ThemeProvider></MemoryRouter>;
const heading = () => screen.getByRole('heading', {level: 1});
function deferred() {
  let resolve!: (value: unknown) => void;
  let reject!: (reason: unknown) => void;
  const promise = new Promise((yes, no) => {resolve = yes; reject = no;});
  return {promise, resolve, reject};
}
beforeEach(() => {
  vi.clearAllMocks();
  mock.high = {state: 2, state_name: 'MOWING', current_area: 2, emergency: false};
  mock.status = {}; mock.odom = {}; mock.settings = {tool_width: 0.18};
  mock.raster = null; mock.session = false; mock.mobile = false;
  mock.call.mockReset().mockResolvedValue({data: {}});
});

describe('Home telemetry and activity', () => {
  it.each([false, true])('keeps missing position/grid unknown and coverage as summary (mobile=%s)', mobile => {
    mock.mobile = mobile; render(page());
    expect(screen.queryByTestId('mini-map-robot')).not.toBeInTheDocument();
    expect(screen.getByText('Robot position unknown')).toBeInTheDocument();
    expect(screen.getByTestId('live-map-mini').querySelector('image, rect')).toBeNull();
    expect(screen.getByText('Area coverage · 40 %')).toBeInTheDocument();
    expect(screen.queryByText('good conditions')).not.toBeInTheDocument();
    expect(screen.queryByText('dry')).not.toBeInTheDocument();
    expect(screen.getAllByText('Status unknown').length).toBeGreaterThan(0);
  });
  it('renders measured position and real grid, and follows rain knownness', () => {
    mock.odom = {pose: {pose: {position: {x: 5, y: 5, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}};
    mock.raster = {width: 200, height: 200, resolution: 0.05, originX: 0, originY: 0, dataUrl: 'data:image/png;base64,grid'};
    mock.status = {rain_detected: false};
    const view = render(page());
    expect(screen.getByTestId('mini-map-robot')).toHaveAttribute('transform', 'translate(300 150)');
    expect(screen.getByTestId('live-map-mini').querySelector('image')).toHaveAttribute('href', mock.raster.dataUrl);
    expect(screen.getByText('dry')).toBeInTheDocument();
    expect(screen.getByText('good conditions')).toBeInTheDocument();
    mock.status = {rain_detected: true}; view.rerender(page());
    expect(screen.getByText('detected')).toBeInTheDocument();
    expect(screen.queryByText('dry')).not.toBeInTheDocument();
    mock.status = {}; mock.odom = {pose: {pose: {position: {x: NaN, y: 5, z: 0}}}};
    view.rerender(page());
    expect(screen.queryByTestId('mini-map-robot')).not.toBeInTheDocument();
    expect(screen.queryByText('good conditions')).not.toBeInTheDocument();
  });
  it.each([
    [2, 'LOW_BATTERY_DOCKING', '', 'Returning to dock'],
    [2, 'RETURNING_HOME', '', 'Returning to dock'],
    [2, 'MOWING_COMPLETE', '', 'Returning to dock'],
    [2, 'CRITICAL_BATTERY_DOCKING', '', 'Returning to dock'],
    [2, 'RAIN_DETECTED_DOCKING', '', 'Returning to dock'],
    [2, 'COVERAGE_FAILED_DOCKING', '', 'Returning to dock'],
    [2, 'MOWING', 'TRANSIT', 'In transit'], [2, 'TRANSIT', '', 'In transit'],
    [3, 'RECORDING', '', 'Recording boundary'], [4, 'MANUAL_MOWING', '', 'Manual mowing'],
    [2, 'PLANNING', '', 'Autonomous activity'], [2, 'NEW_SUBSTATE', '', 'Autonomous activity'],
  ])('describes %s/%s/%s without a mowing ETA', (state, state_name, sub_state_name, label) => {
    mock.high = {...mock.high, state: Number(state), state_name: String(state_name), sub_state_name: String(sub_state_name)};
    mock.odom = {twist: {twist: {linear: {x: 0.3, y: 0, z: 0}}}}; render(page());
    expect(heading()).toHaveTextContent(String(label));
    expect(within(screen.getByRole('banner')).getAllByText(String(label))).toHaveLength(2);
    expect(screen.queryByText(/min of cutting/)).not.toBeInTheDocument();
    expect(screen.getByRole('button', {name: 'Pause'})).toBeEnabled();
  });
  it('labels the rough current-area cutting estimate and suppresses it without its inputs', () => {
    mock.odom = {twist: {twist: {linear: {x: 0.3, y: 0, z: 0}}}};
    const view = render(page());
    expect(heading()).toHaveTextContent('About 19 min of cutting');
    expect(screen.getByText('Orchard only, at current speed. Excludes turns, transit, charging and return to dock.')).toBeInTheDocument();
    mock.high = {...mock.high, current_area: 7}; view.rerender(page());
    expect(heading()).toHaveTextContent('Mowing');
    expect(screen.queryByText(/Area coverage/)).not.toBeInTheDocument();
    mock.high = {...mock.high, current_area: 2};
    for (const speed of [0, -0.3, NaN, Infinity]) {
      mock.odom = {twist: {twist: {linear: {x: speed, y: 0, z: 0}}}}; view.rerender(page());
      expect(heading()).toHaveTextContent('Mowing');
    }
    mock.odom = {twist: {twist: {linear: {x: 0.3, y: 0, z: 0}}}};
    mock.settings = {}; view.rerender(page()); expect(heading()).toHaveTextContent('Mowing');
  });
  it('preserves scan pause, emergency, charge holds and stop reasons', () => {
    mock.high = {...mock.high, sub_state_name: 'SCAN_PAUSED'};
    const view = render(page());
    expect(heading()).toHaveTextContent('Mowing is paused.');
    expect(screen.getByText(/LiDAR scan is temporarily stale/)).toBeInTheDocument();
    mock.high = {...mock.high, emergency: true}; view.rerender(page());
    expect(heading()).toHaveTextContent('Emergency stop');
    expect(screen.getByRole('button', {name: 'Re-arm'})).toBeEnabled();
    mock.high = {state: 1, state_name: 'CHARGING', is_charging: true, emergency: false};
    mock.session = true; view.rerender(page());
    expect(heading()).toHaveTextContent('Mowing paused');
    expect(screen.getByRole('button', {name: 'Resume now'})).toBeInTheDocument();
    expect(screen.getByRole('button', {name: 'Cancel mowing'})).toBeEnabled();
    mock.high = {state: 1, state_name: 'IDLE', stop_reason: 'DOCK_NAV_FAILED', stop_detail: 'Blocked approach'};
    view.rerender(page());
    expect(screen.getByTestId('stop-reason-banner')).toHaveTextContent('Blocked approach');
  });
});

describe('Home command requests', () => {
  it('locks duplicate/competing requests immediately, retains emergency stop and shows service failures', async () => {
    const pending = deferred(); mock.call.mockReturnValueOnce(pending.promise); render(page());
    const pause = screen.getByRole('button', {name: 'Pause'});
    act(() => {pause.click(); pause.click();});
    expect(mock.call).toHaveBeenCalledExactlyOnceWith('high_level_control', {Command: 8});
    expect(pause).toBeDisabled(); expect(screen.getByRole('status')).toHaveTextContent('Command pending…');
    fireEvent.click(screen.getByRole('button', {name: 'Return to base'})); expect(mock.call).toHaveBeenCalledTimes(1);
    fireEvent.click(screen.getByRole('button', {name: 'Emergency stop'})); expect(mock.confirm).toHaveBeenCalledOnce();
    await act(async () => {await mock.confirm.mock.calls[0][0].onOk();});
    expect(mock.call).toHaveBeenLastCalledWith('emergency', {Emergency: 1}); expect(pause).toBeDisabled();
    await act(async () => {pending.reject({error: {error: 'Service refused pause'}}); await Promise.resolve();});
    expect(mock.error).toHaveBeenCalledWith({message: 'Command failed', description: 'Service refused pause'});
    expect(pause).toBeEnabled(); expect(screen.queryByRole('status')).not.toBeInTheDocument();
    await act(async () => {pause.click(); await Promise.resolve();}); expect(mock.call).toHaveBeenCalledTimes(3);
  });
  it.each([
    ['IDLE', 1, false, 'Start mowing', 'high_level_control', {Command: 1}],
    ['MOWING', 2, false, 'Return to base', 'high_level_control', {Command: 2}],
    ['EMERGENCY', 0, false, 'Re-arm', 'emergency', {Emergency: 0}],
    ['CHARGING', 1, true, 'Resume now', 'high_level_control', {Command: 1}],
    ['MANUAL_CHARGING', 1, true, 'Cancel mowing', 'high_level_control', {Command: 8}],
  ] as const)('protects %s/%s %s: %s until response and preserves warnings', async (state_name, state, session, label, command, args) => {
    mock.high = {state, state_name, emergency: state_name === 'EMERGENCY', battery_percent: 60}; mock.session = session;
    const pending = deferred(); mock.call.mockReturnValueOnce(pending.promise); const view = render(page());
    const button = screen.getByRole('button', {name: label}); act(() => {button.click(); button.click();});
    expect(mock.call).toHaveBeenCalledExactlyOnceWith(command, args); expect(button).toBeDisabled();
    expect(screen.getByRole('button', {name: 'Emergency stop'})).toBeEnabled();
    mock.mobile = true; mock.high = {state: 2, state_name: 'MOWING'}; view.rerender(page());
    expect(screen.getByRole('button', {name: 'Pause'})).toBeDisabled();
    await act(async () => {pending.resolve({data: {warning: 'Request accepted with warning'}}); await pending.promise;});
    expect(mock.warning).toHaveBeenCalledWith('Request accepted with warning', 10);
    expect(screen.getByRole('button', {name: 'Pause'})).toBeEnabled();
  });
});


describe('Home dig escalation command coordination', () => {
  beforeEach(() => {
    mock.high = {state: 1, state_name: 'DIG_OBSTRUCTION', emergency: false};
    mock.status = {dig_escalated: true, dig_escalated_distance_m: 1, dig_escalated_required_distance_m: 0.5};
  });

  it('holds the shared lock across clear and START, with emergency stop still available', async () => {
    const clear = deferred();
    const start = deferred();
    mock.call.mockImplementation((command: string) => command === 'clear_dig_escalation'
      ? clear.promise : command === 'high_level_control' ? start.promise : Promise.resolve({data: {}}));
    render(page());
    const resume = screen.getByRole('button', {name: 'Clear & resume'});
    const home = screen.getByRole('button', {name: 'Return to base'});
    const play = screen.getByRole('button', {name: 'Start mowing'});
    act(() => {resume.click(); home.click(); play.click(); resume.click();});
    expect(mock.call).toHaveBeenCalledExactlyOnceWith('clear_dig_escalation', {});
    expect(home).toBeDisabled();
    expect(play).toBeDisabled();
    expect(resume).toBeDisabled();
    expect(screen.getByRole('status')).toHaveTextContent('Command pending…');
    fireEvent.click(screen.getByRole('button', {name: 'Emergency stop'}));
    await act(async () => {await mock.confirm.mock.calls[0][0].onOk();});
    expect(mock.call).toHaveBeenLastCalledWith('emergency', {Emergency: 1});
    await act(async () => {clear.resolve({data: {}}); await clear.promise;});
    expect(mock.call).toHaveBeenLastCalledWith('high_level_control', {Command: 1});
    expect(home).toBeDisabled();
    expect(resume).toBeDisabled();
    await act(async () => {start.resolve({data: {}}); await start.promise;});
    expect(home).toBeEnabled();
    expect(resume).toBeEnabled();
    expect(screen.queryByRole('status')).not.toBeInTheDocument();
    expect(mock.call).toHaveBeenCalledTimes(3);
  });

  it('prevents the banner starting a clear while HOME is unresolved, including same-turn clicks', async () => {
    const home = deferred(); mock.call.mockReturnValueOnce(home.promise);
    render(page());
    const resume = screen.getByRole('button', {name: 'Clear & resume'});
    act(() => {screen.getByRole('button', {name: 'Return to base'}).click(); resume.click();});
    expect(mock.call).toHaveBeenCalledExactlyOnceWith('high_level_control', {Command: 2});
    expect(resume).toBeDisabled();
    expect(screen.getByRole('button', {name: 'Emergency stop'})).toBeEnabled();
    await act(async () => {home.resolve({data: {}}); await home.promise;});
    expect(resume).toBeEnabled();
  });

  it('keeps normal controls locked during acknowledgement and releases them on cancel', async () => {
    mock.status.dig_escalated_distance_m = 0.1;
    render(page());
    fireEvent.click(screen.getByRole('button', {name: 'Clear & resume'}));
    expect(mock.confirm).toHaveBeenCalledOnce();
    expect(mock.call).not.toHaveBeenCalled();
    expect(screen.getByRole('button', {name: 'Return to base'})).toBeDisabled();
    expect(screen.getByRole('button', {name: 'Emergency stop'})).toBeEnabled();
    await act(async () => {mock.confirm.mock.calls[0][0].onCancel?.(); await Promise.resolve();});
    expect(screen.getByRole('button', {name: 'Return to base'})).toBeEnabled();
    expect(mock.call).not.toHaveBeenCalled();
  });

  it.each(['clear', 'start'])('shows %s failure and unlocks the page without retrying', async failedStep => {
    const failure = deferred();
    if (failedStep === 'clear') mock.call.mockReturnValueOnce(failure.promise);
    else mock.call.mockResolvedValueOnce({data: {}}).mockReturnValueOnce(failure.promise);
    const errors = vi.spyOn(console, 'error').mockImplementation(() => {});
    try {
      render(page());
      const resume = screen.getByRole('button', {name: 'Clear & resume'});
      await act(async () => {resume.click(); await Promise.resolve();});
      expect(screen.getByRole('button', {name: 'Return to base'})).toBeDisabled();
      await act(async () => {failure.reject({error: {error: 'Request refused'}}); await Promise.resolve();});
      expect(mock.error).toHaveBeenCalledWith(expect.objectContaining({description: 'Request refused'}));
      expect(screen.getByRole('button', {name: 'Return to base'})).toBeEnabled();
      expect(resume).toBeEnabled();
      expect(mock.call).toHaveBeenCalledTimes(failedStep === 'clear' ? 1 : 2);
      expect(errors).toHaveBeenCalledOnce();
    } finally {errors.mockRestore();}
  });
});
