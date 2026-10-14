import {fireEvent, render, screen, waitFor} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {FcuParamTool} from './FcuParamTool';
const storage = vi.hoisted(() => new Map<string, unknown>());
const storeState = vi.hoisted(() => ({fail: false}));
const urls = vi.hoisted(() => vi.fn((blob: Blob) => `blob:param-test-${blob.size}`));
vi.mock('localforage', () => ({default: {createInstance: vi.fn(() => ({
    keys: () => Promise.resolve([...storage.keys()]), getItem: (id: string) => Promise.resolve(storage.get(id) ?? null),
    setItem: (id: string, value: unknown) => {if (storeState.fail) return Promise.reject(new Error('quota')); storage.set(id, value); return Promise.resolve(value);},
}))}}));
const upload = (text: string, name = 'robot.param') => {
    const file = new File([text], name, {type: 'text/plain'});
    Object.defineProperty(file, 'arrayBuffer', {value: () => Promise.resolve(new TextEncoder().encode(text).buffer)});
    fireEvent.change(screen.getByLabelText('Import .param', {selector: 'input'}), {target: {files: [file]}});
};
const blobText = (blob: Blob) => new Promise<string>((resolve, reject) => {
    const reader = new FileReader(); reader.onload = () => resolve(new TextDecoder('utf-8', {ignoreBOM: true}).decode(reader.result as ArrayBuffer));
    reader.onerror = reject; reader.readAsArrayBuffer(blob);
});
const supported = '# ArduRover 4.7.1\r\n# Board: Pixhawk 5X\r\nMAV_SYSID,1\r\nMAV_GCS_SYSID,42\r\nARMING_REQUIRE,0\r\nATC_SPEED_P,0.20000\r\nSERIAL0_PROTOCOL,0';
const originalURL = URL;
const urlMethods = ['createObjectURL', 'revokeObjectURL'] as const;
const urlDescriptors = urlMethods.map(name => Object.getOwnPropertyDescriptor(URL, name));
beforeEach(() => {
    storage.clear(); urls.mockClear(); storeState.fail = false;
    Object.defineProperty(URL, 'createObjectURL', {configurable: true, value: urls});
    Object.defineProperty(URL, 'revokeObjectURL', {configurable: true, value: vi.fn()});
    expect(URL).toBe(originalURL);
    expect(new URL('/params', 'https://example.test').pathname).toBe('/params');
    vi.spyOn(HTMLAnchorElement.prototype, 'click').mockImplementation(() => undefined);
});
afterEach(() => {
    urlMethods.forEach((name, i) => {
        const descriptor = urlDescriptors[i];
        if (descriptor) Object.defineProperty(URL, name, descriptor);
        else Reflect.deleteProperty(URL, name);
        expect(Object.getOwnPropertyDescriptor(URL, name)).toEqual(descriptor);
    });
    vi.restoreAllMocks();
});
// These exact text/label queries avoid a full-page getByRole visibility/style walk.
// Still assert the real button is visible before interacting with it.
const button = (name: string) => {
    const element = screen.getByText(name, {selector: 'button span'}).closest('button')!;
    expect(element).toBeVisible();
    return element;
};
async function ready() {
    const importButton = screen.getByLabelText('Import .param', {selector: 'button'});
    expect(importButton).toBeVisible();
    await waitFor(() => expect(importButton).not.toBeDisabled());
}
const openGroup = (name: string) => fireEvent.click(screen.getByText(name));
describe('FCU .param tool', () => {
    it('does not expose a working copy or export when immutable backup storage fails', async () => {
        render(<FcuParamTool/>); await ready(); storeState.fail = true; upload(supported);
        expect(await screen.findByText(/Browser backup storage failed/)).toBeInTheDocument();
        expect(screen.queryByRole('button', {name: 'Export selected corrections'})).not.toBeInTheDocument();
        expect(storage.size).toBe(0);
    });
    it('backs up the exact original, exports selectively and restores without altering the backup', async () => {
        render(<FcuParamTool/>); await ready(); upload(supported);
        expect(await screen.findByText('robot.param')).toBeInTheDocument();
        expect([...storage.values()]).toHaveLength(1);
        expect((([...storage.values()][0]) as {original: string}).original).toBe(supported);
        openGroup('MAVLink communication');
        fireEvent.change(screen.getByLabelText('MAVROS logical SERIAL port (if identified)', {selector: 'input'}), {target: {value: '0'}});
        fireEvent.click(screen.getByTestId('diagnostic-link.protocol').querySelector('summary')!);
        fireEvent.click(screen.getByLabelText('Select SERIAL0_PROTOCOL', {selector: 'input'}));
        fireEvent.click(button('Export selected corrections'));
        const blob = urls.mock.calls[urls.mock.calls.length - 1][0];
        expect(await blobText(blob)).toBe(supported.replace('SERIAL0_PROTOCOL,0', 'SERIAL0_PROTOCOL,2'));
        fireEvent.click(button('Restore working copy'));
        expect(screen.getByTestId('diagnostic-link.protocol').querySelector('input[type=checkbox]')).not.toBeChecked();
        expect((([...storage.values()][0]) as {original: string}).original).toBe(supported);
    });
    it('keeps uncertain safety policies read-only rather than proposing the reference value', async () => {
        render(<FcuParamTool/>); await ready(); upload(supported);
        await screen.findByText('robot.param');
        openGroup('Safety and arming');
        expect(screen.getByTestId('diagnostic-arming.require')).toHaveTextContent('To verify');
        expect(screen.queryByLabelText('Select ARMING_REQUIRE')).not.toBeInTheDocument();
        expect(screen.queryByLabelText('I confirm these selected changes and their installed hardware implications.')).not.toBeInTheDocument();
    });
    it('does not treat live FCU identity or default input values as permission to renumber SYSIDs', async () => {
        render(<FcuParamTool info={{state: {connected: true}, vehicle: {available_info: 3, autopilot: 3, type: 10, flight_sw_version: 0x040701ff, sysid: 23}}}/>);
        await ready(); upload(supported); await screen.findByText('robot.param');
        openGroup('MAVLink communication');
        fireEvent.click(button('Use observed identity for this file'));
        fireEvent.click(button('Select admissible corrections'));
        expect(screen.getByTestId('diagnostic-link.target')).toHaveTextContent('To verify');
        expect(screen.getByTestId('diagnostic-link.gcs')).toHaveTextContent('To verify');
        expect(button('Export selected corrections')).toBeDisabled();
    });
    it('does not infer imported firmware from parameter names or apply PX4 rules as ArduPilot', async () => {
        render(<FcuParamTool info={{state: {connected: true}, vehicle: {available_info: 3, autopilot: 12, type: 10, flight_sw_version: 0x040701ff}}}/>);
        await ready(); upload('MAV_SYSID,1\nMAV_GCS_SYSID,42\n');
        await screen.findByText('robot.param');
        expect(button('Export selected corrections')).toBeDisabled();
        openGroup('MAVLink communication');
        expect(screen.queryByLabelText('Select MAV_GCS_SYSID')).not.toBeInTheDocument();
        expect(screen.getAllByText(/No comparison profile/).length).toBeGreaterThan(0);
    });
    it('bulk selection excludes safety and provisional rules, and changing bindings invalidates old selections', async () => {
        render(<FcuParamTool/>); await ready(); upload(supported); await screen.findByText('robot.param');
        openGroup('MAVLink communication');
        fireEvent.change(screen.getByLabelText('MAVROS logical SERIAL port (if identified)', {selector: 'input'}), {target: {value: '0'}});
        fireEvent.click(button('Select admissible corrections'));
        expect(screen.getByTestId('diagnostic-link.protocol').querySelector('input[type=checkbox]')).toBeChecked();
        expect(screen.getByTestId('diagnostic-link.target')).toHaveTextContent('To verify');
        expect(screen.getByTestId('diagnostic-link.gcs')).toHaveTextContent('To verify');
        fireEvent.change(screen.getByLabelText('MAVROS source system ID to verify', {selector: 'input'}), {target: {value: '99'}});
        expect(screen.getByTestId('diagnostic-link.protocol').querySelector('input[type=checkbox]')).not.toBeChecked();
        expect(button('Export selected corrections')).toBeDisabled();
        fireEvent.change(screen.getByLabelText('MAVROS source system ID to verify', {selector: 'input'}), {target: {value: '255'}});
        expect(screen.getByTestId('diagnostic-link.protocol').querySelector('input[type=checkbox]')).not.toBeChecked();
        openGroup('Safety and arming');
        expect(screen.getByTestId('diagnostic-arming.require')).toHaveTextContent('To verify');
        expect(screen.queryByLabelText('Select ARMING_REQUIRE')).not.toBeInTheDocument();
    });
    it('keeps a large personal parameter inventory out of the diagnostic UI and preserves every hidden byte on export', async () => {
        const original = supported + '\r\n' + Array.from({length: 900}, (_, i) => `ATC_PRIVATE_${i},1.23000`).join('\r\n');
        render(<FcuParamTool/>); await ready(); upload(original); await screen.findByText('robot.param');
        for (const label of ['MAVLink communication', 'Propulsion and blade', 'DroneCAN and VESC', 'Safety and arming', 'Navigation and sensors', 'Telemetry and publication frequencies']) expect(screen.getByText(label)).toBeInTheDocument();
        expect(screen.queryByText(/ATC_PRIVATE_/)).not.toBeInTheDocument();
        expect(document.querySelector('.ant-table')).toBeNull();
        openGroup('MAVLink communication');
        fireEvent.change(screen.getByLabelText('MAVROS logical SERIAL port (if identified)', {selector: 'input'}), {target: {value: '0'}});
        fireEvent.click(button('Select admissible corrections'));
        fireEvent.click(button('Export selected corrections'));
        expect(await blobText(urls.mock.calls[urls.mock.calls.length - 1][0])).toBe(original.replace('SERIAL0_PROTOCOL,0', 'SERIAL0_PROTOCOL,2'));
        openGroup('Safety and arming');
        expect(screen.getByTestId('diagnostic-arming.require')).toHaveTextContent('To verify');
        fireEvent.click(screen.getByText('Telemetry and publication frequencies'));
        expect(screen.getByTestId('telemetry-state')).toHaveTextContent('To verify');
        expect(screen.queryByTestId('telemetry-bladeRpm')).not.toBeInTheDocument();
    });

});
