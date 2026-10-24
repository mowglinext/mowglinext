import {cleanup, fireEvent, render} from '@testing-library/react';
import {afterEach, describe, expect, it} from 'vitest';
import {RobotAnatomy, type AnatomyInputs} from './RobotAnatomy';
import {bladeTelemetry} from '../utils/motorTelemetry';
afterEach(cleanup);
const inputs: AnatomyInputs = {batteryPct: 80, vBattery: 28, motorTempC: NaN, escTempC: 40,
    gpsLabel: 'GPS', gpsOk: true, imuYawDeg: 0, imuOk: true, wheelLeftRpm: 12,
    wheelRightRpm: 0, bladeOn: true, rain: false, dockCharging: false};

describe('existing anatomy motor display', () => {
    it.each(['mowgli', 'mavros'])('renders the same existing parts for %s Status', backend => {
        const blade = bladeTelemetry({stamp: {sec: 1, nanosec: 0}, blade_status_stamp: {sec: 1, nanosec: 0},
            mower_motor_rpm: 1200, mower_esc_temperature: 40, mower_motor_temperature: 35}, 10, 11, backend);
        const {container} = render(<RobotAnatomy inputs={{...inputs, motorTempC: blade.motorTemp,
            escTempC: blade.escTemp, bladeOn: blade.rpm > 0}}/>);
        fireEvent.mouseEnter(container.querySelectorAll('svg > g')[7]);
        expect(container.textContent).toContain(backend === 'mavros' ? '—' : '35');
        expect(container.textContent).not.toContain('NaN');
    });
    it('renders missing wheel telemetry as unavailable instead of stopped or NaN', () => {
        const {container} = render(<RobotAnatomy inputs={{...inputs, wheelLeftRpm: NaN}}/>);
        fireEvent.mouseEnter(container.querySelectorAll('svg > g')[5]);
        expect(container.textContent).toContain('—');
        expect(container.textContent).not.toContain('NaN');
    });
});
it('keeps a reported battery percentage when voltage is unavailable',()=>{
    const {container}=render(<RobotAnatomy inputs={{...inputs,vBattery:NaN}}/>);
    fireEvent.mouseEnter(container.querySelectorAll('svg > g')[3]);
    expect(container.textContent).toContain('80');expect(container.textContent).not.toContain('NaN');
});
