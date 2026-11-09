import {describe, expect, it} from 'vitest';
import {parseParamDocument, exportParamDocument} from './fcuParamDocument';
import {referenceParamHardware, reviewParamDocument, selectedParamCorrections} from './fcuParamProfile';
import {diagnosticGroups, groupDiagnosticRows, telemetryDiagnostics, diagnosticState} from './fcuParamDiagnostics';
const identity = {family: 'ardupilot' as const, vehicle: 'rover', version: '4.7.1', board: '', source: 'operator' as const};
const h = referenceParamHardware();
describe('focused FCU diagnostic presentation', () => {
    it('groups required/applicable checks, preserves hidden personal values exactly and retains provisional safety', () => {
        const text = '\uFEFF# synthetic\r\nSERIAL0_PROTOCOL,0\r\nATC_SPEED_P,1.23000\r\nBATT_VOLT_MULT,8.20\r\nARMING_REQUIRE,2\r\nARMING_SKIPCHK,0\r\nMAV_OPTIONS,0\r\nMAV1_OPTIONS,0';
        const doc = parseParamDocument(text);const hardware = {...h, serialPort: 0};
        const rows = reviewParamDocument(doc, identity, hardware); const groups = groupDiagnosticRows(rows, hardware, 'unknown');
        expect(Object.keys(groups)).toEqual(diagnosticGroups);
        expect(Object.values(groups).flat().some(row => row.category === 'calibration' || row.category === 'optional')).toBe(false);
        expect(groups.safety.find(row => row.id === 'arming.require')).toMatchObject({provisional: true});
        expect(groups.safety.some(row => row.id === 'safety.ARMING_SKIPCHK')).toBe(true);
        expect(groups.communication.some(row => row.id === 'link.target')).toBe(true);
        expect(groups.communication.some(row => row.parameter === 'MAV_OPTIONS')).toBe(true);
        expect(groups.telemetry.some(row => row.parameter === 'MAV1_OPTIONS')).toBe(true);
        expect(exportParamDocument(doc, selectedParamCorrections(rows, ['link.protocol']))).toBe(text.replace('SERIAL0_PROTOCOL,0', 'SERIAL0_PROTOCOL,2'));
    });
    it('hides CAN/RPM controls for PWM and GNSS FCU settings for direct Universal GNSS', () => {
        const doc = parseParamDocument('GPS1_TYPE,1\nGPS2_TYPE,2\nCAN_P1_BITRATE,500000\n');
        const hardware = {...h, drive: 'pwm-skid' as const};
        const groups = groupDiagnosticRows(reviewParamDocument(doc, identity, hardware), hardware, 'universal');
        expect(groups.can).toEqual([]);expect(groups.navigation.some(row => row.parameter.startsWith('GPS'))).toBe(false);
        const vesc = {...hardware, drive: 'vesc-dronecan' as const, legacyRpm: true};
        const can = groupDiagnosticRows(reviewParamDocument(doc, identity, vesc), vesc, 'mavros').can;
        expect(can.some(row => row.id === 'rpm.1.type')).toBe(true);
        expect(can.some(row => row.parameter === 'CAN_P1_BITRATE')).toBe(true);
    });
    it('does not turn provisional, ambiguous or missing requirements into false compatibility', () => {
        const rows = reviewParamDocument(parseParamDocument('MAV_SYSID,1\nMAV_SYSID,2\n'), identity, h);
        expect(diagnosticState(rows.find(row => row.id === 'link.target')!)).toBe('review');
        expect(diagnosticState(rows.find(row => row.id === 'link.gcs')!)).toBe('missing');
    });
});
describe('telemetry contracts without invented runtime evidence', () => {
    it.each(['', '4.6.3', '4.8.0'])('does not apply 4.7.1 stream mappings to version %s', version => {
        expect(telemetryDiagnostics(null, {...identity, version}, h, 'mavros')).toEqual([]);
    });
    it('does not apply Rover profiles to PX4', () => {
        expect(telemetryDiagnostics(null, {...identity, family: 'px4'}, h, 'mavros')).toEqual([]);
    });
    it('reads selected MAVx rates but never claims compatible/insufficient from configured rates or duplicates', () => {
        const doc = parseParamDocument('MAV2_EXTRA1,50\nMAV2_EXT_STAT,0\nMAV2_EXTRA3,10\nMAV2_RC_CHAN,10\nMAV2_RC_CHAN,20\n');
        const flows = telemetryDiagnostics(doc, identity, {...h, mavlinkChannel: 2}, 'mavros');
        expect(flows.every(flow => flow.state === 'review')).toBe(true);
        expect(flows.find(flow => flow.id === 'imu')?.parameters[0]).toMatchObject({name: 'MAV2_EXTRA1', configuredHz: 50});
        expect(flows.find(flow => flow.id === 'rc')?.parameters[0]).toMatchObject({ambiguous: true, configuredHz: undefined});
        expect(flows.flatMap(flow => flow.parameters).some(parameter => parameter.name === 'SCHED_LOOP_RATE')).toBe(false);
    });
    it('does not impose FCU GNSS with Universal GNSS, retains blade feedback even on PWM drive', () => {
        const flows = telemetryDiagnostics(null, identity, {...h, drive: 'pwm-skid', blade: true}, 'universal');
        expect(flows.some(flow => flow.id === 'gnss')).toBe(false);
        expect(flows.find(flow => flow.id === 'gnssExternal')?.parameters).toEqual([]);
        expect(flows.find(flow => flow.id === 'bladeRpm')?.freshnessReferenceMs).toBe(1000);
        expect(flows.some(flow => flow.id === 'bladeOutput')).toBe(true);
        expect(flows.some(flow => flow.id === 'esc')).toBe(false);
    });
});
