import {describe, expect, it} from 'vitest';
import reference from './fixtures/synthetic-runtime.param?raw';
import {parseParamDocument, exportParamDocument} from './fcuParamDocument';
import {identityFromMavros, identityFromParamComments, unknownParamIdentity} from './fcuParamIdentity';
import {admissibleParamCorrections, profileForIdentity, referenceParamHardware, reviewParamDocument, selectedParamCorrections} from './fcuParamProfile';
const identity = {family: 'ardupilot' as const, vehicle: 'rover', version: '4.7.1', board: 'Pixhawk 5X', source: 'operator' as const};

describe('lossless Mission Planner .param', () => {
    it('imports synthetic parameters without inferring a version or changing any bytes', () => {
        const doc = parseParamDocument(reference);
        expect(doc.entries.size).toBe(12);
        expect(reference.startsWith('\uFEFF')).toBe(true);
        expect(reference.match(/\r\n/g)).toHaveLength(13);
        expect(reference.endsWith('\n')).toBe(false); expect(exportParamDocument(doc, new Map())).toBe(reference);
        expect(identityFromParamComments(reference)).toEqual(unknownParamIdentity());
    });
    it('preserves BOM, CRLF, comments, ordering, spacing and untouched numeric formatting', () => {
        const text = '\uFEFF# opérateur\r\nMAV_GCS_SYSID , 42 # note\r\nATC_SPEED_P\t0.20000\r\nUSER_VALUE,1.20e+02';
        expect(exportParamDocument(parseParamDocument(text), new Map([['MAV_GCS_SYSID', 255]])))
            .toBe(text.replace('42 # note', '255 # note'));
    });
    it('blocks duplicate/missing corrections but preserves unselected duplicates', () => {
        const doc = parseParamDocument('A,1\nA,2\nB,3\n');
        expect(() => exportParamDocument(doc, new Map([['A', 4]]))).toThrow();
        expect(() => exportParamDocument(doc, new Map([['ABSENT', 4]]))).toThrow();
        expect(exportParamDocument(doc, new Map([['B', 5]]))).toBe('A,1\nA,2\nB,5\n');
    });
    it.each(['', '# comment\n', 'A,NaN', 'A,1e999', 'bad line', 'A,1\0', 'A: 1'])('rejects invalid inputs: %s', text => {
        expect(() => parseParamDocument(text)).toThrow('invalidFile');
    });
});
describe('firmware-specific parameter comparison', () => {
    it('requires real heartbeat/version metadata and never equates MAVROS with ArduPilot', () => {
        expect(identityFromMavros({state: {}, vehicle: {autopilot: 3, type: 10, available_info: 0}}).family).toBe('unknown');
        const info = {state: {connected: true}, vehicle: {autopilot: 3, type: 10, available_info: 3, flight_sw_version: 0x040701ff}};
        expect(identityFromMavros(info)).toMatchObject({family: 'ardupilot', vehicle: 'rover', version: '4.7.1'});
        expect(identityFromMavros({...info, state: {connected: false}}).family).toBe('unknown');
        expect(identityFromMavros({...info, vehicle: {...info.vehicle, autopilot: 12}}).family).toBe('px4');
    });
    it.each([{...identity, family: 'px4' as const}, {...identity, version: ''}, {...identity, version: '4.6.3'}, {...identity, vehicle: 'copter'}])('does not apply Rover rules to unknown/unsupported identities', target => {
        expect(profileForIdentity(target)).toBeUndefined();
        expect(reviewParamDocument(parseParamDocument(reference), target, referenceParamHardware())).toEqual([]);
    });
    it('uses explicit file comments only, without transferring YAML builder metadata', () => {
        expect(identityFromParamComments('# ArduRover v4.7.1\n# Board: Pixhawk 5X\nA,1')).toEqual({...identity, source: 'file'});
        expect(identityFromParamComments('# ArduRover v4.7.1\n# PX4 v1.16.0\nA,1').family).toBe('unknown');
        expect(identityFromParamComments('# ArduRover v4.7.1\n# ArduRover v4.6.3\nA,1').family).toBe('unknown');
        expect(identityFromParamComments('# previous example: ArduRover v4.7.1\nA,1').family).toBe('unknown');
    });
    it('selects conditional CAN/RPM requirements by hardware and feedback, with configurable slots', () => {
        const doc = parseParamDocument(reference);
        const pwm = reviewParamDocument(doc, identity, {...referenceParamHardware(), drive: 'pwm-skid'});
        expect(pwm.some(row => row.id.startsWith('can.') || row.id.startsWith('rpm.'))).toBe(false);
        const vesc = reviewParamDocument(doc, identity, {...referenceParamHardware(), drive: 'vesc-dronecan', legacyRpm: true, rightEscIndex: 4, leftEscIndex: 5});
        expect(vesc.find(row => row.parameter === 'RPM1_ESC_MASK')).toMatchObject({state: 'review', editable: false, applicability: 'vesc-dronecan', versions: ['4.7.1']});
        expect(vesc.find(row => row.parameter === 'RPM2_ESC_MASK')?.proposed).toBeUndefined();
        expect(vesc.find(row => row.parameter === 'CAN_D1_UC_ESC_BM')?.proposed).toBeUndefined(); // never copy the reference routing
    });
    it('shows absent capabilities without inventing a parameter; duplicates cannot be corrected', () => {
        const rows = reviewParamDocument(parseParamDocument('MAV_SYSID,1\nMAV_SYSID,2\n'), identity, referenceParamHardware());
        expect(rows.find(row => row.parameter === 'MAV_GCS_SYSID')).toMatchObject({state: 'missing', editable: false});
        expect(rows.find(row => row.parameter === 'MAV_SYSID')).toMatchObject({state: 'ambiguous', editable: false});
    });
    it('exports only accepted selections; personal calibration and safety masks remain exact', () => {
        const text = 'MAV_SYSID,9\nMAV_GCS_SYSID,42\nARMING_REQUIRE,0\nARMING_SKIPCHK,0\nATC_SPEED_P,0.20000\nSERIAL0_PROTOCOL,0\n';
        const doc = parseParamDocument(text);const rows = reviewParamDocument(doc, identity, {...referenceParamHardware(), serialPort: 0});
        expect(admissibleParamCorrections(rows).map(row => row.parameter)).toEqual(['SERIAL0_PROTOCOL']);
        const output = exportParamDocument(doc, selectedParamCorrections(rows, ['link.protocol']));
        expect(output).toBe(text.replace('SERIAL0_PROTOCOL,0', 'SERIAL0_PROTOCOL,2'));
        expect(rows.find(row => row.id === 'arming.require')).toMatchObject({state: 'review', editable: false, proposed: undefined});
        expect(() => selectedParamCorrections(rows, ['arming.require'])).toThrow('ambiguousParameter');
        // Any future technically established sensitive correction still needs explicit consent.
        const sensitive = [{...rows[0], id: 'verified-sensitive', editable: true, proposed: 1, sensitive: true}];
        expect(() => selectedParamCorrections(sensitive, ['verified-sensitive'])).toThrow('confirmationRequired');
        expect(() => selectedParamCorrections(sensitive, ['verified-sensitive'], {confirmed: true, reason: ' '})).toThrow('confirmationRequired');
        expect(selectedParamCorrections(sensitive, ['verified-sensitive'], {confirmed: true, reason: 'verified contract'}).size).toBe(1);
    });
    it('never treats arbitrary reference differences as compatibility failures or assumed corrections', () => {
        const rows = reviewParamDocument(parseParamDocument('MAV_SYSID,1\nMAV_GCS_SYSID,255\nMAV1_EXTRA1,9\nARMING_REQUIRE,2\nSERVO1_FUNCTION,42\nATC_SPEED_P,123.000\n'), identity, {...referenceParamHardware(), drive: 'pwm-skid'});
        expect(rows.filter(row => row.state === 'mismatch')).toEqual([]);
        expect(rows.find(row => row.parameter === 'ATC_SPEED_P')?.state).toBe('preserved');
        expect(rows.every(row => row.versions.includes('4.7.1') && row.contract.includes('bf5bb0d') && row.source !== 'reference-inventory')).toBe(true);
        const rangeRows = reviewParamDocument(parseParamDocument('MAV_GCS_SYSID,42\nMAV_GCS_SYSID_HI,250\n'), identity, referenceParamHardware());
        expect(rangeRows.find(row => row.id === 'link.gcs')).toMatchObject({state: 'review', editable: false});
    });
    it.each([
        {target: 23, gcs: 201, high: 0},
        {target: 1, gcs: 255, high: 0},
        {target: 23, gcs: 201, high: 240},
    ])('never renumbers unconfirmed SYSIDs, including multi-robot IDs and launch-default matches: %o', ({target, gcs, high}) => {
        const text = `MAV_SYSID,${target}\r\nMAV_GCS_SYSID,${gcs}\r\nMAV_GCS_SYSID_HI,${high}`;
        const doc = parseParamDocument(text);
        // Neither default nor manually entered bindings establish effective IDs/GCS policy.
        for (const hardware of [referenceParamHardware(), {...referenceParamHardware(), targetSysId: target, gcsSysId: gcs}]) {
            const rows = reviewParamDocument(doc, identity, hardware);
            for (const id of ['link.target', 'link.gcs']) {
                expect(rows.find(row => row.id === id)).toMatchObject({state: 'review', editable: false, proposed: undefined});
                expect(() => selectedParamCorrections(rows, [id], {confirmed: true, reason: 'typed IDs'})).toThrow('ambiguousParameter');
            }
            expect(admissibleParamCorrections(rows)).toEqual([]);
            expect(exportParamDocument(doc, selectedParamCorrections(rows, []))).toBe(text);
        }
    });
    it('does not require unused CAN/RPM bindings for a PWM installation', () => {
        const rows = reviewParamDocument(parseParamDocument(reference), identity, {...referenceParamHardware(), drive: 'pwm-skid', canPort: 0, rightEscIndex: 0, leftEscIndex: 0});
        expect(rows.some(row => row.id === 'drive.right')).toBe(true);
    });
    it('preserves MAVLink 1 where already configured and rejects conflicting output bindings', () => {
        const doc = parseParamDocument('SERIAL0_PROTOCOL,1\n');
        expect(reviewParamDocument(doc, identity, {...referenceParamHardware(), serialPort: 0}).find(row => row.parameter === 'SERIAL0_PROTOCOL')?.state).toBe('compliant');
        expect(reviewParamDocument(doc, identity, {...referenceParamHardware(), drive: 'pwm-skid', leftOutput: 1})).toEqual([]);
    });
});
