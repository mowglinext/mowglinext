import sanitizedMavros from '../test/fixtures/gnss-mavros-sanitized.json';
import {describe, expect, it} from 'vitest';
import {GnssStatusConstants as C} from '../types/ros';
import {decodeGnssStatus} from './gnssTelemetry';
import {currentRtkMode, deriveGpsStatus, deriveGnssStatusFromDiagnostics, mergeGnssStatusDiagnosticProjection, readGnssNumber} from './gpsStatus';

const projection = (values: Record<string, string>, message = 'RTCM forwarding active') => deriveGnssStatusFromDiagnostics({status: [
    {name: 'universal_gnss/summary', values: Object.entries(values).map(([key, value]) => ({key, value}))},
    {name: 'GPS', values: [{key: 'fix_status', value: '0'}]},
    {name: 'universal_gnss_ntrip/rtcm_forwarding', message},
]});

describe('GNSS runtime boundary and source authority', () => {
    it.each([[5, '2D_FIX', 20], [6, '3D_FIX', 40], [7, 'DGPS', 60], [2, 'RTK_FLOAT', 80], [3, 'RTK_FIX', 100]])(
        'validates canonical fix %s without changing display percentages', (fix_type, fixType, percent) => {
            const decoded = decodeGnssStatus({fix_type, fix_valid: true});
            expect(decoded.kind).toBe('valid');
            if (decoded.kind === 'valid') expect(deriveGpsStatus(decoded.data)).toMatchObject({fixType, percent});
        });
    it.each([null, [], 'FIX', 42, {fix_type: '3'}, {fix_valid: 1}, {backend: 2}, {rtk_mode: 255},
        {fix_type: 255}, {correction_stream_status: -1}, {baseline_solution_status: 8}, {satellites_used: 1.5},
        {satellites_used: -1}, {satellites_visible: 65536}, {horizontal_accuracy_m: Infinity},
        {horizontal_accuracy_m: NaN}, {baseline_length_m: -1}, {msm_summary_seen: 'false'},
        {msm_summary_age_s: null}, {capability_flags: -1}, {value_flags: 0.5}, {receiver_model: null},
        {header: {stamp: {sec: 1, nanosec: 1000000000}, frame_id: 'gps'}},
        {capability_flags: 0, value_flags: C.CAP_RTK_MODE}, {fix_type: undefined}, {value_flags: C.CAP_RTK_MODE},
        {capability_flags: C.CAP_MSM_SUMMARY, value_flags: C.CAP_MSM_SUMMARY, msm_summary_seen: true}])(
        'rejects malformed consumed fields in %j', raw => expect(decodeGnssStatus(raw).kind).toBe('invalid'));
    it.each([{}, {future_extension: 1}, {header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}}])(
        'does not report an empty/incomplete envelope as a sample', raw => expect(decodeGnssStatus(raw).kind).toBe('unsupported'));
    it('accepts absent optional channels and ignores unconsumed extensions', () => {
        expect(decodeGnssStatus({fix_valid: false, quality_percent: NaN, future_extension: 'new'}))
            .toEqual({kind: 'valid', data: {fix_valid: false}});
    });
    it('preserves MAVROS unavailable float sentinels without manufacturing a measured zero', () => {
        const decoded = decodeGnssStatus({backend: 'mavros_primary', fix_valid: true, fix_type: 6,
            capability_flags: C.CAP_HORIZONTAL_ACCURACY, value_flags: 0,
            horizontal_accuracy_m: NaN, msm_summary_age_s: NaN, quality_percent: NaN});
        expect(decoded).toEqual({kind: 'valid', data: {backend: 'mavros_primary', fix_valid: true,
            fix_type: 6, capability_flags: C.CAP_HORIZONTAL_ACCURACY, value_flags: 0}});
        expect(decodeGnssStatus({value_flags: C.CAP_HORIZONTAL_ACCURACY, horizontal_accuracy_m: NaN}).kind).toBe('invalid');
    });
    it.each([{fix_valid: false, fix_type: 3, rtk_mode: 3}, {fix_valid: false, fix_type: 2, rtk_mode: 2}])(
        'explicit invalid positioning wins over positive codes', status => expect(deriveGpsStatus(status)).toMatchObject({fixType: 'NO_FIX', percent: 0}));
    it.each([{fix_type: 255, rtk_mode: 3, fix_valid: true}, {fix_type: 0, rtk_mode: 3, fix_valid: true},
        {fix_type: 4, rtk_mode: 3, fix_valid: true}, {rtk_mode: 3}, {fix_valid: true},
        {fix_type: 1}, {fix_type: 1, rtk_mode: 255}])(
        'does not promote unknown/unavailable positioning', status => expect(deriveGpsStatus(status).percent).toBe(0));
    it('ignores an unavailable RTK scalar but retains explicit basic fix evidence', () => {
        expect(deriveGpsStatus({fix_type: 6, fix_valid: true, rtk_mode: 3, capability_flags: C.CAP_RTK_MODE, value_flags: 0}).fixType).toBe('3D_FIX');
        expect(deriveGpsStatus({fix_type: 3, fix_valid: true, rtk_mode: 0, capability_flags: 0, value_flags: 0}).fixType).toBe('RTK_FIX');
    });
    it('cannot upgrade typed negative status or receiver identity from diagnostics', () => {
        const typed = {backend: 'mavros_primary', fix_valid: false};
        const merged = mergeGnssStatusDiagnosticProjection(typed, projection({fix_valid: 'true'}));
        expect(merged.fix_type).toBeUndefined(); expect(merged.backend).toBe('mavros_primary');
        expect(deriveGpsStatus(merged).percent).toBe(0);
    });
    it.each([0, C.CAP_CORRECTION_STREAM])('explicit masks %s prohibit diagnostic capability/value invention', capability_flags => {
        const typed = {fix_type: 6, capability_flags, value_flags: 0, correction_stream_status: C.CORRECTION_STREAM_STATUS_UNKNOWN};
        expect(mergeGnssStatusDiagnosticProjection(typed, projection({fix_valid: 'true'}))).toEqual(typed);
    });
    it('does not poison legacy typed RTK evidence with diagnostic-only masks', () => {
        const typed = {fix_type: 1, fix_valid: true, rtk_mode: 3};
        const merged = mergeGnssStatusDiagnosticProjection(typed, projection({fix_valid: 'false'}));
        expect(merged).toEqual(typed); expect(deriveGpsStatus(merged).fixType).toBe('RTK_FIX');
    });
    it('does not fill a typed MSM group around an explicit negative flag', () => {
        expect(mergeGnssStatusDiagnosticProjection({fix_valid: false, msm_summary_seen: false}, {
            capability_flags: C.CAP_MSM_SUMMARY, value_flags: C.CAP_MSM_SUMMARY, msm_summary_seen: true,
            msm_summary_valid: true, msm_summary_decoded: true,
        })).toEqual({fix_valid: false, msm_summary_seen: false});
    });
    it('keeps diagnostic-only fallback generic and respects explicit contradictory negatives', () => {
        expect(deriveGpsStatus(projection({fix_valid: 'false'})).percent).toBe(0);
        expect(deriveGpsStatus(projection({fix_valid: 'true'})).fixType).toBe('UNKNOWN');
        const negativeGps = deriveGnssStatusFromDiagnostics({status: [
            {name: 'universal_gnss/summary', values: [{key: 'fix_valid', value: 'true'}]},
            {name: 'GPS', values: [{key: 'fix_status', value: '-1'}]},
        ]});
        // The unidentified GPS entry cannot override a different summary source.
        expect(negativeGps?.fix_valid).toBe(true);
        expect(negativeGps?.fix_type).toBeUndefined();
    });
    it.each(['0junk', '0x0', '255', '3', ''])('does not coerce unsupported GPS diagnostic %s', value => {
        const status = deriveGnssStatusFromDiagnostics({status: [{name: 'GPS', values: [{key: 'fix_status', value}]}]});
        expect(status?.fix_type).toBeUndefined(); expect(status?.fix_valid).toBeUndefined();
    });
    it('does not fabricate zero/false values for incomplete diagnostic MSM data', () => {
        const status = deriveGnssStatusFromDiagnostics({status: [{name: 'universal_gnss_ntrip/rtcm_semantic/msm_summary',
            values: [{key: 'satellite_count', value: '7junk'}, {key: 'age_s', value: '0.2seconds'}, {key: 'seen', value: 'false'}]}]});
        expect(status?.msm_summary_seen).toBe(false); expect(status?.msm_summary_valid).toBeUndefined();
        expect(status?.msm_summary_satellite_count).toBeUndefined(); expect(status?.msm_summary_age_s).toBeUndefined();
        expect(status?.value_flags).toBeUndefined();
    });
    it('does not confuse inactive with active correction forwarding', () => {
        expect(projection({}, 'RTCM forwarding inactive')?.correction_stream_status).toBeUndefined();
        expect(deriveGnssStatusFromDiagnostics({status: [{name: 'universal_gnss_ntrip/rtcm_forwarding', message: 'RTCM forwarding inactive'}]})?.correction_stream_status).toBe(C.CORRECTION_STREAM_STATUS_UNKNOWN);
    });
    it('guards numeric displays using both masks and finite values', () => {
        expect(readGnssNumber({capability_flags: 0, value_flags: C.CAP_HORIZONTAL_ACCURACY}, C.CAP_HORIZONTAL_ACCURACY, 0)).toBeUndefined();
        expect(readGnssNumber({capability_flags: 8, value_flags: 8}, 8, NaN)).toBeUndefined();
        expect(readGnssNumber({capability_flags: 8, value_flags: 8}, 8, 0)).toBe(0);
    });
});

describe('Lot 25 confirmed presentation/provenance defects', () => {
    it('retains the producer position observation sequence', () => {
        expect(decodeGnssStatus({fix_type: 3, fix_valid: true, position_observation_sequence: 42}))
            .toEqual({kind: 'valid', data: {fix_type: 3, fix_valid: true, position_observation_sequence: 42}});
    });
    it('rejects contradictory Fixed mode without erasing explicit 2D evidence', () => {
        expect(deriveGpsStatus({fix_valid: true, fix_type: 5, rtk_mode: 3}).fixType).toBe('2D_FIX');
        expect(currentRtkMode({fix_valid: true, fix_type: 5, rtk_mode: 3})).toBeUndefined();
    });
    it('does not hide a legacy explicit measured zero solely because masks are absent', () => {
        expect(readGnssNumber({horizontal_accuracy_m: 0}, C.CAP_HORIZONTAL_ACCURACY, 0)).toBe(0);
    });
    it('does not enrich a MAVROS fix with unrelated Universal correction diagnostics', () => {
        const typed = {backend: 'mavros_gps1', fix_valid: false};
        expect(mergeGnssStatusDiagnosticProjection(typed, projection({fix_valid: 'true'}))).toEqual(typed);
    });
});

it('does not merge positioning and correction diagnostics from distinct receivers', () => {
    const status = deriveGnssStatusFromDiagnostics({status: [
        {name: 'GPS', hardware_id: 'receiver-A', values: [{key: 'fix_status', value: '0'}]},
        {name: 'universal_gnss_ntrip/rtcm_forwarding', hardware_id: 'receiver-B', message: 'RTCM forwarding active'},
    ]});
    expect(status?.correction_stream_status).toBeUndefined();
});

it('accepts the actual Foxglove NaN-to-null wire representation only for explicitly unavailable fields', () => {
    const decoded = decodeGnssStatus(sanitizedMavros);
    expect(decoded.kind).toBe('valid');
    if (decoded.kind !== 'valid') throw new Error('Valid sanitized ROS frame rejected');
    expect(deriveGpsStatus(decoded.data).fixType).toBe('RTK_FIX');
    expect(readGnssNumber(decoded.data, C.CAP_HORIZONTAL_ACCURACY, decoded.data.horizontal_accuracy_m)).toBe(0);
    expect(decoded.data.vertical_accuracy_m).toBeUndefined();
    expect(decoded.data.msm_summary_age_s).toBeUndefined();
});

it.each([0, C.CAP_HORIZONTAL_ACCURACY])('honors measured-zero and unavailable flags %s after sanitization', value_flags => {
    for (const value of [0, null, NaN]) {
        const decoded = decodeGnssStatus({capability_flags: C.CAP_HORIZONTAL_ACCURACY, value_flags, horizontal_accuracy_m: value, fix_valid: true, fix_type: 6});
        if (value_flags && value !== 0) expect(decoded.kind).toBe('invalid');
        else {
            expect(decoded.kind).toBe('valid');
            if (decoded.kind !== 'valid') throw new Error('Unexpected rejection');
            expect(readGnssNumber(decoded.data, C.CAP_HORIZONTAL_ACCURACY, decoded.data.horizontal_accuracy_m)).toBe(value_flags ? 0 : undefined);
        }
    }
});
it.each([9007199254740992, -1, 1.5, '42', null])('rejects untrustworthy position identities %s', position_observation_sequence => {
    expect(decodeGnssStatus({fix_valid: true, fix_type: 3, position_observation_sequence}).kind).toBe('invalid');
});
it('preserves same-receiver legacy diagnostic enrichment without inventing a typed fix category', () => {
    const projected = deriveGnssStatusFromDiagnostics({status: [
        {name: 'GPS', hardware_id: 'A', values: [{key: 'fix_status', value: '0'}]},
        {name: 'universal_gnss_ntrip/rtcm_forwarding', hardware_id: 'A', message: 'RTCM forwarding active'},
    ]});
    expect(projected?.correction_stream_status).toBe(C.CORRECTION_STREAM_STATUS_ACTIVE);
    expect(deriveGpsStatus(projected).fixType).toBe('UNKNOWN');
});
