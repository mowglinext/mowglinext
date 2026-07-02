import {describe, expect, it} from 'vitest';
import {GnssStatusConstants, type GnssStatus} from '../types/ros.ts';
// deriveGpsStatus resolves its label via i18n; assert against the active locale
// (pinned to English in test/setup.ts) so reworded copy doesn't break the test.
import en from '../i18n/locales/en.json';
import {
    deriveGpsStatus,
    deriveGnssStatusFromDiagnostics,
    gnssBaselineSolutionStatusLabel,
    gnssCorrectionStreamStatusLabel,
    gnssRtkModeLabel,
    gnssReceiverLabel,
    hasTypedGnssStatusSample,
    hasGnssCapability,
    hasGnssValue,
    mergeGnssStatusDiagnosticProjection,
    readGnssBooleanState,
    readGnssNumber,
} from './gpsStatus.ts';

describe('deriveGpsStatus', () => {
    it('maps RTK fixed status to the highest quality label', () => {
        const status: GnssStatus = {fix_type: GnssStatusConstants.FIX_TYPE_RTK_FIXED};
        expect(deriveGpsStatus(status)).toEqual({
            fixType: 'RTK_FIX',
            label: en.gpsStatus.rtkFixed,
            percent: 100,
        });
    });

    it('maps plain GPS fix status', () => {
        const status: GnssStatus = {fix_type: GnssStatusConstants.FIX_TYPE_GPS_FIX};
        expect(deriveGpsStatus(status)).toEqual({
            fixType: 'GPS_FIX',
            label: en.gpsStatus.gpsFix,
            percent: 25,
        });
    });
    it('prefers RTK fixed mode over plain GPS fix type', () => {
        const status: GnssStatus = {
            fix_valid: true,
            fix_type: GnssStatusConstants.FIX_TYPE_GPS_FIX,
            rtk_mode: GnssStatusConstants.RTK_MODE_FIXED,
        };

        expect(deriveGpsStatus(status)).toEqual({
            fixType: 'RTK_FIX',
            label: en.gpsStatus.rtkFixed,
            percent: 100,
        });
    });

    it('prefers RTK float mode over plain GPS fix type', () => {
        const status: GnssStatus = {
            fix_valid: true,
            fix_type: GnssStatusConstants.FIX_TYPE_GPS_FIX,
            rtk_mode: GnssStatusConstants.RTK_MODE_FLOAT,
        };

        expect(deriveGpsStatus(status)).toEqual({
            fixType: 'RTK_FLOAT',
            label: en.gpsStatus.rtkFloat,
            percent: 50,
        });
    });
    it('prefers fix_valid=false over stale fix_type values', () => {
        const status: GnssStatus = {
            fix_type: GnssStatusConstants.FIX_TYPE_GPS_FIX,
            fix_valid: false,
        };

        expect(deriveGpsStatus(status)).toEqual({
            fixType: 'NO_FIX',
            label: en.gpsStatus.noGps,
            percent: 0,
        });
    });

    it('does not show no-fix when fix_valid=true but fix_type is missing or stale', () => {
        const status: GnssStatus = {
            fix_type: GnssStatusConstants.FIX_TYPE_NO_FIX,
            fix_valid: true,
        };

        expect(deriveGpsStatus(status)).toEqual({
            fixType: 'GPS_FIX',
            label: en.gpsStatus.gpsFix,
            percent: 25,
        });
    });

    it('falls back to no-fix when typed status is absent', () => {
        expect(deriveGpsStatus(undefined)).toEqual({
            fixType: 'NO_FIX',
            label: en.gpsStatus.noGps,
            percent: 0,
        });
    });

    it('detects when a typed GNSS sample is actually populated', () => {
        expect(hasTypedGnssStatusSample({})).toBe(false);
        expect(hasTypedGnssStatusSample({backend: 'universal'})).toBe(true);
        expect(hasTypedGnssStatusSample({fix_type: GnssStatusConstants.FIX_TYPE_GPS_FIX})).toBe(true);
    });

    it('derives a limited fallback GNSS status from diagnostics only when typed status is absent', () => {
        expect(deriveGnssStatusFromDiagnostics({
            status: [
                {
                    name: 'universal_gnss/summary',
                    values: [
                        {key: 'fix_valid', value: 'true'},
                        {key: 'correction_available', value: 'true'},
                    ],
                },
                {
                    name: 'GPS',
                    values: [
                        {key: 'fix_status', value: '0'},
                    ],
                },
            ],
        })).toEqual({
            backend: 'universal',
            fix_type: GnssStatusConstants.FIX_TYPE_GPS_FIX,
            fix_valid: true,
        });
    });

    it('keeps SBAS and GBAS fallback fixes as generic GPS fixes instead of guessing RTK', () => {
        expect(deriveGnssStatusFromDiagnostics({
            status: [
                {
                    name: 'GPS',
                    values: [
                        {key: 'fix_status', value: '1'},
                    ],
                },
            ],
        })).toEqual({
            backend: undefined,
            fix_type: GnssStatusConstants.FIX_TYPE_GPS_FIX,
            fix_valid: true,
        });

        expect(deriveGnssStatusFromDiagnostics({
            status: [
                {
                    name: 'GPS',
                    values: [
                        {key: 'fix_status', value: '2'},
                    ],
                },
            ],
        })).toEqual({
            backend: undefined,
            fix_type: GnssStatusConstants.FIX_TYPE_GPS_FIX,
            fix_valid: true,
        });
    });

    it('distinguishes supported fields from current values', () => {
        const status: GnssStatus = {
            capability_flags: GnssStatusConstants.CAP_HORIZONTAL_ACCURACY,
            value_flags: 0,
            horizontal_accuracy_m: 0.02,
        };

        expect(hasGnssCapability(status, GnssStatusConstants.CAP_HORIZONTAL_ACCURACY)).toBe(true);
        expect(hasGnssValue(status, GnssStatusConstants.CAP_HORIZONTAL_ACCURACY)).toBe(false);
        expect(readGnssNumber(
            status,
            GnssStatusConstants.CAP_HORIZONTAL_ACCURACY,
            status.horizontal_accuracy_m,
        )).toBeUndefined();
    });

    it('maps optional boolean fields through support and value flags', () => {
        const supportedUnknown: GnssStatus = {
            capability_flags: GnssStatusConstants.CAP_JAMMING_STATUS,
            value_flags: 0,
            jamming_detected: false,
        };
        const supportedTrue: GnssStatus = {
            capability_flags: GnssStatusConstants.CAP_JAMMING_STATUS,
            value_flags: GnssStatusConstants.CAP_JAMMING_STATUS,
            jamming_detected: true,
        };

        expect(readGnssBooleanState(
            supportedUnknown,
            GnssStatusConstants.CAP_JAMMING_STATUS,
            supportedUnknown.jamming_detected,
        )).toBe('unknown');
        expect(readGnssBooleanState(
            supportedTrue,
            GnssStatusConstants.CAP_JAMMING_STATUS,
            supportedTrue.jamming_detected,
        )).toBe('true');
        expect(readGnssBooleanState(
            undefined,
            GnssStatusConstants.CAP_JAMMING_STATUS,
            undefined,
        )).toBe('unsupported');
    });

    it('maps RTK mode labels from the public enum values', () => {
        expect(gnssRtkModeLabel({rtk_mode: GnssStatusConstants.RTK_MODE_NONE})).toBe('None');
        expect(gnssRtkModeLabel({rtk_mode: GnssStatusConstants.RTK_MODE_FLOAT})).toBe('Float');
        expect(gnssRtkModeLabel({rtk_mode: GnssStatusConstants.RTK_MODE_FIXED})).toBe('Fixed');
        expect(gnssRtkModeLabel({rtk_mode: GnssStatusConstants.RTK_MODE_UNKNOWN})).toBe('Unknown');
    });

    it('maps baseline solution labels from the public enum values', () => {
        expect(gnssBaselineSolutionStatusLabel({
            baseline_solution_status: GnssStatusConstants.BASELINE_STATUS_COMPUTED,
        })).toBe(en.gpsStatus.baselineComputed);
        expect(gnssBaselineSolutionStatusLabel({
            baseline_solution_status: GnssStatusConstants.BASELINE_STATUS_NOT_SOLVED,
        })).toBe(en.gpsStatus.baselineNotSolved);
    });

    it('maps correction stream labels from the public enum values', () => {
        expect(gnssCorrectionStreamStatusLabel({
            correction_stream_status: GnssStatusConstants.CORRECTION_STREAM_STATUS_ACTIVE,
        })).toBe(en.gpsStatus.correctionStreamActive);
        expect(gnssCorrectionStreamStatusLabel({
            correction_stream_status: GnssStatusConstants.CORRECTION_STREAM_STATUS_WAITING,
        })).toBe(en.gpsStatus.correctionStreamWaiting);
    });

    it('merges diagnostics-derived correction stream fields into a typed GNSS sample when the bridge is older', () => {
        const merged = mergeGnssStatusDiagnosticProjection(
            {
                backend: 'universal',
                fix_type: GnssStatusConstants.FIX_TYPE_GPS_FIX,
                capability_flags: GnssStatusConstants.CAP_HORIZONTAL_ACCURACY,
                value_flags: GnssStatusConstants.CAP_HORIZONTAL_ACCURACY,
            },
            deriveGnssStatusFromDiagnostics({
                status: [
                    {
                        name: 'universal_gnss_ntrip/rtcm_forwarding',
                        message: 'RTCM forwarding active',
                        values: [],
                    },
                ],
            }),
        );

        expect(merged.correction_stream_status).toBe(GnssStatusConstants.CORRECTION_STREAM_STATUS_ACTIVE);
        expect((merged.capability_flags ?? 0) & GnssStatusConstants.CAP_CORRECTION_STREAM).not.toBe(0);
    });

    it('formats user-facing receiver labels without leaking backend ids', () => {
        expect(gnssReceiverLabel({backend: 'unicore', receiver_vendor: 'Unicore'})).toBe('Unicore');
        expect(gnssReceiverLabel({backend: 'ublox', receiver_vendor: 'u-blox'})).toBe('u-blox');
        expect(gnssReceiverLabel({receiver_model: 'F9P'})).toBe('F9P');
        expect(gnssReceiverLabel({backend: 'nmea'})).toBe('GNSS');
    });
});
