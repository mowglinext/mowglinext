import {describe, expect, it} from 'vitest';
import {GnssStatusConstants, type GnssStatus} from '../types/ros.ts';
import {
    deriveGpsStatus,
    gnssReceiverLabel,
    hasGnssCapability,
    hasGnssValue,
    readGnssBooleanState,
    readGnssNumber,
} from './gpsStatus.ts';

describe('deriveGpsStatus', () => {
    it('maps RTK fixed status to the highest quality label', () => {
        const status: GnssStatus = {fix_type: GnssStatusConstants.FIX_TYPE_RTK_FIXED};
        expect(deriveGpsStatus(status)).toEqual({
            fixType: 'RTK_FIX',
            label: 'RTK fixed',
            percent: 100,
        });
    });

    it('maps plain GPS fix status', () => {
        const status: GnssStatus = {fix_type: GnssStatusConstants.FIX_TYPE_GPS_FIX};
        expect(deriveGpsStatus(status)).toEqual({
            fixType: 'GPS_FIX',
            label: 'GPS fix',
            percent: 25,
        });
    });

    it('falls back to no-fix when typed status is absent', () => {
        expect(deriveGpsStatus(undefined)).toEqual({
            fixType: 'NO_FIX',
            label: 'No GPS',
            percent: 0,
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

    it('formats user-facing receiver labels without leaking backend ids', () => {
        expect(gnssReceiverLabel({backend: 'unicore', receiver_vendor: 'Unicore'})).toBe('Unicore GNSS');
        expect(gnssReceiverLabel({backend: 'ublox', receiver_vendor: 'u-blox'})).toBe('u-blox GNSS');
        expect(gnssReceiverLabel({receiver_model: 'F9P'})).toBe('F9P');
        expect(gnssReceiverLabel({backend: 'nmea'})).toBe('GNSS');
    });
});
