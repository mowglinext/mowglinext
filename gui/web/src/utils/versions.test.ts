import {describe, expect, it} from 'vitest';
import {browserBuildDiffers, firmwareInventoryState, imageVersion} from './versions';

describe('installed version identity', () => {
    it('compares the browser to served web assets, not backend source or dates', () => {
        expect(browserBuildDiffers({}, {id: 'build-a'})).toBe(false);
        expect(browserBuildDiffers({id:'build-a'}, {})).toBe(false);
        expect(browserBuildDiffers({id:'build-a'}, {id:'build-a'})).toBe(false);
        expect(browserBuildDiffers({id:'build-a'}, {id:'build-b'})).toBe(true);
    });
    it('handles registry ports and digest-pinned references without inventing tags', () => {
        expect(imageVersion('localhost:5000/gui:feat-test')).toBe('feat-test');
        expect(imageVersion('localhost:5000/gui')).toBe('');
        expect(imageVersion('ghcr.io/example/gui@sha256:abc')).toBe('');
    });
});

describe('firmware inventory freshness', () => {
    const status = {firmware_version: '1.8.213', firmware_protocol_version: 6, firmware_compatible: true};
    it('requires an advancing mainboard timestamp before declaring compatibility', () => {
        expect(firmwareInventoryState(status, true, null, 10000)).toBe('waiting');
        expect(firmwareInventoryState(status, true, 9000, 10000)).toBe('compatible');
    });
    it('does not report a cached compatible firmware as live when stale or disconnected', () => {
        expect(firmwareInventoryState(status, false, 9000, 10000)).toBe('disconnected');
        expect(firmwareInventoryState(status, true, 1000, 10000)).toBe('stale');
    });
    it('distinguishes an unresolved handshake from a known mismatch', () => {
        expect(firmwareInventoryState({...status, firmware_protocol_version: 0, firmware_compatible: false}, true, 9000, 10000)).toBe('waiting');
        expect(firmwareInventoryState({...status, firmware_compatible: false}, true, 9000, 10000)).toBe('incompatible');
    });
});
