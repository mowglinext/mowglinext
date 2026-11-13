import {describe, expect, it} from 'vitest';
import {readBundle} from './api';

describe('bundle input identity', () => {
    it('preserves duplicate keys for strict host rejection instead of silently changing the bundle', async () => {
        const original = '{"manifest":{"id":"first","id":"second"}}';
        expect(await readBundle(new File([original], 'ambiguous.json'))).toBe(original);
    });
});
