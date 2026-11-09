import {beforeEach, describe, expect, it, vi} from 'vitest';
import {listParamOriginals, saveParamOriginal, type ParamBackup} from './fcuParamStorage';
import {unknownParamIdentity} from './fcuParamIdentity';
const storage = vi.hoisted(() => new Map<string, unknown>());
vi.mock('localforage', () => ({default: {createInstance: vi.fn(() => ({
    keys: () => Promise.resolve([...storage.keys()]), getItem: (id: string) => Promise.resolve(storage.get(id) ?? null),
    setItem: (id: string, value: unknown) => {storage.set(id, value); return Promise.resolve(value);},
}))}}));
beforeEach(() => storage.clear());
describe('immutable parameter originals', () => {
    it('retains separate originals and refuses overwrite', async () => {
        const backup: ParamBackup = {id: 'a', filename: 'robot.param', importedAt: '2026-10-11', original: 'A,1\r\n', identity: unknownParamIdentity()};
        await saveParamOriginal(backup);
        await expect(saveParamOriginal({...backup, original: 'A,2'})).rejects.toThrow('backupCollision');
        expect((await listParamOriginals())[0].original).toBe('A,1\r\n');
        backup.identity.board = 'changed working identity';
        expect((await listParamOriginals())[0].identity.board).toBe('');
        const other = {...backup, id: 'concurrent', original: 'A,3'};
        const results = await Promise.allSettled([saveParamOriginal(other), saveParamOriginal({...other, original: 'A,4'})]);
        expect(results.map(result => result.status)).toEqual(['fulfilled', 'rejected']);
        expect((await listParamOriginals()).find(row => row.id === 'concurrent')?.original).toBe('A,3');
    });
});
