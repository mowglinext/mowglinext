import localforage from 'localforage';
import type {ParamIdentity} from './fcuParamIdentity';
export interface ParamBackup {id: string; filename: string; importedAt: string; original: string; identity: ParamIdentity}
// A separate database/store; neither YAML backups nor robot settings are touched.
const originals = () => localforage.createInstance({name: 'mowglinext-fcu-params', storeName: 'immutable_originals_v1'});
let writes: Promise<void> = Promise.resolve();
async function insertOriginal(backup: ParamBackup): Promise<void> {
    const store = originals();
    if (await store.getItem(backup.id) !== null) throw new Error('backupCollision');
    await store.setItem(backup.id, {...backup, identity: {...backup.identity}});
}
/** Serialize insert-only requests in this app; imported UUIDs isolate other tabs. */
export function saveParamOriginal(backup: ParamBackup): Promise<void> {
    const snapshot = {...backup, identity: {...backup.identity}};
    const operation = writes.then(() => insertOriginal(snapshot));
    writes = operation.catch(() => undefined);
    return operation;
}
export async function listParamOriginals(): Promise<ParamBackup[]> {
    const store = originals();
    const rows = await Promise.all((await store.keys()).map(key => store.getItem<ParamBackup>(key)));
    return rows.filter((row): row is ParamBackup => row !== null).sort((a, b) => b.importedAt.localeCompare(a.importedAt));
}
