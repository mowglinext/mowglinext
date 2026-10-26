import {useCallback, useRef, useState} from "react";
import type {SetStateAction} from "react";

/** Partial settings PUTs: serialize saves and acknowledge only the edits sent.
 * Secret-specific withdrawal/clear rules remain with the calling section.
 */
export function useExternalSettingsDraft<T extends object>() {
    const [draft, setState] = useState<T>({} as T);
    const current = useRef(draft);
    const revisions = useRef(new Map<keyof T, number>());
    const acknowledged = useRef(new Map<keyof T, number>());
    const revision = useRef(0);
    const tail = useRef<Promise<unknown>>(Promise.resolve());

    const setDraft = useCallback((action: SetStateAction<T>) => {
        const previous = current.current;
        const next = typeof action === "function" ? action(previous) : action;
        const keys = new Set([...Object.keys(previous), ...Object.keys(next)] as (keyof T)[]);
        const stamp = ++revision.current;
        for (const key of keys) {
            if (Object.prototype.hasOwnProperty.call(previous, key) !== Object.prototype.hasOwnProperty.call(next, key)
                || !Object.is(previous[key], next[key])) revisions.current.set(key, stamp);
        }
        current.current = next;
        setState(next);
    }, []);

    const saveDraft = useCallback((persist: (submitted: T) => Promise<boolean>): Promise<boolean> => {
        const snapshot = {...current.current};
        const sentRevisions = new Map(revisions.current);
        const task = tail.current.then(async () => {
            const submitted = {...snapshot};
            for (const key of Object.keys(submitted) as (keyof T)[]) {
                // A duplicate Save need not resend an already acknowledged
                // revision. Revert can also withdraw an edit not yet sent.
                if (acknowledged.current.has(key) && acknowledged.current.get(key) === sentRevisions.get(key)
                    || !Object.prototype.hasOwnProperty.call(current.current, key)
                    && revisions.current.get(key) !== sentRevisions.get(key)) delete submitted[key];
            }
            if (Object.keys(submitted).length === 0) return true;
            if (!await persist(submitted)) return false;
            const remaining = {...current.current};
            for (const key of Object.keys(submitted) as (keyof T)[]) {
                const sentRevision = sentRevisions.get(key);
                if (sentRevision !== undefined) acknowledged.current.set(key, sentRevision);
                if (sentRevisions.get(key) === revisions.current.get(key)) delete remaining[key];
            }
            setDraft(remaining);
            return true;
        });
        // A rejected request must not poison the order of later retries.
        tail.current = task.then(() => undefined, () => undefined);
        return task;
    }, [setDraft]);

    return {draft, setDraft, saveDraft};
}
