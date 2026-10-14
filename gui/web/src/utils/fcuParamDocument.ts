/** Lossless Mission Planner parameter document; never a firmware builder YAML. */
export interface ParamEntry {name: string; value: number; start: number; end: number}
export interface ParamDocument {original: string; entries: Map<string, ParamEntry>; duplicates: Set<string>}
export function parseParamDocument(original: string): ParamDocument {
    if (original.length > 2_000_000 || original.includes('\0')) throw new Error('invalidFile');
    const entries = new Map<string, ParamEntry>(); const duplicates = new Set<string>();
    let offset = 0;
    for (const raw of original.match(/[^\r\n]*(?:\r\n|\n|\r|$)/g) ?? []) {
        if (!raw) continue;
        const line = raw.replace(/[\r\n]+$/, '');
        if (/^[\s\uFEFF]*(?:#|;|\/\/|$)/.test(line)) {offset += raw.length; continue;}
        const match = /^([\s\uFEFF]*[A-Z][A-Z0-9_]*)([ \t]*,[ \t]*|[ \t]+)([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)([ \t]*(?:(?:#|;|\/\/).*)?)$/.exec(line);
        if (!match || !Number.isFinite(Number(match[3]))) throw new Error('invalidFile');
        const name = match[1].trim().replace(/^\uFEFF/, '');
        if (entries.has(name)) duplicates.add(name);
        entries.set(name, {name, value: Number(match[3]), start: offset + match[1].length + match[2].length,
            end: offset + match[1].length + match[2].length + match[3].length});
        offset += raw.length;
    }
    if (!entries.size) throw new Error('invalidFile');
    return {original, entries, duplicates};
}
export function exportParamDocument(document: ParamDocument, corrections: ReadonlyMap<string, number>): string {
    const edits = [...corrections].map(([name, value]) => {
        const entry = document.entries.get(name);
        if (!entry || document.duplicates.has(name) || !Number.isFinite(value)) throw new Error('ambiguousParameter');
        return {...entry, replacement: String(value)};
    }).sort((a, b) => b.start - a.start);
    return edits.reduce((text, edit) => text.slice(0, edit.start) + edit.replacement + text.slice(edit.end), document.original);
}
