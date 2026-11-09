export interface ChangelogEntry {scope?: string; title: string; pr?: number; breaking?: boolean}
export interface Changelog {features: ChangelogEntry[]; fixes: ChangelogEntry[]; other: number; total: number; truncated: boolean; url: string}

const REVISION = /^[a-f0-9]{40}$/;

/** Both ends must be real commits; a custom image mix or an unknown install has no source range. */
export const changelogAvailable = (installed?: string, available?: string) =>
    !!installed && !!available && installed !== available && REVISION.test(installed) && REVISION.test(available);

export async function fetchChangelog(repository: string, installed: string, available: string, signal?: AbortSignal): Promise<Changelog> {
    const query = new URLSearchParams({repository, installed, available});
    const response = await fetch(`/api/system/updates/changelog?${query.toString()}`, {cache: 'no-store', signal});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    return await response.json() as Changelog;
}
