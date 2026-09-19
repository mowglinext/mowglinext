import {useEffect, useState} from 'react';
import {Skeleton, Tag, Typography} from 'antd';
import {useTranslation} from 'react-i18next';

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

type Props = {repository: string; installed?: string; available?: string};

/**
 * What the update brings, in the operator's terms: new features and fixes read
 * from the merged changes between the installed source and the candidate. The
 * image digests stay available below, collapsed — they say which bytes, never why.
 */
export function UpdateChangelog({repository, installed, available}: Props) {
    const {t} = useTranslation();
    // Keyed by the revision pair, so a result for a previous pair is never shown
    // for the current one and the effect never has to reset state itself.
    const [result, setResult] = useState<{key: string; log?: Changelog}>();
    const usable = changelogAvailable(installed, available);
    const key = `${repository}/${installed ?? ''}...${available ?? ''}`;

    useEffect(() => {
        if (!usable || !installed || !available) return;
        const abort = new AbortController();
        fetchChangelog(repository, installed, available, abort.signal)
            .then(log => setResult({key, log}))
            .catch(() => {if (!abort.signal.aborted) setResult({key});});
        return () => abort.abort();
    }, [usable, repository, installed, available, key]);

    const current = result?.key === key ? result : undefined;
    const log = current?.log;
    const failed = !!current && !current.log;

    if (!usable) return null;
    if (failed) return <Typography.Text type="secondary">{t('hostUpdater.changelog.unavailable')}</Typography.Text>;
    if (!log) return <Skeleton active paragraph={{rows: 3}} title={false}/>;

    const section = (title: string, entries: ChangelogEntry[]) => entries.length > 0 && <div>
        <Typography.Text strong>{title}</Typography.Text>
        <ul style={{margin: '4px 0 0', paddingInlineStart: 20}}>
            {entries.map((entry, index) => <li key={`${entry.pr ?? 'x'}-${index}`}>
                {entry.breaking && <Tag color="red">{t('hostUpdater.changelog.breaking')}</Tag>}
                {entry.scope && <Typography.Text type="secondary">{entry.scope}: </Typography.Text>}
                {entry.title}
                {entry.pr ? <> <a href={`https://github.com/${repository}/pull/${entry.pr}`} target="_blank" rel="noreferrer">#{entry.pr}</a></> : null}
            </li>)}
        </ul>
    </div>;

    return <div data-testid="update-changelog" style={{display: 'flex', flexDirection: 'column', gap: 12}}>
        <Typography.Text strong>{t('hostUpdater.changelog.title')}</Typography.Text>
        {section(t('hostUpdater.changelog.features'), log.features)}
        {section(t('hostUpdater.changelog.fixes'), log.fixes)}
        {log.features.length === 0 && log.fixes.length === 0 && <Typography.Text type="secondary">{t('hostUpdater.changelog.maintenanceOnly')}</Typography.Text>}
        <Typography.Text type="secondary">
            {log.other > 0 && <>{t('hostUpdater.changelog.other', {count: log.other})} </>}
            {log.truncated && <>{t('hostUpdater.changelog.truncated')} </>}
            <a href={log.url} target="_blank" rel="noreferrer">{t('hostUpdater.changelog.full')}</a>
        </Typography.Text>
    </div>;
}
