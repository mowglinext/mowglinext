import {useCallback, useEffect, useState} from 'react';
import {Alert, Button, Form, Select, Space, Tag, Typography} from 'antd';
import {useTranslation} from 'react-i18next';

export interface UpdaterPullRequest {
    number: number;
    title: string;
    branch: string;
    author: string;
    url: string;
    updated_at: string;
    from_fork: boolean;
    labeled: boolean;
    deployment?: string;
    installable: boolean;
}

interface UpdaterPullRequests {repository: string; label: string; pull_requests: UpdaterPullRequest[]}

interface PullRequestPickerProps {
    repository: string;
    branch: string;
    disabled?: boolean;
    /** Called with the PR's head branch: the updater installs it as a custom branch. */
    onSelect: (branch: string) => void;
}

/**
 * Advanced → Update settings, custom track: pick an open pull request instead of
 * typing its branch. A PR is installable once CI has published a complete
 * deployment of it (label `deploy-preview`, deployment-release.yml).
 */
export function PullRequestPicker({repository, branch, disabled, onSelect}: PullRequestPickerProps) {
    const {t} = useTranslation();
    const [data, setData] = useState<UpdaterPullRequests>();
    const [error, setError] = useState<string>();
    const [loading, setLoading] = useState(false);

    const load = useCallback(async () => {
        setLoading(true);
        setError(undefined);
        try {
            const response = await fetch(`/api/system/updater/pull-requests?repository=${encodeURIComponent(repository)}`, {cache: 'no-store'});
            const body = await response.json() as UpdaterPullRequests & {error?: string};
            if (!response.ok) throw new Error(body.error || `HTTP ${response.status}`);
            setData(body);
        } catch (e) {
            setError(e instanceof Error ? e.message : String(e));
        } finally {
            setLoading(false);
        }
    }, [repository]);

    useEffect(() => {void load();}, [load]);

    const pulls = data?.pull_requests ?? [];
    const selected = pulls.find(p => p.branch === branch && !p.from_fork);
    const status = (p: UpdaterPullRequest) => {
        if (p.installable) return t('hostUpdater.pullRequests.installable');
        if (p.from_fork) return t('hostUpdater.pullRequests.fork');
        if (p.labeled) return t('hostUpdater.pullRequests.building');
        return t('hostUpdater.pullRequests.notBuilt', {label: data?.label ?? 'deploy-preview'});
    };

    return <Form.Item label={t('hostUpdater.pullRequests.label')} help={t('hostUpdater.pullRequests.help', {label: data?.label ?? 'deploy-preview'})}>
        <Space direction="vertical" style={{width: '100%'}}>
            <Space.Compact style={{width: '100%'}}>
                <Select
                    aria-label={t('hostUpdater.pullRequests.label')}
                    data-testid="pull-request-picker"
                    showSearch
                    optionFilterProp="search"
                    loading={loading}
                    disabled={disabled}
                    placeholder={t('hostUpdater.pullRequests.placeholder')}
                    value={selected ? selected.number : undefined}
                    options={pulls.map(p => ({
                        value: p.number,
                        disabled: !p.installable,
                        search: `#${p.number} ${p.title} ${p.branch} ${p.author}`,
                        label: <span>
                            #{p.number} {p.title}{' '}
                            <Tag color={p.installable ? 'green' : 'default'}>{status(p)}</Tag>
                            <Typography.Text type="secondary">{p.branch}</Typography.Text>
                        </span>,
                    }))}
                    onChange={(number: number) => {
                        const p = pulls.find(candidate => candidate.number === number);
                        if (p) onSelect(p.branch);
                    }}
                />
                <Button onClick={() => void load()} loading={loading}>{t('hostUpdater.pullRequests.refresh')}</Button>
            </Space.Compact>
            {selected && <Typography.Text type="secondary">
                <a href={selected.url} target="_blank" rel="noreferrer">#{selected.number}</a>
                {selected.deployment && <> · {selected.deployment}</>}
            </Typography.Text>}
            {error && <Alert type="warning" showIcon message={t('hostUpdater.pullRequests.unavailable')} description={error}/>}
        </Space>
    </Form.Item>;
}
