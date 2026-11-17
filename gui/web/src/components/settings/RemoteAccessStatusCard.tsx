import {Alert, Button, Card, Popconfirm, Space, Tag, Typography} from "antd";
import {LinkOutlined, LogoutOutlined, ReloadOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import {remoteAccessVerdict, type RemoteAccessStatus, type RemoteAccessVerdict} from "../../types/remoteAccess.ts";

const {Text, Paragraph, Link} = Typography;

const VERDICT_COLOR: Record<RemoteAccessVerdict, string> = {
    off: "default",
    working: "processing",
    needsLogin: "warning",
    connected: "success",
    error: "error",
    offline: "warning",
};

interface RemoteAccessStatusCardProps {
    status: RemoteAccessStatus | null;
    fetchError: string | null;
    onRefresh: () => void;
    onRetry: () => void;
    onLogout: () => void;
    busy: boolean;
}

/**
 * Lifecycle + tailnet state: a login link while the node waits for the
 * operator, then the URLs that reach the GUI once connected.
 */
export function RemoteAccessStatusCard({status, fetchError, onRefresh, onRetry, onLogout, busy}: RemoteAccessStatusCardProps) {
    const {t, i18n} = useTranslation();
    const verdict = remoteAccessVerdict(status);
    const phaseLabel = status ? t(`settingsRemoteAccess.phase.${status.phase}`) : t("settingsRemoteAccess.phase.unknown");
    const recoveryState = status?.recoveryState ?? (status?.enabled ? "monitoring" : "disabled");
    const formatDate = (value: string) => {
        const date = new Date(value);
        return Number.isNaN(date.getTime()) ? value : new Intl.DateTimeFormat(
            i18n.resolvedLanguage?.startsWith("fr") ? "fr-FR" : "en-US",
            {dateStyle: "medium", timeStyle: "short"},
        ).format(date);
    };

    return (
        <Card
            size="small"
            title={t("settingsRemoteAccess.status")}
            style={{marginBottom: 16}}
            extra={<Button size="small" icon={<ReloadOutlined/>} onClick={onRefresh}>{t("settingsRemoteAccess.refresh")}</Button>}
        >
            <Space wrap style={{marginBottom: 8}}>
                <Tag color={VERDICT_COLOR[verdict]} data-testid="remote-access-verdict">
                    {t(`settingsRemoteAccess.verdict.${verdict}`)}
                </Tag>
                <Text type="secondary" style={{fontSize: 12}}>{phaseLabel}</Text>
                {status?.backendState && (
                    <Text type="secondary" style={{fontSize: 12}}>
                        {t("settingsRemoteAccess.backendState", {state: status.backendState})}
                    </Text>
                )}
                {status?.version && <Text type="secondary" style={{fontSize: 12}}>Tailscale {status.version}</Text>}
            </Space>

            {fetchError && <Alert type="warning" showIcon style={{marginBottom: 8}} message={fetchError}/>}

            {status?.enabled && (
                <div data-testid="remote-access-recovery" style={{marginBottom: 8}}>
                    <Paragraph type="secondary" style={{margin: "0 0 4px"}}>
                        {t(`settingsRemoteAccess.recovery.${recoveryState}`, {
                            count: status.recoveryAttempts ?? 0,
                        })}
                    </Paragraph>
                    {status.lastSuccessAt && (
                        <Text type="secondary" style={{fontSize: 12, display: "block"}}>
                            {t("settingsRemoteAccess.lastSuccess", {value: formatDate(status.lastSuccessAt)})}
                        </Text>
                    )}
                    {status.nextRecoveryAt && (recoveryState === "suspect" || recoveryState === "cooldown") && (
                        <Text type="secondary" style={{fontSize: 12, display: "block"}}>
                            {t("settingsRemoteAccess.nextRecovery", {value: formatDate(status.nextRecoveryAt)})}
                        </Text>
                    )}
                </div>
            )}

            {status?.enabled && status.hostNetworkOnline === false && (
                <Alert
                    type="warning"
                    showIcon
                    style={{marginBottom: 8}}
                    message={t("settingsRemoteAccess.networkUnavailable")}
                />
            )}

            {status?.error && (
                <Alert
                    type="error"
                    showIcon
                    style={{marginBottom: 8}}
                    message={t("settingsRemoteAccess.errorTitle")}
                    description={status.error}
                    action={<Button size="small" onClick={onRetry} loading={busy}>{t("settingsRemoteAccess.retry")}</Button>}
                />
            )}

            {verdict === "needsLogin" && status?.loginUrl && (
                <Alert
                    type="warning"
                    showIcon
                    style={{marginBottom: 8}}
                    message={t("settingsRemoteAccess.needsLoginTitle")}
                    description={
                        <Space direction="vertical" size={4}>
                            <span>{t("settingsRemoteAccess.needsLoginBody")}</span>
                            <Button
                                type="primary"
                                icon={<LinkOutlined/>}
                                href={status.loginUrl}
                                target="_blank"
                                rel="noopener noreferrer"
                                data-testid="remote-access-login-link"
                            >
                                {t("settingsRemoteAccess.openLogin")}
                            </Button>
                            <Text copyable={{text: status.loginUrl}} style={{fontSize: 12, wordBreak: "break-all"}}>{status.loginUrl}</Text>
                        </Space>
                    }
                />
            )}

            {verdict === "working" && (
                <Paragraph type="secondary" style={{margin: "0 0 8px"}}>{t("settingsRemoteAccess.workingHint")}</Paragraph>
            )}

            {verdict === "offline" && (
                <Alert
                    type="warning"
                    showIcon
                    style={{marginBottom: 8}}
                    message={t("settingsRemoteAccess.nodeOffline")}
                />
            )}

            {recoveryState === "operator_action" && status?.backendState === "NeedsMachineAuth" && (
                <Alert
                    type="warning"
                    showIcon
                    style={{marginBottom: 8}}
                    message={t("settingsRemoteAccess.machineAuthRequired")}
                />
            )}

            {recoveryState === "degraded" && !status?.error && (
                <Alert
                    type="error"
                    showIcon
                    style={{marginBottom: 8}}
                    message={t("settingsRemoteAccess.degradedTitle")}
                    description={status?.recoveryError}
                    action={<Button size="small" onClick={onRetry} loading={busy}>{t("settingsRemoteAccess.retry")}</Button>}
                />
            )}

            {verdict === "connected" && status && (
                <div data-testid="remote-access-urls">
                    <Paragraph style={{margin: "0 0 4px"}}>{t("settingsRemoteAccess.reachableAt")}</Paragraph>
                    <Space direction="vertical" size={2} style={{marginBottom: 8}}>
                        {status.httpsUrl && (
                            <Link href={status.httpsUrl} target="_blank" rel="noopener noreferrer" strong copyable>
                                {status.httpsUrl}
                            </Link>
                        )}
                        {status.httpUrls.map((url) => (
                            <Link key={url} href={url} target="_blank" rel="noopener noreferrer" copyable>{url}</Link>
                        ))}
                    </Space>
                    {!status.httpsUrl && (
                        <Paragraph type="secondary" style={{fontSize: 12, margin: "0 0 8px"}}>
                            {t("settingsRemoteAccess.noHttpsHint")}
                        </Paragraph>
                    )}
                    {!status.magicDnsEnabled && (
                        <Paragraph type="secondary" style={{fontSize: 12, margin: "0 0 8px"}}>
                            {t("settingsRemoteAccess.noMagicDnsHint")}
                        </Paragraph>
                    )}
                </div>
            )}

            {status && status.health.length > 0 && (
                <Alert
                    type="warning"
                    showIcon
                    style={{marginBottom: 8}}
                    message={t("settingsRemoteAccess.healthTitle")}
                    description={<ul style={{margin: 0, paddingLeft: 18}}>{status.health.map((h) => <li key={h}>{h}</li>)}</ul>}
                />
            )}

            {(verdict === "connected" || verdict === "needsLogin") && (
                <Popconfirm
                    title={t("settingsRemoteAccess.logoutConfirmTitle")}
                    description={t("settingsRemoteAccess.logoutConfirmBody")}
                    okText={t("settingsRemoteAccess.logout")}
                    okButtonProps={{danger: true}}
                    onConfirm={onLogout}
                >
                    <Button danger icon={<LogoutOutlined/>} loading={busy}>{t("settingsRemoteAccess.logout")}</Button>
                </Popconfirm>
            )}
        </Card>
    );
}
