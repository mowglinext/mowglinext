import {Space, Tag, Typography} from 'antd';
import {useTranslation} from 'react-i18next';
import type {ROSHealth} from '../../hooks/useDiagnosticsSnapshot';

/** HTTP responsiveness and ROS transport evidence have separate failure domains. */
export function TransportHealthStatus({fresh, ros}: {fresh: boolean; ros?: ROSHealth}) {
    const {t} = useTranslation();
    const transport = fresh ? ros?.transport : undefined;
    const rosState = !transport ? 'unknown' : !transport.connected ? 'disconnected'
        : transport.heartbeat_ok ? 'healthy' : 'checking';
    const rosLabel = rosState === 'unknown' ? t('telemetry.statusUnknown')
        : t(`diagnosticsPage.transport${rosState[0].toUpperCase()}${rosState.slice(1)}`);
    return <div style={{marginBottom: 12}}>
        <Space wrap>
            <Tag data-testid="diagnostic-gui-http" color={fresh ? 'success' : 'default'}>
                {t('diagnosticsPage.guiHttp')}: {fresh ? t('diagnosticsPage.guiResponding') : t('telemetry.statusUnknown')}
            </Tag>
            <Tag data-testid="diagnostic-ros-transport" color={rosState === 'healthy' ? 'success'
                : rosState === 'disconnected' || rosState === 'checking' ? 'warning' : 'default'}>
                {t('diagnosticsPage.rosTransport')}: {rosLabel}
            </Tag>
        </Space>
        <Typography.Paragraph type="secondary" style={{fontSize: 12, marginBottom: 0}}>
            {t('diagnosticsPage.transportHint')}
        </Typography.Paragraph>
    </div>;
}
