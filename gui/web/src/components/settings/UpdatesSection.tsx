import {Alert, Button, Card, Empty, Space, Spin, Tag, Typography} from 'antd';
import {ReloadOutlined} from '@ant-design/icons';
import {useTranslation} from 'react-i18next';
import {useNavigate} from 'react-router-dom';
import {browserBuild, useInstalledVersions} from '../../hooks/useInstalledVersions';
import {useFirmwareInventory} from '../../hooks/useFirmwareInventory';
import {browserBuildDiffers, imageVersion} from '../../utils/versions';
import type {ApiInstalledComponent} from '../../api/Api';
import './UpdatesSection.css';
import {UpdateChecks} from './UpdateChecks';
import {HostUpdaterPanel} from './HostUpdaterPanel';

const {Text} = Typography;
const order = ['robot', 'gui', 'gps', 'lidar', 'tfluna-front', 'tfluna-edge', 'mavros', 'ntrip', 'mqtt', 'watchtower', 'vesc'];

export function UpdatesSection({configuredModel}: {configuredModel?: string}) {
    const {t} = useTranslation();
    const navigate = useNavigate();
    const {data, loading, error, refresh} = useInstalledVersions();
    const firmware = useFirmwareInventory();
    const components = [...(data?.components ?? [])].sort((a, b) => order.indexOf(a.component ?? '') - order.indexOf(b.component ?? ''));
    const unknown = t('updates.unknown');
    const buildLabel = (revision?: string, version?: string) => [version, revision?.slice(0, 8)].filter(Boolean).join(' · ') || unknown;
    const versionDetails = JSON.stringify({inventory: data, browser: browserBuild, firmware: {version: firmware.data.firmware_version, protocol: firmware.data.firmware_protocol_version, state: firmware.state}, configured_model: configuredModel}, null, 2);
    const componentCard = (component: ApiInstalledComponent) => (
        <div className="installed-version-row" key={component.name} data-testid={`version-${component.component}`}>
            <div className="installed-version-heading">
                <div><Text strong>{t(`updates.components.${component.component}`, {defaultValue: component.name})}</Text><div><Text type="secondary">{component.name}</Text></div></div>
                <div className="installed-version-identity"><Text code>{buildLabel(component.revision, component.version || imageVersion(component.image ?? ''))}</Text><Tag>{t(`updates.containerStates.${component.state}`, {defaultValue: component.state || unknown})}</Tag></div>
            </div>
            <details><summary>{t('updates.details')}</summary><dl>
                <dt>{t('updates.image')}</dt><dd>{component.image || unknown}</dd>
                <dt>{t('updates.imageId')}</dt><dd>{component.image_id || unknown}</dd>
                <dt>{t('updates.digest')}</dt><dd>{component.digests?.length ? component.digests.join('\n') : unknown}</dd>
                <dt>{t('updates.revision')}</dt><dd>{component.revision || unknown}</dd>
                <dt>{t('updates.built')}</dt><dd>{component.built_at || unknown}</dd>
                <dt>{t('updates.architecture')}</dt><dd>{component.architecture || unknown}</dd>
            </dl>{!component.metadata_available && <Text type="warning">{t('updates.metadataUnavailable')}</Text>}</details>
        </div>
    );
    return (
        <div className="installed-versions" data-testid="installed-versions">
            <HostUpdaterPanel/>
            <UpdateChecks/>
            <div className="installed-version-toolbar"><Button icon={<ReloadOutlined/>} loading={loading} onClick={() => void refresh()}>{t('updates.refresh')}</Button>{data && <Typography.Paragraph style={{margin: 0}} copyable={{text: versionDetails, tooltips: [t('updates.copy'), t('updates.copied')]}}>{t('updates.copy')}</Typography.Paragraph>}</div>
            {error && <Alert type="warning" showIcon message={t('updates.fetchFailed')} description={data ? t('updates.showingPrevious') : undefined}/>}
            {data && !data.docker_available && <Alert type="warning" showIcon message={t('updates.dockerUnavailable')}/>}
            {data && browserBuildDiffers(browserBuild, data.server ?? {}) && <Alert type="warning" showIcon message={t('updates.browserStale')} action={<Button onClick={() => window.location.reload()}>{t('updates.reloadBrowser')}</Button>}/>}
            <Card title={t('updates.installedSoftware')} size="small">
                {loading && !data ? <Spin/> : components.length ? components.map(componentCard) : <Empty image={Empty.PRESENTED_IMAGE_SIMPLE} description={t('updates.noContainers')}/>}
            </Card>
            <Card title={t('updates.mainboard')} size="small">
                <Space direction="vertical" style={{width: '100%'}}>
                    <div className="installed-version-heading"><Text code>{firmware.data.firmware_version || unknown}</Text><Tag color={firmware.state === 'compatible' ? 'success' : firmware.state === 'incompatible' ? 'error' : 'default'}>{t(`updates.firmwareStates.${firmware.state}`)}</Tag></div>
                    <Text type="secondary">{t('updates.firmwareMeaning')}</Text>
                    <dl><dt>{t('updates.protocol')}</dt><dd>{firmware.data.firmware_protocol_version || unknown}</dd><dt>{t('updates.configuredModel')}</dt><dd>{configuredModel || unknown}</dd><dt>{t('updates.boardRevision')}</dt><dd>{t('updates.notReported')}</dd></dl>
                    {firmware.state === 'incompatible' && <Button danger onClick={() => void navigate('/onboarding?step=firmware&flash=1')}>{t('mowgliNextPage.firmwareFlashCta')}</Button>}
                </Space>
            </Card>
            <Card title={t('updates.webBuild')} size="small"><dl>
                <dt>{t('updates.server')}</dt><dd>{buildLabel(data?.server?.revision, data?.server?.version)}{data?.server?.modified && <Tag>{t('updates.modified')}</Tag>}</dd>
                <dt>{t('updates.browser')}</dt><dd>{buildLabel(browserBuild.revision, browserBuild.version)}</dd>
            </dl><Text type="secondary">{t('updates.browserMeaning')}</Text></Card>
            {data?.observed_at && <Text type="secondary">{t('updates.observed', {time: new Date(data.observed_at).toLocaleString()})}</Text>}
        </div>
    );
}
