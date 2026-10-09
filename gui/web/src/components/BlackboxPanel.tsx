import {Alert, App, Button, Card, Col, Form, InputNumber, Row, Space, Spin, Switch, Table, Tag, Typography} from "antd";
import {DeleteOutlined, DownloadOutlined, SaveOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import {BlackboxConfig, BlackboxRecording, useBlackbox} from "../hooks/useBlackbox";
import {useTimeFormat} from "../hooks/useTimeFormat";

const MiB = 1024 * 1024;

export function BlackboxPanel() {
    const {t} = useTranslation();
    const {modal, message} = App.useApp();
    const box = useBlackbox();
    const {formatAbsolute} = useTimeFormat();
    const run = async (action: () => Promise<void>) => {
        try { await action(); } catch (e) { void message.error(e instanceof Error ? e.message : t("blackbox.requestFailed")); }
    };
    const status = box.status;
    const capturing = status?.phase === "capturing" || status?.phase === "writing";
    const remove = (recording: BlackboxRecording) => modal.confirm({
        title: t("blackbox.deleteTitle"), content: recording.name,
        okText: t("blackbox.delete"), cancelText: t("blackbox.cancel"), okButtonProps: {danger: true},
        onOk: () => box.remove(recording.name),
    });
    return <Card size="small" title={t("blackbox.title")}>
        <Space direction="vertical" size="middle" style={{width: "100%"}}>
            <Typography.Paragraph style={{margin: 0}}>{t("blackbox.description")}</Typography.Paragraph>
            {box.error && <Alert type="error" showIcon message={box.error}/>}
            {!status && !box.error && <Spin aria-label={t("blackbox.loading")}/>}
            {status && <>
                <Space wrap>
                    <Tag color={status.config.enabled ? "processing" : "default"}>{t(`blackbox.phase.${status.phase}`, {defaultValue: status.phase})}</Tag>
                    <Typography.Text>{t("blackbox.history", {seconds: status.buffered_seconds.toFixed(1)})}</Typography.Text>
                    <Typography.Text type="secondary">{t("blackbox.resources", {
                        used: (status.buffered_bytes / MiB).toFixed(1), limit: (status.effective_memory_bytes / MiB).toFixed(1), dropped: status.dropped_messages,
                    })}</Typography.Text>
                </Space>
                {(status.warning || status.last_error) && <Alert type="warning" showIcon message={status.warning || status.last_error}/>}
                {status.memory_pressure && <Alert type="warning" showIcon message={t("blackbox.pressure")}/>}
                {status.config.enabled && !status.topics.some(topic => topic.last_received_at) && <Alert type="info" showIcon message={t("blackbox.waiting")}/>}
                <BlackboxSettings key={JSON.stringify(status.config)} config={status.config} busy={box.busy || capturing} configure={config => run(() => box.configure(config))}/>
                <Button type="primary" icon={<SaveOutlined/>} loading={box.busy} disabled={!status.config.enabled || capturing || box.busy} onClick={() => void run(box.save)}>
                    {t("blackbox.save")}
                </Button>
                <Typography.Text type="secondary">{t("blackbox.privacy")}</Typography.Text>
                <Table<BlackboxRecording> size="small" rowKey="name" dataSource={status.recordings} pagination={{pageSize: 10}} scroll={{x: 660}}
                    locale={{emptyText: t("blackbox.empty")}}
                    columns={[
                        {title: t("blackbox.trigger"), key: "trigger", render: (_, row) => <Space direction="vertical" size={0}>
                            <Typography.Text>{row.reasons.join(", ")}</Typography.Text>
                            <Typography.Text type="secondary">{formatAbsolute(row.triggered_at)}</Typography.Text>
                            {row.interrupted && <Tag color="warning">{t("blackbox.interrupted")}</Tag>}
                        </Space>},
                        {title: t("blackbox.duration"), key: "duration", render: (_, row) => <Space direction="vertical" size={0}>
                            <span>{t("blackbox.coverage", {pre: row.actual_pre_seconds.toFixed(1), post: row.actual_post_seconds.toFixed(1)})}</span>
                            {row.capture_dropped_messages > 0 && <Typography.Text type="warning">{t("blackbox.dropped", {count: row.capture_dropped_messages})}</Typography.Text>}
                        </Space>},
                        {title: t("blackbox.size"), key: "size", render: (_, row) => `${(row.size / MiB).toFixed(2)} MiB`},
                        {title: t("blackbox.actions"), key: "actions", render: (_, row) => <Space>
                            <Button size="small" icon={<DownloadOutlined/>} href={box.downloadUrl(row.name)}>{t("blackbox.download")}</Button>
                            <Button size="small" danger icon={<DeleteOutlined/>} aria-label={t("blackbox.deleteNamed", {name: row.name})} disabled={box.busy} onClick={() => { remove(row); }}/>
                        </Space>},
                    ]}/>
            </>}
        </Space>
    </Card>;
}

function BlackboxSettings({config, busy, configure}: {config: BlackboxConfig; busy: boolean; configure: (config: BlackboxConfig) => Promise<void>}) {
    const {t} = useTranslation();
    return <Form<{enabled: boolean; pre: number; post: number; memory: number; count: number; disk: number}> layout="vertical" initialValues={{enabled: config.enabled, pre: config.pre_seconds, post: config.post_seconds,
        memory: config.memory_bytes / MiB, count: config.max_snapshots, disk: config.max_disk_bytes / MiB}}
        onFinish={values => void configure({...config, enabled: values.enabled, pre_seconds: values.pre, post_seconds: values.post,
            memory_bytes: Math.round(values.memory * MiB), max_snapshots: values.count, max_disk_bytes: Math.round(values.disk * MiB)})}>
        <Row gutter={[16, 0]}>
            <Col xs={24} sm={8}><Form.Item label={t("blackbox.enabled")} name="enabled" valuePropName="checked"><Switch disabled={busy}/></Form.Item></Col>
            {([
                ["pre", "pre", 1, 300], ["post", "post", 1, 60], ["memory", "memory", 4, 64],
                ["count", "count", 1, 200], ["disk", "disk", 4, 4096],
            ] as const).map(([name, label, min, max]) => <Col key={name} xs={12} sm={8}>
                <Form.Item label={t(`blackbox.${label}`)} name={name} rules={[{required: true, message: t("blackbox.required")}]}>
                    <InputNumber min={min} max={max} precision={0} disabled={busy} style={{width: "100%"}}/>
                </Form.Item>
            </Col>)}
        </Row>
        <Button htmlType="submit" disabled={busy}>{t("blackbox.apply")}</Button>
    </Form>;
}
