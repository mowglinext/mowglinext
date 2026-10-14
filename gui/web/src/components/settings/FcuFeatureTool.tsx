import {useEffect, useMemo, useRef, useState} from "react";
import localforage from "localforage";
import {
    Alert,
    Button,
    Card,
    Descriptions,
    Divider,
    Empty,
    List,
    Space,
    Tag,
    Typography,
} from "antd";
import {
    DeleteOutlined,
    DownloadOutlined,
    ExportOutlined,
    PlusOutlined,
    ReloadOutlined,
    UploadOutlined,
} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import {
    exportFcuYaml,
    parseFcuYaml,
    requiredFcuFeatures,
    type FcuYamlConfiguration,
} from "../../utils/fcuFeatureYaml.ts";

const {Paragraph, Text} = Typography;
const BACKUP_STORAGE_KEY = "mowglinext.fcu-yaml-backups.v1";

type FcuBackup = {
    id: string;
    createdAt: string;
    filename: string;
    yaml: string;
};

const unique = (values: readonly string[]) => [...new Set(values)];

const downloadText = (filename: string, contents: string) => {
    const url = URL.createObjectURL(new Blob([contents], {type: "application/yaml;charset=utf-8"}));
    const anchor = document.createElement("a");
    anchor.href = url;
    anchor.download = filename;
    anchor.click();
    URL.revokeObjectURL(url);
};

const outputFilename = (filename: string) => {
    const base = filename.replace(/\.(yaml|yml)$/i, "") || "ardupilot-features";
    return `${base}-mowglinext.yaml`;
};

export function FcuFeatureTool() {
    const {t} = useTranslation();
    const inputRef = useRef<HTMLInputElement>(null);
    const [configuration, setConfiguration] = useState<FcuYamlConfiguration | null>(null);
    const [filename, setFilename] = useState("ardupilot-features.yaml");
    const [features, setFeatures] = useState<string[]>([]);
    const [originalFeatures, setOriginalFeatures] = useState<string[]>([]);
    const [backups, setBackups] = useState<FcuBackup[]>([]);
    const [error, setError] = useState<string | null>(null);

    useEffect(() => {
        let active = true;
        void localforage.getItem<FcuBackup[]>(BACKUP_STORAGE_KEY).then((stored) => {
            if (active && Array.isArray(stored)) setBackups(stored);
        }).catch(() => undefined);
        return () => { active = false; };
    }, []);

    const persistBackup = async (backup: FcuBackup) => {
        const stored = await localforage.getItem<FcuBackup[]>(BACKUP_STORAGE_KEY).catch(() => null);
        const next = [backup, ...(Array.isArray(stored) ? stored : [])];
        await localforage.setItem(BACKUP_STORAGE_KEY, next);
        setBackups(next);
    };

    const loadWorkingCopy = (yaml: string, nextFilename: string) => {
        const parsed = parseFcuYaml(yaml);
        setConfiguration(parsed);
        setFeatures(parsed.features);
        setOriginalFeatures(parsed.features);
        setFilename(nextFilename);
        setError(null);
    };

    const importFile = async (file: File) => {
        try {
            const yaml = await file.text();
            const parsed = parseFcuYaml(yaml);
            const backup: FcuBackup = {
                id: `${Date.now()}-${crypto.randomUUID?.() ?? Math.random().toString(36).slice(2)}`,
                createdAt: new Date().toISOString(),
                filename: file.name,
                yaml,
            };
            await persistBackup(backup);
            setConfiguration(parsed);
            setFeatures(parsed.features);
            setOriginalFeatures(parsed.features);
            setFilename(file.name);
            setError(null);
        } catch (caught) {
            setError(caught instanceof Error ? caught.message : t("settingsFcu.invalidYaml"));
        }
    };

    const required = useMemo(() => new Set(requiredFcuFeatures), []);
    const current = useMemo(() => new Set(features), [features]);
    const original = useMemo(() => new Set(originalFeatures), [originalFeatures]);
    const missing = requiredFcuFeatures.filter((feature) => !current.has(feature));
    const extras = features.filter((feature) => !required.has(feature));
    const removed = originalFeatures.filter((feature) => !current.has(feature));
    const removedExtras = removed.filter((feature) => !required.has(feature));
    const added = features.filter((feature) => !original.has(feature));
    const rows = unique([...requiredFcuFeatures, ...extras, ...removedExtras]);

    const addFeature = (feature: string) => setFeatures((previous) =>
        previous.includes(feature) ? previous : [...previous, feature]);
    const removeFeature = (feature: string) => setFeatures((previous) =>
        previous.filter((candidate) => candidate !== feature));
    const addAllMissing = () => setFeatures((previous) => unique([...previous, ...requiredFcuFeatures]));

    return <Space direction="vertical" size={16} style={{width: "100%"}} data-testid="fcu-feature-tool">
        <Alert type="info" showIcon message={t("settingsFcu.scopeTitle")} description={t("settingsFcu.scopeDescription")}/>

        <Card size="small" title={t("settingsFcu.importTitle")}>
            <Space direction="vertical" size={8} style={{width: "100%"}}>
                <Paragraph type="secondary" style={{margin: 0}}>{t("settingsFcu.importDescription")}</Paragraph>
                <input
                    ref={inputRef}
                    type="file"
                    accept=".yaml,.yml,application/yaml,text/yaml,text/x-yaml"
                    hidden
                    aria-label={t("settingsFcu.importButton")}
                    onChange={(event) => {
                        const file = event.target.files?.[0];
                        if (file) void importFile(file);
                        event.target.value = "";
                    }}
                />
                <Button aria-label={t("settingsFcu.importButton")} icon={<UploadOutlined/>} onClick={() => inputRef.current?.click()}>{t("settingsFcu.importButton")}</Button>
                {error && (
                    <Alert type="error" showIcon message={t("settingsFcu.importError")} description={error}/>
                )}
            </Space>
        </Card>

        {configuration && <>
            <Card size="small" title={t("settingsFcu.informationTitle")}>
                <Descriptions size="small" column={{xs: 1, sm: 3}}>
                    <Descriptions.Item label={t("settingsFcu.vehicle")}>{configuration.info.vehicle ?? "—"}</Descriptions.Item>
                    <Descriptions.Item label={t("settingsFcu.board")}>{configuration.info.board ?? "—"}</Descriptions.Item>
                    <Descriptions.Item label={t("settingsFcu.version")}>{configuration.info.version ?? "—"}</Descriptions.Item>
                </Descriptions>
                <Text type="secondary">{t("settingsFcu.informationOnly")}</Text>
            </Card>

            <Card
                size="small"
                title={t("settingsFcu.compatibilityTitle")}
                extra={missing.length > 0 && <Button type="primary" aria-label={t("settingsFcu.addAll")} icon={<PlusOutlined/>} onClick={addAllMissing}>
                    {t("settingsFcu.addAll")}
                </Button>}
            >
                <Alert
                    type={missing.length === 0 ? "success" : "error"}
                    showIcon
                    message={missing.length === 0
                        ? t("settingsFcu.minimumSatisfied")
                        : t("settingsFcu.minimumMissing", {count: missing.length})}
                    style={{marginBottom: 12}}
                />
                {removed.length > 0 && <Alert
                    type="warning"
                    showIcon
                    message={t("settingsFcu.removalWarningTitle")}
                    description={t("settingsFcu.removalWarningDescription")}
                    style={{marginBottom: 12}}
                />}
                <List
                    size="small"
                    dataSource={rows}
                    renderItem={(feature) => {
                        const isRequired = required.has(feature);
                        const isPresent = current.has(feature);
                        const wasRemoved = original.has(feature) && !isPresent;
                        const status = isRequired
                            ? (isPresent ? "requiredPresent" : "requiredMissing")
                            : (wasRemoved ? "removed" : "extra");
                        return <List.Item
                            actions={isPresent
                                ? [<Button key="remove" type="text" danger icon={<DeleteOutlined/>}
                                    aria-label={t("settingsFcu.removeFeature", {feature})}
                                    onClick={() => removeFeature(feature)}/>]
                                : [<Button key="add" type="text" icon={<PlusOutlined/>}
                                    aria-label={t("settingsFcu.addFeature", {feature})}
                                    onClick={() => addFeature(feature)}/>]}
                        >
                            <Text code>{feature}</Text>
                            <Tag color={status === "requiredPresent" ? "success" : status === "requiredMissing" ? "error" : status === "removed" ? "warning" : "default"}>
                                {t(`settingsFcu.status.${status}`)}
                            </Tag>
                        </List.Item>;
                    }}
                />
                {(added.length > 0 || removed.length > 0) && <>
                    <Divider orientation="left" plain>{t("settingsFcu.sessionDiff")}</Divider>
                    <Space direction="vertical" size={2}>
                        {added.map((feature) => <Text key={`+${feature}`} type="success">+ {feature} — {t("settingsFcu.addedDiff")}</Text>)}
                        {removed.map((feature) => <Text key={`-${feature}`} type="warning">− {feature} — {t("settingsFcu.removedDiff")}</Text>)}
                    </Space>
                </>}
            </Card>

            <Card size="small" title={t("settingsFcu.exportTitle")}>
                <Space wrap>
                    <Button aria-label={t("settingsFcu.downloadFinal")} icon={<DownloadOutlined/>} onClick={() => downloadText(
                        outputFilename(filename), exportFcuYaml(configuration, features),
                    )}>{t("settingsFcu.downloadFinal")}</Button>
                    <Button aria-label={t("settingsFcu.openBuilder")} href="https://custom.ardupilot.org/" target="_blank" rel="noopener noreferrer" icon={<ExportOutlined/>}>
                        {t("settingsFcu.openBuilder")}
                    </Button>
                </Space>
                <Paragraph type="secondary" style={{margin: "8px 0 0"}}>{t("settingsFcu.builderDescription")}</Paragraph>
            </Card>
        </>}

        <Card size="small" title={t("settingsFcu.backupsTitle")}>
            {backups.length === 0
                ? <Empty image={Empty.PRESENTED_IMAGE_SIMPLE} description={t("settingsFcu.noBackups")}/>
                : <List size="small" dataSource={backups} renderItem={(backup) => <List.Item actions={[
                    <Button key="download" type="link" aria-label={t("settingsFcu.downloadBackup")} icon={<DownloadOutlined/>} onClick={() => downloadText(backup.filename, backup.yaml)}>
                        {t("settingsFcu.downloadBackup")}
                    </Button>,
                    <Button key="restore" type="link" aria-label={t("settingsFcu.restoreBackup")} icon={<ReloadOutlined/>} onClick={() => {
                        try { loadWorkingCopy(backup.yaml, backup.filename); } catch (caught) {
                            setError(caught instanceof Error ? caught.message : t("settingsFcu.invalidYaml"));
                        }
                    }}>{t("settingsFcu.restoreBackup")}</Button>,
                ]}>
                    <List.Item.Meta
                        title={backup.filename}
                        description={`${t("settingsFcu.originalBackup")} · ${new Date(backup.createdAt).toLocaleString()}`}
                    />
                </List.Item>}/>
            }
        </Card>
    </Space>;
}
