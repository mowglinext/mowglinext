import {useCallback, useEffect, useState} from "react";
import {App, Button, Empty, List, Modal, Spin, Tag, Typography} from "antd";
import {useTranslation} from "react-i18next";
import type {MapBackup} from "../hooks/useMapBackups.ts";

interface RestoreMapBackupModalProps {
    open: boolean;
    onClose: () => void;
    list: () => Promise<{backups: MapBackup[]; keep: number}>;
    restore: (id: string) => Promise<void>;
    /// Called after a backup was restored, so the page can leave the editor and reload.
    onRestored: () => void;
}

/// The last map backups, newest first. A backup is taken automatically every time the
/// editor opens, so each row is "the map as it was before that edit session".
export const RestoreMapBackupModal = ({open, onClose, list, restore, onRestored}: RestoreMapBackupModalProps) => {
    const {t, i18n} = useTranslation();
    const {modal, notification} = App.useApp();
    const [backups, setBackups] = useState<MapBackup[]>([]);
    const [keep, setKeep] = useState(20);
    const [loading, setLoading] = useState(false);
    const [restoring, setRestoring] = useState<string | null>(null);

    useEffect(() => {
        if (!open) return;
        let cancelled = false;
        setLoading(true);
        list()
            .then((res) => { if (!cancelled) { setBackups(res.backups); setKeep(res.keep); } })
            .catch((e: unknown) => {
                if (!cancelled) {
                    notification.error({message: t("mapBackups.listFailed"), description: e instanceof Error ? e.message : undefined});
                }
            })
            .finally(() => { if (!cancelled) setLoading(false); });
        return () => { cancelled = true; };
    }, [open, list, notification, t]);

    const when = useCallback(
        (iso: string) => new Date(iso).toLocaleString(i18n.language, {dateStyle: "medium", timeStyle: "medium"}),
        [i18n.language]);

    const confirmRestore = (backup: MapBackup) => {
        modal.confirm({
            title: t("mapBackups.confirmTitle"),
            content: t("mapBackups.confirmBody", {when: when(backup.created_at), areas: backup.areas}),
            okText: t("mapBackups.restore"),
            okType: "danger",
            cancelText: t("mapBackups.cancel"),
            onOk: async () => {
                setRestoring(backup.id);
                try {
                    await restore(backup.id);
                    notification.success({message: t("mapBackups.restored")});
                    onRestored();
                } catch (e: unknown) {
                    notification.error({
                        message: t("mapBackups.restoreFailed"),
                        description: e instanceof Error ? e.message : undefined,
                    });
                } finally {
                    setRestoring(null);
                }
            },
        });
    };

    return (
        <Modal
            open={open}
            title={t("mapBackups.title")}
            onCancel={onClose}
            footer={<Button onClick={onClose}>{t("mapBackups.close")}</Button>}
            width={560}
        >
            <Typography.Paragraph type="secondary">{t("mapBackups.intro", {keep})}</Typography.Paragraph>
            <Spin spinning={loading}>
                {backups.length === 0 && !loading ? (
                    <Empty description={t("mapBackups.empty")}/>
                ) : (
                    <List
                        dataSource={backups}
                        renderItem={(backup) => (
                            <List.Item
                                actions={[
                                    <Button
                                        key="restore"
                                        size="small"
                                        danger
                                        loading={restoring === backup.id}
                                        disabled={restoring !== null}
                                        onClick={() => confirmRestore(backup)}
                                    >
                                        {t("mapBackups.restore")}
                                    </Button>,
                                ]}
                            >
                                <List.Item.Meta
                                    title={when(backup.created_at)}
                                    description={
                                        <>
                                            <div>{backup.area_names.join(", ")}</div>
                                            <Tag>{t("mapBackups.areas", {count: backup.areas})}</Tag>
                                            <Tag>{t("mapBackups.obstacles", {count: backup.obstacles})}</Tag>
                                            <Tag>{t("mapBackups.ignoreLines", {count: backup.ignore_lines})}</Tag>
                                        </>
                                    }
                                />
                            </List.Item>
                        )}
                    />
                )}
            </Spin>
        </Modal>
    );
};
