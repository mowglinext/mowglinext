import {useState} from "react";
import {Button, Card, Form, Input, Space, Switch, Typography} from "antd";
import {useTranslation} from "react-i18next";
import type {RemoteAccessSettings, RemoteAccessSettingsUpdate} from "../../types/remoteAccess.ts";

const {Text} = Typography;

interface RemoteAccessConnectionCardProps {
    settings: RemoteAccessSettings;
    draft: RemoteAccessSettingsUpdate;
    onChange: (patch: RemoteAccessSettingsUpdate) => void;
}

/**
 * Node name, the write-only auth key, HTTPS publishing and the image pin.
 * The key field is never pre-filled: the backend only returns a masked
 * prefix, so what the operator sees is "set" or "not set" plus a box to
 * paste a new one into.
 */
export function RemoteAccessConnectionCard({settings, draft, onChange}: RemoteAccessConnectionCardProps) {
    const {t} = useTranslation();
    const [keyInput, setKeyInput] = useState("");

    const hostname = draft.hostname ?? settings.hostname;
    const serveHttps = draft.serveHttps ?? settings.serveHttps;
    const image = draft.image ?? settings.image;
    const keyPending = draft.authKey !== undefined;
    const keyCleared = draft.clearAuthKey === true;
    const keyState = keyPending
        ? t("settingsRemoteAccess.keyPending")
        : keyCleared
            ? t("settingsRemoteAccess.keyCleared")
            : settings.authKeySet
                ? t("settingsRemoteAccess.keySet", {masked: settings.authKeyMasked})
                : t("settingsRemoteAccess.keyNotSet");

    const applyKey = () => {
        const trimmed = keyInput.trim();
        if (!trimmed) return;
        onChange({authKey: trimmed, clearAuthKey: undefined});
        setKeyInput("");
    };

    return (
        <Card size="small" title={t("settingsRemoteAccess.connection")} style={{marginBottom: 16}}>
            <Form layout="vertical" size="small">
                <Form.Item label={t("settingsRemoteAccess.hostname")} tooltip={t("settingsRemoteAccess.hostnameTooltip")}>
                    <Input
                        value={hostname}
                        onChange={(e) => onChange({hostname: e.target.value.toLowerCase()})}
                        placeholder="mowgli"
                        maxLength={63}
                        aria-label={t("settingsRemoteAccess.hostname")}
                    />
                </Form.Item>
                <Form.Item label={t("settingsRemoteAccess.key")} tooltip={t("settingsRemoteAccess.keyTooltip")}>
                    <Space.Compact style={{width: "100%"}}>
                        <Input.Password
                            value={keyInput}
                            onChange={(e) => setKeyInput(e.target.value)}
                            onPressEnter={applyKey}
                            placeholder={t("settingsRemoteAccess.keyPlaceholder")}
                            aria-label={t("settingsRemoteAccess.key")}
                            autoComplete="off"
                        />
                        <Button onClick={applyKey} disabled={!keyInput.trim()}>
                            {t("settingsRemoteAccess.keyApply")}
                        </Button>
                        <Button
                            danger
                            disabled={!settings.authKeySet && !keyPending}
                            onClick={() => { onChange({authKey: undefined, clearAuthKey: true}); setKeyInput(""); }}
                        >
                            {t("settingsRemoteAccess.keyClear")}
                        </Button>
                    </Space.Compact>
                    <Text type="secondary" style={{fontSize: 12}} data-testid="remote-access-key-state">{keyState}</Text>
                </Form.Item>
                <Form.Item label={t("settingsRemoteAccess.serveHttps")} tooltip={t("settingsRemoteAccess.serveHttpsTooltip")}>
                    <Space>
                        <Switch
                            checked={serveHttps}
                            onChange={(checked) => onChange({serveHttps: checked})}
                            aria-label={t("settingsRemoteAccess.serveHttps")}
                        />
                        <Text type="secondary" style={{fontSize: 12}}>{t("settingsRemoteAccess.serveHttpsHint")}</Text>
                    </Space>
                </Form.Item>
                <Form.Item label={t("settingsRemoteAccess.image")} tooltip={t("settingsRemoteAccess.imageTooltip")}>
                    <Input
                        value={image}
                        onChange={(e) => onChange({image: e.target.value})}
                        placeholder={settings.defaultImage}
                        aria-label={t("settingsRemoteAccess.image")}
                    />
                </Form.Item>
            </Form>
        </Card>
    );
}
