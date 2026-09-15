import {useState} from "react";
import {Button, Card, Col, Form, Input, Row, Select, Space, Typography} from "antd";
import {SendOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import type {
    NotificationChannel,
    NotificationLanguage,
    NotificationSettings,
    NotificationSettingsUpdate,
} from "../../types/notifications.ts";

const {Text, Paragraph} = Typography;

interface NotificationsChannelCardProps {
    settings: NotificationSettings;
    draft: NotificationSettingsUpdate;
    onChange: (patch: NotificationSettingsUpdate) => void;
    testing: boolean;
    onSendTest: () => void;
}

interface SecretFieldProps {
    label: string;
    tooltip: string;
    placeholder: string;
    stored: boolean;
    masked: string;
    pending: boolean;
    cleared: boolean;
    onApply: (value: string) => void;
    onClear: () => void;
}

/**
 * A write-only secret: the backend only ever returns a masked prefix, so the
 * field shows "set" / "not set" and a box to paste a replacement into.
 */
function SecretField({label, tooltip, placeholder, stored, masked, pending, cleared, onApply, onClear}: SecretFieldProps) {
    const {t} = useTranslation();
    const [input, setInput] = useState("");
    const state = pending
        ? t("settingsNotifications.secretPending")
        : cleared
            ? t("settingsNotifications.secretCleared")
            : stored
                ? t("settingsNotifications.secretSet", {masked})
                : t("settingsNotifications.secretNotSet");
    const apply = () => {
        const trimmed = input.trim();
        if (!trimmed) return;
        onApply(trimmed);
        setInput("");
    };
    return (
        <Form.Item label={label} tooltip={tooltip}>
            <Space.Compact style={{width: "100%"}}>
                <Input.Password
                    value={input}
                    onChange={(e) => setInput(e.target.value)}
                    onPressEnter={apply}
                    placeholder={placeholder}
                    aria-label={label}
                    autoComplete="off"
                />
                <Button onClick={apply} disabled={!input.trim()}>{t("settingsNotifications.secretApply")}</Button>
                {(stored || pending) && !cleared && (
                    <Button danger onClick={onClear}>{t("settingsNotifications.secretClear")}</Button>
                )}
            </Space.Compact>
            <Text type="secondary" style={{fontSize: 12}}>{state}</Text>
        </Form.Item>
    );
}

/** Channel picker plus the fields of the selected channel and the test button. */
export function NotificationsChannelCard({settings, draft, onChange, testing, onSendTest}: NotificationsChannelCardProps) {
    const {t} = useTranslation();
    const channel = draft.channel ?? settings.channel;
    const language = draft.language ?? settings.language;
    const title = draft.title ?? settings.title;

    const channelOptions: {value: NotificationChannel; label: string}[] = [
        {value: "telegram", label: t("settingsNotifications.channelTelegram")},
        {value: "pushover", label: t("settingsNotifications.channelPushover")},
        {value: "ntfy", label: t("settingsNotifications.channelNtfy")},
        {value: "webhook", label: t("settingsNotifications.channelWebhook")},
    ];
    const languageOptions: {value: NotificationLanguage; label: string}[] = [
        {value: "en", label: "English"},
        {value: "fr", label: "Français"},
    ];

    return (
        <Card size="small" title={t("settingsNotifications.channel")} style={{marginBottom: 16}}>
            <Form layout="vertical" size="small">
                <Row gutter={[16, 0]}>
                    <Col xs={24} sm={8}>
                        <Form.Item label={t("settingsNotifications.channelLabel")} tooltip={t("settingsNotifications.channelTooltip")}>
                            <Select
                                value={channel}
                                options={channelOptions}
                                onChange={(v) => onChange({channel: v})}
                                aria-label={t("settingsNotifications.channelLabel")}
                            />
                        </Form.Item>
                    </Col>
                    <Col xs={24} sm={8}>
                        <Form.Item label={t("settingsNotifications.title")} tooltip={t("settingsNotifications.titleTooltip")}>
                            <Input
                                value={title}
                                onChange={(e) => onChange({title: e.target.value})}
                                aria-label={t("settingsNotifications.title")}
                            />
                        </Form.Item>
                    </Col>
                    <Col xs={24} sm={8}>
                        <Form.Item label={t("settingsNotifications.language")} tooltip={t("settingsNotifications.languageTooltip")}>
                            <Select
                                value={language}
                                options={languageOptions}
                                onChange={(v) => onChange({language: v})}
                                aria-label={t("settingsNotifications.language")}
                            />
                        </Form.Item>
                    </Col>
                </Row>

                {channel === "ntfy" && (
                    <>
                        <Paragraph type="secondary" style={{margin: "0 0 12px", fontSize: 12}}>
                            {t("settingsNotifications.ntfyHelp")}
                        </Paragraph>
                        <Row gutter={[16, 0]}>
                            <Col xs={24} sm={12}>
                                <Form.Item label={t("settingsNotifications.ntfyServer")} tooltip={t("settingsNotifications.ntfyServerTooltip")}>
                                    <Input
                                        value={draft.ntfyServer ?? settings.ntfyServer}
                                        onChange={(e) => onChange({ntfyServer: e.target.value})}
                                        placeholder="https://ntfy.sh"
                                        aria-label={t("settingsNotifications.ntfyServer")}
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item label={t("settingsNotifications.ntfyTopic")} tooltip={t("settingsNotifications.ntfyTopicTooltip")}>
                                    <Input
                                        value={draft.ntfyTopic ?? settings.ntfyTopic}
                                        onChange={(e) => onChange({ntfyTopic: e.target.value})}
                                        placeholder="mowgli-garden-a8f3k2"
                                        aria-label={t("settingsNotifications.ntfyTopic")}
                                    />
                                </Form.Item>
                            </Col>
                        </Row>
                        <SecretField
                            label={t("settingsNotifications.ntfyToken")}
                            tooltip={t("settingsNotifications.ntfyTokenTooltip")}
                            placeholder={t("settingsNotifications.secretPlaceholder")}
                            stored={settings.ntfyTokenSet}
                            masked={settings.ntfyTokenMasked}
                            pending={draft.ntfyToken !== undefined}
                            cleared={draft.clearNtfyToken === true}
                            onApply={(v) => onChange({ntfyToken: v, clearNtfyToken: undefined})}
                            onClear={() => onChange({ntfyToken: undefined, clearNtfyToken: true})}
                        />
                    </>
                )}

                {channel === "telegram" && (
                    <>
                        <Paragraph type="secondary" style={{margin: "0 0 12px", fontSize: 12}}>
                            {t("settingsNotifications.telegramHelp")}
                        </Paragraph>
                        <SecretField
                            label={t("settingsNotifications.telegramBotToken")}
                            tooltip={t("settingsNotifications.telegramBotTokenTooltip")}
                            placeholder={t("settingsNotifications.secretPlaceholder")}
                            stored={settings.telegramBotTokenSet}
                            masked={settings.telegramBotTokenMasked}
                            pending={draft.telegramBotToken !== undefined}
                            cleared={draft.clearTelegramBotToken === true}
                            onApply={(v) => onChange({telegramBotToken: v, clearTelegramBotToken: undefined})}
                            onClear={() => onChange({telegramBotToken: undefined, clearTelegramBotToken: true})}
                        />
                        <Form.Item label={t("settingsNotifications.telegramChatId")} tooltip={t("settingsNotifications.telegramChatIdTooltip")}>
                            <Input
                                value={draft.telegramChatId ?? settings.telegramChatId}
                                onChange={(e) => onChange({telegramChatId: e.target.value})}
                                placeholder="123456789"
                                aria-label={t("settingsNotifications.telegramChatId")}
                            />
                        </Form.Item>
                    </>
                )}

                {channel === "pushover" && (
                    <>
                        <Paragraph type="secondary" style={{margin: "0 0 12px", fontSize: 12}}>
                            {t("settingsNotifications.pushoverHelp")}
                        </Paragraph>
                        <SecretField
                            label={t("settingsNotifications.pushoverAppToken")}
                            tooltip={t("settingsNotifications.pushoverAppTokenTooltip")}
                            placeholder={t("settingsNotifications.secretPlaceholder")}
                            stored={settings.pushoverAppTokenSet}
                            masked={settings.pushoverAppTokenMasked}
                            pending={draft.pushoverAppToken !== undefined}
                            cleared={draft.clearPushoverAppToken === true}
                            onApply={(v) => onChange({pushoverAppToken: v, clearPushoverAppToken: undefined})}
                            onClear={() => onChange({pushoverAppToken: undefined, clearPushoverAppToken: true})}
                        />
                        <Form.Item label={t("settingsNotifications.pushoverUserKey")} tooltip={t("settingsNotifications.pushoverUserKeyTooltip")}>
                            <Input
                                value={draft.pushoverUserKey ?? settings.pushoverUserKey}
                                onChange={(e) => onChange({pushoverUserKey: e.target.value})}
                                placeholder="uQiRzpo4DXghDmr9QzzfQu27cmVRsG"
                                aria-label={t("settingsNotifications.pushoverUserKey")}
                            />
                        </Form.Item>
                    </>
                )}

                {channel === "webhook" && (
                    <>
                        <Paragraph type="secondary" style={{margin: "0 0 12px", fontSize: 12}}>
                            {t("settingsNotifications.webhookHelp")}
                        </Paragraph>
                        <Form.Item label={t("settingsNotifications.webhookUrl")} tooltip={t("settingsNotifications.webhookUrlTooltip")}>
                            <Input
                                value={draft.webhookUrl ?? settings.webhookUrl}
                                onChange={(e) => onChange({webhookUrl: e.target.value})}
                                placeholder="http://homeassistant.local:8123/api/webhook/mowgli"
                                aria-label={t("settingsNotifications.webhookUrl")}
                            />
                        </Form.Item>
                    </>
                )}

                <Button icon={<SendOutlined/>} onClick={onSendTest} loading={testing}>
                    {t("settingsNotifications.sendTest")}
                </Button>
            </Form>
        </Card>
    );
}
