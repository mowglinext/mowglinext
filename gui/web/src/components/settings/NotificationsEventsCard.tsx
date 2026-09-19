import {Card, Switch, Typography} from "antd";
import {useTranslation} from "react-i18next";
import {
    NOTIFICATION_EVENT_KINDS,
    type NotificationSettings,
    type NotificationSettingsUpdate,
} from "../../types/notifications.ts";

const {Text, Paragraph} = Typography;

interface NotificationsEventsCardProps {
    settings: NotificationSettings;
    draft: NotificationSettingsUpdate;
    onChange: (patch: NotificationSettingsUpdate) => void;
}

/** One switch per event kind; the draft carries the whole events map. */
export function NotificationsEventsCard({settings, draft, onChange}: NotificationsEventsCardProps) {
    const {t} = useTranslation();
    const events = {...settings.events, ...(draft.events ?? {})};
    const kinds = settings.eventKinds.length > 0 ? settings.eventKinds : [...NOTIFICATION_EVENT_KINDS];

    return (
        <Card size="small" title={t("settingsNotifications.events")} style={{marginBottom: 16}}>
            <Paragraph type="secondary" style={{margin: "0 0 12px", fontSize: 12}}>
                {t("settingsNotifications.eventsDescription")}
            </Paragraph>
            {kinds.map((kind) => (
                <div
                    key={kind}
                    style={{display: "flex", justifyContent: "space-between", alignItems: "center", gap: 12, padding: "6px 0"}}
                >
                    <div>
                        <Text>{t(`settingsNotifications.event.${kind}.label`, {defaultValue: kind})}</Text>
                        <br/>
                        <Text type="secondary" style={{fontSize: 12}}>
                            {t(`settingsNotifications.event.${kind}.description`, {defaultValue: ""})}
                        </Text>
                    </div>
                    <Switch
                        checked={events[kind] ?? true}
                        onChange={(checked) => onChange({events: {...(draft.events ?? {}), [kind]: checked}})}
                        aria-label={t(`settingsNotifications.event.${kind}.label`, {defaultValue: kind})}
                    />
                </div>
            ))}
        </Card>
    );
}
