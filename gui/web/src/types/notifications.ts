/** Mirrors gui/pkg/providers notification kinds — the operator toggles. */
export const NOTIFICATION_EVENT_KINDS = [
    "mowStarted",
    "zoneStarted",
    "zoneFinished",
    "mowComplete",
    "mowStopped",
    "blocked",
    "emergency",
    "battery",
    "rain",
    "waitingForRtk",
] as const;

export type NotificationEventKind = (typeof NOTIFICATION_EVENT_KINDS)[number];

export type NotificationChannel = "telegram" | "pushover" | "ntfy" | "webhook";

/** Display order, mirrors providers.NotifyChannels. */
export const NOTIFICATION_CHANNELS: NotificationChannel[] = ["telegram", "pushover", "ntfy", "webhook"];
export type NotificationLanguage = "en" | "fr";

/** Mirrors api.NotificationSettingsResponse — secrets are never in here. */
export interface NotificationSettings {
    enabled: boolean;
    channel: NotificationChannel;
    language: NotificationLanguage;
    title: string;
    ntfyServer: string;
    ntfyTopic: string;
    ntfyTokenSet: boolean;
    ntfyTokenMasked: string;
    telegramBotTokenSet: boolean;
    telegramBotTokenMasked: string;
    telegramChatId: string;
    pushoverAppTokenSet: boolean;
    pushoverAppTokenMasked: string;
    pushoverUserKey: string;
    webhookUrl: string;
    events: Record<string, boolean>;
    eventKinds: string[];
    channels: string[];
}

/** Mirrors api.NotificationSettingsUpdate — absent fields keep their value. */
export interface NotificationSettingsUpdate
    extends Partial<Omit<NotificationSettings,
        "ntfyTokenSet" | "ntfyTokenMasked" | "telegramBotTokenSet" | "telegramBotTokenMasked"
        | "pushoverAppTokenSet" | "pushoverAppTokenMasked" | "eventKinds" | "channels">> {
    ntfyToken?: string;
    clearNtfyToken?: boolean;
    telegramBotToken?: string;
    clearTelegramBotToken?: boolean;
    pushoverAppToken?: string;
    clearPushoverAppToken?: boolean;
}

/** Mirrors providers.NotifyDeliveryStatus. */
export interface NotificationDeliveryStatus {
    enabled: boolean;
    configured: boolean;
    channel: NotificationChannel;
    sentCount: number;
    failedCount: number;
    lastMessage?: string;
    lastSentAt?: string;
    lastError?: string;
    lastErrorAt?: string;
}
