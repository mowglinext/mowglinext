import {Cloud, CloudRain, Sun, CloudSun, Snowflake, CloudFog, CloudLightning} from "lucide-react";
import {useTranslation} from "react-i18next";
import i18n from "../../i18n";

/**
 * Compact weather pill -- icon + temp + condition. Tone shifts to amber
 * when rain is forecast (we'll auto-dock in that case).
 */

export type Condition = "clear" | "partly" | "cloudy" | "fog" | "rain" | "snow" | "storm";

const ICON: Record<Condition, typeof Cloud> = {
  clear:  Sun,
  partly: CloudSun,
  cloudy: Cloud,
  fog:    CloudFog,
  rain:   CloudRain,
  snow:   Snowflake,
  storm:  CloudLightning,
};

interface WeatherChipProps {
  condition: Condition;
  tempC: number;
  rainSoon?: boolean;
}

export function WeatherChip({condition, tempC, rainSoon}: WeatherChipProps) {
  const {t} = useTranslation();
  const Icon = ICON[condition];
  const warn = rainSoon || condition === "rain" || condition === "storm";
  return (
    <div style={{
      display: "inline-flex", alignItems: "center", gap: 10,
      padding: "8px 14px 8px 12px",
      background: warn
        ? "rgba(243, 168, 92, 0.12)"
        : "rgba(255, 255, 255, 0.04)",
      border: `1px solid ${warn ? "rgba(243, 168, 92, 0.32)" : "var(--border-soft)"}`,
      borderRadius: "var(--radius-pill)",
      color: warn ? "var(--amber)" : "var(--ink)",
      backdropFilter: "blur(20px)",
      fontSize: 13, fontWeight: 600,
    }}>
      <Icon size={16} strokeWidth={2}/>
      <span className="mono">{tempC.toFixed(0)}°C</span>
      <span style={{opacity: 0.6, fontSize: 11, marginLeft: -2}}>
        {warn ? t('weatherChip.rainIncoming') : conditionLabel(condition)}
      </span>
    </div>
  );
}

function conditionLabel(c: Condition) {
  switch (c) {
    case "clear":  return i18n.t('weatherChip.clear');
    case "partly": return i18n.t('weatherChip.partly');
    case "cloudy": return i18n.t('weatherChip.cloudy');
    case "fog":    return i18n.t('weatherChip.fog');
    case "rain":   return i18n.t('weatherChip.rain');
    case "snow":   return i18n.t('weatherChip.snow');
    case "storm":  return i18n.t('weatherChip.storm');
  }
}
