import {Cloud, CloudRain, Sun, CloudSun, Snowflake} from "lucide-react";

/**
 * Compact weather pill -- icon + temp + condition. Tone shifts to amber
 * when rain is forecast (we'll auto-dock in that case).
 */

type Condition = "clear" | "partly" | "cloudy" | "rain" | "snow";

const ICON: Record<Condition, typeof Cloud> = {
  clear:  Sun,
  partly: CloudSun,
  cloudy: Cloud,
  rain:   CloudRain,
  snow:   Snowflake,
};

interface WeatherChipProps {
  condition: Condition;
  tempC: number;
  rainSoon?: boolean;
}

export function WeatherChip({condition, tempC, rainSoon}: WeatherChipProps) {
  const Icon = ICON[condition];
  const warn = rainSoon || condition === "rain";
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
        {warn ? "pluie en approche" : conditionLabel(condition)}
      </span>
    </div>
  );
}

function conditionLabel(c: Condition) {
  switch (c) {
    case "clear":  return "ciel dégagé";
    case "partly": return "ensoleillé";
    case "cloudy": return "nuageux";
    case "rain":   return "pluvieux";
    case "snow":   return "neigeux";
  }
}
