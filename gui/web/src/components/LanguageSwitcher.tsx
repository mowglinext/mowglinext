import {useTranslation} from "react-i18next";
import {SUPPORTED_LANGUAGES} from "../i18n";

/**
 * Compact FR/EN segmented toggle for the app header.
 *
 * The active language is auto-detected on first load (localStorage choice, then
 * navigator.language); this lets the user override it manually. The choice is
 * persisted to localStorage by the language detector.
 */
export function LanguageSwitcher() {
  const {i18n, t} = useTranslation();
  const current = (i18n.resolvedLanguage ?? i18n.language ?? "fr").slice(0, 2);

  return (
    <div
      role="group"
      aria-label={t("language.label")}
      style={{
        display: "inline-flex",
        alignItems: "center",
        gap: 2,
        padding: 2,
        borderRadius: 999,
        background: "rgba(255, 255, 255, 0.04)",
        border: "1px solid rgba(236, 255, 244, 0.10)",
      }}
    >
      {SUPPORTED_LANGUAGES.map(({code, label}) => {
        const active = current === code;
        return (
          <button
            key={code}
            type="button"
            onClick={() => i18n.changeLanguage(code)}
            aria-label={label}
            aria-pressed={active}
            style={{
              border: "none",
              cursor: "pointer",
              borderRadius: 999,
              padding: "4px 9px",
              fontSize: 11,
              fontWeight: 700,
              letterSpacing: "0.04em",
              textTransform: "uppercase",
              lineHeight: 1.1,
              background: active
                ? "linear-gradient(135deg, #7CFFB2 0%, #45D688 100%)"
                : "transparent",
              color: active ? "#02110D" : "rgba(236, 255, 244, 0.6)",
              transition: "background 0.15s, color 0.15s",
            }}
          >
            {code.toUpperCase()}
          </button>
        );
      })}
    </div>
  );
}

export default LanguageSwitcher;
