import i18n from "i18next";
import {initReactI18next} from "react-i18next";
import LanguageDetector from "i18next-browser-languagedetector";

import fr from "./locales/fr.json";
import en from "./locales/en.json";

// Supported UI languages. French is the source language the app was authored
// in, so it stays the fallback; English is the first translation.
export const SUPPORTED_LANGUAGES = [
  {code: "fr", label: "Français"},
  {code: "en", label: "English"},
] as const;

export type LanguageCode = (typeof SUPPORTED_LANGUAGES)[number]["code"];

i18n
  // Detect from localStorage first (an explicit user choice via the in-app
  // switcher), then fall back to the browser language (navigator.language).
  .use(LanguageDetector)
  .use(initReactI18next)
  .init({
    resources: {
      fr: {translation: fr},
      en: {translation: en},
    },
    // Fallback chain (covers both directions if a key is ever missing):
    // a missing English key falls back to French (the source language), and a
    // missing French key falls back to English instead of showing a raw key.
    fallbackLng: ["fr", "en"],
    supportedLngs: SUPPORTED_LANGUAGES.map((l) => l.code),
    // Map regional codes (e.g. "en-US", "fr-CA") onto the base language.
    nonExplicitSupportedLngs: true,
    load: "languageOnly",
    interpolation: {
      // React already escapes values, so i18next must not double-escape.
      escapeValue: false,
    },
    detection: {
      order: ["localStorage", "navigator"],
      lookupLocalStorage: "mowglinext.lang",
      caches: ["localStorage"],
    },
  });

export default i18n;
