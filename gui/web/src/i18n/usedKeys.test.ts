import { describe, expect, it } from "vitest";
import en from "./locales/en.json";
import fr from "./locales/fr.json";
import type { BackendAuthority, HardwareBackend } from "../constants/hardwareBackends";
import * as fieldGroups from "../components/settings/settingsFieldGroups";
import type { SettingsFieldGroup } from "../components/settings/settingsFieldGroups";
import { PARAM_GROUPS } from "../components/settings/paramCatalog";
import { NODES } from "../components/btStateGraphModel";
import { MEASUREMENTS } from "../components/robot/measurements";
import { NOTIFICATION_EVENT_KINDS } from "../types/notifications";
import { diagnosticGroups } from "../utils/fcuParamDiagnostics";

/**
 * A key the UI asks for but neither locale defines renders as its raw dotted
 * path (e.g. "settingsHardwareBackend.title") — exactly what the operator saw.
 * locales.test.ts only proves the two files agree with each other; this guard
 * proves they cover what the source actually uses.
 */
type Json = Record<string, unknown>;

const LOCALES = { en: en as Json, fr: fr as Json };
const PLURAL_SUFFIXES = ["", "_zero", "_one", "_two", "_few", "_many", "_other"];

const lookup = (locale: Json, key: string): unknown =>
    key.split(".").reduce<unknown>(
        (node, part) => (node !== null && typeof node === "object" ? (node as Json)[part] : undefined),
        locale,
    );

const hasLeaf = (locale: Json, key: string): boolean =>
    PLURAL_SUFFIXES.some((suffix) => typeof lookup(locale, key + suffix) === "string");

/** Keys absent from at least one locale, tagged with the locale(s) missing them. */
function missingKeys(keys: Iterable<string>): string[] {
    const missing = new Set<string>();
    for (const key of keys) {
        for (const [name, locale] of Object.entries(LOCALES)) {
            if (!hasLeaf(locale, key)) missing.add(`${key} (${name})`);
        }
    }
    return [...missing].sort();
}

const SOURCES = import.meta.glob<string>(["../**/*.{ts,tsx}", "!../**/*.test.{ts,tsx}", "!../**/__tests__/**", "!../test/**"], {
    query: "?raw",
    import: "default",
    eager: true,
});

const KEY_SHAPE = /^[A-Za-z][\w-]*(\.[\w-]+)+$/;

/** Literal keys handed to t() / i18n.t() / <Trans i18nKey>. */
function literalCallKeys(): Map<string, string> {
    const call = /(?:\bi18n(?:ext)?\.t|\bt)\(\s*(["'`])([^"'`$\n]+)\1|\bi18nKey=\{?\s*(["'`])([^"'`$\n]+)\3/g;
    const found = new Map<string, string>();
    for (const [file, source] of Object.entries(SOURCES)) {
        for (const match of source.matchAll(call)) {
            const key = match[2] ?? match[4];
            if (KEY_SHAPE.test(key)) found.set(key, file);
        }
    }
    return found;
}

/**
 * Fully-qualified keys stored in constants (labelKey / titleKey fields, label
 * maps, …) and only later passed to t(): any string literal whose first segment
 * is a locale namespace. A literal naming a whole sub-tree (a prefix completed
 * at runtime) is not a key and is skipped.
 */
function namespacedLiterals(): Map<string, string> {
    const namespaces = new Set(Object.keys(en));
    const literal = /(["'])([A-Za-z][\w]*(?:\.[\w-]+)+)\1/g;
    const found = new Map<string, string>();
    for (const [file, source] of Object.entries(SOURCES)) {
        for (const match of source.matchAll(literal)) {
            const key = match[2];
            if (!namespaces.has(key.split(".")[0])) continue;
            const node = lookup(en as Json, key);
            if (node !== null && typeof node === "object") continue;
            found.set(key, file);
        }
    }
    return found;
}

// Exhaustive by construction: adding a backend or an authority to the union
// fails to compile until it is listed here, and then fails below until both
// locales name it.
const BACKENDS: Record<HardwareBackend, true> = { mowgli: true, openmower: true, mavros: true, unknown: true };
const UNCONFIRMED_AUTHORITIES: Record<Exclude<BackendAuthority, "confirmed">, true> = {
    loading: true,
    legacy_absent: true,
    unavailable: true,
    malformed: true,
    unsupported: true,
};

const isFieldGroup = (value: unknown): value is SettingsFieldGroup =>
    value !== null && typeof value === "object" && "id" in value && "fields" in value;

describe("i18n keys used by the source", () => {
    it("scans the source tree", () => {
        expect(Object.keys(SOURCES).length).toBeGreaterThan(100);
        expect(literalCallKeys().size).toBeGreaterThan(1000);
    });

    it("defines every literal t() / i18n.t() / i18nKey key in both locales", () => {
        expect(missingKeys(literalCallKeys().keys())).toEqual([]);
    });

    it("defines every namespaced key held in a constant in both locales", () => {
        expect(missingKeys(namespacedLiterals().keys())).toEqual([]);
    });

    it("names and describes every hardware backend and metadata authority", () => {
        const keys = Object.keys(BACKENDS).flatMap((backend) =>
            ["names", "shortNames", "descriptions"].map((family) => `settingsHardwareBackend.${family}.${backend}`),
        );
        keys.push(...Object.keys(UNCONFIRMED_AUTHORITIES).map((a) => `settingsHardwareBackend.authority.${a}`));
        keys.push("settingsHardwareBackend.title");
        expect(missingKeys(keys)).toEqual([]);
    });

    it("labels every declarative settings field and group", () => {
        const groups = Object.values(fieldGroups).filter(isFieldGroup);
        expect(groups.length).toBeGreaterThanOrEqual(fieldGroups.ALL_FIELD_GROUPS.length);
        const keys = groups.flatMap((group) => [
            `settingsFieldGroups.${group.id}.title`,
            `settingsFieldGroups.${group.id}.description`,
            ...group.fields.flatMap((field) => [
                `settingsFields.${field.key}.label`,
                `settingsFields.${field.key}.tooltip`,
            ]),
        ]);
        expect(missingKeys(keys)).toEqual([]);
    });

    it("labels every robot measurement", () => {
        const keys = Object.values(MEASUREMENTS).flatMap(({ label }) => [
            `settingsHardware.${label}`,
            `settingsHardware.${label}Tooltip`,
        ]);
        expect(missingKeys(keys)).toEqual([]);
    });

    it("labels the remaining runtime-built families", () => {
        const keys = [
            ...PARAM_GROUPS.map((group) => `paramGroups.${group}`),
            ...NODES.map((node) => `btStateGraph.${node.labelKey}`),
            ...NOTIFICATION_EVENT_KINDS.flatMap((kind) => [
                `settingsNotifications.event.${kind}.label`,
                `settingsNotifications.event.${kind}.description`,
            ]),
            ...diagnosticGroups.map((group) => `settingsFcuParams.groups.${group}`),
        ];
        expect(missingKeys(keys)).toEqual([]);
    });
});
