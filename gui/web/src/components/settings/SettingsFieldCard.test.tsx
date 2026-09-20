import { describe, expect, it, vi } from "vitest";
import { fireEvent, render, screen } from "@testing-library/react";
import { ThemeProvider } from "../../theme/ThemeContext.tsx";
import en from "../../i18n/locales/en.json";
import fr from "../../i18n/locales/fr.json";
import { SettingsFieldCard } from "./SettingsFieldCard.tsx";
import {
    ALL_FIELD_GROUPS,
    REVERSE_ESCAPE_GROUP,
    TURN_SPEED_GROUP,
    groupKeys,
} from "./settingsFieldGroups.ts";

type FieldStrings = Record<string, { label?: string; tooltip?: string }>;
type GroupStrings = Record<string, { title?: string; description?: string } | string>;

describe("settingsFieldGroups", () => {
    const allKeys = ALL_FIELD_GROUPS.flatMap(groupKeys);

    it("declares each key exactly once", () => {
        expect(new Set(allKeys).size).toBe(allKeys.length);
    });

    // "Every key has a schema default" is pinned on the Go side
    // (pkg/api/settings_field_groups_test.go): the Docker web build typechecks
    // this file with only web/ in its context, so asserts/ is unreachable here.
    it.each([
        ["en", en],
        ["fr", fr],
    ])("has a label, tooltip and group heading for every field in %s", (_lang, locale) => {
        const fields = (locale as { settingsFields: FieldStrings }).settingsFields;
        const groups = (locale as { settingsFieldGroups: GroupStrings }).settingsFieldGroups;
        for (const key of allKeys) {
            expect(fields[key]?.label, `${key}.label`).toBeTruthy();
            expect(fields[key]?.tooltip, `${key}.tooltip`).toBeTruthy();
        }
        for (const group of ALL_FIELD_GROUPS) {
            const heading = groups[group.id];
            expect(typeof heading === "object" && heading.title, `${group.id}.title`).toBeTruthy();
            expect(typeof heading === "object" && heading.description, `${group.id}.description`).toBeTruthy();
        }
    });
});

describe("SettingsFieldCard", () => {
    it("renders one control per field and reports a switch change by key", () => {
        // Arrange
        const onChange = vi.fn();
        render(
            <ThemeProvider>
                <SettingsFieldCard
                    group={REVERSE_ESCAPE_GROUP}
                    values={{
                        obstacle_reverse_enabled: true,
                        obstacle_reverse_max_dist_m: 0.3,
                        obstacle_reverse_speed_mps: 0.15,
                    }}
                    onChange={onChange}
                />
            </ThemeProvider>,
        );

        // Act
        fireEvent.click(screen.getByRole("switch"));

        // Assert
        expect(screen.getAllByRole("spinbutton")).toHaveLength(2);
        expect(onChange).toHaveBeenCalledWith("obstacle_reverse_enabled", false);
    });

    it("warns on a safety-relevant group only", () => {
        const { rerender } = render(
            <ThemeProvider>
                <SettingsFieldCard group={REVERSE_ESCAPE_GROUP} values={{}} onChange={vi.fn()} />
            </ThemeProvider>,
        );
        expect(screen.getByRole("alert")).toBeInTheDocument();

        rerender(
            <ThemeProvider>
                <SettingsFieldCard group={TURN_SPEED_GROUP} values={{}} onChange={vi.fn()} />
            </ThemeProvider>,
        );
        expect(screen.queryByRole("alert")).not.toBeInTheDocument();
    });

    it("offers reset only for an overridden field that has a default", () => {
        const onReset = vi.fn();
        render(
            <ThemeProvider>
                <SettingsFieldCard
                    group={TURN_SPEED_GROUP}
                    values={{ turn_speed_ratio: 0.6 }}
                    onChange={vi.fn()}
                    isOverridden={() => true}
                    hasDefault={() => true}
                    onReset={onReset}
                />
            </ThemeProvider>,
        );

        fireEvent.click(screen.getByRole("button", { name: /reset to default/i }));

        expect(onReset).toHaveBeenCalledWith("turn_speed_ratio");
    });
});
