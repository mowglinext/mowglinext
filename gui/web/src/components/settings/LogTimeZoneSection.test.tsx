import {beforeEach, describe, expect, it} from "vitest";
import {fireEvent, render, screen} from "@testing-library/react";
import {ThemeProvider} from "../../theme/ThemeContext.tsx";
import {TimeFormatProvider, useTimeFormat} from "../../hooks/useTimeFormat.tsx";
import {LOG_TIME_ZONE_STORAGE_KEY} from "../../utils/logTime.ts";
import {LogTimeZoneSection} from "./LogTimeZoneSection.tsx";

/** Probe that exposes the shared preference so the assertion is behavioural. */
function TimeZoneValue() {
    return <output data-testid="time-zone-value">{useTimeFormat().timeZoneMode}</output>;
}

describe("LogTimeZoneSection", () => {
    let storage: Map<string, string>;

    beforeEach(() => {
        storage = new Map<string, string>();
        Object.defineProperty(window, "localStorage", {
            configurable: true,
            value: {
                getItem: (key: string) => storage.get(key) ?? null,
                setItem: (key: string, value: string) => storage.set(key, value),
                removeItem: (key: string) => storage.delete(key),
                clear: () => storage.clear(),
            },
        });
    });

    function renderSection() {
        render(
            <ThemeProvider>
                <TimeFormatProvider>
                    <LogTimeZoneSection/>
                    <TimeZoneValue/>
                </TimeFormatProvider>
            </ThemeProvider>,
        );
    }

    it("offers both zones and defaults to local time", () => {
        // Arrange / Act
        renderSection();

        // Assert
        expect(screen.getByRole("radio", {name: /^Local/})).toBeChecked();
        expect(screen.getByRole("radio", {name: /^UTC/})).toBeInTheDocument();
        expect(screen.getByTestId("time-zone-value")).toHaveTextContent("local");
    });

    it("updates the shared preference when UTC is picked", () => {
        // Arrange
        renderSection();

        // Act
        fireEvent.click(screen.getByRole("radio", {name: /^UTC/}));

        // Assert
        expect(screen.getByTestId("time-zone-value")).toHaveTextContent("utc");
    });

    it("persists the choice to localStorage so it survives a reload", () => {
        // Arrange
        renderSection();

        // Act
        fireEvent.click(screen.getByRole("radio", {name: /^UTC/}));

        // Assert
        expect(storage.get(LOG_TIME_ZONE_STORAGE_KEY)).toBe("utc");
    });
});
