import { App } from "antd";
import { render, screen } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import React, { useState } from "react";
import { describe, expect, it, vi } from "vitest";
import { UniversalGnssAdvancedSettings } from "./UniversalGnssAdvancedSettings.tsx";

const renderSignalGroupField = (initialValue = "") => {
    const onChangeSpy = vi.fn();

    const Wrapper: React.FC = () => {
        const [values, setValues] = useState<Record<string, any>>({
            gnss_signal_group: initialValue,
        });

        return (
            <App>
                <UniversalGnssAdvancedSettings
                    receiverFamily="unicore"
                    values={values}
                    onChange={(key, value) => {
                        setValues((prev) => ({ ...prev, [key]: value }));
                        onChangeSpy(key, value);
                    }}
                />
            </App>
        );
    };

    return {
        onChangeSpy,
        ...render(<Wrapper />),
    };
};

describe("UniversalGnssAdvancedSettings", () => {
    it("lets operators type a space-separated SIGNALGROUP without collapsing it into 36", async () => {
        const user = userEvent.setup();
        const { onChangeSpy } = renderSignalGroupField();

        const input = screen.getByPlaceholderText("e.g. 3 6");
        await user.type(input, "3 6");

        expect(input).toHaveValue("3 6");
        expect(onChangeSpy).toHaveBeenCalledWith("gnss_signal_group", "3 ");
        expect(onChangeSpy).toHaveBeenLastCalledWith("gnss_signal_group", "3 6");
    });
});
