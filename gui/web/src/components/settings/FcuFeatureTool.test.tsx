import {fireEvent, render, screen, waitFor} from "@testing-library/react";
import {beforeEach, describe, expect, it, vi} from "vitest";
import {FcuFeatureTool} from "./FcuFeatureTool.tsx";
import {requiredFcuFeatures} from "../../utils/fcuFeatureYaml.ts";

const storage = vi.hoisted(() => new Map<string, unknown>());
const createObjectURL = vi.hoisted(() => vi.fn(() => "blob:test"));
vi.mock("localforage", () => ({
    default: {
        getItem: vi.fn((key: string) => Promise.resolve(storage.get(key) ?? null)),
        setItem: vi.fn((key: string, value: unknown) => {
            storage.set(key, value);
            return Promise.resolve(value);
        }),
    },
}));

const yamlWith = (features: readonly string[]) => `vehicle: {id: copter, name: Copter}\nboard: {id: Different}\nversion: {name: 9.9}\nunknown: preserved\nselected_features:\n${features.map((feature) => `  - ${feature}`).join("\n")}\n`;

const upload = (contents: string, name = "input.yaml") => {
    const file = new File([contents], name, {type: "application/yaml"});
    Object.defineProperty(file, "text", {value: () => Promise.resolve(contents)});
    fireEvent.change(screen.getByLabelText("Import YAML", {selector: "input"}), {target: {files: [file]}});
};

describe("FcuFeatureTool", () => {
    beforeEach(() => {
        storage.clear();
        vi.stubGlobal("crypto", {randomUUID: () => "test-id"});
        vi.stubGlobal("URL", {
            createObjectURL,
            revokeObjectURL: vi.fn(),
        });
        vi.spyOn(HTMLAnchorElement.prototype, "click").mockImplementation(() => undefined);
    });

    it("imports, backs up and validates features independently of vehicle metadata", async () => {
        render(<FcuFeatureTool/>);
        upload(yamlWith([...requiredFcuFeatures, "USER_EXTRA"]));

        expect(await screen.findByText("All features required by MowgliNext are present.")).toBeInTheDocument();
        expect(screen.getByText("Copter (copter)")).toBeInTheDocument();
        expect(screen.getByText("○ additional")).toBeInTheDocument();
        expect(await screen.findByText("input.yaml")).toBeInTheDocument();
        expect(storage.size).toBe(1);
        expect(screen.getByText("Open the official ArduPilot builder").closest("a")).toHaveAttribute(
            "href", "https://custom.ardupilot.org/",
        );
    });

    it("adds one or all missing features without removing extras", async () => {
        const first = requiredFcuFeatures[0];
        const second = requiredFcuFeatures[1];
        render(<FcuFeatureTool/>);
        upload(yamlWith([...requiredFcuFeatures.slice(2), "USER_EXTRA"]));

        expect(await screen.findByText("2 features required by MowgliNext are missing.")).toBeInTheDocument();
        fireEvent.click(screen.getByRole("button", {name: `Add ${first}`}));
        expect(screen.getByText("1 feature required by MowgliNext is missing.")).toBeInTheDocument();
        fireEvent.click(screen.getByRole("button", {name: "Add all missing MowgliNext features"}));
        expect(screen.getByText("All features required by MowgliNext are present.")).toBeInTheDocument();
        expect(screen.getByText("USER_EXTRA")).toBeInTheDocument();
        expect(screen.getByText(`+ ${second} — added to satisfy the MowgliNext minimum`)).toBeInTheDocument();
    });

    it("warns after explicit removal and makes a removed requirement missing", async () => {
        render(<FcuFeatureTool/>);
        upload(yamlWith([...requiredFcuFeatures, "USER_EXTRA"]));
        await screen.findByText("All features required by MowgliNext are present.");

        fireEvent.click(screen.getByRole("button", {name: "Remove USER_EXTRA"}));
        expect(screen.getByText("Features from the original configuration have been removed.")).toBeInTheDocument();
        expect(screen.getByText("− removed by user")).toBeInTheDocument();

        fireEvent.click(screen.getByRole("button", {name: `Remove ${requiredFcuFeatures[0]}`}));
        expect(screen.getByText("1 feature required by MowgliNext is missing.")).toBeInTheDocument();
    });

    it("keeps the previous working configuration after an invalid import", async () => {
        render(<FcuFeatureTool/>);
        upload(yamlWith(requiredFcuFeatures));
        await screen.findByText("All features required by MowgliNext are present.");
        upload("vehicle: rover\n");

        expect(await screen.findByText("YAML could not be imported")).toBeInTheDocument();
        expect(screen.getByText("All features required by MowgliNext are present.")).toBeInTheDocument();
        await waitFor(() => expect(screen.getAllByText("input.yaml").length).toBeGreaterThan(0));
    });

    it("downloads and restores the immutable original backup", async () => {
        const missingFeature = requiredFcuFeatures[0];
        render(<FcuFeatureTool/>);
        upload(yamlWith(requiredFcuFeatures.slice(1)), "original.yaml");
        await screen.findByText("1 feature required by MowgliNext is missing.");
        fireEvent.click(screen.getByRole("button", {name: `Add ${missingFeature}`}));
        expect(screen.getByText("All features required by MowgliNext are present.")).toBeInTheDocument();

        fireEvent.click(screen.getByRole("button", {name: "Download"}));
        expect(createObjectURL).toHaveBeenCalledOnce();
        fireEvent.click(screen.getByRole("button", {name: "Restore as working configuration"}));
        expect(screen.getByText("1 feature required by MowgliNext is missing.")).toBeInTheDocument();
    });
});
