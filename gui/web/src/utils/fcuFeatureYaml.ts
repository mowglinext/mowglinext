import {Document, parseDocument} from "yaml";
import minimumProfileYaml from "../profiles/ardurover_4.7.1light_1.3_mowglinext.yaml?raw";

export type FcuYamlInfo = {
    vehicle?: string;
    board?: string;
    version?: string;
};

export type FcuYamlConfiguration = {
    document: Document;
    features: string[];
    info: FcuYamlInfo;
};

const displayIdentity = (value: unknown): string | undefined => {
    if (!value || typeof value !== "object") return undefined;
    const record = value as Record<string, unknown>;
    const scalar = (candidate: unknown) => typeof candidate === "string" || typeof candidate === "number"
        ? String(candidate)
        : undefined;
    const name = scalar(record.name);
    const id = scalar(record.id);
    return name && id && name !== id ? `${name} (${id})` : name ?? id;
};

export const parseFcuYaml = (source: string): FcuYamlConfiguration => {
    const document = parseDocument(source, {keepSourceTokens: true});
    if (document.errors.length > 0) throw new Error(document.errors[0].message);
    const data = document.toJS() as unknown;
    if (!data || typeof data !== "object" || Array.isArray(data)) throw new Error("YAML root must be a mapping");
    const record = data as Record<string, unknown>;
    const features = record.selected_features;
    if (!Array.isArray(features)) throw new Error("selected_features must be a YAML list");
    if (features.some((feature) => typeof feature !== "string" || feature.length === 0)) {
        throw new Error("selected_features must contain only non-empty feature names");
    }
    if (new Set(features as string[]).size !== features.length) {
        throw new Error("selected_features must not contain duplicates");
    }

    return {
        document,
        features: features as string[],
        info: {
            vehicle: displayIdentity(record.vehicle),
            board: displayIdentity(record.board),
            version: displayIdentity(record.version),
        },
    };
};

export const exportFcuYaml = (configuration: FcuYamlConfiguration, features: string[]): string => {
    const document = configuration.document.clone();
    document.set("selected_features", features);
    return String(document);
};

export const requiredFcuFeatures = Object.freeze(parseFcuYaml(minimumProfileYaml).features);
