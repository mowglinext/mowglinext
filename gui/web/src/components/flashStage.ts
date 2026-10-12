/** Mirrors the backend's types.FlashStageEvent (the `stage` SSE event). */
export type FlashStage = {
    stages: string[];
    current: number;
};

/** Validates a `stage` event payload; anything malformed yields null. */
export const parseFlashStage = (data: string): FlashStage | null => {
    try {
        const parsed: unknown = JSON.parse(data);
        if (typeof parsed !== "object" || parsed === null) return null;
        const {stages, current} = parsed as Record<string, unknown>;
        if (!Array.isArray(stages) || !stages.every((s) => typeof s === "string")) return null;
        if (typeof current !== "number" || current < 0 || current >= stages.length) return null;
        return {stages, current};
    } catch {
        return null;
    }
};

/** Whole-stage granularity: the bar advances as each stage is entered. */
export const flashPercent = (stage: FlashStage | null, done: boolean): number => {
    if (done) return 100;
    if (!stage || stage.stages.length === 0) return 0;
    return Math.round((stage.current / stage.stages.length) * 100);
};
