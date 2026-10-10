import {describe, expect, it, vi} from "vitest";
import type {Deployment, HostUpdater} from "../hooks/useHostUpdater.ts";
import {AGENT_RESTART_TIMEOUT_MS, agentUpgradeFor, upgradeAgentAndWait} from "./updaterAgent.ts";

const PLATFORM = "linux/arm64";

function release(id: string, publishedAt: string, build: string): Deployment {
    return {
        id, revision: id.padEnd(40, "0"), published_at: publishedAt,
        source: {repository: "mowglinext/mowglinext", track: "dev", branch: "dev"},
        updater: {[PLATFORM]: {version: `v-${build}`, build_id: build}},
    };
}

const OLD = release("old", "2026-10-01T00:00:00Z", "b-old");
const NEW = release("new", "2026-10-10T00:00:00Z", "b-new");
const agent = (build: string, error?: string): HostUpdater["agent"] =>
    ({version: `v-${build}`, build_id: build, revision: "r", platform: PLATFORM, error});

describe("agentUpgradeFor", () => {
    it("runs a newer release's updater before planning it", () => {
        expect(agentUpgradeFor(NEW, agent("b-old"), [NEW, OLD])).toBe(NEW);
    });

    it("does nothing when the release ships the running updater", () => {
        expect(agentUpgradeFor(NEW, agent("b-new"), [NEW, OLD])).toBeUndefined();
    });

    it("never moves the updater backwards to an older release", () => {
        expect(agentUpgradeFor(OLD, agent("b-new"), [NEW, OLD])).toBeUndefined();
    });

    it("replaces an installer-built updater that matches no release", () => {
        expect(agentUpgradeFor(NEW, agent("b-installer"), [NEW, OLD])).toBe(NEW);
    });

    it("does nothing for a release without an updater for this platform", () => {
        const noBinary = {...NEW, updater: {}};
        expect(agentUpgradeFor(noBinary, agent("b-old"), [noBinary, OLD])).toBeUndefined();
        expect(agentUpgradeFor(undefined, agent("b-old"), [OLD])).toBeUndefined();
    });
});

function io(states: (HostUpdater["agent"] | Error)[]) {
    let clock = 0;
    const calls: string[] = [];
    const request = vi.fn((operation: string) => {
        calls.push(operation);
        if (operation === "agent-update") return Promise.resolve({restarting: true});
        const next = states.length > 1 ? states.shift()! : states[0];
        if (next instanceof Error) return Promise.reject(next);
        return Promise.resolve({agent: next} as unknown);
    });
    return {
        calls,
        io: {
            request: request as <T>(operation: string, body?: unknown) => Promise<T>,
            sleep: (ms: number) => {clock += ms; return Promise.resolve();},
            now: () => clock,
        },
    };
}

describe("upgradeAgentAndWait", () => {
    it("waits through the restart until the new updater answers", async () => {
        const {io: deps, calls} = io([new Error("socket down"), agent("b-old"), agent("b-new")]);
        await upgradeAgentAndWait(NEW, agent("b-old"), deps);
        expect(calls).toEqual(["agent-update", "state", "state", "state"]);
    });

    it("fails with the launcher's reason when the new updater is rolled back", async () => {
        const {io: deps} = io([agent("b-old", "Candidate updater failed its health probe")]);
        await expect(upgradeAgentAndWait(NEW, agent("b-old"), deps)).rejects.toThrow("health probe");
    });

    it("ignores an error that was already there before", async () => {
        const {io: deps} = io([agent("b-old", "old failure"), agent("b-new", "old failure")]);
        await expect(upgradeAgentAndWait(NEW, agent("b-old", "old failure"), deps)).resolves.toBeUndefined();
    });

    it("gives up after the restart timeout", async () => {
        const {io: deps, calls} = io([agent("b-old")]);
        await expect(upgradeAgentAndWait(NEW, agent("b-old"), deps)).rejects.toThrow("did not restart");
        expect(calls.filter(c => c === "state").length).toBeLessThanOrEqual(AGENT_RESTART_TIMEOUT_MS / 2000);
    });
});
