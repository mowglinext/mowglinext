import {sameUpdaterBuild} from "../hooks/useHostUpdater.ts";
import type {Deployment, HostUpdater} from "../hooks/useHostUpdater.ts";

type Agent = HostUpdater["agent"];

/**
 * The release whose host updater must run before `target` is planned, or
 * undefined. The installed updater is the one that checks a release, so a
 * check it gets wrong blocks the very release that fixes it (2026-10-10: an
 * OpenMower robot refused every update over its /dev mount until the updater
 * was updated by hand). Only ever moves FORWARD: an older updater may not read
 * the newer one's journal, so a target published before the running updater's
 * own release keeps the running one. A running updater from no known release
 * (installer build) is replaced by any release that ships one.
 */
export function agentUpgradeFor(target: Deployment | undefined, agent: Agent, versions: Deployment[]): Deployment | undefined {
    const candidate = target?.updater[agent.platform];
    if (!target || !candidate || sameUpdaterBuild(candidate, agent)) return undefined;
    const running = versions.find(r => {
        const binary = r.updater[agent.platform];
        return !!binary && sameUpdaterBuild(binary, agent);
    });
    if (running && Date.parse(target.published_at) <= Date.parse(running.published_at)) return undefined;
    return target;
}

export interface AgentUpgradeIO {
    request: <T>(operation: string, body?: unknown) => Promise<T>;
    sleep: (ms: number) => Promise<void>;
    now: () => number;
}

export const AGENT_RESTART_TIMEOUT_MS = 120_000;
export const AGENT_POLL_MS = 2_000;

/**
 * Install `release`'s updater and wait until it answers. The launcher starts
 * the new worker and keeps the previous one if it fails its health probe,
 * reporting why in agent.error: a NEW error, or no switch before the timeout,
 * is a failure. The socket is down while the worker restarts; those polls
 * simply retry.
 */
export async function upgradeAgentAndWait(release: Deployment, agent: Agent, io: AgentUpgradeIO): Promise<void> {
    const wanted = release.updater[agent.platform];
    if (!wanted) throw new Error("This release has no updater for this platform");
    const previousError = agent.error ?? "";
    await io.request("agent-update", {deployment: release.id});
    const deadline = io.now() + AGENT_RESTART_TIMEOUT_MS;
    while (io.now() < deadline) {
        await io.sleep(AGENT_POLL_MS);
        let state: HostUpdater;
        try {
            state = await io.request<HostUpdater>("state");
        } catch {
            continue;  // the worker is restarting
        }
        if (sameUpdaterBuild(wanted, state.agent)) return;
        if (state.agent.error && state.agent.error !== previousError) throw new Error(state.agent.error);
    }
    throw new Error("The update service did not restart in time; the previous one is still running");
}
