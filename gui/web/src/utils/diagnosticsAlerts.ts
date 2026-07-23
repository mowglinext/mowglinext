/**
 * Group ROS diagnostics alerts by source component so a flood of related
 * warnings (e.g. 8 universal_gnss NTRIP messages) renders as one collapsible
 * row instead of a page-long stack.
 */

export interface DiagnosticAlertLike {
    level: number;
    name?: string;
    message?: string;
}

export interface AlertGroup<T extends DiagnosticAlertLike = DiagnosticAlertLike> {
    component: string;
    /** Highest-severity level in the group (2 error > 1 warn > 3 stale). */
    worstLevel: number;
    items: T[];
}

/** ROS diagnostic levels ranked by severity (error > warn > stale > ok). */
const LEVEL_SEVERITY: Record<number, number> = {2: 3, 1: 2, 3: 1, 0: 0};

export function alertSeverity(level: number): number {
    return LEVEL_SEVERITY[level] ?? 0;
}

export function groupAlertsByComponent<T extends DiagnosticAlertLike>(alerts: T[]): AlertGroup<T>[] {
    const groups = new Map<string, AlertGroup<T>>();
    for (const alert of alerts) {
        const component = (alert.name ?? "").split("/")[0] || "unknown";
        const existing = groups.get(component);
        if (existing === undefined) {
            groups.set(component, {component, worstLevel: alert.level, items: [alert]});
        } else {
            groups.set(component, {
                component,
                worstLevel: alertSeverity(alert.level) > alertSeverity(existing.worstLevel)
                    ? alert.level
                    : existing.worstLevel,
                items: [...existing.items, alert],
            });
        }
    }
    return [...groups.values()].sort(
        (a, b) => alertSeverity(b.worstLevel) - alertSeverity(a.worstLevel),
    );
}
