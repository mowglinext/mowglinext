export const MAP_ROTATION_LOCKED_KEY = "gui.map.display.rotation_locked";

export function mapRotationLocked(config: Record<string, unknown>): boolean {
    return config[MAP_ROTATION_LOCKED_KEY] === "true";
}
