import {DOCK_APPEARANCES, type DockAppearanceId} from "./mowerAppearances.ts";

const DOCK_APPEARANCE_MENU_KEY_PREFIX = "dockAppearance:";

/** Return a registered dock appearance id encoded in a toolbar menu key. */
export function parseDockAppearanceMenuKey(key: string): DockAppearanceId | undefined {
    if (!key.startsWith(DOCK_APPEARANCE_MENU_KEY_PREFIX)) return undefined;

    const requestedId = key.slice(DOCK_APPEARANCE_MENU_KEY_PREFIX.length);
    return Object.values(DOCK_APPEARANCES).find(({id}) => id === requestedId)?.id;
}
