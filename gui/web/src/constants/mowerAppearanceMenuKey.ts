import {MOWER_APPEARANCES, type MowerAppearanceId} from "./mowerAppearances.ts";

const MOWER_APPEARANCE_MENU_KEY_PREFIX = "mowerAppearance:";

/** Return a registered appearance id encoded in a toolbar menu key. */
export function parseMowerAppearanceMenuKey(key: string): MowerAppearanceId | undefined {
    if (!key.startsWith(MOWER_APPEARANCE_MENU_KEY_PREFIX)) return undefined;

    const requestedId = key.slice(MOWER_APPEARANCE_MENU_KEY_PREFIX.length);
    return Object.values(MOWER_APPEARANCES).find(({id}) => id === requestedId)?.id;
}
