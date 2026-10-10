import type {MowerStyle} from "../../hooks/useMowerVisual";

type Shell = {size: [number, number]; top: [number, number, number, number]; side: [number, number, number, number]; topRotation?: number};
/** Visible shell bounds in source pixels. Padding is deliberately excluded from
 * physical scaling. Both opacity modes use these exact same atlas regions. */
export const SHELLS: Record<MowerStyle, Shell> = {
    rounded: {size: [1774,887], top: [75,63,620,788], side: [911,339,825,284]},
    sculpted: {size: [1774,887], top: [106,32,629,828], side: [795,301,929,338]},
    utility: {size: [1774,887], top: [117,48,607,816], side: [831,356,874,261]},
    yardforce: {size: [1774,887], top: [113,29,629,818], side: [852,274,857,371]},
};

