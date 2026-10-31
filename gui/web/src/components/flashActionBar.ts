import type {CSSProperties} from "react";
import type {getColors} from "../theme/colors.ts";

/**
 * `page` (default): the onboarding wizard step — the action bar is pinned to
 * the bottom of the screen and the form scrolls in a fixed-height pane.
 * `modal`: embedded in a dialog (Settings → Updates) — a fixed bar would
 * escape the dialog and float at the bottom of the viewport, so the actions
 * stay in the normal flow under a content pane that sizes to its content.
 */
export type FlashBoardVariant = "page" | "modal";

/** Style of the flash screens' action bar (config form and progress view). */
export const flashActionBarStyle = (
    inModal: boolean,
    isMobile: boolean,
    colors: ReturnType<typeof getColors>,
): CSSProperties => inModal ? {marginTop: 16} : {
    position: "fixed",
    bottom: isMobile ? "calc(56px + env(safe-area-inset-bottom, 0px))" : 20,
    left: isMobile ? 0 : undefined,
    right: isMobile ? 0 : undefined,
    padding: isMobile ? "8px 12px" : undefined,
    background: isMobile ? colors.bgCard : undefined,
    borderTop: isMobile ? `1px solid ${colors.border}` : undefined,
    zIndex: 50,
};
