import {useEffect, useState} from "react";

/** Read the current visibility, tolerating a non-browser/jsdom-less host. */
const readVisible = (): boolean =>
    typeof document === "undefined" || document.visibilityState !== "hidden";

/**
 * useDocumentVisible — true while this tab is in the foreground.
 *
 * Gate for expensive live subscriptions. A backgrounded tab still holds its
 * websocket open, so foxglove_bridge keeps serialising and the backend keeps
 * forwarding at full rate for a page nobody is looking at — measured 207 kB/s
 * for the mow-progress grid alone (351 kB per message at ~0.6 Hz).
 *
 * Deliberately only reports `hidden` (tab switched away, window minimised,
 * phone screen off). A merely unfocused but visible window keeps its
 * subscriptions: a second monitor showing the dashboard is still being read.
 */
export const useDocumentVisible = (): boolean => {
    const [visible, setVisible] = useState(readVisible);

    useEffect(() => {
        if (typeof document === "undefined") return;
        const onChange = () => setVisible(readVisible());
        // Re-read on mount: visibility may have changed between the initial
        // useState and this effect running.
        onChange();
        document.addEventListener("visibilitychange", onChange);
        return () => document.removeEventListener("visibilitychange", onChange);
    }, []);

    return visible;
};
