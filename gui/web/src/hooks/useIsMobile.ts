import {useSyncExternalStore} from "react";

const MOBILE_QUERY = '(max-width: 767px)';

function subscribe(onChange: () => void) {
    const query = window.matchMedia(MOBILE_QUERY);
    query.addEventListener('change', onChange);
    return () => query.removeEventListener('change', onChange);
}

const getSnapshot = () => window.matchMedia(MOBILE_QUERY).matches;
const getServerSnapshot = () => false;

/** Media-query changes are external state; subscribe without a second mount-time setState. */
export function useIsMobile() {
    return useSyncExternalStore(subscribe, getSnapshot, getServerSnapshot);
}
