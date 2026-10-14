import {useState} from "react";

function isIOSSafari(): boolean {
    if (typeof navigator === "undefined") return false;
    const ua = navigator.userAgent;
    const isIOS = /iPad|iPhone|iPod/.test(ua) || (navigator.platform === "MacIntel" && navigator.maxTouchPoints > 1);
    const isSafari = /Safari/.test(ua) && !/CriOS|FxiOS|OPiOS|EdgiOS/.test(ua);
    return isIOS && isSafari;
}

function isStandalone(): boolean {
    if (typeof window === "undefined") return false;
    return (
        ("standalone" in window.navigator && (window.navigator as any).standalone === true) ||
        window.matchMedia("(display-mode: standalone)").matches
    );
}

const DISMISSED_KEY = "pwa-install-dismissed";

export function useIOSInstallPrompt() {
    const [showPrompt, setShowPrompt] = useState(() => {
        if (!isIOSSafari() || isStandalone()) return false;
        return !sessionStorage.getItem(DISMISSED_KEY);
    });

    const dismiss = () => {
        setShowPrompt(false);
        sessionStorage.setItem(DISMISSED_KEY, "1");
    };

    return {showPrompt, dismiss};
}
