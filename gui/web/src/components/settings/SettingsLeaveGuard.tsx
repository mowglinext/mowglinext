import {useEffect} from "react";
import {useBlocker} from "react-router-dom";
import {Modal} from "antd";
import {useTranslation} from "react-i18next";

/** Section navigation keeps drafts alive; leaving the page needs a decision. */
export function SettingsLeaveGuard({dirty}: {dirty: boolean}) {
    const {t} = useTranslation();
    const blocker = useBlocker(({currentLocation, nextLocation}) =>
        dirty && currentLocation.pathname !== nextLocation.pathname);

    useEffect(() => {
        if (!dirty) return;
        const beforeUnload = (event: BeforeUnloadEvent) => {
            event.preventDefault();
            event.returnValue = "";
        };
        window.addEventListener("beforeunload", beforeUnload);
        return () => window.removeEventListener("beforeunload", beforeUnload);
    }, [dirty]);

    return <Modal
        open={blocker.state === "blocked"}
        title={t("settingsPage.leaveTitle")}
        okText={t("settingsPage.discardAndLeave")}
        cancelText={t("settingsPage.keepEditing")}
        okButtonProps={{danger: true}}
        onOk={() => { if (blocker.state === "blocked") blocker.proceed(); }}
        onCancel={() => { if (blocker.state === "blocked") blocker.reset(); }}
    >
        {t("settingsPage.leaveBody")}
    </Modal>;
}
