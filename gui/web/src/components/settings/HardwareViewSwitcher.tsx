import {useMavrosInfo} from "../../hooks/useMavrosInfo.ts";
import {identityFromMavros} from "../../utils/fcuParamIdentity.ts";
import {FcuParamTool} from "./FcuParamTool.tsx";
import type {ReactNode} from "react";
import {useState} from "react";
import {Segmented, Space} from "antd";
import {useTranslation} from "react-i18next";
import type {HardwareBackend} from "../../constants/hardwareBackends.ts";

export function HardwareViewSwitcher({backend, chassis, fcu}: {
    backend: HardwareBackend;
    chassis: ReactNode;
    fcu: ReactNode;
}) {
    const {t} = useTranslation();
    const [view, setView] = useState<"chassis" | "fcu" | "parameters">("chassis");
    const mavros = useMavrosInfo(backend === "mavros");
    const firmware = identityFromMavros(mavros).family;
    const parameterLabel = firmware === "ardupilot" ? "ArduPilot" : firmware === "px4" ? "PX4" : t("settingsFcuParams.genericTab");
    if (backend !== "mavros") return chassis;

    return <Space direction="vertical" size={16} style={{width: "100%"}}>
        <Segmented
            aria-label={t("settingsFcu.viewMode")}
            value={view}
            onChange={(value) => setView(value as "chassis" | "fcu" | "parameters")}
            options={[
                {value: "chassis", label: t("settingsFcu.chassisTab")},
                {value: "fcu", label: t("settingsFcu.fcuTab")},
                {value: "parameters", label: parameterLabel},
            ]}
        />
        {view === "chassis" ? chassis : view === "fcu" ? fcu : <FcuParamTool info={mavros}/>}
    </Space>;
}
