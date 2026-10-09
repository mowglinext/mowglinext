import React from "react";
import { Tooltip } from "antd";
import { ApiOutlined } from "@ant-design/icons";
import { useNavigate } from "react-router-dom";
import { useTranslation } from "react-i18next";
import { useHardwareBackend } from "../hooks/useHardwareBackend.ts";
import { useThemeMode } from "../theme/ThemeContext.tsx";

/**
 * Which robot this GUI drives, at a glance: its name and the hardware backend
 * (Mowgli board, OpenMower v1, MAVROS). Opens the hardware settings, where the
 * backend's wiring lives.
 */
export function HardwareBackendBadge(): React.ReactElement | null {
    const { t } = useTranslation();
    const { colors } = useThemeMode();
    const navigate = useNavigate();
    const { backend, robotName, loading } = useHardwareBackend();
    if (loading) return null;

    const shortName = t(`settingsHardwareBackend.shortNames.${backend}`);
    const fullName = t(`settingsHardwareBackend.names.${backend}`);
    const label = robotName ? `${robotName} · ${shortName}` : shortName;

    return (
        <Tooltip title={t("settingsHardwareBackend.badgeTooltip", { name: fullName })}>
            <button
                type="button"
                data-testid="hardware-backend-badge"
                data-backend={backend}
                onClick={() => {
                    void navigate({ pathname: "/settings", search: "?section=hardware" });
                }}
                style={{
                    display: "inline-flex",
                    alignItems: "center",
                    gap: 6,
                    maxWidth: 220,
                    padding: "3px 10px",
                    borderRadius: 999,
                    border: `1px solid ${colors.borderSubtle}`,
                    background: "transparent",
                    color: colors.text,
                    fontSize: 12,
                    lineHeight: "18px",
                    cursor: "pointer",
                    whiteSpace: "nowrap",
                    overflow: "hidden",
                    textOverflow: "ellipsis",
                }}
            >
                <ApiOutlined aria-hidden />
                <span style={{ overflow: "hidden", textOverflow: "ellipsis" }}>{label}</span>
            </button>
        </Tooltip>
    );
}
