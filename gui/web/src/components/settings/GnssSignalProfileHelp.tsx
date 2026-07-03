import React from "react";
import { Typography } from "antd";
import { useTranslation } from "react-i18next";
import {
    GNSS_SIGNAL_PROFILE_CUSTOM_HELP_TEXT,
    GNSS_SIGNAL_PROFILE_HELP_TEXT,
    gnssSignalProfileDescription,
    gnssSignalProfileLabel,
    normalizeGnssSignalProfile,
} from "./gnssConfig.ts";

const { Text } = Typography;

type Props = {
    selectedProfile: unknown;
};

export const GnssSignalProfileHelp: React.FC<Props> = ({ selectedProfile }) => {
    const { t } = useTranslation();
    const normalized = normalizeGnssSignalProfile(selectedProfile);
    const selectedLabel = gnssSignalProfileLabel(normalized);
    const selectedDescription = gnssSignalProfileDescription(normalized);

    return (
        <div style={{ marginTop: 2 }}>
            <Text type="secondary" style={{ display: "block", fontSize: 12 }}>
                {t(GNSS_SIGNAL_PROFILE_HELP_TEXT)}
            </Text>
            <Text type="secondary" style={{ display: "block", fontSize: 12, marginTop: 4 }}>
                <Text strong style={{ fontSize: 12 }}>
                    {t(selectedLabel)}:
                </Text>{" "}
                {t(selectedDescription)}
            </Text>
            {normalized === "custom" && (
                <Text type="secondary" style={{ display: "block", fontSize: 12, marginTop: 4 }}>
                    {t(GNSS_SIGNAL_PROFILE_CUSTOM_HELP_TEXT)}
                </Text>
            )}
        </div>
    );
};
