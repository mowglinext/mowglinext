import React from "react";
import { Badge, Menu, Tabs } from "antd";
import {
    AimOutlined,
    CloudOutlined,
    CodeOutlined,
    CompassOutlined,
    HomeOutlined,
    NodeIndexOutlined,
    SafetyOutlined,
    ScissorOutlined,
    ThunderboltOutlined,
    ToolOutlined,
    GlobalOutlined,
    WifiOutlined,
} from "@ant-design/icons";
import { useIsMobile } from "../../hooks/useIsMobile.ts";
import { useThemeMode } from "../../theme/ThemeContext.tsx";
import { SettingsSection, SectionMeta } from "../../hooks/useSettingsManager.ts";

const SECTION_ICONS: Record<string, React.ReactNode> = {
    tool: <ToolOutlined />,
    global: <GlobalOutlined />,
    wifi: <WifiOutlined />,
    aim: <AimOutlined />,
    "node-index": <NodeIndexOutlined />,
    scissor: <ScissorOutlined />,
    home: <HomeOutlined />,
    thunderbolt: <ThunderboltOutlined />,
    safety: <SafetyOutlined />,
    compass: <CompassOutlined />,
    cloud: <CloudOutlined />,
    code: <CodeOutlined />,
};

type Props = {
    sections: SectionMeta[];
    activeSection: SettingsSection;
    onSectionChange: (section: SettingsSection) => void;
    isSectionDirty: (section: SettingsSection) => boolean;
};

export const SettingsNav: React.FC<Props> = ({
    sections,
    activeSection,
    onSectionChange,
    isSectionDirty,
}) => {
    const isMobile = useIsMobile();
    const { colors } = useThemeMode();

    if (isMobile) {
        return (
            <Tabs
                activeKey={activeSection}
                onChange={(key) => onSectionChange(key as SettingsSection)}
                size="small"
                tabBarStyle={{ marginBottom: 12, paddingLeft: 4, paddingRight: 4 }}
                items={sections.map((section) => ({
                    key: section.id,
                    label: (
                        <Badge dot={isSectionDirty(section.id)} offset={[4, 0]}>
                            {SECTION_ICONS[section.icon]}
                            <span style={{ marginLeft: 4, fontSize: 12 }}>{section.label}</span>
                        </Badge>
                    ),
                }))}
            />
        );
    }

    return (
        <Menu
            mode="inline"
            selectedKeys={[activeSection]}
            onClick={({ key }) => onSectionChange(key as SettingsSection)}
            style={{
                border: "none",
                background: "transparent",
                position: "sticky",
                top: 0,
            }}
            items={sections.map((section) => ({
                key: section.id,
                icon: SECTION_ICONS[section.icon],
                label: (
                    <span style={{ display: "flex", alignItems: "center", gap: 6 }}>
                        {section.label}
                        {isSectionDirty(section.id) && (
                            <span
                                style={{
                                    width: 6,
                                    height: 6,
                                    borderRadius: "50%",
                                    background: colors.accent,
                                    display: "inline-block",
                                }}
                            />
                        )}
                    </span>
                ),
            }))}
        />
    );
};
