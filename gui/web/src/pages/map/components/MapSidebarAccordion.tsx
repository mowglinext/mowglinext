import type {ReactNode} from "react";
import {DownOutlined, RightOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import {useThemeMode} from "../../../theme/ThemeContext.tsx";

export interface SidebarSection {
    /// Stable id: the value `openKey` holds while this section is open.
    key: string;
    title: string;
    /// Shown muted next to the title, e.g. an item count.
    badge?: string | number;
    /// Extra control on the right of the header (an info tooltip, say). It is a
    /// SIBLING of the toggle button, never inside it — nested interactive
    /// elements are invalid HTML and a click on it must not fold the section.
    extra?: ReactNode;
    content: ReactNode;
}

interface MapSidebarAccordionProps {
    sections: SidebarSection[];
    /// The one open section, or null when all are folded. Controlled by the
    /// parent so it can open a section on its own (e.g. when a corridor is being
    /// drawn and its Finish/Cancel buttons live in a folded section).
    openKey: string | null;
    onOpenChange: (key: string | null) => void;
}

/// The map's right-hand sidebar on desktop: a stack of collapsible sections where
/// at most ONE is open. Opening a section closes the one that was open; clicking
/// the open section folds it again. Only the open section's content is rendered,
/// and it scrolls inside the sidebar when it is taller than the space left.
export const MapSidebarAccordion = ({sections, openKey, onOpenChange}: MapSidebarAccordionProps) => {
    const {colors} = useThemeMode();
    const {t} = useTranslation();

    return (
        <div style={{display: 'flex', flexDirection: 'column', minHeight: 0, flex: '1 1 auto'}}>
            {sections.map((section, index) => {
                const open = section.key === openKey;
                const headerId = `map-sidebar-header-${section.key}`;
                const regionId = `map-sidebar-region-${section.key}`;
                return (
                    <div
                        key={section.key}
                        style={{
                            display: 'flex',
                            flexDirection: 'column',
                            minHeight: 0,
                            flex: open ? '1 1 auto' : '0 0 auto',
                            borderTop: index === 0 ? undefined : `1px solid ${colors.borderSubtle}`,
                        }}
                    >
                        <div style={{display: 'flex', alignItems: 'center', gap: 6, paddingRight: 12}}>
                            <button
                                type="button"
                                id={headerId}
                                aria-expanded={open}
                                aria-controls={regionId}
                                aria-label={`${section.title} — ${open ? t('mapSidebar.collapse') : t('mapSidebar.expand')}`}
                                onClick={() => onOpenChange(open ? null : section.key)}
                                style={{
                                    flex: 1,
                                    minWidth: 0,
                                    display: 'flex',
                                    alignItems: 'center',
                                    gap: 8,
                                    padding: '10px 12px',
                                    background: 'transparent',
                                    border: 0,
                                    cursor: 'pointer',
                                    textAlign: 'left',
                                    fontSize: 12,
                                    fontWeight: 600,
                                    color: open ? colors.text : colors.muted,
                                    textTransform: 'uppercase',
                                    letterSpacing: '0.05em',
                                }}
                            >
                                <span aria-hidden="true" style={{fontSize: 10, width: 10, flexShrink: 0}}>
                                    {open ? <DownOutlined/> : <RightOutlined/>}
                                </span>
                                <span style={{overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap'}}>
                                    {section.title}
                                </span>
                                {section.badge !== undefined && (
                                    <span style={{fontWeight: 500, color: colors.muted, textTransform: 'none', letterSpacing: 0}}>
                                        {section.badge}
                                    </span>
                                )}
                            </button>
                            {section.extra}
                        </div>
                        {open && (
                            <div
                                id={regionId}
                                role="region"
                                aria-labelledby={headerId}
                                className="scrollbar-thin"
                                style={{overflowY: 'auto', minHeight: 0, flex: '1 1 auto'}}
                            >
                                {section.content}
                            </div>
                        )}
                    </div>
                );
            })}
        </div>
    );
};
