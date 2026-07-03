import {CloseOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import {useThemeMode} from "../theme/ThemeContext.tsx";

interface IOSInstallBannerProps {
    onDismiss: () => void;
}

const ShareIcon = () => (
    <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" style={{verticalAlign: 'middle'}}>
        <path d="M4 12v8a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2v-8"/>
        <polyline points="16 6 12 2 8 6"/>
        <line x1="12" y1="2" x2="12" y2="15"/>
    </svg>
);

export const IOSInstallBanner = ({onDismiss}: IOSInstallBannerProps) => {
    const {colors} = useThemeMode();
    const {t} = useTranslation();
    return (<div style={{
        position: 'fixed',
        bottom: 'calc(56px + env(safe-area-inset-bottom, 0px) + 8px)',
        left: 12,
        right: 12,
        zIndex: 300,
        background: colors.glassBackground,
        backdropFilter: 'blur(16px) saturate(180%)',
        WebkitBackdropFilter: 'blur(16px) saturate(180%)',
        borderRadius: 14,
        border: colors.glassBorder,
        boxShadow: colors.glassShadow,
        padding: '14px 16px',
        display: 'flex',
        alignItems: 'flex-start',
        gap: 12,
    }}>
        <div style={{
            width: 40,
            height: 40,
            borderRadius: 10,
            background: colors.primaryBg,
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            flexShrink: 0,
            fontSize: 20,
        }}>
            🤖
        </div>
        <div style={{flex: 1, minWidth: 0}}>
            <div style={{fontWeight: 600, fontSize: 14, color: colors.text, marginBottom: 4}}>
                {t('iosInstallBanner.title')}
            </div>
            <div style={{fontSize: 12, color: colors.muted, lineHeight: 1.4}}>
                {t('iosInstallBanner.tapPrefix')} <ShareIcon/> {t('iosInstallBanner.then')} <strong style={{color: colors.text}}>{t('iosInstallBanner.addToHomeScreen')}</strong> {t('iosInstallBanner.suffix')}
            </div>
        </div>
        <button
            onClick={onDismiss}
            aria-label={t('iosInstallBanner.dismiss')}
            style={{
                background: 'none',
                border: 'none',
                color: colors.muted,
                fontSize: 14,
                padding: 4,
                cursor: 'pointer',
                flexShrink: 0,
            }}
        >
            <CloseOutlined/>
        </button>
    </div>
    );
};
