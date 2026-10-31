import {useSyncExternalStore} from "react";
import {Joystick} from "react-joystick-component";
import {IJoystickUpdateEvent} from "react-joystick-component/build/lib/Joystick";
import {CheckOutlined, CloseOutlined, HomeOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import AsyncButton from "../../../components/AsyncButton.tsx";
import {useThemeMode} from "../../../theme/ThemeContext.tsx";
import {limeAlpha} from "../../../theme/colors.ts";

const SHORT_SCREEN_QUERY = "(max-height: 500px) and (min-width: 600px)";
const shortScreen = () => window.matchMedia(SHORT_SCREEN_QUERY).matches;
const subscribeShortScreen = (onChange: () => void) => {
    const media = window.matchMedia(SHORT_SCREEN_QUERY);
    media.addEventListener("change", onChange);
    return () => media.removeEventListener("change", onChange);
};

interface JoystickOverlayProps {
    visible: boolean;
    isRecording?: boolean;
    mobile?: boolean;
    desktopBottom?: number;
    onStart?: () => void;
    onMove: (event: IJoystickUpdateEvent) => void;
    onStop: () => void;
    onFinishRecording?: () => Promise<void>;
    onCancelRecording?: () => Promise<void>;
    onHome?: () => Promise<void>;
}

export const JoystickOverlay = ({
    visible, isRecording, mobile, desktopBottom = 88,
    onStart, onMove, onStop, onFinishRecording, onCancelRecording, onHome,
}: JoystickOverlayProps) => {
    const {colors, displayMode} = useThemeMode();
    const {t} = useTranslation();
    const compact = useSyncExternalStore(subscribeShortScreen, shortScreen, () => false);
    if (!visible) return null;

    const size = compact ? 82 : mobile ? 132 : 110;

    // The default react-joystick-component look is a flat grey puck. We dress
    // it up with a glassy base ring + a glowing brand-green stick, sized up for
    // thumbs. Tokenised from the palette (lime/mint/emerald + deep bg).
    const baseColor = `radial-gradient(circle at 50% 40%, ${limeAlpha(0.10)}, ${colors.bgBase} 70%)`;
    const stickColor = `radial-gradient(circle at 38% 32%, ${colors.primaryLight}, ${colors.mint} 55%, ${colors.emeraldDeep})`;

    // Mobile: bottom-left (natural left thumb), lifted clear of the centred
    // toolbar + bottom-nav. The toolbar pins itself at safe-area + 100px and is
    // ~60px tall, so the control stack starts above that band rather than at a
    // magic offset tuned by hand. Desktop: above its measured toolbar, left of the sidebar.
    const TOOLBAR_BAND_PX = 100 + 60 + 12; // toolbar bottom + height + gap
    const anchor: React.CSSProperties = mobile
        ? {left: 18, bottom: `calc(env(safe-area-inset-bottom, 0px) + ${TOOLBAR_BAND_PX}px)`}
        : {left: 16, bottom: desktopBottom};

    // A single flex "control stack" so the Finish/Cancel/Home buttons derive
    // their position from the joystick row instead of their own magic offsets.
    const actionButtonStyle: React.CSSProperties = {height: 44, minWidth: 44, borderRadius: 10, fontWeight: 600};

    return (
        <div style={{
            // Mobile controls follow the viewport above navigation. Desktop
            // controls share the map container with its measured toolbar.
            position: mobile ? "fixed" : "absolute",
            ...anchor,
            zIndex: 60,
            display: "flex",
            alignItems: "flex-end",
            gap: 12,
            // Don't let dragging the stick pan/scroll the map underneath.
            touchAction: "none",
        }}>
            <div style={{
                position: "relative",
                padding: 6,
                borderRadius: "50%",
                background: colors.glassBackground,
                border: `1px solid ${limeAlpha(0.28)}`,
                boxShadow: `0 10px 30px -10px rgba(0,0,0,0.6), inset 0 0 24px ${limeAlpha(0.06)}`,
                backdropFilter: displayMode === 'visual' ? 'blur(8px)' : undefined,
                WebkitBackdropFilter: displayMode === 'visual' ? 'blur(8px)' : undefined,
            }}>
                <Joystick
                    size={size}
                    baseColor={baseColor}
                    stickColor={stickColor}
                    stickSize={Math.max(44, Math.round(size * 0.42))}
                    start={onStart}
                    move={onMove}
                    stop={onStop}
                    throttle={50}
                />
            </div>

            {isRecording && (
                <div style={{display: "flex", flexDirection: compact ? "row" : "column", gap: 8, marginBottom: 4}}>
                    <AsyncButton
                        type="primary"
                        icon={<CheckOutlined/>}
                        onAsyncClick={onFinishRecording!}
                        style={actionButtonStyle}
                    >
                        {t('mapJoystick.finish')}
                    </AsyncButton>
                    <AsyncButton
                        danger
                        icon={<CloseOutlined/>}
                        onAsyncClick={onCancelRecording!}
                        style={actionButtonStyle}
                    >
                        {t('mapJoystick.cancel')}
                    </AsyncButton>
                    <AsyncButton
                        icon={<HomeOutlined/>}
                        onAsyncClick={onHome!}
                        style={actionButtonStyle}
                    >
                        {t('mapJoystick.home')}
                    </AsyncButton>
                </div>
            )}
        </div>
    );
};
