import {Joystick} from "react-joystick-component";
import {IJoystickUpdateEvent} from "react-joystick-component/build/lib/Joystick";
import {CheckOutlined, CloseOutlined, HomeOutlined} from "@ant-design/icons";
import AsyncButton from "../../../components/AsyncButton.tsx";

interface JoystickOverlayProps {
    visible: boolean;
    isRecording?: boolean;
    mobile?: boolean;
    onMove: (event: IJoystickUpdateEvent) => void;
    onStop: () => void;
    onFinishRecording?: () => Promise<void>;
    onCancelRecording?: () => Promise<void>;
    onHome?: () => Promise<void>;
}

// The default react-joystick-component look is a flat grey puck. We dress it up
// with a glassy base ring + a glowing lime stick, and size it up for thumbs.
const BASE_COLOR = "radial-gradient(circle at 50% 40%, rgba(124,255,178,0.10), rgba(2,17,13,0.55) 70%)";
const STICK_COLOR = "radial-gradient(circle at 38% 32%, #BFFFD8, #45D688 55%, #2BAA66)";

export const JoystickOverlay = ({
    visible, isRecording, mobile,
    onMove, onStop, onFinishRecording, onCancelRecording, onHome,
}: JoystickOverlayProps) => {
    if (!visible) return null;

    const size = mobile ? 132 : 110;

    // Mobile: bottom-left (natural left thumb) and lifted clear of the centered
    // toolbar + bottom-nav. Desktop: bottom-right corner.
    const anchor: React.CSSProperties = mobile
        ? {left: 18, bottom: "calc(env(safe-area-inset-bottom, 0px) + 172px)"}
        : {right: 30, bottom: 30};

    return (
        <div style={{
            // Fixed to the viewport (not the bleeding map container) so it lands
            // reliably above the nav on iOS Safari — same reason as the toolbar.
            position: "fixed",
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
                background: "rgba(2,17,13,0.35)",
                border: "1px solid rgba(124,255,178,0.28)",
                boxShadow: "0 10px 30px -10px rgba(0,0,0,0.6), inset 0 0 24px rgba(124,255,178,0.06)",
                backdropFilter: "blur(8px)",
            }}>
                <Joystick
                    size={size}
                    baseColor={BASE_COLOR}
                    stickColor={STICK_COLOR}
                    stickSize={Math.round(size * 0.42)}
                    move={onMove}
                    stop={onStop}
                    throttle={50}
                />
            </div>

            {isRecording && (
                <div style={{display: "flex", flexDirection: "column", gap: 8, marginBottom: 4}}>
                    <AsyncButton
                        type="primary"
                        icon={<CheckOutlined/>}
                        onAsyncClick={onFinishRecording!}
                        style={{height: 44, borderRadius: 10, fontWeight: 600}}
                    >
                        Finish
                    </AsyncButton>
                    <AsyncButton
                        danger
                        icon={<CloseOutlined/>}
                        onAsyncClick={onCancelRecording!}
                        style={{height: 44, borderRadius: 10}}
                    >
                        Cancel
                    </AsyncButton>
                    <AsyncButton
                        icon={<HomeOutlined/>}
                        onAsyncClick={onHome!}
                        style={{height: 44, borderRadius: 10}}
                    >
                        Home
                    </AsyncButton>
                </div>
            )}
        </div>
    );
};
