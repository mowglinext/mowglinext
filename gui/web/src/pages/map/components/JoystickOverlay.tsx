import {Joystick} from "react-joystick-component";
import {IJoystickUpdateEvent} from "react-joystick-component/build/lib/Joystick";
import {CheckOutlined, CloseOutlined, HomeOutlined} from "@ant-design/icons";
import AsyncButton from "../../../components/AsyncButton.tsx";

interface JoystickOverlayProps {
    visible: boolean;
    isRecording?: boolean;
    onMove: (event: IJoystickUpdateEvent) => void;
    onStop: () => void;
    onFinishRecording?: () => Promise<void>;
    onCancelRecording?: () => Promise<void>;
    onHome?: () => Promise<void>;
    bottomOffset?: number;
}

export const JoystickOverlay = ({visible, isRecording, onMove, onStop, onFinishRecording, onCancelRecording, onHome, bottomOffset = 30}: JoystickOverlayProps) => {
    if (!visible) return null;
    return (
        <div style={{position: "absolute", bottom: bottomOffset, right: 30, zIndex: 100, display: 'flex', alignItems: 'flex-end', gap: 12}}>
            {isRecording && (
                <div style={{
                    display: 'flex',
                    flexDirection: 'column',
                    gap: 8,
                    marginBottom: 10,
                }}>
                    <AsyncButton
                        type="primary"
                        icon={<CheckOutlined />}
                        onAsyncClick={onFinishRecording!}
                        style={{height: 44, borderRadius: 10, fontWeight: 600}}
                    >
                        Finish
                    </AsyncButton>
                    <AsyncButton
                        danger
                        icon={<CloseOutlined />}
                        onAsyncClick={onCancelRecording!}
                        style={{height: 44, borderRadius: 10}}
                    >
                        Cancel
                    </AsyncButton>
                    <AsyncButton
                        icon={<HomeOutlined />}
                        onAsyncClick={onHome!}
                        style={{height: 44, borderRadius: 10}}
                    >
                        Home
                    </AsyncButton>
                </div>
            )}
            <Joystick move={onMove} stop={onStop}/>
        </div>
    );
};
