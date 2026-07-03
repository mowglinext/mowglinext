import {describe, it, expect, vi} from 'vitest';
import {render, screen} from '@testing-library/react';
import {JoystickOverlay} from './JoystickOverlay.tsx';
import en from "../../../i18n/locales/en.json";

const noop = () => {};
const asyncNoop = async () => {};

describe('JoystickOverlay', () => {
    it('renders nothing when not visible', () => {
        const {container} = render(
            <JoystickOverlay visible={false} onMove={noop} onStop={noop}/>,
        );
        expect(container).toBeEmptyDOMElement();
    });

    it('renders the recording action buttons only while recording', () => {
        const {rerender} = render(
            <JoystickOverlay visible={true} onMove={noop} onStop={noop}/>,
        );
        expect(screen.queryByText(en.mapJoystick.finish)).toBeNull();

        rerender(
            <JoystickOverlay
                visible={true}
                isRecording={true}
                onMove={noop}
                onStop={noop}
                onFinishRecording={asyncNoop}
                onCancelRecording={asyncNoop}
                onHome={asyncNoop}
            />,
        );
        expect(screen.getByText(en.mapJoystick.finish)).toBeTruthy();
        expect(screen.getByText(en.mapJoystick.cancel)).toBeTruthy();
        expect(screen.getByText(en.mapJoystick.home)).toBeTruthy();
    });

    it('does not crash with the mobile layout flag', () => {
        const onMove = vi.fn();
        const {container} = render(
            <JoystickOverlay visible={true} mobile={true} onMove={onMove} onStop={noop}/>,
        );
        expect(container.firstChild).not.toBeNull();
    });
});
