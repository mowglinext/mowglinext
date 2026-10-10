import {render, screen} from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import {describe, expect, it, vi} from 'vitest';
import {MapRotationPanel} from './MapOffsetPanel.tsx';

describe('MapRotationPanel', () => {
    it('keeps the angle controls enabled while rotation gestures are locked', async () => {
        const onChangeRotationLocked = vi.fn();
        render(
            <MapRotationPanel
                bearing={33}
                onChangeBearing={vi.fn()}
                rotationLocked
                onChangeRotationLocked={onChangeRotationLocked}
            />,
        );

        expect(screen.getByRole('slider')).toBeEnabled();
        expect(screen.getByRole('spinbutton')).toBeEnabled();
        const button = screen.getByRole('button', {name: 'Unlock map rotation'});
        expect(button).toHaveAttribute('aria-pressed', 'true');

        await userEvent.click(button);
        expect(onChangeRotationLocked).toHaveBeenCalledWith(false);
    });

    it('offers to lock rotation gestures while unlocked', async () => {
        const onChangeRotationLocked = vi.fn();
        render(
            <MapRotationPanel
                bearing={0}
                onChangeBearing={vi.fn()}
                rotationLocked={false}
                onChangeRotationLocked={onChangeRotationLocked}
            />,
        );

        const button = screen.getByRole('button', {name: 'Lock map rotation'});
        expect(button).toHaveAttribute('aria-pressed', 'false');

        await userEvent.click(button);
        expect(onChangeRotationLocked).toHaveBeenCalledWith(true);
    });
});
