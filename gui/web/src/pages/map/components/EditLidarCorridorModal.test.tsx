import {describe, it, expect, vi} from 'vitest';
import {render, screen, waitFor} from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import {EditLidarCorridorModal} from './EditLidarCorridorModal.tsx';
import en from "../../../i18n/locales/en.json";
import "../../../i18n";

const corridor = {id: 1, name: 'Hedge', width_m: 0.4, polyline: {points: []}};

describe('EditLidarCorridorModal width validation', () => {
    const save = en.mapLidarCorridors.save;

    it('accepts the 120 cm maximum', async () => {
        const onSave = vi.fn();
        const user = userEvent.setup();
        render(<EditLidarCorridorModal corridor={corridor} busy={false} onSave={onSave} onCancel={vi.fn()}/>);
        const input = screen.getByDisplayValue('40');
        await user.clear(input);
        await user.type(input, '120');
        await user.click(screen.getByText(save));
        await waitFor(() => expect(onSave).toHaveBeenCalledWith('Hedge', 1.2));
    });

    it('rejects more than 120 cm with an error instead of clamping it', async () => {
        const onSave = vi.fn();
        const user = userEvent.setup();
        render(<EditLidarCorridorModal corridor={corridor} busy={false} onSave={onSave} onCancel={vi.fn()}/>);
        const input = screen.getByDisplayValue('40');
        await user.clear(input);
        await user.type(input, '150');
        await user.click(screen.getByText(save));
        expect(await screen.findByText('Enter a whole distance between 5 and 120 cm')).toBeInTheDocument();
        expect(onSave).not.toHaveBeenCalled();
    });
});
