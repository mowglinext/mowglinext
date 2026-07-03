import {describe, it, expect, vi} from 'vitest';
import {render, screen} from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import {AreasListPanel} from './AreasListPanel.tsx';
import type {AreaListItem} from '../utils/types.ts';
import i18n from "../../../i18n";

describe('AreasListPanel', () => {
    const areas: AreaListItem[] = [
        {id: 'area-0-area-0', name: 'Front Yard', ftype: 'workarea', areaLabel: '45 m²', mowingOrder: 1},
        {id: 'navigation-0-area-0', name: 'Nav Zone', ftype: 'navigation', areaLabel: '12 m²'},
        {id: 'area-0-obstacle-0', name: 'Tree', ftype: 'obstacle', areaLabel: '3 m²'},
    ];

    it('shows area count in header', () => {
        render(<AreasListPanel areas={areas} />);
        expect(screen.getByText(i18n.t('mapAreasList.areasHeader', {count: 3}))).toBeInTheDocument();
    });

    it('renders all areas immediately (no expand needed)', () => {
        render(<AreasListPanel areas={areas} />);
        expect(screen.getByText('Front Yard')).toBeInTheDocument();
        expect(screen.getByText('Nav Zone')).toBeInTheDocument();
        expect(screen.getByText('Tree')).toBeInTheDocument();
    });

    it('shows area sizes', () => {
        render(<AreasListPanel areas={areas} />);
        expect(screen.getByText('45 m²')).toBeInTheDocument();
        expect(screen.getByText('12 m²')).toBeInTheDocument();
        expect(screen.getByText('3 m²')).toBeInTheDocument();
    });

    it('calls onAreaClick when area is clicked', async () => {
        const user = userEvent.setup();
        const onAreaClick = vi.fn();
        render(<AreasListPanel areas={areas} onAreaClick={onAreaClick} />);
        await user.click(screen.getByText('Front Yard'));
        expect(onAreaClick).toHaveBeenCalledWith('area-0-area-0');
    });

    it('returns null when no areas', () => {
        const {container} = render(<AreasListPanel areas={[]} />);
        expect(container.innerHTML).toBe('');
    });

    it('shows mowing order badge for work areas', () => {
        render(<AreasListPanel areas={areas} />);
        expect(screen.getByText('1')).toBeInTheDocument();
    });
});
