import {useState} from 'react';
import {describe, it, expect, vi} from 'vitest';
import {render, screen} from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import {MapSidebarAccordion, type SidebarSection} from './MapSidebarAccordion.tsx';

const sections: SidebarSection[] = [
    {key: 'areas', title: 'Mowing areas (2)', content: <div>area rows</div>},
    {key: 'obstacles', title: 'Obstacles', badge: 3, content: <div>obstacle rows</div>},
    {key: 'lidar', title: 'LiDAR ignore lines (1)', extra: <button type="button">info</button>, content: <div>line rows</div>},
];

const Harness = ({initial = 'areas'}: {initial?: string | null}) => {
    const [open, setOpen] = useState<string | null>(initial);
    return <MapSidebarAccordion sections={sections} openKey={open} onOpenChange={setOpen}/>;
};

const header = (title: string) => screen.getByRole('button', {name: new RegExp(title)});

describe('MapSidebarAccordion', () => {
    it('shows every section header but only the open section content', () => {
        render(<Harness/>);
        expect(screen.getByText('Mowing areas (2)')).toBeInTheDocument();
        expect(screen.getByText('Obstacles')).toBeInTheDocument();
        expect(screen.getByText('LiDAR ignore lines (1)')).toBeInTheDocument();
        expect(screen.getByText('area rows')).toBeInTheDocument();
        expect(screen.queryByText('obstacle rows')).not.toBeInTheDocument();
        expect(screen.queryByText('line rows')).not.toBeInTheDocument();
    });

    it('marks the open section with aria-expanded', () => {
        render(<Harness/>);
        expect(header('Mowing areas')).toHaveAttribute('aria-expanded', 'true');
        expect(header('Obstacles')).toHaveAttribute('aria-expanded', 'false');
    });

    it('closes the open section when another one is opened', async () => {
        const user = userEvent.setup();
        render(<Harness/>);
        await user.click(header('Obstacles'));
        expect(screen.getByText('obstacle rows')).toBeInTheDocument();
        expect(screen.queryByText('area rows')).not.toBeInTheDocument();
        await user.click(header('LiDAR ignore lines'));
        expect(screen.getByText('line rows')).toBeInTheDocument();
        expect(screen.queryByText('obstacle rows')).not.toBeInTheDocument();
    });

    it('folds the open section when its header is clicked again', async () => {
        const user = userEvent.setup();
        render(<Harness/>);
        await user.click(header('Mowing areas'));
        expect(screen.queryByText('area rows')).not.toBeInTheDocument();
        expect(header('Mowing areas')).toHaveAttribute('aria-expanded', 'false');
    });

    it('can start with everything folded', () => {
        render(<Harness initial={null}/>);
        expect(screen.queryByText('area rows')).not.toBeInTheDocument();
        expect(screen.queryByText('obstacle rows')).not.toBeInTheDocument();
    });

    it('reports the key that was asked for, or null when the open one is clicked', async () => {
        const user = userEvent.setup();
        const onOpenChange = vi.fn();
        render(<MapSidebarAccordion sections={sections} openKey="areas" onOpenChange={onOpenChange}/>);
        await user.click(header('Obstacles'));
        expect(onOpenChange).toHaveBeenLastCalledWith('obstacles');
        await user.click(header('Mowing areas'));
        expect(onOpenChange).toHaveBeenLastCalledWith(null);
    });

    it('shows the badge next to the title', () => {
        render(<Harness/>);
        expect(screen.getByText('3')).toBeInTheDocument();
    });

    it('keeps the extra header control independent of the toggle', async () => {
        const user = userEvent.setup();
        const onOpenChange = vi.fn();
        render(<MapSidebarAccordion sections={sections} openKey="areas" onOpenChange={onOpenChange}/>);
        await user.click(screen.getByRole('button', {name: 'info'}));
        expect(onOpenChange).not.toHaveBeenCalled();
    });
});
