import {fireEvent, render, screen, within} from '@testing-library/react';
import {beforeEach, describe, expect, it, vi} from 'vitest';
import {CoveragePreviewPanel} from './CoveragePreviewPanel.tsx';
import {MowingAreaFeature} from '../../../types/map.ts';
import type {AreaChoices} from '../coveragePreview.ts';
import type {useCoveragePreview} from '../hooks/useCoveragePreview.ts';

vi.mock('../../../theme/ThemeContext.tsx', () => ({useThemeMode: () => ({colors: {}})}));

type Preview = ReturnType<typeof useCoveragePreview>;

const area = (id: string, name: string) => {
    const a = new MowingAreaFeature(id, 1);
    a.setName(name);
    return a;
};

const fns = {
    selectArea: vi.fn(), setAngleMode: vi.fn(), setAngleDeg: vi.fn(), commitAngle: vi.fn(),
    setDirection: vi.fn(), setStart: vi.fn(), setEnabled: vi.fn(),
};

const follow: AreaChoices = {angleMode: 'global', angleDeg: 0, direction: 'global', start: null};

const makePreview = (over: Partial<Preview> = {}): Preview => {
    const areas = [area('a', 'Front lawn')];
    return {
        enabled: true,
        areas,
        area: areas[0],
        editMode: true,
        startAdjustable: true,
        choices: follow,
        robotWideAngle: -1,
        robotWideDirection: 0,
        shownAngle: 72.4,
        changedFromServer: false,
        loading: false,
        error: undefined,
        result: {success: true, swaths: [{}, {}, {}], headland_passes: 2, planned_fraction: 0.934, mow_angle_deg: 72.4},
        layers: {lines: {type: 'FeatureCollection', features: []}, arrows: {type: 'FeatureCollection', features: []}},
        ...fns,
        ...over,
    } as unknown as Preview;
};

const show = (preview: Preview, resumeAvailable = false) => render(
    <CoveragePreviewPanel preview={preview} areaLabel={(i, name) => name || `Area ${i + 1}`} resumeAvailable={resumeAvailable}/>);

const button = (name: string) => screen.getByRole('button', {name});

describe('coverage preview panel', () => {
    beforeEach(() => vi.clearAllMocks());

    it('asks for an area when there is none to preview', () => {
        show(makePreview({areas: [], area: undefined}));
        expect(screen.getByText('Draw a mowing area first.')).toBeInTheDocument();
    });

    it('summarises what the planner drew', () => {
        show(makePreview());
        expect(screen.getByText('3 lines')).toBeInTheDocument();
        expect(screen.getByText('2 perimeter rounds')).toBeInTheDocument();
        expect(screen.getByText('93% of the area planned')).toBeInTheDocument();
    });

    it('never reports more than 100% of the area planned', () => {
        show(makePreview({result: {success: true, swaths: [{}], headland_passes: 2, planned_fraction: 1.08}} as unknown as Partial<Preview>));
        expect(screen.getByText('100% of the area planned')).toBeInTheDocument();
    });

    it('shows the planner refusal instead of stale numbers', () => {
        show(makePreview({error: 'field too small after insets', result: undefined}));
        expect(screen.getByText('field too small after insets')).toBeInTheDocument();
        expect(screen.queryByText(/^\d+ perimeter rounds$/)).not.toBeInTheDocument();
    });

    it('only offers an area picker when there is a choice', () => {
        show(makePreview());
        expect(screen.queryByText('Area')).not.toBeInTheDocument();
    });

    it('says an area follows the robot-wide settings, and what they are', () => {
        show(makePreview({robotWideAngle: 40, robotWideDirection: 2}));
        expect(screen.getByText('This area follows the robot-wide settings.')).toBeInTheDocument();
        expect(screen.getByText('Robot-wide (40°)')).toBeInTheDocument();
        expect(screen.getByText('Robot-wide (Counter-clockwise)')).toBeInTheDocument();
    });

    it('names robot-wide auto as auto, and shows the angle the planner chose for it', () => {
        show(makePreview({robotWideAngle: -1}));
        expect(screen.getByText('Robot-wide (auto)')).toBeInTheDocument();
        expect(screen.getByText('Auto (now 72°)')).toBeInTheDocument();
    });

    it('says an area has its own lines once it overrides something', () => {
        show(makePreview({choices: {...follow, angleMode: 'fixed', angleDeg: 30}}));
        expect(screen.getByText('This area has its own lines.')).toBeInTheDocument();
        show(makePreview({choices: {...follow, start: {x: 1, y: 2}}}));
        expect(screen.getAllByText('This area has its own lines.')).toHaveLength(2);
    });

    describe('in edit mode', () => {
        it('says the changes are part of the map edit', () => {
            show(makePreview());
            expect(screen.getByText(/Save map keeps them, Cancel drops them/)).toBeInTheDocument();
            expect(screen.queryByText(/Choose Edit map to change/)).not.toBeInTheDocument();
        });

        it('has no save or reset buttons of its own: Save map and Cancel do that', () => {
            show(makePreview());
            expect(screen.queryByRole('button', {name: /save/i})).not.toBeInTheDocument();
            expect(screen.queryByRole('button', {name: 'Reset'})).not.toBeInTheDocument();
            expect(screen.queryByRole('button', {name: 'Use for all areas'})).not.toBeInTheDocument();
        });

        it('enables every control', () => {
            show(makePreview());
            for (const combo of screen.getAllByRole('combobox')) expect(combo).toBeEnabled();
            expect(screen.getByRole('spinbutton')).toBeEnabled();
        });

        it('picking a direction from the list sends that choice', () => {
            show(makePreview());
            fireEvent.mouseDown(screen.getAllByRole('combobox')[1]);
            fireEvent.click(within(document.body).getByText('Counter-clockwise', {selector: '.ant-select-item-option-content'}));
            expect(fns.setDirection).toHaveBeenCalledExactlyOnceWith(2);
        });

        it('picking a fixed angle from the list sends that mode', () => {
            show(makePreview());
            fireEvent.mouseDown(screen.getAllByRole('combobox')[0]);
            fireEvent.click(within(document.body).getByText('Fixed angle', {selector: '.ant-select-item-option-content'}));
            expect(fns.setAngleMode).toHaveBeenCalledExactlyOnceWith('fixed');
        });

        it('typing an angle moves the preview and commits when the field is left or Enter is pressed', () => {
            show(makePreview({choices: {...follow, angleMode: 'fixed', angleDeg: 30}, shownAngle: 30}));
            const input = screen.getByRole('spinbutton');
            fireEvent.change(input, {target: {value: '45'}});
            expect(fns.setAngleDeg).toHaveBeenCalledWith(45);
            expect(fns.commitAngle).not.toHaveBeenCalled();
            fireEvent.blur(input);
            expect(fns.commitAngle).toHaveBeenCalledTimes(1);
            fireEvent.keyDown(input, {key: 'Enter', code: 'Enter', keyCode: 13});
            expect(fns.commitAngle).toHaveBeenCalledTimes(2);
        });

        it('says the planner picks the start until the operator chooses one', () => {
            show(makePreview());
            expect(screen.getByText('Start point')).toBeInTheDocument();
            expect(screen.getByText('Automatic: the middle of the longest side.')).toBeInTheDocument();
            expect(screen.getByText(/Drag the green dot to change where mowing starts/)).toBeInTheDocument();
            expect(screen.queryByRole('button', {name: 'Back to automatic'})).not.toBeInTheDocument();
        });

        it('shows a chosen start and lets the operator go back to automatic', () => {
            show(makePreview({choices: {...follow, start: {x: 3, y: 4}}}));
            expect(screen.getByText('You chose where mowing starts.')).toBeInTheDocument();
            fireEvent.click(button('Back to automatic'));
            expect(fns.setStart).toHaveBeenCalledExactlyOnceWith(null);
        });

        it('explains why there is no start point to place with the perimeter rounds off', () => {
            show(makePreview({startAdjustable: false, choices: {...follow, start: {x: 3, y: 4}}}));
            expect(screen.getByText(/A start point needs the perimeter rounds/)).toBeInTheDocument();
            expect(screen.queryByText(/Drag the green dot/)).not.toBeInTheDocument();
            expect(screen.queryByRole('button', {name: 'Back to automatic'})).not.toBeInTheDocument();
        });

        it('warns that a paused mow resumes on a re-planned area, only when this session changed the lines', () => {
            const {rerender} = show(makePreview({changedFromServer: true}), true);
            expect(screen.getByText(/choose Start fresh instead of Resume/)).toBeInTheDocument();
            rerender(<CoveragePreviewPanel preview={makePreview({changedFromServer: false})} areaLabel={() => ''} resumeAvailable/>);
            expect(screen.queryByText(/choose Start fresh instead of Resume/)).not.toBeInTheDocument();
            rerender(<CoveragePreviewPanel preview={makePreview({changedFromServer: true})} areaLabel={() => ''} resumeAvailable={false}/>);
            expect(screen.queryByText(/choose Start fresh instead of Resume/)).not.toBeInTheDocument();
        });
    });

    describe('outside edit mode', () => {
        it('tells the operator to choose Edit map', () => {
            show(makePreview({editMode: false}));
            expect(screen.getByText(/Choose Edit map to change the angle, direction or start point/)).toBeInTheDocument();
            expect(screen.queryByText(/Save map keeps them/)).not.toBeInTheDocument();
        });

        it('shows the same lines and numbers, read-only', () => {
            show(makePreview({editMode: false}));
            expect(screen.getByText('3 lines')).toBeInTheDocument();
            for (const combo of screen.getAllByRole('combobox')) expect(combo).toBeDisabled();
            expect(screen.getByRole('spinbutton')).toBeDisabled();
        });

        it('does not offer to change the start point', () => {
            show(makePreview({editMode: false, choices: {...follow, start: {x: 3, y: 4}}}));
            expect(screen.getByText('You chose where mowing starts.')).toBeInTheDocument();
            expect(screen.queryByRole('button', {name: 'Back to automatic'})).not.toBeInTheDocument();
            expect(screen.queryByText(/Drag the green dot/)).not.toBeInTheDocument();
        });
    });
});
