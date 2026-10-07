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
    selectArea: vi.fn(), setAngleMode: vi.fn(), setAngleDeg: vi.fn(), setDirection: vi.fn(),
    reset: vi.fn(), saveArea: vi.fn(), saveRobotWide: vi.fn(), setEnabled: vi.fn(), setStart: vi.fn(),
};

const follow: AreaChoices = {angleMode: 'global', angleDeg: 0, direction: 'global', start: null};

const makePreview = (over: Partial<Preview> = {}): Preview => {
    const areas = [area('a', 'Front lawn')];
    return {
        enabled: true,
        areas,
        area: areas[0],
        hasAreaId: true,
        canEdit: true,
        startAdjustable: true,
        choices: follow,
        robotWideAngle: -1,
        robotWideDirection: 0,
        shownAngle: 72.4,
        dirty: false,
        differsFromRobotWide: false,
        saving: false,
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
        show(makePreview({choices: {angleMode: 'fixed', angleDeg: 30, direction: 'global', start: null}}));
        expect(screen.getByText('This area has its own lines.')).toBeInTheDocument();
    });

    it('keeps save and reset disabled until something changed', () => {
        show(makePreview());
        expect(button('Save for this area')).toBeDisabled();
        expect(button('Reset')).toBeDisabled();
    });

    it('saves this area and resets once the preview differs from what is stored', () => {
        show(makePreview({dirty: true}));
        expect(screen.getByText('Not saved yet. This only changes the preview.')).toBeInTheDocument();
        fireEvent.click(button('Save for this area'));
        expect(fns.saveArea).toHaveBeenCalledOnce();
        fireEvent.click(button('Reset'));
        expect(fns.reset).toHaveBeenCalledOnce();
    });

    it('cannot save an area while the mower is not idle, and says why', () => {
        show(makePreview({dirty: true, canEdit: false}));
        expect(button('Save for this area')).toBeDisabled();
        expect(screen.getByText(/Stop the mower to change an area's lines/)).toBeInTheDocument();
        // The preview itself stays usable.
        expect(button('Reset')).toBeEnabled();
    });

    it('cannot save an area that has no id yet', () => {
        show(makePreview({dirty: true, hasAreaId: false}));
        expect(button('Save for this area')).toBeDisabled();
        expect(screen.getByText('Save the map first: this area has no id yet.')).toBeInTheDocument();
    });

    it('warns that a paused mow resumes with a re-planned area, only when it matters', () => {
        const {rerender} = show(makePreview({dirty: true}), true);
        expect(screen.getByText(/choose Start fresh instead of Resume/)).toBeInTheDocument();
        rerender(<CoveragePreviewPanel preview={makePreview({dirty: false})} areaLabel={() => ''} resumeAvailable/>);
        expect(screen.queryByText(/choose Start fresh instead of Resume/)).not.toBeInTheDocument();
        rerender(<CoveragePreviewPanel preview={makePreview({dirty: true})} areaLabel={() => ''} resumeAvailable={false}/>);
        expect(screen.queryByText(/choose Start fresh instead of Resume/)).not.toBeInTheDocument();
    });

    it('offers the robot-wide save only when what is shown differs from it', () => {
        const {rerender} = show(makePreview({differsFromRobotWide: false}));
        expect(button('Use for all areas')).toBeDisabled();
        rerender(<CoveragePreviewPanel preview={makePreview({differsFromRobotWide: true})} areaLabel={() => ''}/>);
        fireEvent.click(button('Use for all areas'));
        expect(fns.saveRobotWide).toHaveBeenCalledOnce();
    });

    it('picking a direction from the list sends that choice', () => {
        show(makePreview());
        const selects = screen.getAllByRole('combobox');
        fireEvent.mouseDown(selects[1]);
        fireEvent.click(within(document.body).getByText('Counter-clockwise', {selector: '.ant-select-item-option-content'}));
        expect(fns.setDirection).toHaveBeenCalledWith(2);
    });

    it('picking a fixed angle from the list sends that mode', () => {
        show(makePreview());
        const selects = screen.getAllByRole('combobox');
        fireEvent.mouseDown(selects[0]);
        fireEvent.click(within(document.body).getByText('Fixed angle', {selector: '.ant-select-item-option-content'}));
        expect(fns.setAngleMode).toHaveBeenCalledWith('fixed');
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
});
