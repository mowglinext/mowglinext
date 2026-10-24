import {fireEvent, render, screen} from '@testing-library/react';
import {beforeEach, expect, it, vi} from 'vitest';
import {HardwareSection} from './HardwareSection';

const mocks = vi.hoisted(() => ({confirm: vi.fn<(options: {onOk: () => void}) => void>(), change: vi.fn(), bulk: vi.fn()}));
vi.mock('react-i18next', () => ({useTranslation: () => ({t: (key: string) => key})}));
vi.mock('../../theme/ThemeContext', () => ({useThemeMode: () => ({colors: {primary: 'blue', border: 'grey'}})}));
vi.mock('../robot/MowerPreview', () => ({MowerPreview: () => null}));
vi.mock('antd', async importOriginal => {
    const actual = await importOriginal<typeof import('antd')>();
    return {...actual, App: {...actual.App, useApp: () => ({modal: {confirm: mocks.confirm}})}};
});
beforeEach(() => vi.clearAllMocks());

it('unconfirmed backend blocks model presets through click and keyboard', () => {
    render(<HardwareSection values={{mower_model: 'custom'}} onChange={mocks.change} onBulkChange={mocks.bulk}/>);
    const model = screen.getAllByRole('radio')[0];
    expect(model).toHaveAttribute('aria-disabled', 'true');
    fireEvent.click(model); fireEvent.keyDown(model, {key: 'Enter'});
    expect(mocks.confirm).not.toHaveBeenCalled();
    expect(mocks.change).not.toHaveBeenCalled(); expect(mocks.bulk).not.toHaveBeenCalled();
});

it('confirmed model presets use the backend override after confirmation', () => {
    render(<HardwareSection values={{mower_model: 'custom'}} backendConfirmed backendDefaultOverrides={{ticks_per_meter: 1600}}
        onChange={mocks.change} onBulkChange={mocks.bulk}/>);
    fireEvent.click(screen.getAllByRole('radio')[0]);
    expect(mocks.confirm).toHaveBeenCalledTimes(1);
    mocks.confirm.mock.calls[0][0].onOk();
    expect(mocks.bulk).toHaveBeenCalledWith(expect.objectContaining({ticks_per_meter: 1600}));
});
