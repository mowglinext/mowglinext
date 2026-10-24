import {act, cleanup, fireEvent, render, screen} from '@testing-library/react';
import {afterEach, expect, it, vi} from 'vitest';
import {App} from 'antd';
import {ImportOpenMowerModal} from './ImportOpenMowerModal';
import type {ImportOpenMowerSummary} from '../hooks/useMapFiles';
afterEach(cleanup);
const summary: ImportOpenMowerSummary = {mowing_areas: 1, navigation_areas: 0, obstacles: 0, orphan_obstacles: 0,
    datum_shift_east_m: 0, datum_shift_north_m: 0, warnings: [], areas: [], applied: false};
it('forwards each explicit Enter/blur to one preview reprojection, without applying the map', async () => {
    const reproject = vi.fn<(lat?: number, lon?: number) => Promise<ImportOpenMowerSummary>>().mockResolvedValue(summary);
    const apply = vi.fn(); const error = vi.spyOn(console, 'error');
    try {
        render(<App><ImportOpenMowerModal preview={summary} onApply={apply} onReproject={reproject} onClose={vi.fn()}/></App>);
        const latitude = screen.getByPlaceholderText('OM_DATUM_LAT');
        const longitude = screen.getByPlaceholderText('OM_DATUM_LONG');
        fireEvent.change(latitude, {target: {value: '0'}}); fireEvent.change(longitude, {target: {value: '2'}});
        await act(async () => {fireEvent.keyDown(latitude, {key: 'Enter', code: 'Enter'}); await Promise.resolve();});
        expect(reproject).toHaveBeenCalledExactlyOnceWith(0, 2);
        await act(async () => {fireEvent.blur(latitude); await Promise.resolve();});
        expect(reproject).toHaveBeenCalledTimes(2); // Existing two distinct commit events; no new wrapper listeners.
        expect(apply).not.toHaveBeenCalled();
        expect(error.mock.calls.some(args => args.some(arg => typeof arg === 'string' && arg.includes('[antd: InputNumber]') && /addonBefore|addonAfter/.test(arg)))).toBe(false);
    } finally {error.mockRestore();}
});

it('keeps the dialog and error after partial apply failure; Cancel never retries or rolls back', async () => {
    const apply = vi.fn<() => Promise<void>>().mockRejectedValue(new Error('map replaced, but setting the dock failed — HTTP 500'));
    const close = vi.fn(), reproject = vi.fn<() => Promise<ImportOpenMowerSummary>>().mockResolvedValue(summary);
    render(<App><ImportOpenMowerModal preview={summary} onApply={apply} onReproject={reproject} onClose={close}/></App>);
    await act(async () => {fireEvent.click(screen.getByRole('button', {name: /Apply/})); await Promise.resolve();});
    expect(await screen.findByText('map replaced, but setting the dock failed — HTTP 500')).toBeInTheDocument();
    expect(close).not.toHaveBeenCalled(); expect(apply).toHaveBeenCalledOnce();
    expect(screen.getByRole('button', {name: /Apply/})).toBeEnabled();
    fireEvent.click(screen.getByRole('button', {name: 'Cancel'}));
    expect(close).toHaveBeenCalledOnce(); expect(apply).toHaveBeenCalledOnce(); expect(reproject).not.toHaveBeenCalled();
});
it('allows explicit apply retry after an error, without an implicit second request', async () => {
    const apply = vi.fn<() => Promise<void>>().mockRejectedValueOnce('Connection lost').mockResolvedValueOnce();
    const close = vi.fn();
    render(<App><ImportOpenMowerModal preview={summary} onApply={apply} onReproject={vi.fn()} onClose={close}/></App>);
    await act(async () => {fireEvent.click(screen.getByRole('button', {name: /Apply/})); await Promise.resolve();});
    expect(await screen.findByText('Connection lost')).toBeInTheDocument();
    expect(apply).toHaveBeenCalledOnce(); expect(close).not.toHaveBeenCalled();
    await act(async () => {fireEvent.click(screen.getByRole('button', {name: /Apply/})); await Promise.resolve();});
    expect(apply).toHaveBeenCalledTimes(2); expect(close).toHaveBeenCalledOnce();
});

it('sends one apply for rapid confirmation events and cannot close while it is pending', async () => {
    let rejectApply: (reason: Error) => void = () => {throw new Error('Not initialized');};
    const apply = vi.fn<() => Promise<void>>().mockImplementation(() => new Promise<void>((_resolve, reject) => {rejectApply = reject;}));
    const close = vi.fn();
    render(<App><ImportOpenMowerModal preview={summary} onApply={apply} onReproject={vi.fn()} onClose={close}/></App>);
    const confirm = screen.getByRole('button', {name: /Apply/});
    const cancel = screen.getByRole('button', {name: 'Cancel'});
    act(() => {fireEvent.click(confirm); fireEvent.click(confirm); fireEvent.click(cancel);});
    expect(apply).toHaveBeenCalledOnce();
    expect(cancel).toBeDisabled();
    fireEvent.click(cancel);
    fireEvent.keyDown(screen.getByRole('dialog'), {key: 'Escape', code: 'Escape'});
    expect(close).not.toHaveBeenCalled();
    await act(async () => {rejectApply(new Error('Map saved; dock unconfirmed. Verify both.')); await Promise.resolve();});
    // jsdom does not run the CSS entrance animation; Chromium checks visibility.
    expect(await screen.findByText('Map saved; dock unconfirmed. Verify both.')).toBeInTheDocument();
    expect(close).not.toHaveBeenCalled(); expect(apply).toHaveBeenCalledOnce();
    expect(screen.getByRole('button', {name: /Apply/})).toBeEnabled();
});

it('keeps zero datum fields and the operator choice after a partial result', async () => {
    const apply = vi.fn<(...args: [number?, number?, boolean?]) => Promise<void>>().mockRejectedValue(new Error('Partial import; verify map and dock.'));
    const close = vi.fn();
    render(<App><ImportOpenMowerModal preview={{...summary, mn_datum_configured: false}}
        onApply={apply} onReproject={vi.fn()} onClose={close}/></App>);
    const lat = screen.getByPlaceholderText('OM_DATUM_LAT'), lon = screen.getByPlaceholderText('OM_DATUM_LONG');
    fireEvent.change(lat, {target: {value: '0'}}); fireEvent.change(lon, {target: {value: '0'}});
    await act(async () => {fireEvent.click(screen.getByRole('button', {name: /Apply/})); await Promise.resolve();});
    expect(apply).toHaveBeenCalledExactlyOnceWith(0, 0, true);
    expect(lat).toHaveValue('0.000000000'); expect(lon).toHaveValue('0.000000000');
    expect(screen.getByRole('checkbox')).toBeChecked();
    expect(close).not.toHaveBeenCalled();
    fireEvent.click(screen.getByRole('button', {name: 'Cancel'}));
    expect(close).toHaveBeenCalledOnce(); expect(apply).toHaveBeenCalledOnce();
});

it('cancels a preview without any apply or reprojection', () => {
    const apply = vi.fn(), reproject = vi.fn(), close = vi.fn();
    render(<App><ImportOpenMowerModal preview={summary} onApply={apply} onReproject={reproject} onClose={close}/></App>);
    fireEvent.click(screen.getByRole('button', {name: 'Cancel'}));
    expect(close).toHaveBeenCalledOnce(); expect(apply).not.toHaveBeenCalled(); expect(reproject).not.toHaveBeenCalled();
});

it('does not let a read-only preview erase an unconfirmed write outcome', async () => {
    const apply = vi.fn<() => Promise<void>>().mockRejectedValue(new Error('Map saved; dock unconfirmed.'));
    const reproject = vi.fn<() => Promise<ImportOpenMowerSummary>>().mockResolvedValueOnce(summary).mockRejectedValueOnce(new Error('Preview unavailable'));
    render(<App><ImportOpenMowerModal preview={summary} onApply={apply} onReproject={reproject} onClose={vi.fn()}/></App>);
    await act(async () => {fireEvent.click(screen.getByRole('button', {name: /Apply/})); await Promise.resolve();});
    const latitude = screen.getByPlaceholderText('OM_DATUM_LAT');
    await act(async () => {fireEvent.blur(latitude); await Promise.resolve();});
    expect(screen.getByText('Map saved; dock unconfirmed.')).toBeInTheDocument();
    await act(async () => {fireEvent.blur(latitude); await Promise.resolve();});
    expect(screen.getByText(/Map saved; dock unconfirmed.*Preview unavailable/)).toBeInTheDocument();
    expect(apply).toHaveBeenCalledOnce(); expect(reproject).toHaveBeenCalledTimes(2);
});
