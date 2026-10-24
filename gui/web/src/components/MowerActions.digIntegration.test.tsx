import {cleanup, fireEvent, render, screen, waitFor, within} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {App, ConfigProvider} from 'antd';
import {MowerActions} from './MowerActions';
import en from '../i18n/locales/en.json';
const mock = vi.hoisted(() => ({call: vi.fn(), escalated: true}));
vi.mock('../hooks/useApi.ts', () => ({useApi: () => ({mowglinext: {callCreate: mock.call}})}));
vi.mock('../hooks/useHighLevelStatus.ts', () => ({useHighLevelStatus: () => ({highLevelStatus: {state: 1, state_name: 'DIG_OBSTRUCTION'}})}));
vi.mock('../hooks/useStatus.ts', () => ({useStatus: () => ({dig_escalated: mock.escalated, dig_escalated_distance_m: 0, dig_escalated_required_distance_m: 0.5})}));
vi.mock('../hooks/useCoverageResumeAvailable.ts', () => ({useCoverageResumeAvailable: () => false}));
vi.mock('../theme/ThemeContext.tsx', () => ({useThemeMode: () => ({colors: {}})}));
beforeEach(() => {mock.call.mockReset().mockResolvedValue({}); mock.escalated = true;});
afterEach(cleanup);

describe('MowerActions real DIG_OBSTRUCTION acknowledgement integration', () => {
    it.each(['start', 'continue'] as const)('cancelled %s does not send clear or start', async path => {
        render(<ConfigProvider theme={{token: {motion: false}}}><App><MowerActions bare/></App></ConfigProvider>);
        if (path === 'start') fireEvent.click(screen.getByRole('button', {name: new RegExp(`${en.mowerActions.start}$`)}));
        else {fireEvent.click(screen.getByText(en.mowerActions.more)); fireEvent.click(await screen.findByText(en.mowerActions.continue, {exact: true}));}
        const dialog = await screen.findByRole('dialog');
        fireEvent.click(within(dialog).getByRole('button', {name: en.digObstructionAck.cancel}));
        await waitFor(() => expect(screen.queryByRole('dialog')).not.toBeInTheDocument());
        expect(mock.call).not.toHaveBeenCalled();
    });
    it.each(['start', 'continue'] as const)('confirmed %s releases before starting and never starts after a failed release', async path => {
        mock.call.mockResolvedValueOnce({error: {error: 'Release refused'}});
        render(<ConfigProvider theme={{token: {motion: false}}}><App><MowerActions bare/></App></ConfigProvider>);
        if (path === 'start') fireEvent.click(screen.getByRole('button', {name: new RegExp(`${en.mowerActions.start}$`)}));
        else {fireEvent.click(screen.getByText(en.mowerActions.more)); fireEvent.click(await screen.findByText(en.mowerActions.continue, {exact: true}));}
        const dialog = await screen.findByRole('dialog');
        fireEvent.click(within(dialog).getByRole('button', {name: en.digObstructionAck.ok}));
        expect(await screen.findByText('Release refused')).toBeVisible();
        expect(mock.call.mock.calls).toEqual([['clear_dig_escalation', {}]]);
    });
    it.each(['start', 'continue'] as const)('confirmed successful %s preserves the clear/start order and parameters', async path => {
        render(<ConfigProvider theme={{token: {motion: false}}}><App><MowerActions bare/></App></ConfigProvider>);
        if (path === 'start') fireEvent.click(screen.getByRole('button', {name: new RegExp(`${en.mowerActions.start}$`)}));
        else {fireEvent.click(screen.getByText(en.mowerActions.more)); fireEvent.click(await screen.findByText(en.mowerActions.continue, {exact: true}));}
        const dialog = await screen.findByRole('dialog');
        fireEvent.click(within(dialog).getByRole('button', {name: en.digObstructionAck.ok}));
        await waitFor(() => expect(mock.call.mock.calls).toEqual([['clear_dig_escalation', {}], ['high_level_control', {Command: 1}]]));
    });
});
