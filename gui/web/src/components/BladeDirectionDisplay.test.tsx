import {act, cleanup, fireEvent, render, screen} from '@testing-library/react';
import {afterEach, describe, expect, it, vi} from 'vitest';
import {BladeDirectionDisplay} from './BladeDirectionDisplay';
import type {Status} from '../types/ros';

const topic = vi.hoisted(() => ({data: {} as Status, lastMessageAt: null as number | null}));
vi.mock('../hooks/useTopic', () => ({useTopic: () => topic}));
afterEach(() => { cleanup(); vi.useRealTimers(); topic.data = {}; topic.lastMessageAt = null; });

describe('blade direction display', () => {
    it.each(['forward', 'reverse', 'off'] as const)('labels %s as a request, independent of RPM', (direction) => {
        topic.data = {blade_requested_direction: direction, mower_motor_rpm: 3200};
        topic.lastMessageAt = Date.now();
        render(<BladeDirectionDisplay/>);
        expect(screen.getByText('Requested blade direction')).toBeInTheDocument();
        expect(screen.getByText(direction[0].toUpperCase() + direction.slice(1))).toBeInTheDocument();
        expect(screen.getByText(/physical rotation is not reported/)).toBeInTheDocument();
    });
    it('provides firmware help by touch in compact mode', async () => {
        render(<BladeDirectionDisplay compact/>);
        fireEvent.click(screen.getByRole('button', {name: 'Requested direction: Unknown'}));
        // jsdom has no popup layout; the mobile browser test verifies visibility.
        expect(await screen.findByText(/Older firmware may still run forward/)).toBeInTheDocument();
        expect(screen.getByRole('button')).toHaveAttribute('aria-describedby');
    });
    it('does not guess forward on an old stack or on missing/invalid telemetry', () => {
        topic.data = {mower_motor_rpm: 3200}; topic.lastMessageAt = Date.now();
        const {rerender} = render(<BladeDirectionDisplay/>);
        expect(screen.getByText('Unknown')).toBeInTheDocument();
        topic.data.blade_requested_direction = 'invalid';
        rerender(<BladeDirectionDisplay/>);
        expect(screen.getByText('Unknown')).toBeInTheDocument();
    });
    it('ages out cached telemetry and updates on fresh data in compact dashboard form', () => {
        vi.useFakeTimers();
        topic.data = {blade_requested_direction: 'reverse'}; topic.lastMessageAt = Date.now();
        const {rerender} = render(<BladeDirectionDisplay compact/>);
        expect(screen.getByText('Requested direction: Reverse')).toBeInTheDocument();
        act(() => { vi.advanceTimersByTime(5000); });
        expect(screen.getByText('Requested direction: Unknown')).toBeInTheDocument();
        topic.data = {blade_requested_direction: 'forward'}; topic.lastMessageAt = Date.now();
        rerender(<BladeDirectionDisplay compact/>);
        expect(screen.getByText('Requested direction: Forward')).toBeInTheDocument();
    });
});
