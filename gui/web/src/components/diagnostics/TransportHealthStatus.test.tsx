import {cleanup, render, screen} from '@testing-library/react';
import {afterEach, expect, it} from 'vitest';
import {TransportHealthStatus} from './TransportHealthStatus';
import type {ROSHealth} from '../../hooks/useDiagnosticsSnapshot';

afterEach(cleanup);

it.each([
    {ready: false, transport: {connected: false, heartbeat_ok: false}},
    {ready: false, transport: {connected: true, heartbeat_ok: false}},
    {ready: true, transport: {connected: true, heartbeat_ok: true}},
] satisfies ROSHealth[])('keeps HTTP responsiveness separate from ROS evidence: %j', ros => {
    render(<TransportHealthStatus fresh ros={ros}/>);
    expect(screen.getByTestId('diagnostic-gui-http')).toHaveTextContent('GUI HTTP: Responding');
    expect(screen.getByTestId('diagnostic-ros-transport')).toHaveTextContent(!ros.transport?.connected
        ? 'Disconnected' : ros.transport.heartbeat_ok ? 'Heartbeat verified' : 'Waiting for heartbeat');
});

it.each([undefined, {ready: false, transport: null}] satisfies (ROSHealth | undefined)[])(
    'keeps legacy or unknown ROS status unknown: %j', ros => {
        render(<TransportHealthStatus fresh ros={ros}/>);
        expect(screen.getByTestId('diagnostic-gui-http')).toHaveTextContent('Responding');
        expect(screen.getByTestId('diagnostic-ros-transport')).toHaveTextContent('Status unknown');
    });

it('does not preserve healthy labels after the HTTP observation becomes stale', () => {
    render(<TransportHealthStatus fresh={false} ros={{ready: true, transport: {connected: true, heartbeat_ok: true}}}/>);
    expect(screen.getByTestId('diagnostic-gui-http')).toHaveTextContent('Status unknown');
    expect(screen.getByTestId('diagnostic-ros-transport')).toHaveTextContent('Status unknown');
});
