import {cleanup, fireEvent, render, screen, within} from '@testing-library/react';
import {afterEach, describe, expect, it} from 'vitest';
import {ConfigProvider} from 'antd';
import {DiagnosticAlertsPanel} from './DiagnosticAlertsPanel';
import type {CurrentDiagnosticAlert} from '../../utils/diagnosticAlertHistory';

const report = (level = 1, name = 'component/detail', hardware_id = 'source-A'): CurrentDiagnosticAlert => ({
    name, hardware_id, level, reportedLevel: level, message: 'Preserved diagnostic detail', receivedAt: 0,
    values: [{key: 'reading', value: '0'}], notApplicable: false,
});
const wrapper = ({children}: {children: React.ReactNode}) => <ConfigProvider theme={{token: {motion: false}}}>{children}</ConfigProvider>;
afterEach(cleanup);
function panel() {return within(screen.getByTestId('diagnostic-alert-panel'));}
function group(name = 'component') {return panel().getByRole('button', {name: new RegExp(name)});}

describe('stable diagnostic consultation', () => {
    it('always mounts a panel including when there are no alerts', () => {
        render(<DiagnosticAlertsPanel alerts={[]} connected now={0}/>, {wrapper});
        expect(screen.getByTestId('diagnostic-alert-panel')).toBeVisible();
        expect(screen.getByTestId('diagnostic-alert-empty')).toBeVisible();
    });
    it('keeps the same focused open group and last fault through OK, reappearance and ERROR', () => {
        const alert = report();
        const view = render(<DiagnosticAlertsPanel alerts={[alert]} observations={[alert]} connected now={0}/>, {wrapper});
        const header = group(); fireEvent.click(header); header.focus();
        const row = screen.getByTestId('diagnostic-alert-episode');
        view.rerender(<DiagnosticAlertsPanel alerts={[]} observations={[{...alert, level: 0, message: 'OK'}]} connected now={1000}/>);
        expect(header).toHaveFocus(); expect(header).toHaveAttribute('aria-expanded', 'true');
        expect(screen.getByTestId('diagnostic-alert-episode')).toBe(row);
        expect(row).toHaveAttribute('data-state', 'resolved'); expect(row).toHaveTextContent('Preserved diagnostic detail');
        expect(row).toHaveTextContent('0');
        view.rerender(<DiagnosticAlertsPanel alerts={[{...alert, level: 2, reportedLevel: 2, message: 'New error'}]} connected now={2000}/>);
        expect(header).toHaveFocus(); expect(row).toHaveAttribute('data-episode', '2');
        expect(row).toHaveAttribute('data-state', 'active'); expect(row).toHaveTextContent('New error');
        expect(row).toHaveClass('ant-alert-error');
    });
    it('does not lose an open resolved group on expiry, then permits explicit collapse and purge', () => {
        const alert = report();
        const view = render(<DiagnosticAlertsPanel alerts={[alert]} connected now={0}/>, {wrapper});
        fireEvent.click(group());
        const ok = [{...alert, level: 0}];
        view.rerender(<DiagnosticAlertsPanel alerts={[]} observations={ok} connected now={1000}/>);
        view.rerender(<DiagnosticAlertsPanel alerts={[]} observations={ok} connected now={900000}/>);
        expect(group()).toHaveAttribute('aria-expanded', 'true'); expect(screen.getByTestId('diagnostic-alert-message')).toBeVisible();
        fireEvent.click(panel().getByRole('button', {name: 'Collapse all groups'}));
        expect(screen.queryByTestId('diagnostic-alert-group')).not.toBeInTheDocument();
        expect(screen.getByTestId('diagnostic-alert-empty')).toBeVisible();
    });
    it('provides explicit access to recent resolved alerts and preserves their details until expiry', () => {
        const alert = report(2);
        const view = render(<DiagnosticAlertsPanel alerts={[alert]} connected now={0}/>, {wrapper});
        const ok = [{...alert, level: 0}];
        view.rerender(<DiagnosticAlertsPanel alerts={[]} observations={ok} connected now={1000}/>);
        view.rerender(<DiagnosticAlertsPanel alerts={[]} observations={ok} connected now={31000}/>);
        expect(screen.queryByTestId('diagnostic-alert-group')).not.toBeInTheDocument();
        const recent = panel().getByRole('button', {name: 'Show recent alerts (1)'}); fireEvent.click(recent);
        expect(recent).toHaveAttribute('aria-pressed', 'true');
        fireEvent.click(group());
        expect(screen.getByTestId('diagnostic-alert-episode')).toHaveTextContent('Error');
        expect(screen.getByTestId('diagnostic-alert-episode')).toHaveAttribute('data-state', 'resolved');
        // Hiding history does not remove a group the operator is consulting.
        fireEvent.click(panel().getByRole('button', {name: 'Hide recent alerts'}));
        expect(group()).toHaveAttribute('aria-expanded', 'true');
        fireEvent.click(group()); expect(screen.queryByTestId('diagnostic-alert-group')).not.toBeInTheDocument();
    });
    it('distinguishes loss of the stream from an explicit OK on reconnect', () => {
        const alert = report(2);
        const view = render(<DiagnosticAlertsPanel alerts={[alert]} connected now={0}/>, {wrapper});
        fireEvent.click(group());
        view.rerender(<DiagnosticAlertsPanel alerts={[]} connected={false} now={1000}/>);
        const row = screen.getByTestId('diagnostic-alert-episode');
        expect(row).toHaveAttribute('data-state', 'unverified'); expect(row).toHaveTextContent('State unverified');
        view.rerender(<DiagnosticAlertsPanel alerts={[]} connected now={2000}/>);
        expect(row).toHaveAttribute('data-state', 'unverified');
        view.rerender(<DiagnosticAlertsPanel alerts={[]} observations={[{...alert, level: 0}]} connected now={3000}/>);
        expect(row).toHaveAttribute('data-state', 'resolved');
    });
    it('keeps distinct component sources and immediately updates severity without reordering headers', () => {
        const first = report(), second = report(1, 'other/detail', 'source-B');
        const view = render(<DiagnosticAlertsPanel alerts={[first, second]} connected now={0}/>, {wrapper});
        const headers = panel().getAllByRole('button').filter(button => button.hasAttribute('aria-expanded'));
        view.rerender(<DiagnosticAlertsPanel alerts={[first, {...second, level: 2, reportedLevel: 2}]} connected now={1000}/>);
        expect(panel().getAllByRole('button').filter(button => button.hasAttribute('aria-expanded'))).toEqual(headers);
        expect(headers[1]).toHaveTextContent('Error'); expect(headers[1]).toHaveTextContent('source-B');
    });
});
it('does not let a resolved ERROR label a currently warning group as ERROR', () => {
    const error = report(2, 'component/old-error'), warning = report(1, 'component/current-warning');
    const view = render(<DiagnosticAlertsPanel alerts={[error, warning]} connected now={0}/>, {wrapper});
    view.rerender(<DiagnosticAlertsPanel alerts={[warning]} observations={[{...error, level: 0}, warning]} connected now={1000}/>);
    const header = screen.getByTestId('diagnostic-alert-group');
    expect(header).toHaveTextContent('Warning'); expect(header).not.toHaveTextContent('Error');
    fireEvent.click(group());
    const rows = screen.getAllByTestId('diagnostic-alert-episode');
    expect(rows[0]).toHaveAttribute('data-state', 'resolved'); expect(rows[0]).toHaveTextContent('Error');
    expect(rows[1]).toHaveAttribute('data-state', 'active');
});
