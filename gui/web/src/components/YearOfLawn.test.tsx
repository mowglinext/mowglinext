import {afterEach, describe, expect, it, vi} from 'vitest';
import {render, screen} from '@testing-library/react';
import {ThemeProvider} from '../theme/ThemeContext.tsx';
import {YearOfLawn} from './YearOfLawn.tsx';

vi.mock('../hooks/useTimeFormat.tsx', () => ({useTimeFormat: () => ({timeZoneMode: 'utc'})}));

afterEach(() => vi.restoreAllMocks());

describe('YearOfLawn calendar', () => {
    it.each([0, 1, 2, 3, 4, 5, 6])('aligns weekday %i and counts activity without odometry', weekday => {
        const today = Date.UTC(2026, 9, 4 + weekday, 12); // Sunday through Saturday.
        vi.spyOn(Date, 'now').mockReturnValue(today);
        const sessions = [-1, 0, 1].map(offset => ({
            start_time: new Date(today + offset * 86400000).toISOString(),
            duration_sec: 3600, coverage_percent: 50, status: 'aborted',
        }));
        const {container} = render(<ThemeProvider><YearOfLawn sessions={sessions}/></ThemeProvider>);

        expect(screen.getByText('2 sessions')).toBeVisible();
        expect(screen.getByText('Active days').parentElement).toHaveTextContent('2');
        expect(screen.getByText('Current streak').parentElement).toHaveTextContent('2 days');
        const title = `${new Date(today).toLocaleDateString('en', {timeZone: 'UTC'})}: 1 session`;
        const todayCell = screen.getByText(title).parentElement;
        expect(todayCell).toHaveAttribute('y', String(18 + weekday * 15));
        expect(todayCell).toHaveAttribute('x', String(28 + 51 * 15));
        expect(container.querySelectorAll('rect')).toHaveLength(364);
    });
});
