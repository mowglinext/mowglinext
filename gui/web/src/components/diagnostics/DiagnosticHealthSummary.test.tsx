import {cleanup, render, screen} from '@testing-library/react';
import {afterEach, expect, it, vi} from 'vitest';
import {DiagnosticHealthSummary} from './DiagnosticHealthSummary';
import {deriveDiagnosticHealth} from '../../utils/diagnosticHealth';
import {healthy} from '../../test/diagnosticHealthFixture';
import {getColors} from '../../theme/colors';
vi.mock('../../theme/ThemeContext',()=>({useThemeMode:()=>({colors:getColors('dark')})}));
afterEach(cleanup);
it.each(['ok','unknown','warning','critical'] as const)('uses one %s conclusion for the title and relevant badge',level=>{
    const input=healthy();
    if(level==='unknown')input.snapshotFresh=false;
    if(level==='warning')input.batteryPercent=50;
    if(level==='critical')input.emergency={clearanceUnverified:true};
    render(<DiagnosticHealthSummary health={deriveDiagnosticHealth(input)} gpsLabel="Fixture Fixed" batteryPercent={input.batteryPercent} cpuTemp={40}/>);
    expect(screen.getByTestId('diagnostic-health-verdict')).toHaveAttribute('data-level',level);
    const id=level==='critical'?'emergency':level==='warning'?'battery':'cpu';
    expect(screen.getByTestId(`diagnostic-health-${id}`)).toHaveAttribute('data-level',level);
    expect(screen.getByTestId('diagnostic-health-cause')).not.toBeEmptyDOMElement();
});
it('does not let an unknown container mask an unresolved emergency cause',()=>{
    const input={...healthy(),snapshotFresh:false,emergency:{clearanceUnverified:true}};
    render(<DiagnosticHealthSummary health={deriveDiagnosticHealth(input)} gpsLabel="Unknown" batteryPercent={80}/>);
    expect(screen.getByTestId('diagnostic-health-cause')).toHaveTextContent(/emergency|urgence/i);
    expect(screen.getByTestId('diagnostic-health-containers')).toHaveAttribute('data-level','unknown');
    expect(screen.getByTestId('diagnostic-health-emergency')).toHaveAttribute('data-level','critical');
});

it('keeps the current cause keyboard accessible and severity width reserved without changing its conclusion', () => {
    const input = healthy();
    render(<DiagnosticHealthSummary health={deriveDiagnosticHealth(input)} gpsLabel="Fixture Fixed" batteryPercent={80}/>);
    const cause = screen.getByRole('region', {name: 'Current verdict cause'});
    expect(cause).toHaveAttribute('tabindex', '0'); expect(cause).toHaveStyle({overflowY: 'auto'});
    const badge = screen.getByTestId('diagnostic-health-ros');
    expect(badge.querySelectorAll('[aria-hidden="true"]')).toHaveLength(5);
    expect(badge).toHaveAttribute('data-level', 'ok');
});
