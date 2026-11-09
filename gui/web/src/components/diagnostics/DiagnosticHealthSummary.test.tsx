import {cleanup, render, screen} from '@testing-library/react';
import {afterEach, expect, it, vi} from 'vitest';
import {DiagnosticHealthSummary} from './DiagnosticHealthSummary';
import {deriveDiagnosticHealth} from '../../utils/diagnosticHealth';
import {healthy} from '../../test/diagnosticHealthFixture';
import {getColors} from '../../theme/colors';
vi.mock('../../theme/ThemeContext',()=>({useThemeMode:()=>({colors:getColors('dark')})}));
afterEach(cleanup);
it.each([true,false])('does not turn internal freshness=%s into normal display noise', snapshotFresh=>{
    const input={...healthy(),snapshotFresh};
    const view=render(<DiagnosticHealthSummary health={deriveDiagnosticHealth(input)} gpsLabel="FIXED" batteryPercent={80}/>);
    expect(screen.getByTestId('diagnostic-health-verdict')).toHaveTextContent('—');
    expect(screen.getByTestId('diagnostic-health-gnss')).toHaveTextContent('FIXED');
    expect(screen.getByTestId('diagnostic-health-containers')).not.toHaveClass('ant-tag-success');
    expect(view.container.querySelectorAll('.ant-tag')).toHaveLength(6);
    expect(view.container).not.toHaveTextContent(/unverified|stale|resolved|\bOK\b/i);
});
it.each(['warning','critical'] as const)('still presents %s immediately',level=>{
    const input=healthy();
    if(level==='warning')input.batteryPercent=50;
    else input.emergency={clearanceUnverified:true};
    render(<DiagnosticHealthSummary health={deriveDiagnosticHealth(input)} gpsLabel="FIXED" batteryPercent={input.batteryPercent}/>);
    expect(screen.getByTestId('diagnostic-health-verdict')).toHaveAttribute('data-level',level);
    expect(screen.getByTestId('diagnostic-health-cause')).not.toBeEmptyDOMElement();
});
