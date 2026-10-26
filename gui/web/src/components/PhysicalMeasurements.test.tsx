import {render, screen} from '@testing-library/react';
import {Statistic} from 'antd';
import {describe, expect, it} from 'vitest';
import {powerDisplayValue} from '../utils/physicalTelemetry';

describe('power Statistic fallback', () => {
    it.each([undefined, null, NaN, Infinity, '0'])('renders unknown %j as a dash rather than AntD default zero', value => {
        const {container} = render(<Statistic value={powerDisplayValue(value)} precision={2}/>);
        expect(screen.getByText('—')).toBeInTheDocument();
        expect(container.textContent).toBe('—');
        expect(screen.queryByText('NaN')).not.toBeInTheDocument();
    });
    it('renders an actually measured zero current', () => {
        const {container} = render(<Statistic value={powerDisplayValue(0)} precision={2}/>);
        expect(container.textContent).toBe('0.00');
        expect(screen.queryByText('—')).not.toBeInTheDocument();
    });
});
