import {render, screen} from '@testing-library/react';
import {describe, expect, it} from 'vitest';
import {
    booleanFormatter,
    booleanFormatterInverted,
    progressFormatter,
    progressFormatterSmall,
} from './utils';

describe('boolean formatters', () => {
    it.each(['On', 'Yes'])('recognizes the exact affirmative value %s', value => {
        const {rerender} = render(booleanFormatter(value));
        expect(screen.getByRole('img', {name: 'check-circle'})).toBeInTheDocument();
        rerender(booleanFormatterInverted(value));
        expect(screen.getByRole('img', {name: 'check-circle'})).toBeInTheDocument();
    });

    it.each(['Off', 'No', 'on', true, false, 0, null, undefined, {}, []].map(value => ({value})))(
        'keeps non-affirmative input $value negative without coercion', ({value}) => {
            const {rerender} = render(booleanFormatter(value));
            expect(screen.getByRole('img', {name: 'close-circle'})).toBeInTheDocument();
            rerender(booleanFormatterInverted(value));
            expect(screen.getByRole('img', {name: 'close-circle'})).toBeInTheDocument();
        },
    );
});

describe.each([
    ['normal', progressFormatter, 25],
    ['small', progressFormatterSmall, 11],
] as const)('%s progress formatter', (_name, format, size) => {
    it.each([0, 50, 100])('preserves progress %s and three-step dimensions', percent => {
        const {container} = render(format(percent));
        expect(screen.getByRole('progressbar')).toHaveAttribute('aria-valuenow', String(percent));
        const steps = container.querySelectorAll('.ant-progress-steps-item');
        expect(steps).toHaveLength(3);
        expect(steps[0]).toHaveStyle({height: `${size}px`});
        expect(container.querySelector('.ant-progress-text')).not.toBeInTheDocument();
    });
});
