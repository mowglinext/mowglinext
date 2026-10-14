import {act, cleanup, fireEvent, render, screen} from '@testing-library/react';
import {afterEach, describe, expect, it, vi} from 'vitest';
import {createRef} from 'react';
import {ConfigProvider, Form, Space} from 'antd';
import type {InputNumberRef} from 'rc-input-number';
import {NumberWithAddon} from './NumberWithAddon';

afterEach(() => {cleanup(); vi.restoreAllMocks();});
const deprecation = (calls: unknown[][]) => calls.some(args => args.some(arg => typeof arg === 'string'
    && arg.includes('[antd: InputNumber]') && /addonAfter|addonBefore/.test(arg)));

describe('official compact numeric units', () => {
    it.each(['m', 's', 'V', 'A', 'PWM', '°', '%', 'rad²'])('renders accessible numeric input and visible %s without deprecated props', unit => {
        const errors = vi.spyOn(console, 'error'); const warnings = vi.spyOn(console, 'warn');
        const view = render(<NumberWithAddon id="value" aria-label="Measurement" aria-describedby="help" after={unit} value={0}
            min={-10} max={10} step={.5} precision={2} size="small" style={{width: '100%'}}/>);
        const input = screen.getByRole('spinbutton', {name: 'Measurement'});
        expect(input).toHaveValue('0.00'); expect(input).toHaveAttribute('id', 'value');
        expect(input).toHaveAttribute('aria-describedby', 'help');
        expect(input).toHaveAttribute('aria-valuemin', '-10'); expect(input).toHaveAttribute('aria-valuemax', '10');
        expect(view.container.querySelector('.ant-space-compact')).toHaveStyle({width: '100%', maxWidth: '100%'});
        expect(screen.getByText(unit)).toHaveClass('ant-space-addon');
        expect(deprecation(errors.mock.calls)).toBe(false); expect(deprecation(warnings.mock.calls)).toBe(false);
    });
    it('forwards editing, blur, Enter, disabled/status and the imperative ref', () => {
        const change = vi.fn(), blur = vi.fn(), enter = vi.fn(); const ref = createRef<InputNumberRef>();
        const view = render(<NumberWithAddon ref={ref} before="PWM" after="%" aria-label="Duty" defaultValue={2}
            min={0} max={10} step={.5} precision={2} onChange={change} onBlur={blur} onPressEnter={enter} status="error"/>);
        const input = screen.getByRole('spinbutton', {name: 'Duty'});
        act(() => ref.current?.focus()); expect(input).toHaveFocus();
        fireEvent.change(input, {target: {value: '4.5'}});
        expect(change).toHaveBeenLastCalledWith(4.5);
        fireEvent.keyDown(input, {key: 'Enter', code: 'Enter'}); fireEvent.blur(input);
        expect(enter).toHaveBeenCalledOnce(); expect(blur).toHaveBeenCalledOnce(); expect(input).toHaveValue('4.50');
        expect(view.container.querySelector('.ant-input-number')).toHaveClass('ant-input-number-status-error');
        view.rerender(<NumberWithAddon aria-label="Duty" after="%" disabled value={0}/>);
        expect(screen.getByRole('spinbutton', {name: 'Duty'})).toBeDisabled();
    });
    it('receives Form.Item injection, label association, initial values and numeric validation', async () => {
        const submit = vi.fn(), change = vi.fn();
        render(<Form initialValues={{distance: 0}} onFinish={submit} onValuesChange={change}>
            <Form.Item name="distance" label="Distance" rules={[{required: true}, {type: 'number', min: 0}]}>
                <NumberWithAddon after="m" style={{width: '100%'}}/>
            </Form.Item>
            <button type="submit">Submit</button>
        </Form>);
        const input = screen.getByRole('spinbutton', {name: 'Distance'});
        expect(input).toHaveValue('0');
        expect(screen.getByLabelText('Distance')).toBe(input);
        fireEvent.change(input, {target: {value: '2.5'}}); expect(change).toHaveBeenLastCalledWith({distance: 2.5}, {distance: 2.5});
        await act(async () => {fireEvent.click(screen.getByText('Submit')); await Promise.resolve();});
        expect(submit).toHaveBeenCalledWith({distance: 2.5});
        submit.mockClear();
        fireEvent.change(input, {target: {value: '-1'}});
        await act(async () => {fireEvent.click(screen.getByText('Submit')); await Promise.resolve();});
        await screen.findByText(/cannot be less than 0/);
        expect(input).toHaveAttribute('aria-invalid', 'true');
        expect(submit).not.toHaveBeenCalled();
    });
    it('preserves explicit compact and provider sizes and returns a plain input when optional units are unavailable', () => {
        const view = render(<ConfigProvider componentSize="large"><Space.Compact size="small">
            <NumberWithAddon aria-label="Small" after="s" value={1} size="small"/>
        </Space.Compact></ConfigProvider>);
        expect(view.container.querySelector('.ant-input-number')).toHaveClass('ant-input-number-sm');
        expect(screen.getByText('s')).toHaveClass('ant-space-addon-small');
        view.rerender(<ConfigProvider componentSize="large"><NumberWithAddon aria-label="Large" after="m" value={1}/></ConfigProvider>);
        expect(view.container.querySelector('.ant-input-number')).toHaveClass('ant-input-number-lg');
        view.rerender(<NumberWithAddon aria-label="Plain" after={undefined} value={1}/>);
        expect(view.container.querySelector('.ant-space-addon')).not.toBeInTheDocument();
        expect(view.container.querySelector('.ant-space-compact')).not.toBeInTheDocument();
        expect(screen.getByRole('spinbutton', {name: 'Plain'})).toHaveValue('1');
    });
});
