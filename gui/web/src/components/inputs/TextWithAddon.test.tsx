import {act, cleanup, fireEvent, render, screen, waitFor} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi, type MockInstance} from 'vitest';
import {createRef} from 'react';
import {ConfigProvider, Form, type InputRef} from 'antd';
import {TextWithAddon} from './TextWithAddon';

let errors: MockInstance<typeof console.error>, warnings: MockInstance<typeof console.warn>;
beforeEach(() => {errors = vi.spyOn(console, 'error'); warnings = vi.spyOn(console, 'warn');});
afterEach(() => {
    cleanup();
    for (const spy of [errors, warnings]) expect(spy.mock.calls.flat().join(' ')).not.toMatch(/\[antd: Input\].*addon(Before|After)/);
    vi.restoreAllMocks();
});

describe('public compact text addons', () => {
    it('keeps controlled values, both addons, ARIA, ref, focus and keyboard callbacks', () => {
        const ref = createRef<InputRef>(), change = vi.fn(), blur = vi.fn(), enter = vi.fn(), focus = vi.fn();
        const props = {ref, before: 'Host', after: '.test', id: 'host', 'aria-label': 'Caster', 'aria-describedby': 'hint',
            value: '', placeholder: 'caster.example.test', onChange: change, onBlur: blur, onPressEnter: enter, onFocus: focus};
        const view = render(<TextWithAddon {...props} status="error" size="small" style={{width: 240}}/>);
        const input = screen.getByRole('textbox', {name: 'Caster'});
        expect(input).toHaveAttribute('id', 'host'); expect(input).toHaveAttribute('aria-describedby', 'hint');
        expect(input).toHaveAttribute('placeholder', 'caster.example.test'); expect(input).toHaveValue('');
        act(() => ref.current?.focus()); expect(input).toHaveFocus(); expect(focus).toHaveBeenCalledOnce();
        fireEvent.change(input, {target: {value: 'fictional'}}); expect(change).toHaveBeenCalledOnce();
        view.rerender(<TextWithAddon {...props} value="fictional" status="error" size="small" style={{width: 240}}/>);
        expect(input).toHaveValue('fictional');
        fireEvent.keyDown(input, {key: 'Enter', code: 'Enter'}); fireEvent.blur(input);
        expect(enter).toHaveBeenCalledOnce(); expect(blur).toHaveBeenCalledOnce();
        expect(input).toHaveClass('ant-input-status-error', 'ant-input-sm');
        expect(screen.getByText('Host')).toHaveClass('ant-space-addon'); expect(screen.getByText('.test')).toHaveClass('ant-space-addon');
        expect(view.container.querySelector('.ant-space-compact')).toHaveStyle({width: '240px'});
    });
    it('preserves Password visibility, typing and ref without emitting edits on toggles', () => {
        const ref = createRef<InputRef>(), change = vi.fn(), visible = vi.fn();
        render(<TextWithAddon password ref={ref} before="Password" aria-label="Fictional password" defaultValue="fake-secret"
            onChange={change} visibilityToggle={{onVisibleChange: visible}}/>);
        const input = screen.getByLabelText('Fictional password');
        expect(input).toHaveAttribute('type', 'password');
        act(() => ref.current?.focus()); expect(input).toHaveFocus();
        fireEvent.change(input, {target: {value: 'fake-edited'}}); expect(change).toHaveBeenCalledOnce();
        fireEvent.mouseDown(screen.getByRole('img', {name: 'eye-invisible'}));
        fireEvent.click(screen.getByRole('img', {name: 'eye-invisible'}));
        expect(input).toHaveAttribute('type', 'text'); expect(input).toHaveValue('fake-edited'); expect(input).toHaveFocus();
        fireEvent.click(screen.getByRole('img', {name: 'eye'}));
        expect(input).toHaveAttribute('type', 'password'); expect(visible.mock.calls).toEqual([[true], [false]]);
        expect(change).toHaveBeenCalledOnce();
    });
    it.each([false, true])('preserves disabled controls including password mode %s', password => {
        const props = {before: 'Disabled', disabled: true, 'aria-label': 'Disabled field', value: 'fake'};
        const view = render(password ? <TextWithAddon {...props} password/> : <TextWithAddon {...props}/>);
        const input = screen.getByLabelText('Disabled field'); expect(input).toBeDisabled();
        if (password) {
            fireEvent.click(screen.getByRole('img', {name: 'eye-invisible'}));
            expect(input).toHaveAttribute('type', 'password');
        }
        expect(view.container.querySelector('.ant-space-addon')).toBeInTheDocument();
    });
    it('forwards Form.Item injection, required validation and exactly one submit with fictional credentials', async () => {
        const submit = vi.fn();
        render(<Form initialValues={{host: 'caster.example.test', password: 'fake-secret'}} onFinish={submit}>
            <Form.Item name="host" label="Caster" rules={[{required: true, message: 'Host required'}]}>
                <TextWithAddon before="Host" style={{width: '100%'}}/>
            </Form.Item>
            <Form.Item name="password" label="Credential" rules={[{required: true}]}>
                <TextWithAddon password before="Password"/>
            </Form.Item>
            <button type="submit">Save</button>
        </Form>);
        const host = screen.getByRole('textbox', {name: 'Caster'}), password = screen.getByLabelText('Credential');
        expect(host).toHaveAttribute('id', 'host'); expect(password).toHaveValue('fake-secret');
        fireEvent.change(host, {target: {value: 'edited.example.test'}});
        fireEvent.change(password, {target: {value: 'fake-updated'}});
        fireEvent.click(screen.getByRole('button', {name: 'Save'}));
        await waitFor(() => expect(submit).toHaveBeenCalledExactlyOnceWith({host: 'edited.example.test', password: 'fake-updated'}));
        submit.mockClear(); fireEvent.change(host, {target: {value: ''}});
        fireEvent.click(screen.getByRole('button', {name: 'Save'}));
        await screen.findByText('Host required'); expect(host).toHaveAttribute('aria-invalid', 'true');
        expect(submit).not.toHaveBeenCalled();
    });
    it('preserves whole-group layout styles, inherited size and plain inputs when addons are absent', () => {
        const view = render(<ConfigProvider componentSize="large"><TextWithAddon before="Email" aria-label="Email"
            style={{maxWidth: 380, marginBottom: 12}} value="fictional@example.test"/></ConfigProvider>);
        expect(view.container.querySelector('.ant-space-compact')).toHaveStyle({width: '100%', maxWidth: '380px', marginBottom: '12px'});
        expect(screen.getByRole('textbox')).toHaveClass('ant-input-lg');
        view.rerender(<TextWithAddon after={undefined} aria-label="Plain" value="unchanged" style={{width: 220}}/>);
        expect(view.container.querySelector('.ant-space-compact')).not.toBeInTheDocument();
        expect(screen.getByRole('textbox')).toHaveStyle({width: '220px'});
        view.rerender(<TextWithAddon password before="" aria-label="Plain password" value="fake-secret"/>);
        expect(view.container.querySelector('.ant-space-compact')).not.toBeInTheDocument();
        expect(screen.getByLabelText('Plain password')).toHaveAttribute('type', 'password');
    });
});
