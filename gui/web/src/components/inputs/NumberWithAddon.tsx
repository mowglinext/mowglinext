import type {ReactNode, Ref} from 'react';
import {InputNumber, Space, type InputNumberProps} from 'antd';
import type {InputNumberRef} from 'rc-input-number';

type Props<T extends number | string> = Omit<InputNumberProps<T>, 'addonBefore' | 'addonAfter'> & {
    before?: ReactNode;
    after?: ReactNode;
    ref?: Ref<InputNumberRef>;
};

/** A single form control: Form.Item may inject value, onChange, id and ref here.
 * Units remain siblings of the numeric input, using Ant Design's compact addon. */
export function NumberWithAddon<T extends number | string = number>({before, after, ref, style, size, ...props}: Props<T>) {
    const hasBefore = before !== undefined && before !== null && before !== '';
    const hasAfter = after !== undefined && after !== null && after !== '';
    if (!hasBefore && !hasAfter) return <InputNumber<T> {...props} ref={ref} size={size} style={style}/>;
    const {width, ...inputStyle} = style ?? {};
    return <Space.Compact size={size} style={{width, maxWidth: '100%', minWidth: 0}}>
        {hasBefore && <Space.Addon style={{flex: '0 0 auto', whiteSpace: 'nowrap'}}>{before}</Space.Addon>}
        <InputNumber<T> {...props} ref={ref} size={size} style={{...inputStyle, width: 'auto', flex: '1 1 auto', minWidth: 0}}/>
        {hasAfter && <Space.Addon style={{flex: '0 0 auto', whiteSpace: 'nowrap'}}>{after}</Space.Addon>}
    </Space.Compact>;
}
