import type {ComponentProps, ReactNode} from 'react';
import {Input, Space} from 'antd';

type Addons = {before?: ReactNode; after?: ReactNode};
type Props = Addons & (
    | (Omit<ComponentProps<typeof Input>, 'addonBefore' | 'addonAfter'> & {password?: false})
    | (Omit<ComponentProps<typeof Input.Password>, 'addonBefore' | 'addonAfter'> & {password: true})
);

/** Keep one form control: injected values, events, IDs and refs reach Input.
 * Password mode retains Ant Design's visibility control and focus handling. */
export function TextWithAddon({before, after, password, style, size, ...props}: Props) {
    const hasBefore = before !== undefined && before !== null && before !== '';
    const hasAfter = after !== undefined && after !== null && after !== '';
    if (!hasBefore && !hasAfter) {
        return password ? <Input.Password {...props} size={size} style={style}/>
            : <Input {...props} size={size} style={style}/>;
    }
    const inputStyle = {flex: '1 1 auto', minWidth: 0};
    return <Space.Compact size={size} style={{width: '100%', maxWidth: '100%', minWidth: 0, ...style}}>
        {hasBefore && <Space.Addon style={{flex: '0 0 auto', whiteSpace: 'nowrap'}}>{before}</Space.Addon>}
        {password ? <Input.Password {...props} size={size} style={inputStyle}/>
            : <Input {...props} size={size} style={inputStyle}/>}
        {hasAfter && <Space.Addon style={{flex: '0 0 auto', whiteSpace: 'nowrap'}}>{after}</Space.Addon>}
    </Space.Compact>;
}
