import {Popconfirm, type PopconfirmProps} from 'antd';
import type {ComponentType, MouseEvent} from 'react';

// AntD's ActionButton observes returned promises (spinner, close on resolution).
// Its public onConfirm type says void. Widen that type without wrapping the
// callback or changing the runtime component. Callers must report failures.
export const AsyncPopconfirm: ComponentType<Omit<PopconfirmProps, 'onConfirm'> & {
    onConfirm?: (event?: MouseEvent<HTMLElement>) => void | Promise<unknown>;
}> = Popconfirm;
