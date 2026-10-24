import {App} from 'antd';
import {useCallback} from 'react';
import {useTranslation} from 'react-i18next';
import {asyncEvent, reportAsyncError} from '../utils/async';
import {errorMessage} from '../utils/errors';

/** Existing handlers retain their specific errors; unexpected rejections get an operator toast. */
export function useAsyncHandler() {
    const {notification} = App.useApp();
    const {t} = useTranslation();
    const report = useCallback((error: unknown) => {
        reportAsyncError(error);
        notification.error({message: t('asyncButton.errorOccurred'), description: errorMessage(error)});
    }, [notification, t]);
    return useCallback(<Args extends unknown[], Result>(callback: (...args: Args) => Result) =>
        asyncEvent(callback, report), [report]);
}
