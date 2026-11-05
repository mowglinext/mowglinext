import {useCallback} from 'react';
import {useTranslation} from 'react-i18next';
import type {MapDisplayPreferenceOptions} from './useMapDisplayPreference';
import {MAP_ROTATION_LOCKED_KEY, mapRotationLocked} from './mapRotationConfig';
import {observeAsync} from '../../../utils/async';
import {requestErrorMessage} from '../../../utils/errors';

/** Mower-wide map rotation lock. `setConfig` rethrows on failure, so a failed
 * save is reported here instead of leaving the toggle silently unchanged. */
export function useMapRotationLock({config, setConfig, notification}: MapDisplayPreferenceOptions) {
    const {t} = useTranslation();
    const rotationLocked = mapRotationLocked(config);
    const setRotationLocked = useCallback((locked: boolean) => {
        const message = t('mapOffsetPanel.rotationLockSaveError');
        observeAsync(setConfig({[MAP_ROTATION_LOCKED_KEY]: String(locked)}), (error: unknown) => {
            notification.error({message, description: requestErrorMessage(error) ?? message});
        });
    }, [notification, setConfig, t]);
    return {rotationLocked, setRotationLocked};
}
