import {useEffect, useState} from 'react';
import {isJsonObject} from '../utils/json';
import {useApi} from './useApi';
import {parseHardwareBackendInfo, unconfirmedHardwareBackend, type HardwareBackendInfo} from '../constants/hardwareBackends';

export const useHardwareBackend = (): HardwareBackendInfo & {loading: boolean} => {
    const guiApi = useApi();
    const [info, setInfo] = useState<HardwareBackendInfo>(() => unconfirmedHardwareBackend('loading'));
    useEffect(() => {
        let active = true;
        const failed = (error: unknown) => {
            const absent = typeof error === 'object' && error !== null && 'status' in error && (error.status === 404 || error.status === 405);
            const payload = typeof error === 'object' && error !== null && 'error' in error ? error.error : undefined;
            const unsupported = isJsonObject(payload) && payload.state === 'unsupported';
            if (active) setInfo(unconfirmedHardwareBackend(unsupported ? 'unsupported' : absent ? 'legacy_absent' : 'unavailable'));
        };
        void guiApi.request<unknown>({path: '/settings/hardware-backend', method: 'GET', format: 'json'})
            .then(response => {
                if (!active) return;
                if (response.error || !response.ok) { failed(response); return; }
                setInfo(parseHardwareBackendInfo(response.data));
            }).catch(failed);
        return () => { active = false; };
    }, [guiApi]);
    return {...info, loading: info.authority === 'loading'};
};
