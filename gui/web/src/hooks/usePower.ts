import type {Power} from '../types/ros';
import {useValidatedTelemetry} from './useValidatedTelemetry';
import {decodePower} from '../utils/physicalTelemetry';
import type {TelemetryDecode} from '../utils/statusTelemetry';

const decode = (raw: unknown): TelemetryDecode<Power> => {
    const data = decodePower(raw);
    return data === undefined ? {kind: 'invalid'} : {kind: 'valid', data};
};
const replace = (_previous: Power, next: Power): Power => next;

export const usePower = (): Power =>
    useValidatedTelemetry<Power>('power', {}, decode, replace).telemetry.data;
