import type {Status} from '../types/ros';
import {useValidatedTelemetry} from './useValidatedTelemetry';
import {decodeStatus, retainStatusWarning} from '../utils/statusTelemetry';

export const useStatus = (): Status =>
    useValidatedTelemetry<Status>('status', {}, decodeStatus, retainStatusWarning).telemetry.data;
