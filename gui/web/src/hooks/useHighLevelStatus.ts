import type {HighLevelStatus} from '../types/ros';
import {useValidatedTelemetry} from './useValidatedTelemetry';
import {decodeHighLevelStatus, retainHighLevelEmergency, accumulateHighLevelAssertions, type HighLevelDisplay} from '../utils/statusTelemetry';

export const useHighLevelStatus = () => {
    const {telemetry, start, stop} = useValidatedTelemetry<HighLevelStatus>(
        'highLevelStatus', {}, decodeHighLevelStatus, retainHighLevelEmergency, accumulateHighLevelAssertions,
    );
    const highLevelStatus: HighLevelDisplay = {
        ...telemetry.data,
        clearanceUnverified: telemetry.assertions.emergency === true && telemetry.data.emergency !== true,
    };
    return {highLevelStatus, telemetry, start, stop};
};
