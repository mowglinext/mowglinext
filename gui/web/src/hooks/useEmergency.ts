import type {Emergency} from '../types/ros';
import {useValidatedTelemetry} from './useValidatedTelemetry';
import {decodeEmergency, retainEmergency, accumulateEmergencyAssertions, type EmergencyDisplay} from '../utils/statusTelemetry';

export const useEmergencyState = () => {
    const {telemetry} = useValidatedTelemetry<Emergency>(
        'emergency', {}, decodeEmergency, retainEmergency, accumulateEmergencyAssertions,
    );
    const {data, assertions} = telemetry;
    const clearanceUnverified = (assertions.active_emergency === true && data.active_emergency !== true)
        || (assertions.latched_emergency === true && data.latched_emergency !== true);
    // Keep actual payload bits unchanged for existing command-authorization consumers.
    const emergency: EmergencyDisplay = {...data, clearanceUnverified};
    return {emergency, telemetry};
};

export const useEmergency = (): EmergencyDisplay => useEmergencyState().emergency;
