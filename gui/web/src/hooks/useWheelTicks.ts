import type {WheelTick} from '../types/ros';
import {useTopic} from './useTopic';
import {decodeWheelTicks} from '../utils/physicalTelemetry';

export const useWheelTicks = (): WheelTick =>
    useTopic<WheelTick>('ticks', {}, {select: raw => decodeWheelTicks(raw) ?? {}}).data;
