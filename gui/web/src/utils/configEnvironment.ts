import type {ApiGetConfigResponse} from '../api/Api';
import {isJsonObject} from './json';

/** The generated environment contract is sparse; missing tileUri means no local overlay. */
export function decodeConfigEnvironment(value: unknown): ApiGetConfigResponse {
    if (!isJsonObject(value)) throw new Error('Expected a configuration environment object');
    if (!Object.prototype.hasOwnProperty.call(value, 'tileUri')) return {};
    if (typeof value.tileUri !== 'string') throw new Error('Configuration tileUri must be a string');
    return {tileUri: value.tileUri};
}
