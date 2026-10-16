import {act,cleanup,renderHook,waitFor} from '@testing-library/react';
import {afterEach,expect,it,vi} from 'vitest';
import {useCalibrationStatus} from './useCalibrationStatus';
const mock=vi.hoisted(()=>({request:vi.fn()}));
vi.mock('./useApi.ts',()=>({useApi:()=>mock}));
afterEach(()=>{cleanup();mock.request.mockReset();});
it('distinguishes absent optional resources and transport loss from malformed reads and server errors',async()=>{
    mock.request.mockRejectedValueOnce({status:404});
    const hook=renderHook(useCalibrationStatus);
    await waitFor(()=>expect(hook.result.current.error).not.toBeNull());
    expect(hook.result.current.readError).toBeNull();
    for(const failure of [{status:503},new TypeError('Failed to fetch')]){
        mock.request.mockRejectedValueOnce(failure);await act(()=>hook.result.current.refresh());
        expect(hook.result.current.readError).toBeNull();
    }
    mock.request.mockResolvedValueOnce({data:{dock:null}});await act(()=>hook.result.current.refresh());
    expect(hook.result.current.readError).toBe('Invalid calibration status');
    mock.request.mockRejectedValueOnce({status:500,error:{error:'cannot read calibration'}});await act(()=>hook.result.current.refresh());
    expect(hook.result.current.readError).toBe('cannot read calibration');
    const data={dock:{present:false},imu:{present:false,error:'invalid artifact'},mag:{present:false}};
    mock.request.mockResolvedValueOnce({data});await act(()=>hook.result.current.refresh());
    expect(hook.result.current.readError).toBeNull();expect(hook.result.current.status).toEqual(data);
});
