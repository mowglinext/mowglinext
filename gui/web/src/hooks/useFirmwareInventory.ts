import {useEffect, useState, useSyncExternalStore} from 'react';
import type {Status} from '../types/ros';
import {getMultiplexedSocket} from './multiplexedSocket';
import {firmwareInventoryState} from '../utils/versions';

const subscribeConnection = (notify: () => void) => getMultiplexedSocket().onStatusChange(notify);
const connectionSnapshot = () => getMultiplexedSocket().getStatus();

export function useFirmwareInventory() {
    // React rechecks the snapshot after subscribing, covering a connection change
    // between render and subscription. Connection is not evidence of acquisition.
    const connected = useSyncExternalStore(subscribeConnection, connectionSnapshot) === 'open';
    const [sample, setSample] = useState<{data: Status; lastAdvance: number | null}>({data: {}, lastAdvance: null});
    const [now, setNow] = useState(Date.now);
    useEffect(() => {
        const socket = getMultiplexedSocket();
        let stamp: string | null = null;
        let lastAdvance: number | null = null;
        const unsubscribeStatus = socket.onStatusChange(status => {
            if (status !== 'open') { stamp = null; lastAdvance = null; }
            setSample(previous => ({...previous, lastAdvance}));
        });
        const unsubscribe = socket.subscribe('status', raw => {
            if (!raw || typeof raw !== 'object') return;
            const data = raw as Status;
            if (data.stamp && (data.stamp.sec || data.stamp.nanosec)) {
                const nextStamp = `${data.stamp.sec}:${data.stamp.nanosec}`;
                if (stamp !== null && stamp !== nextStamp) lastAdvance = Date.now();
                stamp = nextStamp;
            }
            setSample({data, lastAdvance});
        });
        const timer = window.setInterval(() => setNow(Date.now()), 1000);
        return () => { unsubscribeStatus(); unsubscribe(); window.clearInterval(timer); };
    }, []);
    return {data: sample.data, state: firmwareInventoryState(sample.data, connected, sample.lastAdvance, now)};
}
