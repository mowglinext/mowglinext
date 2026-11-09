import {useState} from 'react';
import {createRoot} from 'react-dom/client';
import {ConfigProvider, Space} from 'antd';
import '../../../src/i18n';
import {DiagnosticAlertsPanel} from '../../../src/components/diagnostics/DiagnosticAlertsPanel';
import type {CurrentDiagnosticAlert} from '../../../src/utils/diagnosticAlertHistory';

// Deterministic presentation clock only. This harness supplies already projected
// reports; it neither replaces telemetry freshness nor simulates physical time.
const burst: CurrentDiagnosticAlert[] = Array.from({length: 110}, (_, index) => ({
    name: `Burst/detail-${index}`, hardware_id: 'fictional-source', level: 2, reportedLevel: 2,
    message: `Fictional retained message ${index}`, values: [{key: 'reading', value: '0'}], receivedAt: 0, notApplicable: false,
}));
function Fixture() {
    const [now, setNow] = useState(0);
    const [alerts, setAlerts] = useState<CurrentDiagnosticAlert[]>([]);
    const [observations, setObservations] = useState<CurrentDiagnosticAlert[]>([]);
    return <ConfigProvider><main style={{padding: 12}}>
        <Space wrap style={{marginBottom: 12}}>
            <button onClick={() => {setAlerts(burst); setObservations(burst);}}>Begin fictional burst</button>
            <button onClick={() => {setAlerts([]); setObservations(burst.map(entry => ({...entry, level: 0, message: 'OK'}))); setNow(1000);}}>Resolve burst</button>
            <button onClick={() => setNow(value => value + 31000)}>Advance clock by 31 seconds</button>
            <button onClick={() => setNow(value => value + 360000)}>Advance clock by 6 minutes</button>
        </Space>
        <DiagnosticAlertsPanel alerts={alerts} observations={observations} connected now={now}/>
    </main></ConfigProvider>;
}
const root = document.getElementById('root');
if (root) createRoot(root).render(<Fixture/>);
