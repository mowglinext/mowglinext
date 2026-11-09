import '@ant-design/v5-patch-for-react-19';
import {StrictMode, useState} from 'react';
import {createRoot} from 'react-dom/client';
import {ThemeProvider} from '../../../src/theme/ThemeContext';
import {NotificationCenterProvider} from '../../../src/hooks/NotificationCenterProvider';
import {TimeFormatProvider} from '../../../src/hooks/TimeFormatProvider';
import {useNotificationCenter} from '../../../src/hooks/useNotificationCenter';
import {useTimeFormat} from '../../../src/hooks/useTimeFormat';
import {BTStateGraph} from '../../../src/components/BTStateGraph';
import {ObstaclesSection} from '../../../src/components/settings/ObstaclesSection';
import {UpdateChangelog} from '../../../src/components/settings/UpdateChangelog';
import '../../../src/i18n';

function Probe() {
    const [count, setCount] = useState(0);
    const notices = useNotificationCenter();
    const time = useTimeFormat();
    return <>
        <button onClick={() => setCount(n => n + 1)}>Increment</button>
        <button onClick={() => notices.push({level: 'info', title: 'local notice'})}>Push notice</button>
        <button onClick={() => time.setTimeZoneMode('utc')}>Use UTC</button>
        <output data-testid="local-count">{count}</output>
        <output data-testid="notice-count">{notices.items.length}</output>
        <output data-testid="zone">{time.timeZoneMode}</output>
        <BTStateGraph current="MOWING"/>
        <ObstaclesSection values={{}} onChange={() => {}}/>
        <UpdateChangelog repository="owner/repo"/>
    </>;
}

const root = document.getElementById('root');
if (!root) throw new Error('missing fixture root');
if (import.meta.hot) {
    let updates = 0;
    import.meta.hot.on('vite:afterUpdate', event => {
        root.dataset.refreshUpdates = String(++updates);
        root.dataset.refreshPaths = event.updates.map(update => update.path).join(',');
    });
}
createRoot(root).render(<StrictMode><ThemeProvider><NotificationCenterProvider><TimeFormatProvider>
    <Probe/>
</TimeFormatProvider></NotificationCenterProvider></ThemeProvider></StrictMode>);
