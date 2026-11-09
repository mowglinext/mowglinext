import '@ant-design/v5-patch-for-react-19';
import React, {StrictMode, useEffect, useRef} from 'react';
import {createRoot} from 'react-dom/client';
import {App, Button, ConfigProvider, Modal, message, notification} from 'antd';

const lifecycle = {setups: 0, cleanups: 0, confirmed: 0, cancelled: 0, messagesClosed: 0, notificationsClosed: 0};
Object.assign(window, {compatLifecycle: lifecycle});
const root = createRoot(document.getElementById('root')!);

function Fixture() {
    const modal = useRef<ReturnType<typeof Modal.confirm> | null>(null);
    useEffect(() => {
        lifecycle.setups++;
        return () => {
            lifecycle.cleanups++;
            Modal.destroyAll();
            message.destroy();
            notification.destroy();
        };
    }, []);
    return <App>
        <Button type="primary">Wave baseline</Button>
        <Button onClick={() => {
            modal.current = Modal.confirm({
                title: 'Static confirmation', content: <div>Original content
                    <Button onClick={() => modal.current?.update({content: 'Updated content'})}>Update inside modal</Button>
                    <Button onClick={() => modal.current?.destroy()}>Destroy inside modal</Button>
                </div>,
                onOk: () => { lifecycle.confirmed++; },
                onCancel: () => { lifecycle.cancelled++; },
            });
        }}>Static confirm</Button>
        <Button onClick={() => modal.current?.update({content: 'Updated content'})}>Update modal</Button>
        <Button onClick={() => modal.current?.destroy()}>Destroy modal</Button>
        <Button onClick={() => {
            void message.success({key: 'compat-message', content: 'Static message', duration: 0,
                onClose: () => { lifecycle.messagesClosed++; }});
        }}>Static message</Button>
        <Button onClick={() => message.destroy('compat-message')}>Close message</Button>
        <Button onClick={() => notification.success({key: 'compat-notification', message: 'Static notification',
            description: 'Notification detail', duration: 0,
            onClose: () => { lifecycle.notificationsClosed++; }})}>Static notification</Button>
        <Button onClick={() => root.unmount()}>Unmount fixture</Button>
    </App>;
}

root.render(<StrictMode><ConfigProvider><Fixture/></ConfigProvider></StrictMode>);
