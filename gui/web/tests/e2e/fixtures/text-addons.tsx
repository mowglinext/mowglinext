import {useState} from 'react';
import {createRoot} from 'react-dom/client';
import {ConfigProvider, Form, Modal} from 'antd';
import {TextWithAddon} from '../../../src/components/inputs/TextWithAddon';
function Fixture() {
    const [saved, setSaved] = useState(false);
    return <ConfigProvider><Modal open title="Text layout fixture" width={720} footer={null}>
        <Form layout="vertical" initialValues={{host: 'caster.example.test', password: 'fake-secret'}} onFinish={() => setSaved(true)}>
            <Form.Item name="host" label="Fixed caster" rules={[{required: true}]}>
                <TextWithAddon before="Host" after=".test" style={{width: 240}}/>
            </Form.Item>
            <Form.Item name="password" label="Fluid password" rules={[{required: true}]}>
                <TextWithAddon password before="Fictional credential" style={{width: '100%'}}/>
            </Form.Item>
            <TextWithAddon before="Your email (RTK2go username)" aria-label="Bounded email" style={{maxWidth: 380, marginBottom: 12}} value="fictional@example.test"/>
            <TextWithAddon password before="Disabled" aria-label="Disabled password" disabled value="fake-secret"/>
            <button type="submit">Save fixture</button><output>{saved ? 'Saved' : 'Not saved'}</output>
        </Form>
    </Modal></ConfigProvider>;
}
const root = document.getElementById('root');
if (root) createRoot(root).render(<Fixture/>);
