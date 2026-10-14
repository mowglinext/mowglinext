import React, {useState} from 'react';
import {createRoot} from 'react-dom/client';
import {ConfigProvider, Form, Modal} from 'antd';
import {NumberWithAddon} from '../../../src/components/inputs/NumberWithAddon';

function Fixture() {
    const [saved, setSaved] = useState<Record<string, number>>({});
    return <ConfigProvider>
        <Modal open title="Numeric layout fixture" width={720} footer={null}>
            <Form layout="vertical" initialValues={{fixed: 0, fluid: 1}} onFinish={setSaved}>
                <Form.Item name="fixed" label="Fixed-width datum">
                    <NumberWithAddon before="OpenMower latitude" style={{width: 280}} precision={3}/>
                </Form.Item>
                <Form.Item name="fluid" label="Fluid-width gain">
                    <NumberWithAddon after="PWM per metre per second" style={{width: '100%'}} size="small"/>
                </Form.Item>
                <button type="submit">Save fixture</button>
                <output>{JSON.stringify(saved)}</output>
            </Form>
        </Modal>
    </ConfigProvider>;
}
const root = document.getElementById('root');
if (root) createRoot(root).render(<Fixture/>);
