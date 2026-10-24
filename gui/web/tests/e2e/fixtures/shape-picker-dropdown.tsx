import '@ant-design/v5-patch-for-react-19';
import {StrictMode} from 'react';
import {createRoot} from 'react-dom/client';
import {Button, ConfigProvider, theme} from 'antd';
import {ShapePickerDropdown} from '../../../src/pages/map/components/ShapePickerDropdown';
import {ThemeProvider} from '../../../src/theme/ThemeContext';
import i18n from '../../../src/i18n';

export type Choice = {kind: 'shape'; shape: string; size: number}
    | {kind: 'emoji'; emoji: string; size: number} | {kind: 'corridor'};
declare global {interface Window {shapeChoices: Choice[]}}
window.shapeChoices = [];
const query = new URLSearchParams(location.search);
const mobile = query.has('mobile');
await i18n.changeLanguage(query.get('lang') ?? 'en');

createRoot(document.getElementById('root')!).render(
    <StrictMode><ThemeProvider><ConfigProvider theme={{algorithm: theme.darkAlgorithm}}>
        <button id="outside" style={{position: 'absolute', left: 10, top: 10}}>Outside</button>
        <div style={{position: 'absolute', left: mobile ? 170 : 500, top: query.has('edge') ? 12 : mobile ? 270 : 600}}>
            <ShapePickerDropdown
                placement={mobile ? 'top' : 'topLeft'}
                onDrawShape={(shape, size) => window.shapeChoices.push({kind: 'shape', shape, size})}
                onDrawEmoji={(emoji, size) => window.shapeChoices.push({kind: 'emoji', emoji, size})}
                onDrawLidarCorridor={() => window.shapeChoices.push({kind: 'corridor'})}
            >
                {mobile ? <Button aria-label="Mobile add">+</Button> : undefined}
            </ShapePickerDropdown>
        </div>
    </ConfigProvider></ThemeProvider></StrictMode>,
);
