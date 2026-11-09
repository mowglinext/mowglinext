import '@ant-design/v5-patch-for-react-19';
import {StrictMode} from 'react';
import {createRoot} from 'react-dom/client';
import {ConfigProvider, theme} from 'antd';
import {LidarCorridorsInfo} from '../../../src/pages/map/components/LidarCorridorsPanel';
import {ThemeProvider, useThemeMode} from '../../../src/theme/ThemeContext';
import i18n from '../../../src/i18n';

const query = new URLSearchParams(location.search);
await i18n.changeLanguage(query.get('lang') ?? 'en');

function Fixture() {
    const {colors} = useThemeMode();
    const edge = query.has('edge');
    return <ConfigProvider theme={{algorithm: theme.darkAlgorithm, token: {
        colorPrimary: colors.primary, colorText: colors.text,
        colorTextSecondary: colors.textSecondary, colorBgContainer: colors.bgCard,
        borderRadius: 12,
        fontFamily: '"Satoshi", "Inter", -apple-system, BlinkMacSystemFont, sans-serif',
    }}}>
        <div style={{position: 'absolute', left: edge ? 20 : 500, top: edge ? 12 : 300}}>
            <LidarCorridorsInfo editable={query.get('editable') === 'true'}/>
        </div>
    </ConfigProvider>;
}

createRoot(document.getElementById('root')!).render(
    <StrictMode><ThemeProvider><Fixture/></ThemeProvider></StrictMode>,
);
