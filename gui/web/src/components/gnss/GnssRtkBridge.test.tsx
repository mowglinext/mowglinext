import {cleanup, render, screen} from '@testing-library/react';
import {afterEach, describe, expect, it} from 'vitest';
import {App} from 'antd';
import i18n from '../../i18n';
import en from '../../i18n/locales/en.json';
import fr from '../../i18n/locales/fr.json';
import {bridgeFixedWireFixture} from '../../test/gnssBridgeFixedFixture';
import {decodeGnssStatus} from '../../utils/gnssTelemetry';
import {GnssLiveDiagnosticsCard} from './GnssLiveDiagnosticsCard';
import {GnssLiveStatusSummaryCard} from './GnssLiveStatusSummaryCard';

afterEach(async () => {cleanup(); await i18n.changeLanguage('en');});
describe.each(['en', 'fr'])('bridge RTK presentation in %s', language => {
    it.each([2, 3])('keeps the diagnostic and compact summary consistent for 3D + mode %s', async mode => {
        await i18n.changeLanguage(language);
        const locale = language === 'fr' ? fr : en;
        const decoded = decodeGnssStatus({...bridgeFixedWireFixture, rtk_mode: mode});
        if (decoded.kind !== 'valid') throw new Error('Fixture rejected');
        render(<App>
            <GnssLiveDiagnosticsCard gnssStatus={decoded.data} latitude={undefined} longitude={undefined} altitudeM={undefined} horizontalAccuracyM={decoded.data.horizontal_accuracy_m}/>
            <GnssLiveStatusSummaryCard gnssStatus={decoded.data} selectedReceiverFamily="auto"/>
        </App>);
        const compact = mode === 3 ? locale.gpsStatus.compactRtkFixed : locale.gpsStatus.compactRtkFloat;
        const label = mode === 3 ? locale.gpsStatus.rtkFixed : locale.gpsStatus.rtkFloat;
        expect(screen.getByTestId('gnss-fix-status-value')).toHaveTextContent(compact);
        expect(screen.getByTestId('gnss-rtk-status-group')).toHaveTextContent(label);
        expect(screen.getAllByText(compact).length).toBeGreaterThanOrEqual(2);
        expect(screen.getAllByText(label).length).toBeGreaterThanOrEqual(2);
    });
});
