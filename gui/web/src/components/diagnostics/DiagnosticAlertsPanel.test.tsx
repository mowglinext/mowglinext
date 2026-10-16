import {cleanup, render, screen} from '@testing-library/react';
import {afterEach, expect, it} from 'vitest';
import {DiagnosticAlertsPanel} from './DiagnosticAlertsPanel';

afterEach(cleanup);
it('does not turn a ten-second delivery gap into a visible stale/history alert', () => {
    render(<DiagnosticAlertsPanel connected now={10000} observations={[]} alerts={[{
        name: 'IMU', hardware_id: 'mowgli/imu', message: 'No data', level: 3,
        reportedLevel: 0, receivedAt: 0, values: [], notApplicable: false,
    }]}/>);
    expect(screen.queryByTestId('diagnostic-alert-group')).not.toBeInTheDocument();
    expect(screen.queryByText('State unverified')).not.toBeInTheDocument();
});

import {fireEvent} from '@testing-library/react';
import i18n from '../../i18n';
import en from '../../i18n/locales/en.json';
import fr from '../../i18n/locales/fr.json';
import {describe} from 'vitest';
import type {RequiredDiagnosticSource} from '../../utils/diagnosticAlertPresentation';
const required = (publication: number | null): RequiredDiagnosticSource[] => [{key:'imu',label:'IMU',publication,ageMs:0}];
describe.each(['en','fr'])('quiet reception alerts in %s', lang => {
    it('renders nothing at 10/30/59, one stable alert at 60/61/70/90, removes on valid recovery', async () => {
        await i18n.changeLanguage(lang);
        const locale=lang==='fr'?fr:en;
        const view=render(<DiagnosticAlertsPanel connected now={0} alerts={[]} sources={required(0)}/>);
        let row: HTMLElement | null=null;
        for (const second of [10,30,59,60,61,70,90]) {
            view.rerender(<DiagnosticAlertsPanel connected now={second*1000} alerts={[]} sources={required(null)}/>);
            const rows=screen.queryAllByTestId('diagnostic-reception-alert');
            expect(rows).toHaveLength(second<60?0:1);
            if(second===60)row=rows[0];
            if(second>=60){expect(rows[0]).toBe(row);expect(row).toHaveTextContent(locale.diagnosticReception.missing.replace('{{source}}','IMU'));}
        }
        view.rerender(<DiagnosticAlertsPanel connected now={90001} alerts={[]} sources={required(90001)}/>);
        expect(screen.queryByTestId('diagnostic-reception-alert')).not.toBeInTheDocument();
        expect(view.container).not.toHaveTextContent(/Resolved|Résolue|Unverified|non vérifié|Stale|Périmé/);
        await i18n.changeLanguage('en');
    });
    it('shows a confirmed error immediately, clears on OK with no retained episode', async () => {
        await i18n.changeLanguage(lang);
        const alert={name:'Motor',hardware_id:'fictional-motor',level:2,reportedLevel:2,message:'Confirmed fault',receivedAt:0,values:[],notApplicable:false};
        const view=render(<DiagnosticAlertsPanel connected now={0} alerts={[alert]}/>);
        fireEvent.click(screen.getByRole('button',{name:/Motor/}));
        expect(screen.getByTestId('diagnostic-alert-message')).toHaveTextContent('Confirmed fault');
        view.rerender(<DiagnosticAlertsPanel connected now={1} alerts={[]}/>);
        expect(screen.queryByTestId('diagnostic-alert-group')).not.toBeInTheDocument();
        expect(screen.queryByTestId('diagnostic-alert-episode')).not.toBeInTheDocument();
        await i18n.changeLanguage('en');
    });
});

it('coalesces a global disconnect after 60 seconds and keeps the indication until admissible ROS delivery resumes', () => {
    const sources=[...required(0),{key:'operation',label:'Robot',publication:0,ageMs:0}];
    const view=render(<DiagnosticAlertsPanel connected now={0} alerts={[]} sources={sources}/>);
    view.rerender(<DiagnosticAlertsPanel connected={false} now={1} alerts={[]} sources={sources.map(s=>({...s,publication:null}))}/>);
    view.rerender(<DiagnosticAlertsPanel connected={false} now={60001} alerts={[]} sources={sources.map(s=>({...s,publication:null}))}/>);
    expect(screen.getAllByTestId('diagnostic-reception-alert')).toHaveLength(1);
    expect(screen.getByTestId('diagnostic-reception-alert')).toHaveAttribute('data-source','communication');
    view.rerender(<DiagnosticAlertsPanel connected now={70000} alerts={[]} sources={sources}/>);
    expect(screen.getByTestId('diagnostic-reception-alert')).toHaveAttribute('data-source','communication');
    view.rerender(<DiagnosticAlertsPanel connected now={71000} alerts={[]} sources={sources.map(s=>({...s,publication:71000,ageMs:0}))}/>);
    expect(screen.queryByTestId('diagnostic-reception-alert')).not.toBeInTheDocument();
});
