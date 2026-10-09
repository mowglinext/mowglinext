import {describe, it, expect, vi, beforeEach} from 'vitest';
import {render, screen, fireEvent} from '@testing-library/react';
import {App} from 'antd';
import {HardwareBackendBadge} from './HardwareBackendBadge.tsx';
import en from '../i18n/locales/en.json';

const mockBackend = vi.fn();
vi.mock('../hooks/useHardwareBackend.ts', () => ({useHardwareBackend: (): unknown => mockBackend()}));
const mockNavigate = vi.fn();
vi.mock('react-router-dom', () => ({useNavigate: () => mockNavigate}));
vi.mock('../theme/ThemeContext.tsx', () => ({
    useThemeMode: () => ({colors: {borderSubtle: '#333', text: '#fff'}}),
}));

const renderBadge = () => render(<App><HardwareBackendBadge/></App>);

describe('HardwareBackendBadge', () => {
    beforeEach(() => {
        mockNavigate.mockReset();
    });

    it('names the robot and its hardware backend', () => {
        mockBackend.mockReturnValue({backend: 'openmower', robotName: 'Garden-East', loading: false});
        renderBadge();
        const badge = screen.getByTestId('hardware-backend-badge');
        expect(badge).toHaveTextContent(`Garden-East · ${en.settingsHardwareBackend.shortNames.openmower}`);
        expect(badge).toHaveAttribute('data-backend', 'openmower');
    });

    it('shows the backend alone when the robot has no name yet', () => {
        mockBackend.mockReturnValue({backend: 'mowgli', robotName: '', loading: false});
        renderBadge();
        expect(screen.getByTestId('hardware-backend-badge'))
            .toHaveTextContent(en.settingsHardwareBackend.shortNames.mowgli);
    });

    it('renders nothing until the backend is known (no flash of the mowgli fallback)', () => {
        mockBackend.mockReturnValue({backend: 'mowgli', robotName: '', loading: true});
        renderBadge();
        expect(screen.queryByTestId('hardware-backend-badge')).toBeNull();
    });

    it('opens the hardware settings', () => {
        mockBackend.mockReturnValue({backend: 'openmower', robotName: 'R', loading: false});
        renderBadge();
        fireEvent.click(screen.getByTestId('hardware-backend-badge'));
        expect(mockNavigate).toHaveBeenCalledWith({pathname: '/settings', search: '?section=hardware'});
    });
});
