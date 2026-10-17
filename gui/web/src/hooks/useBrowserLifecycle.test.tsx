import {StrictMode, createRef} from 'react';
import {act, render, renderHook} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {useIsMobile} from './useIsMobile';
import {useIOSInstallPrompt} from './useIOSInstallPrompt';
import {useSectionFocus} from './useSectionFocus';

class TestMediaQuery extends EventTarget {
    matches = false;
    media = '(max-width: 767px)';
    onchange = null;
    addListener() { /* legacy API unused */ }
    removeListener() { /* legacy API unused */ }
}

const touchDescriptor = Object.getOwnPropertyDescriptor(navigator, 'maxTouchPoints');
const standaloneDescriptor = Object.getOwnPropertyDescriptor(navigator, 'standalone');
let previousDismissal: string | null;
beforeEach(() => {
    previousDismissal = sessionStorage.getItem('pwa-install-dismissed');
    sessionStorage.removeItem('pwa-install-dismissed');
});
const scrollDescriptor = Object.getOwnPropertyDescriptor(HTMLElement.prototype, 'scrollTo');
afterEach(() => {
    vi.restoreAllMocks();
    if (previousDismissal === null) sessionStorage.removeItem('pwa-install-dismissed');
    else sessionStorage.setItem('pwa-install-dismissed', previousDismissal);
    if (touchDescriptor) Object.defineProperty(navigator, 'maxTouchPoints', touchDescriptor);
    else Reflect.deleteProperty(navigator, 'maxTouchPoints');
    if (standaloneDescriptor) Object.defineProperty(navigator, 'standalone', standaloneDescriptor);
    else Reflect.deleteProperty(navigator, 'standalone');
    if (scrollDescriptor) Object.defineProperty(HTMLElement.prototype, 'scrollTo', scrollDescriptor);
    else Reflect.deleteProperty(HTMLElement.prototype, 'scrollTo');
});

describe('browser lifecycle', () => {
    it('tracks the media query with one effective listener through StrictMode and rerenders', () => {
        const query = new TestMediaQuery();
        query.matches = true;
        vi.spyOn(window, 'matchMedia').mockImplementation(() => query);
        const add = vi.spyOn(query, 'addEventListener');
        const remove = vi.spyOn(query, 'removeEventListener');
        const {result, rerender, unmount} = renderHook(() => useIsMobile(), {wrapper: StrictMode});
        expect(result.current).toBe(true);
        const registrations = add.mock.calls.length;
        rerender();
        expect(add).toHaveBeenCalledTimes(registrations);
        act(() => { query.matches = false; query.dispatchEvent(new Event('change')); });
        expect(result.current).toBe(false);
        unmount();
        expect(remove).toHaveBeenCalledTimes(registrations);
        for (const [event, listener] of add.mock.calls) expect(remove).toHaveBeenCalledWith(event, listener);
    });

    it('initializes the iOS prompt once and remembers explicit dismissal on remount', () => {
        vi.spyOn(navigator, 'userAgent', 'get').mockReturnValue('iPhone Safari');
        vi.spyOn(window, 'matchMedia').mockImplementation(() => new TestMediaQuery());
        sessionStorage.removeItem('pwa-install-dismissed');
        const {result, rerender, unmount} = renderHook(() => useIOSInstallPrompt(), {wrapper: StrictMode});
        expect(result.current.showPrompt).toBe(true);
        act(() => result.current.dismiss());
        rerender();
        expect(result.current.showPrompt).toBe(false);
        unmount();
        const next = renderHook(() => useIOSInstallPrompt());
        expect(next.result.current.showPrompt).toBe(false);
    });

    it('resets only the section scrollers and preserves horizontal scroll and focus', () => {
        const ref = createRef<HTMLDivElement>();
        const Fixture = ({context}: {context: string}) => {
            useSectionFocus(ref, context);
            return <main><div data-step-scroll><div ref={ref} tabIndex={-1}/></div></main>;
        };
        const scroll = vi.fn();
        Object.defineProperty(HTMLElement.prototype, 'scrollTo', {value: scroll, configurable: true});
        const view = render(<Fixture context="first"/>);
        expect(document.activeElement).toBe(ref.current);
        expect(scroll).toHaveBeenCalledTimes(2);
        expect(scroll).toHaveBeenCalledWith({top: 0, behavior: 'instant'});
        view.rerender(<Fixture context="first"/>);
        expect(scroll).toHaveBeenCalledTimes(2);
        view.rerender(<Fixture context="second"/>);
        expect(scroll).toHaveBeenCalledTimes(4);
    });
});

describe('iOS install prompt browser contract', () => {
    beforeEach(() => {
        vi.spyOn(navigator, 'userAgent', 'get').mockReturnValue('iPhone Safari');
    });

    const standaloneCases = [
        {name: 'absent', present: false, value: undefined},
        {name: 'false', present: true, value: false},
        {name: 'true', present: true, value: true},
        {name: 'string true', present: true, value: 'true'},
        {name: 'number one', present: true, value: 1},
        {name: 'null', present: true, value: null},
        {name: 'undefined', present: true, value: undefined},
    ];
    describe.each([false, true])('with display-mode standalone=%s', mediaMatches => {
        it.each(standaloneCases)('handles standalone $name without coercion', ({present, value}) => {
            if (present) Object.defineProperty(navigator, 'standalone', {configurable: true, value});
            else Reflect.deleteProperty(navigator, 'standalone');
            expect('standalone' in navigator).toBe(present);
            const query = new TestMediaQuery();
            query.matches = mediaMatches;
            const matchMedia = vi.spyOn(window, 'matchMedia').mockReturnValue(query);
            const {result} = renderHook(() => useIOSInstallPrompt(), {wrapper: StrictMode});
            expect(result.current.showPrompt).toBe(value !== true && !mediaMatches);
            if (value === true) expect(matchMedia).not.toHaveBeenCalled();
            else expect(matchMedia).toHaveBeenCalledWith('(display-mode: standalone)');
            expect(sessionStorage.getItem('pwa-install-dismissed')).toBeNull();
        });
    });

    it.each([
        {name: 'iPhone Safari', ua: 'iPhone Safari', platform: 'iPhone', touch: 1, show: true},
        {name: 'iPad desktop UA', ua: 'Macintosh Safari', platform: 'MacIntel', touch: 5, show: true},
        {name: 'desktop Safari', ua: 'Macintosh Safari', platform: 'MacIntel', touch: 0, show: false},
        ...['CriOS', 'FxiOS', 'OPiOS', 'EdgiOS'].map(browser => ({
            name: browser, ua: `iPhone Safari ${browser}`, platform: 'iPhone', touch: 1, show: false,
        })),
        {name: 'Android Chrome', ua: 'Android Chrome Safari', platform: 'Linux', touch: 5, show: false},
    ])('preserves browser eligibility for $name', ({ua, platform, touch, show}) => {
        vi.spyOn(navigator, 'userAgent', 'get').mockReturnValue(ua);
        vi.spyOn(navigator, 'platform', 'get').mockReturnValue(platform);
        Object.defineProperty(navigator, 'maxTouchPoints', {configurable: true, value: touch});
        Object.defineProperty(navigator, 'standalone', {configurable: true, value: false});
        const matchMedia = vi.spyOn(window, 'matchMedia').mockReturnValue(new TestMediaQuery());
        const {result} = renderHook(() => useIOSInstallPrompt(), {wrapper: StrictMode});
        expect(result.current.showPrompt).toBe(show);
        if (!show) expect(matchMedia).not.toHaveBeenCalled();
        expect(sessionStorage.getItem('pwa-install-dismissed')).toBeNull();
    });
});
