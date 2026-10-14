import {StrictMode, createRef} from 'react';
import {act, render, renderHook} from '@testing-library/react';
import {afterEach, describe, expect, it, vi} from 'vitest';
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

const scrollDescriptor = Object.getOwnPropertyDescriptor(HTMLElement.prototype, 'scrollTo');
afterEach(() => {
    vi.restoreAllMocks();
    sessionStorage.removeItem('pwa-install-dismissed');
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
