import '@testing-library/jest-dom/vitest'
import i18n from '../i18n'

// Pin the UI language to English so component tests are deterministic
// regardless of the jsdom navigator locale, and assert against one consistent
// language. (English is the fully-translated set; the language detector would
// otherwise vary with navigator.language under jsdom.)
i18n.changeLanguage('en')

// Mock window.matchMedia for antd's responsive observer
Object.defineProperty(window, 'matchMedia', {
    writable: true,
    value: (query: string) => ({
        matches: false,
        media: query,
        onchange: null,
        addListener: () => {},
        removeListener: () => {},
        addEventListener: () => {},
        removeEventListener: () => {},
        dispatchEvent: () => false,
    }),
});
