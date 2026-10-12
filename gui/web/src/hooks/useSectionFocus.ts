import {useEffect, type RefObject} from 'react';

/** Reset the actual shell/section scrollers when the user changes context. */
export function useSectionFocus(ref: RefObject<HTMLElement | null>, context: string | number, ready = true) {
    useEffect(() => {
        const section = ref.current;
        if (!section || !ready) return;
        for (let parent = section.parentElement; parent; parent = parent.parentElement) {
            if (parent.tagName === 'MAIN' || parent.hasAttribute('data-step-scroll')) parent.scrollTop = 0;
        }
        section.focus({preventScroll: true});
    }, [context, ready, ref]);
}
