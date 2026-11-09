import {useEffect, useRef, useState, type KeyboardEvent} from "react";
import {useIsMobile} from "../../hooks/useIsMobile";
import {resolveCasterPosition} from "../../utils/casterPosition";
import {DOCK_VISUAL_DEFAULTS} from "./dockGeometry";
import {MEASUREMENTS, validMeasurement, type MeasurementKey} from "./measurements";

export function useMeasurementEditor(values: Record<string, unknown>, onChange: (key: string, value: number) => void) {
    const narrow = useIsMobile();
    const [shortTouch, setShortTouch] = useState(() => window.matchMedia("(pointer: coarse) and (max-height: 600px)").matches);
    useEffect(() => {
        const query = window.matchMedia("(pointer: coarse) and (max-height: 600px)");
        const update = () => setShortTouch(query.matches);
        query.addEventListener("change",update);
        return () => query.removeEventListener("change",update);
    }, []);
    const mobile = narrow || shortTouch;
    const [invalidInput, setInvalidInput] = useState<MeasurementKey | null>(null);
    const [active, setActive] = useState<MeasurementKey | null>(null);
    const [session, setSession] = useState<{key: MeasurementKey; value: number | null} | null>(null);
    useEffect(() => {
        if (active && !mobile) {
            // Focus may scroll the field underneath the now-sticky preview.
            // Align it above the page's save bar after the compact preview lays out.
            document.getElementById("setting-"+active)?.scrollIntoView({block:"end",behavior:"instant"});
        }
    }, [active,mobile]);
    const trigger = useRef<HTMLElement | null>(null);
    const close = () => {
        setSession(null);
        requestAnimationFrame(() => {if(trigger.current?.isConnected) trigger.current.focus({preventScroll:true});});
    };
    const open = (key: MeasurementKey) => {
        trigger.current = document.getElementById("setting-"+key);
        const raw = key === "caster_x_offset" ? resolveCasterPosition(values) : values[key] ??
            DOCK_VISUAL_DEFAULTS[key as keyof typeof DOCK_VISUAL_DEFAULTS];
        const n = raw == null || raw === "" ? NaN : Number(raw);
        setSession({key, value: Number.isFinite(n) ? n : null});
    };
    return {
        active: mobile ? null : active, session, invalid: invalidInput === active,
        close, dismiss: () => setActive(null),
        apply: (key: MeasurementKey, value: number) => {onChange(key,value); close();},
        fieldProps: (key: MeasurementKey) => ({
            min: MEASUREMENTS[key].min,
            onFocus: () => {if (!mobile) {setActive(key); setInvalidInput(null);}},
            onInput: (text: string) => {
                if (!mobile) setInvalidInput(text.trim() && validMeasurement(key,Number(text)) ? null : key);
            },
            onBlur: () => setInvalidInput(null),
            onClick: () => {if (mobile) open(key);},
            onKeyDown: (event: KeyboardEvent<HTMLInputElement>) => {
                if (mobile && (event.key === "Enter" || event.key === " ")) {
                    event.preventDefault(); open(key);
                }
                if (!mobile && event.key === "Escape") setActive(null);
            },
            readOnly: mobile,
            controls: !mobile,
            inputMode: mobile ? "none" as const : "decimal" as const,
            "aria-haspopup": mobile ? "dialog" as const : undefined,
        }),
    };
}
export type MeasurementEditorState = ReturnType<typeof useMeasurementEditor>;
