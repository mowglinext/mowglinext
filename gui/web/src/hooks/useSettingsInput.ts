import {createContext, useContext, useState} from 'react';

export interface SettingsInputs {
    values: Record<string, string>;
    setValue: (key: string, value: string) => void;
}

export const SettingsInputsContext = createContext<SettingsInputs | null>(null);

/** Unapplied secret text stays in memory with its section, never in a PUT. */
export function useSettingsInput(key: string): [string, (value: string) => void] {
    const inputs = useContext(SettingsInputsContext);
    const [local, setLocal] = useState('');
    return inputs ? [inputs.values[key] ?? '', value => inputs.setValue(key, value)] : [local, setLocal];
}
