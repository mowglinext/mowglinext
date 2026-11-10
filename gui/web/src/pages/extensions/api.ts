import {httpBase} from '../../utils/apiHost';

export interface ExtensionManifest {
    id: string;
    version: string;
    source: {repository: string; revision: string};
    runtime: {kind: string; units?: string[]};
    profile: string;
    grants: string[];
    health: string;
}
export interface ExtensionStatus {
    id: string;
    version: string;
    managed: boolean;
    enabled: boolean;
    compatible: boolean;
    blockers: string[];
    recovery_required: boolean;
    error?: string;
    manifest: ExtensionManifest;
    digest: string;
    previous_version?: string;
    previous_digest?: string;
    observation: {process: string; application: string; data: string; identity?: string; services?: Record<string, string>};
}
export interface ExtensionOperation {
    action: string;
    id: string;
    phase: 'running' | 'succeeded' | 'failed';
    error?: string;
}
export interface ExtensionState {
    api: number;
    configured: boolean;
    entries: ExtensionStatus[];
    operation?: ExtensionOperation | null;
    error?: string;
}
export type ImportAction = 'install' | 'adopt' | 'upgrade';
export type ExtensionAction = 'enable' | 'disable' | 'rollback' | 'remove' | 'forget' | 'recover';
export interface ImportRequest {
    action: ImportAction;
    bundle: string;
    binding?: {project?: string; config_dir?: string; data_dir?: string};
}
export interface ExtensionReview {
    manifest: ExtensionManifest;
    digest: string;
    binding: {project: string; config_dir: string; data_dir: string};
    action: ImportAction;
    blockers: string[];
}

export async function extensionRequest<T>(path: string, body?: unknown, signal?: AbortSignal): Promise<T> {
    const response = await fetch(`${httpBase()}/api/system/extensions/${path}`, {
        method: body === undefined ? 'GET' : 'POST',
        headers: body === undefined ? undefined : {'Content-Type': 'application/json', 'X-Mowgli-Update': '1'},
        body: body === undefined ? undefined : JSON.stringify(body),
        signal,
    });
    const value: unknown = await response.json();
    if (!response.ok) {
        const reason = typeof value === 'object' && value !== null && 'error' in value && typeof value.error === 'string' ? value.error : `HTTP ${response.status}`;
        throw new Error(reason);
    }
    return value as T;
}

export function readBundle(file: File): Promise<string> {
    return new Promise((resolve, reject) => {
        const reader = new FileReader();
        reader.onerror = () => reject(new Error('read'));
        reader.onload = () => {
            try {
                if (typeof reader.result !== 'string') throw new Error('read');
                const value: unknown = JSON.parse(reader.result);
                if (typeof value !== 'object' || value === null || Array.isArray(value)) throw new Error('object');
                // Preserve duplicate keys and exact input for the host's strict parser.
                resolve(reader.result);
            } catch { reject(new Error('json')); }
        };
        reader.readAsText(file);
    });
}
