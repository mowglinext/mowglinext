export type Severity = 'ERROR' | 'WARN' | 'INFO' | 'DEBUG' | 'OTHER';

const LEVEL_PATTERN = /\b(ERROR|ERR|FATAL|CRITICAL|WARN(?:ING)?|INFO|DEBUG|TRACE)\b/i;
const LAUNCH_SIGINT_PATTERN = /user interrupted with ctrl-c \(SIGINT\)/i;
const PROCESS_SIGINT_PATTERN = /process has died.*exit code -2\b/i;

export function detectLogSeverity(line: string): Severity {
    // ROS 2 launch logs an intentional SIGINT shutdown as WARNING followed by
    // ERROR. Handle only those two known messages before reading the level.
    if (LAUNCH_SIGINT_PATTERN.test(line) || PROCESS_SIGINT_PATTERN.test(line)) {
        return 'INFO';
    }

    const match = LEVEL_PATTERN.exec(line);
    if (!match) return 'OTHER';

    const token = match[1].toUpperCase();
    if (token === 'ERROR' || token === 'ERR' || token === 'FATAL' || token === 'CRITICAL') {
        return 'ERROR';
    }
    if (token === 'WARN' || token === 'WARNING') return 'WARN';
    if (token === 'INFO') return 'INFO';
    if (token === 'DEBUG' || token === 'TRACE') return 'DEBUG';
    return 'OTHER';
}
