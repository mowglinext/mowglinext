export type Severity = 'ERROR' | 'WARN' | 'INFO' | 'DEBUG' | 'OTHER';

const LEVEL_PATTERN = /\b(ERROR|ERR|FATAL|CRITICAL|WARN(?:ING)?|INFO|DEBUG|TRACE)\b/i;
const LAUNCH_SIGINT_PATTERN = /user interrupted with ctrl-c \(SIGINT\)/i;
const PROCESS_SIGINT_PATTERN = /process has died.*exit code -2\b/i;

function detectLevelSeverity(line: string): Severity {
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

export interface LogSeverityClassifier {
    detect(line: string): Severity;
    reset(): void;
}

export function createLogSeverityClassifier(): LogSeverityClassifier {
    let launchSigintSeen = false;

    return {
        detect(line: string): Severity {
            // ROS 2 launch reports an intentional shutdown as this WARNING,
            // followed by one ERROR per SIGINT-terminated node.
            if (LAUNCH_SIGINT_PATTERN.test(line)) {
                launchSigintSeen = true;
                return 'INFO';
            }
            if (launchSigintSeen && PROCESS_SIGINT_PATTERN.test(line)) {
                return 'INFO';
            }

            return detectLevelSeverity(line);
        },
        reset(): void {
            launchSigintSeen = false;
        },
    };
}
