set -u
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUTDIR="/tmp/unicore_freset_signalgroup_probe_${STAMP}"
mkdir -p "$OUTDIR"
cat > "$OUTDIR/probe.py" <<'PY'
import os, sys, time, json, termios, select
DEV = os.environ.get('UNICORE_DEV', '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
LOG = os.environ['UNICORE_LOG']
STATE = os.environ['UNICORE_STATE']
BAUD_MAP = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
    460800: termios.B460800,
    921600: termios.B921600,
}

def log(msg):
    with open(LOG, 'a', encoding='utf-8', errors='replace') as f:
        f.write(msg)
        if not msg.endswith('\n'):
            f.write('\n')

def record(event, **kwargs):
    with open(STATE, 'a', encoding='utf-8') as f:
        f.write(json.dumps({'ts': time.time(), 'event': event, **kwargs}) + '\n')

class SerialFD:
    def __init__(self, path, baud):
        self.path = path
        self.baud = baud
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = attrs[2] | termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[2] = attrs[2] & ~termios.PARENB
        attrs[2] = attrs[2] & ~termios.CSTOPB
        attrs[3] = 0
        attrs[4] = BAUD_MAP[baud]
        attrs[5] = BAUD_MAP[baud]
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        record('open', baud=baud)
    def write(self, data):
        return os.write(self.fd, data)
    def read(self, n=4096, timeout=0.25):
        r,_,_ = select.select([self.fd], [], [], timeout)
        if self.fd not in r:
            return b''
        try:
            return os.read(self.fd, n)
        except BlockingIOError:
            return b''
    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass

def collect(ser, seconds=1.0):
    end = time.time() + seconds
    chunks = []
    while time.time() < end:
        data = ser.read(4096, timeout=0.2)
        if data:
            chunks.append(data)
        else:
            time.sleep(0.05)
    return b''.join(chunks)

def drain(ser, seconds=0.5):
    raw = collect(ser, seconds)
    record('drain', baud=ser.baud, bytes=len(raw))
    if raw:
        log(f"DRAIN@{ser.baud}: {raw!r}")
    return raw

def send_and_collect(ser, label, payload, wait=2.0, drain_first=True):
    if drain_first:
        drain(ser, 0.5)
    log(f"SEND {label}@{ser.baud}: {payload!r}")
    ser.write(payload)
    record('send', label=label, baud=ser.baud, bytes=list(payload))
    raw = collect(ser, wait)
    log(f"RECV {label}@{ser.baud}: {raw!r}")
    record('recv', label=label, baud=ser.baud, size=len(raw))
    return raw

def scan_versiona(bauds, wait=1.5):
    results = []
    for baud in bauds:
        try:
            ser = SerialFD(DEV, baud)
            raw = send_and_collect(ser, f'VERSIONA_SCAN_{baud}', b'VERSIONA\r\n', wait=wait)
            ser.close()
            text = raw.decode('ascii', errors='replace')
            ok = ('#VERSIONA' in text) or ('$command,VERSIONA,response: OK' in text)
            results.append({'baud': baud, 'ok': ok, 'text': text})
            record('scan_result', baud=baud, ok=ok)
            if ok:
                return baud, results
        except Exception as e:
            results.append({'baud': baud, 'ok': False, 'error': str(e)})
            record('scan_error', baud=baud, error=str(e))
    return None, results

bauds = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
record('start', device=DEV)
active, scan = scan_versiona(bauds)
record('pre_freset_detected', baud=active)
log('PRE_FRESET_SCAN=' + json.dumps(scan))
if active is None:
    log('FAILED: no active baud before FRESET')
    sys.exit(3)
ser = SerialFD(DEV, active)
send_and_collect(ser, 'PRE_CONFIG', b'CONFIG\r\n', wait=2.0)
send_and_collect(ser, 'PRE_VERSIONA', b'VERSIONA\r\n', wait=2.0)
send_and_collect(ser, 'FRESET', b'FRESET\r\n', wait=3.0, drain_first=False)
ser.close()
time.sleep(10)
active2, scan2 = scan_versiona(bauds, wait=2.0)
record('post_freset_detected', baud=active2)
log('POST_FRESET_SCAN=' + json.dumps(scan2))
if active2 is None:
    log('FAILED: no active baud after FRESET')
    sys.exit(4)
ser = SerialFD(DEV, active2)
send_and_collect(ser, 'POST_FRESET_VERSIONA', b'VERSIONA\r\n', wait=2.0)
send_and_collect(ser, 'POST_FRESET_CONFIG', b'CONFIG\r\n', wait=2.0)
send_and_collect(ser, 'RECOVER_COM1_460800', b'CONFIG COM1 460800 8 n 1\r\n', wait=2.0)
ser.close()
time.sleep(5)
active3, scan3 = scan_versiona(bauds, wait=2.0)
record('post_com1_detected', baud=active3)
log('POST_COM1_SCAN=' + json.dumps(scan3))
if active3 is None:
    log('FAILED: no active baud after COM1 recovery')
    sys.exit(5)
ser = SerialFD(DEV, active3)
send_and_collect(ser, 'POST_COM1_VERSIONA', b'VERSIONA\r\n', wait=2.0)
send_and_collect(ser, 'POST_COM1_CONFIG', b'CONFIG\r\n', wait=2.5)
send_and_collect(ser, 'SIGNALGROUP_2_0', b'CONFIG SIGNALGROUP 2 0\r\n', wait=4.0)
raw_after = collect(ser, 3.0)
log(f'POST_SIGNALGROUP_COLLECT@{ser.baud}: {raw_after!r}')
record('post_signalgroup_collect', baud=ser.baud, bytes=len(raw_after))
ser.close()
time.sleep(8)
active4, scan4 = scan_versiona(bauds, wait=2.0)
record('post_signalgroup_detected', baud=active4)
log('POST_SIGNALGROUP_SCAN=' + json.dumps(scan4))
if active4 is not None:
    ser = SerialFD(DEV, active4)
    send_and_collect(ser, 'POST_SIGNALGROUP_VERSIONA', b'VERSIONA\r\n', wait=2.0)
    send_and_collect(ser, 'POST_SIGNALGROUP_CONFIG', b'CONFIG\r\n', wait=2.5)
    ser.close()
record('done')
PY
if docker ps --format '{{.Names}}' | grep -qx mowgli-gps; then
  echo before_stop_running > "$OUTDIR/mowgli_gps_state.txt"
  docker stop -t 20 mowgli-gps > "$OUTDIR/mowgli_gps_stop.log" 2>&1
else
  echo before_stop_not_running > "$OUTDIR/mowgli_gps_state.txt"
fi
UNICORE_DEV=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
UNICORE_LOG="$OUTDIR/probe.log" \
UNICORE_STATE="$OUTDIR/probe_state.jsonl" \
python3 "$OUTDIR/probe.py" > "$OUTDIR/probe.stdout" 2> "$OUTDIR/probe.stderr" || true
if ! docker ps --format '{{.Names}}' | grep -qx mowgli-gps; then
  docker start mowgli-gps > "$OUTDIR/mowgli_gps_start.log" 2>&1 || true
fi
printf '%s\n' "$OUTDIR"
