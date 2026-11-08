"""Run the OpenMower v1 rig in its own process, behind a same-shaped proxy.

The rig's 500 Hz thread stands in for microcontrollers: the LowLevel board must
report status every 100 ms and the xESCs every 20 ms. In the scenario process
it shared one GIL with the probe's rclpy executor threads, and on a GitHub
runner it was starved for up to ~0.5 s at a time while the C++ bridge's own
control tick never slipped more than a few ms. The bridge then correctly saw the
LowLevel board go silent (``ll_status_timeout_s`` 0.5 s), held the wheels and
re-ramped, and the speed/IMU-rate checks failed on the emulator's latency, not
the bridge's behaviour. In its own process the rig has its own interpreter.

``RemoteRig`` exposes what the scenarios use, unchanged:

* reading an attribute or item returns a COPY when it is plain data (numbers,
  strings, lists/tuples/deques/dicts of those), a proxy for an object, and a
  remote function for a method — so ``rig.ll.config['lift_period']``,
  ``rig.escs['left'].commands.maxlen`` and ``rig.escs['left'].reboot()`` work;
* assigning an attribute sets it in the rig (``rig.ll.paused = True``);
* ``with rig.lock:`` holds the rig's own lock in the child for the block, so a
  group of reads stays as consistent as it was in-process.

Copies are snapshots: mutating a returned list does not reach the rig (no
scenario does). Time bases match across processes: ``time.monotonic()`` is the
system-wide CLOCK_MONOTONIC on Linux, so ``rig._t0`` / ``rig.now_ms()`` compare
with the parent's monotonic stamps exactly as before.
"""

from __future__ import annotations

import collections
import io
import multiprocessing
import pickle
import threading
import time
import traceback
from typing import Any

_START_TIMEOUT_S = 20.0
_CLOSE_TIMEOUT_S = 5.0
_SWITCH_INTERVAL_S = 0.0005
# Builtins pickle can carry by value besides the ones it never offers to
# reducer_override (None, bools, int, float, str, bytes, list, tuple, dict, set).
_PLAIN_EXTRA = (collections.deque, complex, bytearray, range, slice)


class _NotPlain(Exception):
    pass


class _PlainPickler(pickle.Pickler):
    """Pickle builtin data only, at C speed: anything else aborts the dump.

    A per-element Python check of a 20000-entry command deque cost ~7 ms per
    read inside the rig's own interpreter — the starvation this module removes.
    """

    def reducer_override(self, obj):
        # A deque pickles by reference to its class, so the class comes here too.
        if type(obj) in _PLAIN_EXTRA or obj in _PLAIN_EXTRA:
            return NotImplemented
        raise _NotPlain


def _encode(value) -> bytes:
    """One reply: ('val', copy) for plain data, else a marker for a proxy."""
    buffer = io.BytesIO()
    try:
        _PlainPickler(buffer, pickle.HIGHEST_PROTOCOL).dump(('val', value))
        return buffer.getvalue()
    except _NotPlain:
        pass
    if isinstance(value, dict):
        # A mapping of objects (rig.escs): keys travel, values stay remote.
        return _reply('map', list(value.keys()))
    return _reply('fn' if callable(value) else 'ref', None)


def _reply(kind, value) -> bytes:
    return pickle.dumps((kind, value), pickle.HIGHEST_PROTOCOL)


def _resolve(root, path):
    obj = root
    for kind, key in path:
        obj = getattr(obj, key) if kind == 'attr' else obj[key]
    return obj


def _serve(conn, workdir: str, esc_type: str, ll_generation: str):
    """Child process: own the rig and answer requests until 'close'."""
    import sys

    from openmower_v1_emulator import OpenMowerV1Rig

    # The rig thread still shares this interpreter with the request thread.
    sys.setswitchinterval(_SWITCH_INTERVAL_S)
    try:
        rig = OpenMowerV1Rig(workdir, esc_type, ll_generation)
    except Exception:
        conn.send(('err', traceback.format_exc()))
        return
    conn.send(('ok', rig.paths))
    lock_depth = 0
    done = _reply('val', None)
    while True:
        try:
            op, path, args, kwargs = conn.recv()
        except EOFError:
            break
        try:
            if op == 'get':
                reply = _encode(_resolve(rig, path))
            elif op == 'set':
                setattr(_resolve(rig, path[:-1]), path[-1][1], args[0])
                reply = done
            elif op == 'call':
                reply = _encode(_resolve(rig, path)(*args, **kwargs))
            elif op == 'acquire':
                rig.lock.acquire()
                lock_depth += 1
                reply = done
            elif op == 'release':
                rig.lock.release()
                lock_depth -= 1
                reply = done
            elif op == 'close':
                while lock_depth > 0:
                    rig.lock.release()
                    lock_depth -= 1
                rig.close()
                conn.send_bytes(done)
                break
            else:
                reply = _reply('err', f'unknown rig op {op!r}')
        except Exception:
            reply = _reply('err', traceback.format_exc())
        # Already pickled (by _encode / _reply); the client's recv() unpickles.
        conn.send_bytes(reply)


class RemoteRigError(RuntimeError):
    """An exception raised inside the rig process, with its traceback."""


class _Client:
    def __init__(self, conn):
        self._conn = conn
        # The executor threads and the scenario thread may both touch the rig.
        self._mutex = threading.Lock()
        self.slow_calls = collections.deque(maxlen=2000)

    def request(self, op, path=(), args=(), kwargs=None):
        with self._mutex:
            started = time.monotonic()
            self._conn.send((op, tuple(path), tuple(args), kwargs or {}))
            raw = self._conn.recv_bytes()
            received = time.monotonic()
            kind, value = pickle.loads(raw)
            done = time.monotonic()
            # Diagnostics only: calls whose round trip or unpickle exceeds 20 ms.
            if done - started > 0.02:
                self.slow_calls.append((started, op, '.'.join(str(k) for _, k in path),
                                        len(raw), received - started, done - received))
        if kind == 'err':
            raise RemoteRigError(value)
        return kind, value


def _materialize(client, path, kind, value) -> Any:
    if kind == 'val':
        return value
    if kind == 'fn':
        return lambda *a, **k: _materialize(client, path, *client.request('call', path, a, k))
    if kind == 'map':
        return _RemoteMap(client, path, value)
    return _RemoteRef(client, path)


class _RemoteRef:
    _client: _Client
    _path: tuple

    def __init__(self, client, path):
        object.__setattr__(self, '_client', client)
        object.__setattr__(self, '_path', tuple(path))

    def _get(self, step) -> Any:
        path = self._path + (step,)
        return _materialize(self._client, path, *self._client.request('get', path))

    def __getattr__(self, name) -> Any:
        if name.startswith('__'):
            raise AttributeError(name)
        return self._get(('attr', name))

    def __setattr__(self, name, value):
        self._client.request('set', self._path + (('attr', name),), (value,))

    def __getitem__(self, key) -> Any:
        return self._get(('item', key))


class _RemoteMap(_RemoteRef):
    _keys: list

    def __init__(self, client, path, keys):
        super().__init__(client, path)
        object.__setattr__(self, '_keys', list(keys))

    def keys(self):
        return list(self._keys)

    def __iter__(self):
        return iter(self.keys())

    def __len__(self):
        return len(self._keys)

    def __contains__(self, key):
        return key in self._keys

    def items(self):
        return [(k, self[k]) for k in self._keys]

    def values(self):
        return [self[k] for k in self._keys]


class _RemoteLock:
    def __init__(self, client):
        self._client = client

    def __enter__(self):
        self._client.request('acquire')
        return self

    def __exit__(self, *exc):
        self._client.request('release')
        return False


class RemoteRig(_RemoteRef):
    """``OpenMowerV1Rig(workdir, esc_type, ll_generation)`` in a child process."""

    _process: multiprocessing.process.BaseProcess
    paths: dict
    lock: _RemoteLock

    def __init__(self, workdir: str, esc_type: str, ll_generation: str = 'v1-fw'):
        # spawn, not fork: the parent already runs rclpy/DDS threads.
        ctx = multiprocessing.get_context('spawn')
        parent_conn, child_conn = ctx.Pipe()
        process = ctx.Process(target=_serve, args=(child_conn, workdir, esc_type, ll_generation),
                              name=f'om-v1-rig-{esc_type}', daemon=True)
        process.start()
        child_conn.close()
        if not parent_conn.poll(_START_TIMEOUT_S):
            process.kill()
            raise RemoteRigError('rig process did not start')
        kind, value = parent_conn.recv()
        if kind == 'err':
            process.join(_CLOSE_TIMEOUT_S)
            raise RemoteRigError(value)
        super().__init__(_Client(parent_conn), ())
        object.__setattr__(self, '_process', process)
        object.__setattr__(self, 'paths', value)
        object.__setattr__(self, 'lock', _RemoteLock(self._client))

    def close(self):
        process = self._process
        try:
            if process.is_alive():
                self._client.request('close')
        except (EOFError, OSError, RemoteRigError):
            pass
        process.join(_CLOSE_TIMEOUT_S)
        if process.is_alive():
            process.kill()
            process.join(_CLOSE_TIMEOUT_S)
