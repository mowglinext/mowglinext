package api

import (
	"encoding/json"
	"fmt"
	"net/http/httptest"
	"strings"
	"testing"
	"time"

	"github.com/gorilla/websocket"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"github.com/vmihailenco/msgpack/v5"
)

type testDiagStatus struct {
	Name       string            `json:"name"`
	HardwareID string            `json:"hardware_id"`
	Level      int               `json:"level"`
	Message    string            `json:"message"`
	Values     []json.RawMessage `json:"values"`
}

type testDiagArray struct {
	Header struct {
		Stamp struct {
			Sec     int `json:"sec"`
			Nanosec int `json:"nanosec"`
		} `json:"stamp"`
	} `json:"header"`
	Status []testDiagStatus `json:"status"`
}

// diagArray builds a DiagnosticArray stamped sec; each status is
// "name:level:message:value".
func diagArray(t *testing.T, sec int, statuses ...string) []byte {
	t.Helper()
	var a testDiagArray
	a.Header.Stamp.Sec = sec
	for _, s := range statuses {
		parts := strings.SplitN(s, ":", 4)
		require.Len(t, parts, 4)
		var level int
		_, err := fmt.Sscan(parts[1], &level)
		require.NoError(t, err)
		value, err := json.Marshal(map[string]string{"key": "v", "value": parts[3]})
		require.NoError(t, err)
		a.Status = append(a.Status, testDiagStatus{Name: parts[0], HardwareID: "hw", Level: level,
			Message: parts[2], Values: []json.RawMessage{value}})
	}
	raw, err := json.Marshal(a)
	require.NoError(t, err)
	return raw
}

func decodeDiag(t *testing.T, frame []byte) testDiagArray {
	t.Helper()
	var a testDiagArray
	require.NoError(t, json.Unmarshal(frame, &a))
	return a
}

// diagSummary renders a frame as "sec|name=level/value,..." for compact asserts.
func diagSummary(t *testing.T, frame []byte) string {
	t.Helper()
	a := decodeDiag(t, frame)
	parts := make([]string, 0, len(a.Status))
	for _, s := range a.Status {
		var v map[string]string
		require.NoError(t, json.Unmarshal(s.Values[0], &v))
		parts = append(parts, fmt.Sprintf("%s=%d/%s", s.Name, s.Level, v["value"]))
	}
	return fmt.Sprintf("%d|%s", a.Header.Stamp.Sec, strings.Join(parts, ","))
}

var diagT0 = time.Unix(1_000, 0)

func TestDiagnosticsRate_HoldsUnchangedReportsAndDeliversTheNewestLater(t *testing.T) {
	l := newDiagnosticsRateLimiter(time.Second)

	first := l.admit(diagT0, diagArray(t, 1, "a:0:ok:1", "b:0:ok:1"))
	assert.Equal(t, "1|a=0/1,b=0/1", diagSummary(t, first))

	assert.Nil(t, l.admit(diagT0.Add(100*time.Millisecond), diagArray(t, 2, "a:0:ok:2", "b:0:ok:2")))
	assert.Nil(t, l.admit(diagT0.Add(200*time.Millisecond), diagArray(t, 3, "a:0:ok:3", "b:0:ok:3")))
	assert.Empty(t, l.flush(diagT0.Add(900*time.Millisecond)), "nothing is due before the interval ends")

	wait, ok := l.nextFlush(diagT0.Add(900 * time.Millisecond))
	require.True(t, ok)
	assert.Equal(t, 100*time.Millisecond, wait)

	// The producer went quiet: the newest held report still arrives, with
	// the header of the array it was published in.
	frames := l.flush(diagT0.Add(time.Second))
	require.Len(t, frames, 1)
	assert.Equal(t, "3|a=0/3,b=0/3", diagSummary(t, frames[0]))
	_, ok = l.nextFlush(diagT0.Add(time.Second))
	assert.False(t, ok, "nothing is held after the flush")
	assert.Empty(t, l.arrays, "flushed arrays are forgotten")
}

func TestDiagnosticsRate_LevelAndMessageChangesPassAtOnce(t *testing.T) {
	l := newDiagnosticsRateLimiter(time.Second)
	require.NotNil(t, l.admit(diagT0, diagArray(t, 1, "a:0:ok:1", "b:0:ok:1")))

	warn := l.admit(diagT0.Add(50*time.Millisecond), diagArray(t, 2, "a:1:degraded:2", "b:0:ok:2"))
	assert.Equal(t, "2|a=1/2", diagSummary(t, warn), "only the changed status goes out early")

	back := l.admit(diagT0.Add(100*time.Millisecond), diagArray(t, 3, "a:0:ok:3", "b:0:ok:3"))
	assert.Equal(t, "3|a=0/3", diagSummary(t, back), "the return to OK is a change too")

	text := l.admit(diagT0.Add(150*time.Millisecond), diagArray(t, 4, "a:0:other text:4", "b:0:ok:4"))
	assert.Equal(t, "4|a=0/4", diagSummary(t, text), "a message change at the same level passes")

	frames := l.flush(diagT0.Add(time.Second))
	require.Len(t, frames, 1)
	assert.Equal(t, "4|b=0/4", diagSummary(t, frames[0]))
}

func TestDiagnosticsRate_DueReportsGoOutWithTheirOwnArray(t *testing.T) {
	l := newDiagnosticsRateLimiter(time.Second)
	require.NotNil(t, l.admit(diagT0, diagArray(t, 1, "a:0:ok:1", "b:0:ok:1")))
	// a is due again after one interval, a newly seen c goes out at once.
	out := l.admit(diagT0.Add(time.Second), diagArray(t, 2, "a:0:ok:2", "c:2:bad:2"))
	assert.Equal(t, "2|a=0/2,c=2/2", diagSummary(t, out))
}

func TestDiagnosticsRate_PassesThroughWhatItCannotInterpret(t *testing.T) {
	l := newDiagnosticsRateLimiter(time.Second)
	for _, raw := range []string{`not json`, `{"header":{}}`, `{"status":[]}`, `{"status":"x"}`} {
		assert.Equal(t, raw, string(l.admit(diagT0, []byte(raw))))
		assert.Equal(t, raw, string(l.admit(diagT0, []byte(raw))), "repeats are not held either")
	}

	// A status without a string name is the client's malformed case: never held.
	malformed := `{"header":{"stamp":{"sec":5,"nanosec":0}},"status":[{"name":7,"level":0}]}`
	for range 3 {
		out := l.admit(diagT0, []byte(malformed))
		assert.JSONEq(t, malformed, string(out))
	}
	assert.Empty(t, l.forwarded)
}

func TestDiagnosticsRate_KeepsUnknownTopLevelFields(t *testing.T) {
	l := newDiagnosticsRateLimiter(time.Second)
	in := `{"header":{"stamp":{"sec":1,"nanosec":2},"frame_id":"f"},"extra":true,"status":[{"name":"a","hardware_id":"","level":0,"message":"","values":[]}]}`
	assert.JSONEq(t, in, string(l.admit(diagT0, []byte(in))))
}

// The properties the client's freshness, transition and episode tracking
// depend on, checked over an interleaved stream of three producers.
func TestDiagnosticsRate_PreservesOrderingTransitionsAndTheFinalValue(t *testing.T) {
	l := newDiagnosticsRateLimiter(time.Second)
	type report struct {
		sec   int
		level int
		value string
	}
	in := map[string][]report{}
	var out [][]byte
	now := diagT0
	sec := 0
	for tick := range 300 { // 30 s at 10 Hz
		now = diagT0.Add(time.Duration(tick) * 100 * time.Millisecond)
		out = append(out, l.flush(now)...)
		var statuses []string
		producer := tick % 3
		for i := range 4 {
			name := fmt.Sprintf("p%d/s%d", producer, i)
			level := 0
			if (tick/7+i)%5 == 0 {
				level = 1 + i%2
			}
			value := fmt.Sprint(tick)
			statuses = append(statuses, fmt.Sprintf("%s:%d:m%d:%s", name, level, level, value))
			in[name] = append(in[name], report{sec + 1, level, value})
		}
		sec++
		if frame := l.admit(now, diagArray(t, sec, statuses...)); frame != nil {
			out = append(out, frame)
		}
	}
	out = append(out, l.flush(now.Add(time.Second))...)

	got := map[string][]report{}
	forwardedAt := map[string][]int{}
	for i, frame := range out {
		a := decodeDiag(t, frame)
		for _, s := range a.Status {
			var v map[string]string
			require.NoError(t, json.Unmarshal(s.Values[0], &v))
			got[s.Name] = append(got[s.Name], report{a.Header.Stamp.Sec, s.Level, v["value"]})
			forwardedAt[s.Name] = append(forwardedAt[s.Name], i)
		}
	}
	levels := func(rs []report) []int {
		var seq []int
		for _, r := range rs {
			if len(seq) == 0 || seq[len(seq)-1] != r.level {
				seq = append(seq, r.level)
			}
		}
		return seq
	}
	require.Len(t, got, len(in))
	for name, reports := range got {
		for i := 1; i < len(reports); i++ {
			assert.Greater(t, reports[i].sec, reports[i-1].sec, "%s: stamps only move forward", name)
		}
		assert.Equal(t, levels(in[name]), levels(reports), "%s: every level transition reaches the client", name)
		assert.Equal(t, in[name][len(in[name])-1], reports[len(reports)-1], "%s: the newest report is delivered", name)
		assert.Less(t, len(reports), len(in[name])/2, "%s: unchanged reports are thinned", name)
	}
}

func TestDiagnosticsRate_BoundsASteadyTenHertzArray(t *testing.T) {
	l := newDiagnosticsRateLimiter(time.Second)
	names := make([]string, 18)
	for i := range names {
		names[i] = fmt.Sprintf("universal_gnss/s%02d:0:steady:%d", i, i)
	}
	frames, statuses := 0, 0
	for tick := range 100 { // 10 s at 10 Hz
		now := diagT0.Add(time.Duration(tick) * 100 * time.Millisecond)
		batch := l.flush(now)
		if frame := l.admit(now, diagArray(t, tick+1, names...)); frame != nil {
			batch = append(batch, frame)
		}
		for _, frame := range batch {
			frames++
			statuses += len(decodeDiag(t, frame).Status)
		}
	}
	assert.Equal(t, 10, frames, "one array per second instead of ten")
	assert.Equal(t, 10*len(names), statuses)
}

func TestDiagnosticsRate_UntrackedKeysBeyondTheBoundAreNotHeld(t *testing.T) {
	l := newDiagnosticsRateLimiter(time.Second)
	for i := range diagnosticsMaxTrackedKeys {
		require.NotNil(t, l.admit(diagT0, diagArray(t, 1, fmt.Sprintf("k%d:0:ok:1", i))))
	}
	for range 3 {
		out := l.admit(diagT0, diagArray(t, 2, "overflow:0:ok:1"))
		assert.Equal(t, "2|overflow=0/1", diagSummary(t, out))
	}
	assert.Len(t, l.forwarded, diagnosticsMaxTrackedKeys)
}

func subscribeMultiplexDiagnostics(t *testing.T) (*types.MockRosProvider, *websocket.Conn, func()) {
	t.Helper()
	mock := types.NewMockRosProvider()
	server := httptest.NewServer(setupMowgliNextRouter(mock))
	conn := dialMultiplex(t, server)
	require.NoError(t, conn.WriteJSON(map[string]string{"op": "subscribe", "topic": "diagnostics"}))
	time.Sleep(50 * time.Millisecond)
	return mock, conn, func() { _ = conn.Close(); server.Close() }
}

func expectNoFrame(t *testing.T, conn *websocket.Conn, within time.Duration) {
	t.Helper()
	_ = conn.SetReadDeadline(time.Now().Add(within))
	_, _, err := conn.ReadMessage()
	require.Error(t, err, "no frame expected")
}

func TestMultiplexRoute_LimitsDiagnosticsPerStatus(t *testing.T) {
	mock, conn, done := subscribeMultiplexDiagnostics(t)
	defer done()

	start := time.Now()
	mock.Dispatch("diagnostics", diagArray(t, 1, "gnss:0:ok:1"))
	_, payload := readMultiplexFrame(t, conn)
	assert.Equal(t, "1|gnss=0/1", diagSummary(t, payload))

	mock.Dispatch("diagnostics", diagArray(t, 2, "gnss:0:ok:2"))
	mock.Dispatch("diagnostics", diagArray(t, 3, "gnss:0:ok:3"))
	// The held report arrives when its interval ends, without another publish.
	_ = conn.SetReadDeadline(time.Now().Add(2 * time.Second))
	_, raw, err := conn.ReadMessage()
	require.NoError(t, err)
	assert.GreaterOrEqual(t, time.Since(start), 900*time.Millisecond, "held until the interval ends")
	payload = multiplexPayload(t, raw)
	assert.Equal(t, "3|gnss=0/3", diagSummary(t, payload))

	// A level change is not delayed by the interval that just restarted.
	mock.Dispatch("diagnostics", diagArray(t, 4, "gnss:2:error:4"))
	_ = conn.SetReadDeadline(time.Now().Add(300 * time.Millisecond))
	_, raw, err = conn.ReadMessage()
	require.NoError(t, err, "a level change must not wait for the interval")
	assert.Equal(t, "4|gnss=2/4", diagSummary(t, multiplexPayload(t, raw)))
}

func TestMultiplexRoute_UnsubscribeStopsHeldDiagnostics(t *testing.T) {
	mock, conn, done := subscribeMultiplexDiagnostics(t)
	defer done()

	mock.Dispatch("diagnostics", diagArray(t, 1, "gnss:0:ok:1"))
	readMultiplexFrame(t, conn)
	mock.Dispatch("diagnostics", diagArray(t, 2, "gnss:0:ok:2"))
	require.NoError(t, conn.WriteJSON(map[string]string{"op": "unsubscribe", "topic": "diagnostics"}))
	expectNoFrame(t, conn, 1500*time.Millisecond)
}

func TestSubscriberRoute_DiagnosticsAreNotLimited(t *testing.T) {
	mock := types.NewMockRosProvider()
	server := httptest.NewServer(setupMowgliNextRouter(mock))
	defer server.Close()
	conn := dialSubscribe(t, server, "diagnostics")
	defer conn.Close()
	time.Sleep(50 * time.Millisecond)

	for sec := 1; sec <= 3; sec++ {
		mock.Dispatch("diagnostics", diagArray(t, sec, "gnss:0:ok:1"))
	}
	for range 3 {
		_ = conn.SetReadDeadline(time.Now().Add(time.Second))
		_, _, err := conn.ReadMessage()
		require.NoError(t, err)
	}
}

// multiplexPayload decodes a raw multiplex frame the same way as
// readMultiplexFrame, for tests that need their own read deadline.
func multiplexPayload(t *testing.T, raw []byte) []byte {
	t.Helper()
	var frame struct {
		Topic string      `msgpack:"topic"`
		Data  interface{} `msgpack:"data"`
	}
	require.NoError(t, msgpack.Unmarshal(raw, &frame))
	require.Equal(t, "diagnostics", frame.Topic)
	payload, err := json.Marshal(frame.Data)
	require.NoError(t, err)
	return payload
}
