package providers

import (
	"encoding/json"
	"fmt"
	"sort"
	"strings"
	"sync"
	"testing"
	"time"

	"github.com/stretchr/testify/require"
)

// Hold the actual subscriber callback, as in the mission 02 characterization.
// Channels establish the ordering; the timeout is only a failure watchdog.
func blockedDiagnosticConsumer(t *testing.T, topic string) (*RosSubscriber, func(), <-chan string) {
	t.Helper()
	entered, release := make(chan struct{}), make(chan struct{})
	output := make(chan string, 1024)
	var once sync.Once
	unblock := func() { once.Do(func() { close(release) }) }
	sub := NewRosSubscriber(topic, t.Name(), -time.Millisecond, func(msg []byte) {
		if string(msg) == "initial" {
			close(entered)
			<-release
		}
		output <- string(msg)
	})
	t.Cleanup(func() {
		unblock()
		sub.Close()
		if sub.diagnostics != nil {
			awaitSubscription(t, sub.diagnostics.done)
		}
	})
	sub.Publish([]byte("initial"))
	awaitSubscription(t, entered)
	return sub, unblock, output
}

func nextDiagnosticDelivery(t *testing.T, output <-chan string) string {
	t.Helper()
	select {
	case msg := <-output:
		return msg
	case <-time.After(time.Second):
		t.Fatal("subscriber did not deliver the accepted diagnostic")
		return ""
	}
}

func diagnosticArray(source string, sequence, level int) string {
	return fmt.Sprintf(`{"header":{"stamp":{"sec":%d,"nanosec":0}},"status":[{"name":%q,"hardware_id":%q,"level":%d,"message":"state","values":[]}]}`, sequence, source, source, level)
}

func TestRosSubscriberDiagnosticsFIFO(t *testing.T) {
	cases := map[string][]string{
		"WARN then OK": {diagnosticArray("A", 1, 1), diagnosticArray("A", 2, 0)},
		"OK then WARN": {diagnosticArray("A", 1, 0), diagnosticArray("A", 2, 1)},
		"OK WARN OK":   {diagnosticArray("A", 1, 0), diagnosticArray("A", 2, 1), diagnosticArray("A", 3, 0)},
		"WARN OK WARN": {diagnosticArray("A", 1, 1), diagnosticArray("A", 2, 0), diagnosticArray("A", 3, 1)},
		"different sources": {diagnosticArray("A", 1, 2), diagnosticArray("B", 2, 0),
			diagnosticArray("A", 3, 1), diagnosticArray("C", 4, 2)},
		"same source": {diagnosticArray("A", 1, 0), diagnosticArray("A", 2, 1),
			diagnosticArray("A", 3, 0), diagnosticArray("A", 4, 2)},
		"identical payloads remain distinct deliveries": {diagnosticArray("A", 1, 1), diagnosticArray("A", 1, 1)},
	}
	for name, burst := range cases {
		t.Run(name, func(t *testing.T) {
			sub, release, output := blockedDiagnosticConsumer(t, "diagnostics")
			for _, msg := range burst {
				sub.Publish([]byte(msg))
			}
			// A final callback is a barrier, not a sleep to guess when draining ends.
			sub.Publish([]byte("barrier"))
			release()
			want := append([]string{"initial"}, burst...)
			want = append(want, "barrier")
			var got []string
			for {
				msg := nextDiagnosticDelivery(t, output)
				got = append(got, msg)
				if msg == "barrier" {
					break
				}
			}
			require.Equal(t, want, got, "every accepted array must be delivered once, unchanged and in order")
		})
	}
}

func TestRosSubscriberDiagnosticsAcknowledgedPublications(t *testing.T) {
	output := make(chan string, 1)
	sub := NewRosSubscriber("diagnostics", t.Name(), 0, func(msg []byte) { output <- string(msg) })
	defer func() {
		sub.Close()
		awaitSubscription(t, sub.diagnostics.done)
	}()
	for i, level := range []int{0, 1, 0, 1} {
		msg := diagnosticArray("A", i+1, level)
		sub.Publish([]byte(msg))
		require.Equal(t, msg, nextDiagnosticDelivery(t, output))
	}
}

func TestRosSubscriberDiagnosticsMessageLimit(t *testing.T) {
	sub, release, output := blockedDiagnosticConsumer(t, "diagnostics")
	var accepted []string
	for i := 0; i < diagnosticQueueMaxMessages; i++ {
		msg := diagnosticArray("A", i+1, i%3)
		accepted = append(accepted, msg)
		sub.Publish([]byte(msg))
	}
	full := sub.DiagnosticQueueStats()
	require.Equal(t, diagnosticQueueMaxMessages, full.PendingMessages)
	require.Zero(t, full.DroppedMessages)
	var rejectedBytes int
	// A full queue must not hold the caller until the consumer resumes.
	published := make(chan struct{})
	go func() {
		defer close(published)
		for i := 0; i < 3; i++ {
			msg := diagnosticArray("rejected", i+1, i)
			rejectedBytes += len(msg)
			sub.Publish([]byte(msg))
		}
	}()
	awaitSubscription(t, published)
	stats := sub.DiagnosticQueueStats()
	require.Equal(t, full.PendingMessages, stats.PendingMessages)
	require.Equal(t, full.PendingBytes, stats.PendingBytes)
	require.EqualValues(t, 3, stats.DroppedMessages)
	require.EqualValues(t, rejectedBytes, stats.DroppedBytes)
	release()
	require.Equal(t, "initial", nextDiagnosticDelivery(t, output))
	for _, msg := range accepted {
		require.Equal(t, msg, nextDiagnosticDelivery(t, output))
	}
	// Space becoming available permits new arrivals; it does not recover the gap.
	sub.Publish([]byte("after-gap"))
	require.Equal(t, "after-gap", nextDiagnosticDelivery(t, output))
	stats = sub.DiagnosticQueueStats()
	require.Zero(t, stats.PendingMessages)
	require.Zero(t, stats.PendingBytes)
	require.EqualValues(t, 3, stats.DroppedMessages, "loss knowledge must not reset after draining")
}

func sizedDiagnosticArray(size int) []byte {
	const prefix, suffix = `{"status":[],"padding":"`, `"}`
	return []byte(prefix + strings.Repeat("x", size-len(prefix)-len(suffix)) + suffix)
}

func TestRosSubscriberDiagnosticsByteLimit(t *testing.T) {
	sub, release, output := blockedDiagnosticConsumer(t, "diagnostics")
	msg := sizedDiagnosticArray(diagnosticQueueMaxBytes / 2)
	sub.Publish(msg)
	sub.Publish(msg)
	rejected := diagnosticArray("rejected", 1, 2)
	sub.Publish([]byte(rejected))
	stats := sub.DiagnosticQueueStats()
	require.Equal(t, 2, stats.PendingMessages, "the byte budget must work before the message count limit")
	require.Equal(t, diagnosticQueueMaxBytes, stats.PendingBytes)
	require.EqualValues(t, 1, stats.DroppedMessages)
	require.EqualValues(t, len(rejected), stats.DroppedBytes)
	release()
	require.Equal(t, "initial", nextDiagnosticDelivery(t, output))
	require.Equal(t, string(msg), nextDiagnosticDelivery(t, output))
	require.Equal(t, string(msg), nextDiagnosticDelivery(t, output))
}

func TestRosSubscriberDiagnosticsOversizedArray(t *testing.T) {
	sub, release, output := blockedDiagnosticConsumer(t, "diagnostics")
	sub.Publish(sizedDiagnosticArray(diagnosticQueueMaxBytes + 1))
	stats := sub.DiagnosticQueueStats()
	require.Zero(t, stats.PendingMessages)
	require.Zero(t, stats.PendingBytes)
	require.EqualValues(t, 1, stats.DroppedMessages)
	require.EqualValues(t, diagnosticQueueMaxBytes+1, stats.DroppedBytes)
	sub.Publish([]byte("accepted"))
	release()
	require.Equal(t, "initial", nextDiagnosticDelivery(t, output))
	require.Equal(t, "accepted", nextDiagnosticDelivery(t, output))
}

func TestRosSubscriberDiagnosticsOwnsAcceptedBytes(t *testing.T) {
	sub, release, output := blockedDiagnosticConsumer(t, "diagnostics")
	want := diagnosticArray("A", 1, 2)
	msg := []byte(want)
	sub.Publish(msg)
	for i := range msg {
		msg[i] = 'x'
	}
	release()
	require.Equal(t, "initial", nextDiagnosticDelivery(t, output))
	require.Equal(t, want, nextDiagnosticDelivery(t, output))
}

func TestRosSubscriberDiagnosticsCloseCancelsPending(t *testing.T) {
	sub, release, output := blockedDiagnosticConsumer(t, "diagnostics")
	for i := 0; i < 3; i++ {
		sub.Publish([]byte(diagnosticArray("A", i+1, i)))
	}
	closed := make(chan struct{})
	go func() {
		sub.Close()
		sub.Close()
		close(closed)
	}()
	awaitSubscription(t, closed) // Close must not wait for the blocked callback.
	sub.Publish([]byte("after-close"))
	stats := sub.DiagnosticQueueStats()
	require.True(t, stats.Closed)
	require.Zero(t, stats.PendingMessages)
	require.Zero(t, stats.PendingBytes)
	require.EqualValues(t, 3, stats.CanceledMessages)
	require.Zero(t, stats.DroppedMessages, "explicit cancellation is not queue saturation")
	for _, msg := range sub.diagnostics.messages {
		require.Nil(t, msg, "cancellation must release queued payload references")
	}
	release()
	awaitSubscription(t, sub.diagnostics.done)
	require.Equal(t, "initial", nextDiagnosticDelivery(t, output))
	select {
	case msg := <-output:
		t.Fatalf("canceled callback delivered %q", msg)
	default:
	}
}

func TestRosSubscriberDiagnosticsCloseIdleOrWaiting(t *testing.T) {
	for _, deliveredFirst := range []bool{false, true} {
		t.Run(fmt.Sprint(deliveredFirst), func(t *testing.T) {
			output := make(chan string, 2)
			sub := NewRosSubscriber("diagnostics", t.Name(), time.Hour, func(msg []byte) { output <- string(msg) })
			t.Cleanup(sub.Close)
			if deliveredFirst {
				sub.Publish([]byte("first"))
				require.Equal(t, "first", nextDiagnosticDelivery(t, output))
				sub.Publish([]byte("waiting-for-interval"))
			}
			sub.Close()
			awaitSubscription(t, sub.diagnostics.done)
			select {
			case msg := <-output:
				t.Fatalf("delivery after cancellation: %q", msg)
			default:
			}
		})
	}
}

func TestRosSubscriberDiagnosticsCloseAfterSaturation(t *testing.T) {
	sub, release, output := blockedDiagnosticConsumer(t, "diagnostics")
	for i := 0; i < diagnosticQueueMaxMessages+1; i++ {
		sub.Publish([]byte("pending"))
	}
	sub.Close()
	release()
	awaitSubscription(t, sub.diagnostics.done)
	stats := sub.DiagnosticQueueStats()
	require.EqualValues(t, 1, stats.DroppedMessages, "Close cannot erase evidence of prior saturation")
	require.EqualValues(t, diagnosticQueueMaxMessages, stats.CanceledMessages)
	require.Zero(t, stats.PendingBytes)
	require.Equal(t, "initial", nextDiagnosticDelivery(t, output))
	select {
	case msg := <-output:
		t.Fatalf("canceled queued delivery: %q", msg)
	default:
	}
}

func TestRosSubscriberDiagnosticsNilAndEmptyMessages(t *testing.T) {
	sub, release, output := blockedDiagnosticConsumer(t, "diagnostics")
	sub.Publish(nil)
	sub.Publish([]byte{})
	release()
	require.Equal(t, "initial", nextDiagnosticDelivery(t, output))
	require.Empty(t, nextDiagnosticDelivery(t, output), "a non-nil empty payload keeps its existing callback semantics")
	require.Zero(t, sub.DiagnosticQueueStats().DroppedMessages)
}

func TestRosSubscriberDiagnosticsConcurrentPublish(t *testing.T) {
	sub, release, output := blockedDiagnosticConsumer(t, "diagnostics")
	const producers, perProducer = 8, 64
	var wg sync.WaitGroup
	for p := 0; p < producers; p++ {
		wg.Go(func() {
			for i := 0; i < perProducer; i++ {
				sub.Publish([]byte(fmt.Sprintf("producer:%d:%d", p, i)))
			}
		})
	}
	published := make(chan struct{})
	go func() { wg.Wait(); close(published) }()
	awaitSubscription(t, published)
	stats := sub.DiagnosticQueueStats()
	require.Equal(t, diagnosticQueueMaxMessages, stats.PendingMessages)
	require.EqualValues(t, producers*perProducer-diagnosticQueueMaxMessages, stats.DroppedMessages)
	release()
	require.Equal(t, "initial", nextDiagnosticDelivery(t, output))
	seen := make(map[string]bool)
	last := make(map[int]int)
	for i := 0; i < diagnosticQueueMaxMessages; i++ {
		msg := nextDiagnosticDelivery(t, output)
		require.False(t, seen[msg], "accepted publications cannot be duplicated")
		seen[msg] = true
		var p, index int
		_, err := fmt.Sscanf(msg, "producer:%d:%d", &p, &index)
		require.NoError(t, err)
		require.GreaterOrEqual(t, p, 0)
		require.Less(t, p, producers)
		require.GreaterOrEqual(t, index, 0)
		require.Less(t, index, perProducer)
		if previous, ok := last[p]; ok {
			require.Greater(t, index, previous, "a producer's accepted order must be preserved")
		}
		last[p] = index
	}
}

func TestRosSubscriberDiagnosticsConcurrentCloseAndPublish(t *testing.T) {
	sub, release, output := blockedDiagnosticConsumer(t, "diagnostics")
	start := make(chan struct{})
	var wg sync.WaitGroup
	for i := 0; i < 8; i++ {
		wg.Go(func() {
			<-start
			for j := 0; j < 64; j++ {
				sub.Publish([]byte("pending"))
			}
		})
	}
	wg.Go(func() { <-start; sub.Close() })
	close(start)
	finished := make(chan struct{})
	go func() { wg.Wait(); close(finished) }()
	awaitSubscription(t, finished)
	release()
	awaitSubscription(t, sub.diagnostics.done)
	stats := sub.DiagnosticQueueStats()
	require.True(t, stats.Closed)
	require.Zero(t, stats.PendingMessages)
	require.Zero(t, stats.PendingBytes)
	require.Equal(t, "initial", nextDiagnosticDelivery(t, output))
	select {
	case msg := <-output:
		t.Fatalf("queued callback survived cancellation: %q", msg)
	default:
	}
}

func TestRosProviderDiagnosticsSlowSubscriberIsIsolated(t *testing.T) {
	r := newSubscriptionProvider(t, &controlledSubscriptions{})
	entered, release := make(chan struct{}), make(chan struct{})
	var once sync.Once
	unblock := func() { once.Do(func() { close(release) }) }
	t.Cleanup(unblock)
	var slowDeliveries int
	require.NoError(t, r.Subscribe("diagnostics", "slow", -1, func([]byte) {
		slowDeliveries++
		close(entered)
		<-release
	}))
	slow := r.subscribers["diagnostics"]["slow"]
	r.fanOut("diagnostics", []byte("initial"))
	awaitSubscription(t, entered)
	fast, imu, emergency := make(chan string, 16), make(chan string, 1), make(chan string, 1)
	require.NoError(t, r.Subscribe("diagnostics", "fast", -1, func(msg []byte) { fast <- string(msg) }))
	require.Equal(t, "initial", nextDiagnosticDelivery(t, fast), "cached replay is preserved")
	require.NoError(t, r.Subscribe("imu", "imu", 100, func(msg []byte) { imu <- string(msg) }))
	require.NoError(t, r.Subscribe("emergency", "emergency", -1, func(msg []byte) { emergency <- string(msg) }))
	var burst []string
	for i, level := range []int{0, 1, 0, 1, 2, 0} {
		burst = append(burst, diagnosticArray("A", i+1, level))
	}
	published := make(chan struct{})
	go func() {
		for _, msg := range burst {
			r.fanOut("diagnostics", []byte(msg))
		}
		r.fanOut("imu", []byte("imu"))
		r.fanOut("emergency", []byte("urgent"))
		close(published)
	}()
	awaitSubscription(t, published)
	for _, msg := range burst {
		require.Equal(t, msg, nextDiagnosticDelivery(t, fast))
	}
	require.Equal(t, "imu", nextDiagnosticDelivery(t, imu))
	require.Equal(t, "urgent", nextDiagnosticDelivery(t, emergency))
	removed := make(chan struct{})
	go func() { r.UnSubscribe("diagnostics", "slow"); close(removed) }()
	awaitSubscription(t, removed)
	unblock()
	awaitSubscription(t, slow.diagnostics.done)
	require.Equal(t, 1, slowDeliveries, "no queued callbacks after unsubscribe")
	r.UnSubscribe("diagnostics", "fast")
	require.NotContains(t, r.lastMessage, "diagnostics", "last listener still clears live cache")
}

func TestRosSubscriberNonDiagnosticsStillCoalesce(t *testing.T) {
	for _, topic := range []string{"imu", "gnssStatus", "emergency", "btLog", "fusionDiag", "map"} {
		t.Run(topic, func(t *testing.T) {
			sub, release, output := blockedDiagnosticConsumer(t, topic)
			sub.Publish([]byte("one"))
			sub.Publish([]byte("two"))
			sub.Publish([]byte("latest"))
			release()
			require.Equal(t, "initial", nextDiagnosticDelivery(t, output))
			require.Equal(t, "latest", nextDiagnosticDelivery(t, output))
		})
	}
}

func TestRosProviderDiagnosticsBacklogLatencyAndRecovery(t *testing.T) {
	r := newSubscriptionProvider(t, &controlledSubscriptions{})
	type delivery struct {
		message string
		at      time.Time
	}
	entered, release := make(chan struct{}), make(chan struct{})
	var once sync.Once
	unblock := func() { once.Do(func() { close(release) }) }
	t.Cleanup(unblock)
	deliveries := make(chan delivery, diagnosticQueueMaxMessages+2)
	require.NoError(t, r.Subscribe("diagnostics", "slow", -1, func(msg []byte) {
		if string(msg) == "initial" {
			close(entered)
			<-release
			return
		}
		deliveries <- delivery{message: string(msg), at: time.Now()}
	}))
	sub := r.subscribers["diagnostics"]["slow"]
	t.Cleanup(func() {
		unblock()
		sub.Close()
		awaitSubscription(t, sub.diagnostics.done)
	})
	r.fanOut("diagnostics", []byte("initial"))
	awaitSubscription(t, entered)

	// Independent producer goroutines use acknowledgments to establish a known
	// admission order. Callback progress cannot affect the published sequence.
	type publication struct {
		message string
		ack     chan time.Time
	}
	var producers [3]chan publication
	var wg sync.WaitGroup
	for i := range producers {
		producers[i] = make(chan publication)
		commands := producers[i]
		wg.Go(func() {
			for command := range commands {
				at := time.Now()
				r.fanOut("diagnostics", []byte(command.message))
				command.ack <- at
			}
		})
	}
	t.Cleanup(func() {
		unblock()
		for _, commands := range producers {
			close(commands)
		}
		done := make(chan struct{})
		go func() { wg.Wait(); close(done) }()
		awaitSubscription(t, done)
	})
	publish := func(index int, message string) time.Time {
		ack := make(chan time.Time, 1)
		select {
		case producers[index] <- publication{message: message, ack: ack}:
		case <-time.After(time.Second):
			t.Fatal("producer could not accept a publication command")
		}
		select {
		case at := <-ack:
			return at
		case <-time.After(time.Second):
			t.Fatal("diagnostics backlog blocked a publisher")
			return time.Time{}
		}
	}
	var accepted []string
	enqueued := make(map[string]time.Time)
	for i := 0; i < diagnosticQueueMaxMessages+2; i++ {
		source := i % len(producers)
		message := diagnosticArray(fmt.Sprintf("producer-%d", source), i+1, i%3)
		at := publish(source, message)
		if i < diagnosticQueueMaxMessages {
			accepted = append(accepted, message)
			enqueued[message] = at
		}
	}
	full := sub.DiagnosticQueueStats()
	require.Equal(t, diagnosticQueueMaxMessages, full.PendingMessages)
	require.EqualValues(t, 2, full.DroppedMessages)

	// These subscriptions must progress while diagnostics remain saturated.
	for _, topic := range []string{"imu", "emergency", "highLevelStatus"} {
		output := make(chan string, 1)
		require.NoError(t, r.Subscribe(topic, topic, -1, func(msg []byte) { output <- string(msg) }))
		published := make(chan struct{})
		go func() { r.fanOut(topic, []byte(topic)); close(published) }()
		awaitSubscription(t, published)
		require.Equal(t, topic, nextDiagnosticDelivery(t, output))
	}
	// This deliberate hold models a slow consumer. Correctness relies on the
	// gate and acknowledgments, not a scheduler deadline or a latency ceiling.
	const modeledHold = 30 * time.Millisecond
	holdStart := time.Now()
	timer := time.NewTimer(modeledHold)
	<-timer.C
	held := time.Since(holdStart)
	unblock()
	var latencies []time.Duration
	for _, want := range accepted {
		select {
		case got := <-deliveries:
			require.Equal(t, want, got.message, "accepted arrays must retain their exact cross-producer order")
			latencies = append(latencies, got.at.Sub(enqueued[want]))
		case <-time.After(time.Second):
			t.Fatal("consumer did not drain the accepted backlog")
		}
	}
	recovered := diagnosticArray("producer-0", 1000, 0)
	recoveryStart := publish(0, recovered)
	var recoveryLatency time.Duration
	select {
	case got := <-deliveries:
		require.Equal(t, recovered, got.message)
		recoveryLatency = got.at.Sub(recoveryStart)
	case <-time.After(time.Second):
		t.Fatal("normal delivery did not resume after the backlog drained")
	}
	stats := sub.DiagnosticQueueStats()
	require.Zero(t, stats.PendingMessages)
	require.Zero(t, stats.PendingBytes)
	require.EqualValues(t, 2, stats.DroppedMessages, "draining does not restore lost events")
	sort.Slice(latencies, func(i, j int) bool { return latencies[i] < latencies[j] })
	require.GreaterOrEqual(t, latencies[0], held, "the measured queue wait includes the explicit consumer hold")
	observation, err := json.Marshal(map[string]interface{}{
		"accepted_arrays": len(accepted), "dropped_arrays": stats.DroppedMessages,
		"peak_pending_bytes": full.PendingBytes,
		"modeled_hold_ns":    modeledHold.Nanoseconds(), "observed_hold_ns": held.Nanoseconds(),
		"latency_min_ns":      latencies[0].Nanoseconds(),
		"latency_median_ns":   latencies[len(latencies)/2].Nanoseconds(),
		"latency_max_ns":      latencies[len(latencies)-1].Nanoseconds(),
		"recovery_latency_ns": recoveryLatency.Nanoseconds(),
		"other_subscriptions_progressed_while_saturated": true,
	})
	require.NoError(t, err)
	t.Logf("latency_observation=%s", observation)
}

func TestRosSubscriberDiagnosticsConcurrentClosersDuringDelivery(t *testing.T) {
	sub, release, output := blockedDiagnosticConsumer(t, "diagnostics")
	const queuedBeforeClose = 32
	for i := 0; i < queuedBeforeClose; i++ {
		sub.Publish([]byte(diagnosticArray("A", i+1, i%3)))
	}
	start := make(chan struct{})
	var wg sync.WaitGroup
	for i := 0; i < 8; i++ {
		wg.Go(func() {
			<-start
			for j := 0; j < 3; j++ {
				sub.Close()
			}
		})
	}
	for i := 0; i < 4; i++ {
		wg.Go(func() {
			<-start
			for j := 0; j < 64; j++ {
				sub.Publish([]byte("racing-publication"))
			}
		})
	}
	close(start)
	closed := make(chan struct{})
	go func() { wg.Wait(); close(closed) }()
	awaitSubscription(t, closed)
	stats := sub.DiagnosticQueueStats()
	require.True(t, stats.Closed)
	require.Zero(t, stats.PendingMessages)
	require.Zero(t, stats.PendingBytes)
	require.GreaterOrEqual(t, stats.CanceledMessages, uint64(queuedBeforeClose))
	require.LessOrEqual(t, stats.CanceledMessages, uint64(diagnosticQueueMaxMessages), "concurrent Close cannot count cancellation twice")
	sub.Publish([]byte("after-all-closes"))
	require.Equal(t, stats, sub.DiagnosticQueueStats())
	select {
	case <-sub.diagnostics.done:
		t.Fatal("Close must not pretend that the active callback has returned")
	default:
	}
	release()
	awaitSubscription(t, sub.diagnostics.done)
	require.Equal(t, "initial", nextDiagnosticDelivery(t, output), "the delivery already started is allowed to finish")
	select {
	case msg := <-output:
		t.Fatalf("queued callback survived cancellation: %q", msg)
	default:
	}
	for _, message := range sub.diagnostics.messages {
		require.Nil(t, message, "closed worker must retain no queued payload reference")
	}
}

func TestRosSubscriberDiagnosticsCallbackCanCloseItself(t *testing.T) {
	entered, release, returned := make(chan struct{}), make(chan struct{}), make(chan struct{})
	var once sync.Once
	unblock := func() { once.Do(func() { close(release) }) }
	var sub *RosSubscriber
	sub = NewRosSubscriber("diagnostics", t.Name(), 0, func([]byte) {
		close(entered)
		<-release
		sub.Close()
		sub.Close()
		close(returned)
	})
	t.Cleanup(func() {
		unblock()
		sub.Close()
		awaitSubscription(t, sub.diagnostics.done)
	})
	sub.Publish([]byte("active"))
	awaitSubscription(t, entered)
	sub.Publish([]byte("pending"))
	unblock()
	awaitSubscription(t, returned)
	awaitSubscription(t, sub.diagnostics.done)
	stats := sub.DiagnosticQueueStats()
	require.True(t, stats.Closed)
	require.EqualValues(t, 1, stats.CanceledMessages)
	require.Zero(t, stats.PendingMessages)
	require.Zero(t, stats.PendingBytes)
}
