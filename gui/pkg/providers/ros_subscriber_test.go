package providers

import (
	"sync"
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestRosSubscriber_PublishAndReceive(t *testing.T) {
	var received []byte
	var mu sync.Mutex
	done := make(chan struct{})

	sub := NewRosSubscriber("/test/topic", "test-id", 0, func(msg []byte) {
		mu.Lock()
		defer mu.Unlock()
		received = msg
		close(done)
	})
	defer sub.Close()

	sub.Publish([]byte(`{"test": true}`))

	select {
	case <-done:
		mu.Lock()
		assert.Equal(t, `{"test": true}`, string(received))
		mu.Unlock()
	case <-time.After(2 * time.Second):
		t.Fatal("timed out waiting for message")
	}
}

func TestRosSubscriber_LastMessageWins(t *testing.T) {
	var received []byte
	var mu sync.Mutex
	callCount := 0
	done := make(chan struct{}, 10)

	sub := NewRosSubscriber("/test/topic", "test-id", 0, func(msg []byte) {
		mu.Lock()
		defer mu.Unlock()
		received = msg
		callCount++
		done <- struct{}{}
	})
	defer sub.Close()

	// Publish multiple messages rapidly - only the last should be delivered
	sub.Publish([]byte(`"msg1"`))
	sub.Publish([]byte(`"msg2"`))
	sub.Publish([]byte(`"msg3"`))

	// Wait for at least one delivery
	select {
	case <-done:
	case <-time.After(2 * time.Second):
		t.Fatal("timed out waiting for message")
	}

	// Give time for any additional deliveries
	time.Sleep(300 * time.Millisecond)

	mu.Lock()
	// The subscriber should have received the latest message
	assert.Equal(t, `"msg3"`, string(received))
	mu.Unlock()
}

func TestRosSubscriber_Close(t *testing.T) {
	sub := NewRosSubscriber("/test/topic", "test-id", 0, func(msg []byte) {})

	// Close should not panic
	require.NotPanics(t, func() {
		sub.Close()
	})
}

func TestRosSubscriber_Fields(t *testing.T) {
	sub := NewRosSubscriber("/my/topic", "my-id", 0, func(msg []byte) {})
	defer sub.Close()

	assert.Equal(t, "/my/topic", sub.Topic)
	assert.Equal(t, "my-id", sub.Id)
}

func TestRosSubscriber_NilMessageNoCallback(t *testing.T) {
	callCount := 0
	var mu sync.Mutex

	sub := NewRosSubscriber("/test/topic", "test-id", 0, func(msg []byte) {
		mu.Lock()
		defer mu.Unlock()
		callCount++
	})
	defer sub.Close()

	// Don't publish anything, wait a bit
	time.Sleep(300 * time.Millisecond)

	mu.Lock()
	assert.Equal(t, 0, callCount, "callback should not be called when no message is published")
	mu.Unlock()
}
