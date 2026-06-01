package types

import (
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestMockDBProvider_SetGetDelete(t *testing.T) {
	db := NewMockDBProvider()

	// Set
	err := db.Set("key1", []byte("value1"))
	require.NoError(t, err)

	// Get
	val, err := db.Get("key1")
	require.NoError(t, err)
	assert.Equal(t, "value1", string(val))

	// Get nonexistent
	_, err = db.Get("nonexistent")
	assert.Error(t, err)

	// Delete
	err = db.Delete("key1")
	require.NoError(t, err)

	_, err = db.Get("key1")
	assert.Error(t, err)
}

func TestMockDBProvider_KeysWithSuffix(t *testing.T) {
	db := NewMockDBProvider()
	db.Set("system.ros.foxgloveUrl", []byte("test"))
	db.Set("system.ros.nodeHost", []byte("test"))
	db.Set("other.key", []byte("test"))

	keys, err := db.KeysWithSuffix("system")
	require.NoError(t, err)
	assert.Len(t, keys, 2)
}

func TestMockRosProvider_CallService(t *testing.T) {
	mock := NewMockRosProvider()

	err := mock.CallService(nil, "/test/service", "req", nil)
	require.NoError(t, err)

	require.Len(t, mock.ServiceCalls, 1)
	assert.Equal(t, "/test/service", mock.ServiceCalls[0].Service)
	assert.Equal(t, "req", mock.ServiceCalls[0].Req)
}

func TestMockRosProvider_CallServiceError(t *testing.T) {
	mock := NewMockRosProvider()
	mock.ServiceErr = assert.AnError

	err := mock.CallService(nil, "/test/service", nil, nil)
	assert.Error(t, err)
}

func TestMockRosProvider_SubscribeAndDispatch(t *testing.T) {
	mock := NewMockRosProvider()

	var received []byte
	err := mock.Subscribe("status", "sub1", 0, func(msg []byte) {
		received = msg
	})
	require.NoError(t, err)

	mock.Dispatch("status", []byte("hello"))
	assert.Equal(t, "hello", string(received))
}

func TestMockRosProvider_UnSubscribe(t *testing.T) {
	mock := NewMockRosProvider()

	callCount := 0
	err := mock.Subscribe("status", "sub1", 0, func(msg []byte) {
		callCount++
	})
	require.NoError(t, err)

	mock.UnSubscribe("status", "sub1")

	mock.Dispatch("status", []byte("should not arrive"))
	assert.Equal(t, 0, callCount)
}

func TestMockRosProvider_MultipleSubscribers(t *testing.T) {
	mock := NewMockRosProvider()

	var received1, received2 []byte
	mock.Subscribe("map", "sub1", 0, func(msg []byte) { received1 = msg })
	mock.Subscribe("map", "sub2", 0, func(msg []byte) { received2 = msg })

	mock.Dispatch("map", []byte("broadcast"))

	assert.Equal(t, "broadcast", string(received1))
	assert.Equal(t, "broadcast", string(received2))
}
