package providers

import (
	"context"
	"encoding/json"
	"errors"
	"sync"
	"testing"
	"time"

	"github.com/mowglinext/mowglinext/pkg/foxglove"
	"github.com/stretchr/testify/require"
)

type controlledSubscriptions struct {
	*foxglove.Client
	mu                sync.Mutex
	active            map[string]func(json.RawMessage)
	ops               []string
	beforeSubscribe   func(string) error
	beforeUnsubscribe func(string)
	callService       func(context.Context, string, interface{}, ...string) (json.RawMessage, error)
}

func (c *controlledSubscriptions) CallService(ctx context.Context, service string, request interface{}, serviceType ...string) (json.RawMessage, error) {
	if c.callService == nil {
		return nil, errors.New("unexpected service call")
	}
	return c.callService(ctx, service, request, serviceType...)
}

func (c *controlledSubscriptions) Subscribe(topic, msgType, id string, cb func(json.RawMessage), opts ...int) error {
	if c.beforeSubscribe != nil {
		if err := c.beforeSubscribe(topic); err != nil {
			return err
		}
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	c.active[topic] = cb
	c.ops = append(c.ops, "subscribe "+topic)
	return nil
}

func (c *controlledSubscriptions) Unsubscribe(topic, id string) {
	if c.beforeUnsubscribe != nil {
		c.beforeUnsubscribe(topic)
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	delete(c.active, topic)
	c.ops = append(c.ops, "unsubscribe "+topic)
}

func newSubscriptionProvider(t *testing.T, client *controlledSubscriptions) *RosProvider {
	t.Helper()
	client.active = make(map[string]func(json.RawMessage))
	r := &RosProvider{client: client, subscribers: make(map[string]map[string]*RosSubscriber), lastMessage: make(map[string][]byte)}
	t.Cleanup(func() {
		r.mtx.Lock()
		defer r.mtx.Unlock()
		for _, subs := range r.subscribers {
			for _, sub := range subs {
				sub.Close()
			}
		}
	})
	return r
}

func awaitSubscription(t *testing.T, done <-chan struct{}) {
	t.Helper()
	select {
	case <-done:
	case <-time.After(2 * time.Second):
		t.Fatal("subscription operation did not complete")
	}
}

func TestRosProviderBlockedBridgeDoesNotBlockFanOut(t *testing.T) {
	for _, operation := range []string{"subscribe", "unsubscribe"} {
		t.Run(operation, func(t *testing.T) {
			entered, release := make(chan struct{}), make(chan struct{})
			var once sync.Once
			defer once.Do(func() { close(release) })
			client := &controlledSubscriptions{}
			r := newSubscriptionProvider(t, client)
			received := make(chan []byte, 1)
			require.NoError(t, r.Subscribe("map", "map", 0, func(msg []byte) { received <- msg }))
			if operation == "unsubscribe" {
				require.NoError(t, r.Subscribe("status", "status", 0, func([]byte) {}))
				client.beforeUnsubscribe = func(string) { close(entered); <-release }
			} else {
				client.beforeSubscribe = func(string) error { close(entered); <-release; return nil }
			}
			done := make(chan struct{})
			go func() {
				defer close(done)
				if operation == "subscribe" {
					_ = r.Subscribe("status", "status", 0, func([]byte) {})
				} else {
					r.UnSubscribe("status", "status")
				}
			}()
			awaitSubscription(t, entered)
			// The virtual map stream uses the same state lock as all telemetry.
			go r.fanOut("map", []byte(`{"areas":[]}`))
			select {
			case msg := <-received:
				require.JSONEq(t, `{"areas":[]}`, string(msg))
			case <-time.After(time.Second):
				t.Fatal("bridge I/O blocked unrelated message delivery")
			}
			once.Do(func() { close(release) })
			awaitSubscription(t, done)
		})
	}
}

func TestRosProviderSubscriptionRacesConvergeToCurrentListeners(t *testing.T) {
	for _, initial := range []bool{false, true} {
		name := "remove-during-subscribe"
		if initial {
			name = "add-during-unsubscribe"
		}
		t.Run(name, func(t *testing.T) {
			client := &controlledSubscriptions{}
			r := newSubscriptionProvider(t, client)
			if initial {
				require.NoError(t, r.Subscribe("status", "old", 0, func([]byte) {}))
			}
			entered, release := make(chan struct{}), make(chan struct{})
			var once sync.Once
			t.Cleanup(func() { once.Do(func() { close(release) }) })
			if initial {
				client.beforeUnsubscribe = func(string) { close(entered); <-release }
			} else {
				client.beforeSubscribe = func(string) error { close(entered); <-release; return nil }
			}
			firstDone, secondDone := make(chan struct{}), make(chan struct{})
			go func() {
				defer close(firstDone)
				if initial {
					r.UnSubscribe("status", "old")
				} else {
					_ = r.Subscribe("status", "old", 0, func([]byte) {})
				}
			}()
			awaitSubscription(t, entered)
			go func() {
				defer close(secondDone)
				if initial {
					_ = r.Subscribe("status", "new", 0, func([]byte) {})
				} else {
					r.UnSubscribe("status", "old")
				}
			}()
			require.Eventually(t, func() bool {
				r.mtx.Lock()
				defer r.mtx.Unlock()
				return (len(r.subscribers["status"]) > 0) == initial
			}, time.Second, time.Millisecond)
			once.Do(func() { close(release) })
			awaitSubscription(t, firstDone)
			awaitSubscription(t, secondDone)
			client.mu.Lock()
			defer client.mu.Unlock()
			_, active := client.active[topicMap["status"].ROS2Topic]
			require.Equal(t, initial, active)
			require.Len(t, client.ops, map[bool]int{false: 2, true: 3}[initial])
		})
	}
}

func TestRosProviderSubscriptionRefcountsAndCache(t *testing.T) {
	client := &controlledSubscriptions{}
	r := newSubscriptionProvider(t, client)
	require.NoError(t, r.Subscribe("status", "one", 0, func([]byte) {}))
	r.fanOut("status", []byte(`{"battery":24}`))
	received := make(chan []byte, 1)
	require.NoError(t, r.Subscribe("status", "two", 0, func(msg []byte) { received <- msg }))
	select {
	case msg := <-received:
		require.JSONEq(t, `{"battery":24}`, string(msg))
	case <-time.After(time.Second):
		t.Fatal("active-topic cached message was not replayed")
	}
	r.UnSubscribe("status", "one")
	require.Len(t, client.ops, 1)
	r.UnSubscribe("status", "two")
	require.Len(t, client.ops, 2)
	require.NotContains(t, r.lastMessage, "status")
	r.fanOut("map", []byte(`{"areas":[]}`))
	require.NoError(t, r.Subscribe("map", "one", 0, func([]byte) {}))
	r.UnSubscribe("map", "one")
	require.Contains(t, r.lastMessage, "map")
	require.Len(t, client.ops, 2)
}

func TestRosProviderFailedRegistrationCanBeRetried(t *testing.T) {
	client := &controlledSubscriptions{}
	r := newSubscriptionProvider(t, client)
	client.beforeSubscribe = func(string) error { return errors.New("registration failed") }
	require.NoError(t, r.Subscribe("status", "one", 0, func([]byte) {}))
	require.Empty(t, client.active)
	client.beforeSubscribe = nil
	require.NoError(t, r.Subscribe("status", "two", 0, func([]byte) {}))
	require.Len(t, client.active, 1)
	require.Len(t, client.ops, 1)
}

func TestRosProviderMavrosVehicleInfoUsesNativeServiceLazily(t *testing.T) {
	client := &controlledSubscriptions{}
	r := newSubscriptionProvider(t, client)
	called := make(chan struct{}, 1)
	client.callService = func(_ context.Context, service string, request interface{}, serviceType ...string) (json.RawMessage, error) {
		require.Equal(t, "/mavros/vehicle_info_get", service)
		require.Equal(t, []string{"mavros_msgs/srv/VehicleInfoGet"}, serviceType)
		require.Equal(t, map[string]interface{}{"sysid": 0, "compid": 0, "get_all": false}, request)
		called <- struct{}{}
		return json.RawMessage(`{"success":true,"vehicles":[{"autopilot":3,"flight_sw_version":67438079}]}`), nil
	}

	// An ordinary Mowgli subscriber must not touch a MAVROS-only service.
	require.NoError(t, r.Subscribe("status", "native", 0, func([]byte) {}))
	select {
	case <-called:
		t.Fatal("MAVROS service called without a MAVROS inventory listener")
	case <-time.After(20 * time.Millisecond):
	}

	received := make(chan []byte, 1)
	require.NoError(t, r.Subscribe("mavrosVehicleInfo", "browser", 0, func(msg []byte) { received <- msg }))
	select {
	case msg := <-received:
		require.JSONEq(t, `{"success":true,"vehicles":[{"autopilot":3,"flight_sw_version":67438079}]}`, string(msg))
	case <-time.After(time.Second):
		t.Fatal("native MAVROS vehicle info response was not delivered")
	}
	r.UnSubscribe("mavrosVehicleInfo", "browser")
}
