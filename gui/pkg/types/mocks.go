package types

import (
	"context"
	"fmt"
	"sync"
)

// MockDBProvider is an in-memory mock of IDBProvider for testing.
type MockDBProvider struct {
	mu   sync.RWMutex
	data map[string][]byte
}

func NewMockDBProvider() *MockDBProvider {
	return &MockDBProvider{
		data: make(map[string][]byte),
	}
}

func (m *MockDBProvider) Set(key string, value []byte) error {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.data[key] = value
	return nil
}

func (m *MockDBProvider) Get(key string) ([]byte, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()
	v, ok := m.data[key]
	if !ok {
		return nil, fmt.Errorf("key %s not found", key)
	}
	return v, nil
}

func (m *MockDBProvider) Delete(key string) error {
	m.mu.Lock()
	defer m.mu.Unlock()
	delete(m.data, key)
	return nil
}

func (m *MockDBProvider) KeysWithSuffix(suffix string) ([]string, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()
	var keys []string
	for k := range m.data {
		if len(k) >= len(suffix) && k[:len(suffix)] == suffix {
			keys = append(keys, k)
		}
	}
	return keys, nil
}

// MockRosProvider is a mock of IRosProvider for testing API handlers.
type MockRosProvider struct {
	mu           sync.Mutex
	subscribers  map[string]map[string]func(msg []byte)
	ServiceCalls []ServiceCall
	ServiceErr   error
	PublishErr   error
	SubscribeErr error
}

type ServiceCall struct {
	Service string
	Req     any
}

func NewMockRosProvider() *MockRosProvider {
	return &MockRosProvider{
		subscribers:  make(map[string]map[string]func(msg []byte)),
		ServiceCalls: []ServiceCall{},
	}
}

func (m *MockRosProvider) CallService(_ context.Context, service string, req any, _ any, _ ...string) error {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.ServiceCalls = append(m.ServiceCalls, ServiceCall{Service: service, Req: req})
	return m.ServiceErr
}

func (m *MockRosProvider) Subscribe(topic string, id string, intervalMs int, cb func(msg []byte)) error {
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.SubscribeErr != nil {
		return m.SubscribeErr
	}
	if m.subscribers[topic] == nil {
		m.subscribers[topic] = make(map[string]func(msg []byte))
	}
	m.subscribers[topic][id] = cb
	return nil
}

func (m *MockRosProvider) UnSubscribe(topic string, id string) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.subscribers[topic] != nil {
		delete(m.subscribers[topic], id)
	}
}

func (m *MockRosProvider) Publish(_ string, _ string, _ interface{}) error {
	return m.PublishErr
}

// Dispatch simulates delivering a message to all subscribers of a logical topic key.
func (m *MockRosProvider) Dispatch(topic string, msg []byte) {
	m.mu.Lock()
	defer m.mu.Unlock()
	for _, cb := range m.subscribers[topic] {
		cb(msg)
	}
}
