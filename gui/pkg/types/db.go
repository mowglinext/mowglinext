package types

import "errors"

// ErrKeyNotFound identifies an absent key after any provider fallbacks.
var ErrKeyNotFound = errors.New("key not found")

// KeyNotFoundError preserves the existing key-specific message and classification.
type KeyNotFoundError struct {
	Key string
}

func (e KeyNotFoundError) Error() string { return "key " + e.Key + " not found" }
func (e KeyNotFoundError) Unwrap() error { return ErrKeyNotFound }

type IDBProvider interface {
	// Set sets the value for the given key.
	Set(key string, value []byte) error

	// Get returns the value for the given key. Missing keys match ErrKeyNotFound
	// through errors.Is; other storage failures must not match that sentinel.
	Get(key string) ([]byte, error)

	// Delete deletes the value for the given key.
	Delete(key string) error

	// KeysWithSuffix returns a list keys with the give suffix.
	KeysWithSuffix(suffix string) ([]string, error)
}
