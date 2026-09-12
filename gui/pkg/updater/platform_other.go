//go:build !linux

package updater

import (
	"errors"
	"os"
)

func freeBytes(string) (uint64, error)   { return 0, errors.New("host operations require Linux") }
func processLock(string) (func(), error) { return nil, errors.New("host operations require Linux") }

func inheritFileOwner(*os.File, string) error { return nil }
