//go:build !linux

package extensions

import (
	"errors"
	"os"
)

var errLinuxStore = errors.New("extension state storage requires Linux")

func openStoreDir(string, bool) (*os.File, error)      { return nil, errLinuxStore }
func openStoreFile(*os.File, string) (*os.File, error) { return nil, errLinuxStore }
func lockStore(*os.File) (func(), error)               { return nil, errLinuxStore }
func writeStoreState(*os.File, []byte) error           { return errLinuxStore }
