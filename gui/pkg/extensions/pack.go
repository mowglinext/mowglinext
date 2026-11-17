package extensions

import (
	"context"
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
)

// Pack resolves a local Compose definition to a closed, pinned bundle without
// starting services. Configuration may contain secrets: never print the result
// as diagnostics. The caller chooses a private destination for the bundle.
func Pack(ctx context.Context, runner CommandRunner, manifest []byte, composePath, envPath string, binding Binding) (Bundle, error) {
	composePath, err := filepath.Abs(composePath)
	if err != nil {
		return Bundle{}, err
	}
	if err = ValidateBinding(binding); err != nil {
		return Bundle{}, err
	}
	if err = noSymlink(composePath); err != nil {
		return Bundle{}, err
	}
	args := []string{"compose", "--project-name", binding.Project, "--project-directory", filepath.Dir(composePath), "-f", composePath}
	if envPath != "" {
		envPath, err = filepath.Abs(envPath)
		if err != nil {
			return Bundle{}, err
		}
		args = append(args, "--env-file", envPath)
	}
	args = append(args, "config", "--format", "json", "--resolve-image-digests")
	raw, err := runner.Run(ctx, "docker", args...)
	if err != nil {
		return Bundle{}, errors.New("could not resolve the local Compose definition")
	}
	raw, err = NormalizeCompose(raw)
	if err != nil {
		return Bundle{}, err
	}
	// Wrapping the raw manifest lets the normal strict parser reject unknown,
	// duplicate, case-aliased and trailing fields before using any of its values.
	wrapped := append([]byte(`{"manifest":`), manifest...)
	wrapped = append(wrapped, []byte(`,"compose":`)...)
	wrapped = append(wrapped, raw...)
	wrapped = append(wrapped, '}')
	bundle, err := ParseBundle(wrapped)
	if err != nil {
		return Bundle{}, err
	}
	if err = ValidateCompose(bundle, binding); err != nil {
		return Bundle{}, err
	}
	return bundle, nil
}

// WriteBundle creates a new private file, never overwriting a configuration or
// following a symlink. The approved bundle contains fully resolved configuration.
func WriteBundle(path string, bundle Bundle) error {
	path, err := filepath.Abs(path)
	if err != nil {
		return err
	}
	if err = noSymlink(filepath.Dir(path)); err != nil {
		return err
	}
	data, err := json.MarshalIndent(bundle, "", "  ")
	if err != nil {
		return err
	}
	f, err := os.OpenFile(path, os.O_WRONLY|os.O_CREATE|os.O_EXCL, 0600)
	if err != nil {
		return err
	}
	defer f.Close()
	if _, err = f.Write(append(data, '\n')); err != nil {
		return err
	}
	if err = f.Sync(); err != nil {
		return err
	}
	dir, err := os.Open(filepath.Dir(path))
	if err != nil {
		return err
	}
	defer dir.Close()
	return dir.Sync()
}
