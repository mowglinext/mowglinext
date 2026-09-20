package updater

import (
	"context"
	"encoding/json"
	"strings"
	"testing"
)

// docker writes warnings to stderr on a successful run. Merged into the output
// they broke every plan: "invalid character 'i' in literal true (expecting 'r')"
// is json.Unmarshal reading `time="..." level=warning ...`.
func TestCommandReturnsStdoutOnlySoWarningsDoNotCorruptJSON(t *testing.T) {
	script := `echo 'time="2026-09-19T10:00:00Z" level=warning msg="The \"GNSS_DEVICE\" variable is not set."' >&2; echo '{"services":{}}'`
	data, err := command(context.Background(), "sh", "-c", script)
	if err != nil {
		t.Fatal(err)
	}
	var doc map[string]any
	if err = json.Unmarshal(data, &doc); err != nil {
		t.Fatalf("stdout is not clean JSON: %v (%q)", err, data)
	}
}

func TestCommandErrorCarriesStderr(t *testing.T) {
	_, err := command(context.Background(), "sh", "-c", `echo 'no such service: gps' >&2; exit 3`)
	if err == nil || !strings.Contains(err.Error(), "no such service: gps") {
		t.Fatalf("stderr missing from the error: %v", err)
	}
}
