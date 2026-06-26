package mowgli

import (
	"encoding/json"
	"testing"
)

// Regression guard for the GUI blade-command binding bug: the web UI sent
// PascalCase keys ({MowEnabled,MowDirection}) which never bound to the
// snake_case JSON tags, so every blade command decoded to {0,0} and manual
// mowing silently commanded the blade OFF. The fix is the snake_case keys in
// the web client; this pins the contract on the backend side so a future
// rename of either side fails loudly.
func TestMowerControlReqBindsSnakeCase(t *testing.T) {
	var req MowerControlReq
	if err := json.Unmarshal([]byte(`{"mow_enabled":1,"mow_direction":1}`), &req); err != nil {
		t.Fatalf("unmarshal: %v", err)
	}
	if req.MowEnabled != 1 || req.MowDirection != 1 {
		t.Fatalf("snake_case body did not bind: got MowEnabled=%d MowDirection=%d, want 1/1",
			req.MowEnabled, req.MowDirection)
	}
}

// The old PascalCase body must NOT bind (documents why the GUI bug was silent:
// Go's case-insensitive fallback needs equal-length names, so "MowEnabled"
// never matched "mow_enabled").
func TestMowerControlReqPascalCaseDoesNotBind(t *testing.T) {
	var req MowerControlReq
	if err := json.Unmarshal([]byte(`{"MowEnabled":1,"MowDirection":1}`), &req); err != nil {
		t.Fatalf("unmarshal: %v", err)
	}
	if req.MowEnabled != 0 || req.MowDirection != 0 {
		t.Fatalf("expected PascalCase body to decode to 0/0 (the silent-OFF bug); got %d/%d",
			req.MowEnabled, req.MowDirection)
	}
}
