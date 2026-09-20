package api

import (
	"context"
	"encoding/json"
	"os"
	"testing"
	"time"

	"github.com/mowglinext/mowglinext/pkg/foxglove"
)

// Explicitly opt-in and read-only: no firmware, motion or update operations.
// Use against a stationary mower and retain its image/receiver baseline with
// the result. This proves the live snapshot wire contract, not position quality.
func TestReceiverSnapshotLive(t *testing.T) {
	url := os.Getenv("MOWGLI_UPDATER_LIVE_FOXGLOVE")
	if url == "" {
		t.Skip("set MOWGLI_UPDATER_LIVE_FOXGLOVE for a read-only receiver check")
	}
	ctx, cancel := context.WithTimeout(context.Background(), 15*time.Second)
	defer cancel()
	client := foxglove.NewClient(url)
	if err := client.Connect(ctx); err != nil {
		t.Fatal(err)
	}
	defer client.Close()
	var progress receiverProgress
	for ctx.Err() == nil {
		probeCtx, stop := context.WithTimeout(ctx, 2*time.Second)
		data, err := client.CallService(probeCtx, "/universal_gnss_receiver/get_snapshot", struct{}{}, "universal_gnss_msgs/srv/GetReceiverSnapshot")
		stop()
		if err == nil {
			var snapshot receiverSnapshot
			if err := json.Unmarshal(data, &snapshot); err != nil {
				t.Fatal(err)
			}
			if ok, reason := progress.observe(snapshot, time.Now()); ok {
				t.Logf("Live receiver identity and progressing observation counter verified (%d)", progress.count)
				return
			} else {
				t.Log(reason)
			}
		} else {
			t.Log(err)
		}
		time.Sleep(500 * time.Millisecond)
	}
	t.Fatal("Live receiver did not supply healthy, advancing observations")
}
