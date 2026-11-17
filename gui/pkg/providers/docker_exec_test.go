package providers

import (
	"context"
	"fmt"
	"io"
	"net/http"
	"net/http/httptest"
	"strings"
	"sync/atomic"
	"testing"
	"time"

	"github.com/docker/docker/client"
	"github.com/docker/docker/pkg/stdcopy"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// Exercise the real Docker client, including its hijacked stdout stream. Exec
// attach already starts the process; a second start is rejected by Docker.
func TestContainerExecStartsOnceAndHonorsStreamDeadline(t *testing.T) {
	for _, hang := range []bool{false, true} {
		t.Run(fmt.Sprint("hang=", hang), func(t *testing.T) {
			var starts atomic.Int32
			streamClosed := make(chan struct{})
			srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				switch {
				case strings.HasSuffix(r.URL.Path, "/containers/sidecar/exec"):
					_, _ = io.WriteString(w, `{"Id":"probe"}`)
				case strings.HasSuffix(r.URL.Path, "/exec/probe/start"):
					if starts.Add(1) > 1 {
						http.Error(w, "exec already running", http.StatusConflict)
						return
					}
					conn, rw, err := w.(http.Hijacker).Hijack()
					if err != nil {
						t.Error(err)
						return
					}
					defer conn.Close()
					defer close(streamClosed)
					_, _ = rw.WriteString("HTTP/1.1 101 UPGRADED\r\nContent-Type: application/vnd.docker.raw-stream\r\nConnection: Upgrade\r\nUpgrade: tcp\r\n\r\n")
					_ = rw.Flush()
					if hang {
						_, _ = io.Copy(io.Discard, conn)
						return
					}
					_, _ = io.WriteString(stdcopy.NewStdWriter(conn, stdcopy.Stdout), "ready")
				case strings.HasSuffix(r.URL.Path, "/exec/probe/json"):
					_, _ = io.WriteString(w, `{"ID":"probe","Running":false,"ExitCode":0}`)
				default:
					http.Error(w, r.URL.Path, http.StatusNotFound)
				}
			}))
			defer srv.Close()
			cli, err := client.NewClientWithOpts(client.WithHost("tcp://"+strings.TrimPrefix(srv.URL, "http://")), client.WithVersion("1.41"))
			require.NoError(t, err)
			defer cli.Close()
			p := &DockerProvider{client: cli}
			ctx, cancel := context.WithTimeout(context.Background(), time.Second)
			defer cancel()
			result, err := p.ContainerExec(ctx, "sidecar", types.ContainerExecSpec{Cmd: []string{"tailscale", "status", "--json"}})
			if hang {
				require.ErrorIs(t, err, context.DeadlineExceeded)
			} else {
				require.NoError(t, err)
				assert.Equal(t, "ready", result.Stdout)
			}
			assert.Equal(t, int32(1), starts.Load())
			select {
			case <-streamClosed:
			case <-time.After(time.Second):
				t.Fatal("Docker exec stream leaked after cancellation")
			}
		})
	}
}
