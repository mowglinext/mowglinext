package api

import (
	"bufio"
	"context"
	"encoding/base64"
	"errors"
	"fmt"
	types2 "github.com/mowglinext/mowglinext/pkg/types"
	"github.com/docker/docker/api/types"
	"github.com/docker/docker/pkg/stdcopy"
	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
	"github.com/samber/lo"
	"io"
	"log"
	"net/http"
	"net/url"
)

func ContainersRoutes(r *gin.RouterGroup, provider types2.IDockerProvider) {
	group := r.Group("/containers")
	ContainerListRoutes(group, provider)
	ContainerLogsRoutes(group, provider)
	ContainerCommandRoutes(group, provider)
}

// ContainerListRoutes list all containers
//
// @Name list
// @Summary list all containers
// @Description list all containers
// @Tags containers
// @Produce  json
// @Success 200 {object} ContainerListResponse
// @Failure 500 {object} ErrorResponse
// @Router /containers [get]
func ContainerListRoutes(group *gin.RouterGroup, provider types2.IDockerProvider) {
	group.GET("/", func(c *gin.Context) {
		containers, err := provider.ContainerList(c.Request.Context())
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(200, ContainerListResponse{Containers: lo.Map(containers, func(container types.Container, idx int) Container {
			if container.Labels == nil {
				container.Labels = map[string]string{}
			}
			if lo.Contains(container.Names, "/mowglinext") {
				container.Labels["project"] = "mowglinext"
				container.Labels["app"] = "gui"
			}
			return Container{
				ID:     container.ID,
				Names:  container.Names,
				Labels: container.Labels,
				State:  container.State,
			}
		})})
	})
}

// ContainerCommandRoutes execute a command on a container
//
// @Summary execute a command on a container
// @Description execute a command on a container
// @Tags containers
// @Produce  json
// @Param containerId path string true "container id"
// @Param command path string true "command to execute (start/stop/restart)"
// @Success 200 {object} OkResponse
// @Failure 500 {object} ErrorResponse
// @Router /containers/{containerId}/{command} [post]
func ContainerCommandRoutes(group *gin.RouterGroup, provider types2.IDockerProvider) {
	group.POST("/:containerId/:command", func(c *gin.Context) {
		containerID := c.Param("containerId")
		command := c.Param("command")
		var err error

		switch command {
		case "restart":
			err = provider.ContainerRestart(c.Request.Context(), containerID)
		case "stop":
			err = provider.ContainerStop(c.Request.Context(), containerID)
		case "start":
			err = provider.ContainerStart(c.Request.Context(), containerID)
		default:
			c.JSON(400, ErrorResponse{Error: fmt.Sprintf("unknown command: %s", command)})
			return
		}
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(200, OkResponse{})
	})
}

// ContainerLogsRoutes stream container logs
//
// @Summary get container logs
// @Description get container logs
// @Tags containers
// @Produce text/event-stream
// @Param containerId path string true "container id"
// @Router /containers/{containerId}/logs [get]
func ContainerLogsRoutes(group *gin.RouterGroup, provider types2.IDockerProvider) {
	var upgrader = websocket.Upgrader{
		ReadBufferSize:  1024,
		WriteBufferSize: 1024,
		// Same exact-host Origin check as the main API upgrader
		// (mowglinext.go): the API has no auth layer, so accepting any
		// origin let a malicious page on the same network open this
		// stream cross-site. Empty Origin (non-browser clients) allowed.
		CheckOrigin: func(r *http.Request) bool {
			origin := r.Header.Get("Origin")
			if origin == "" {
				return true
			}
			u, err := url.Parse(origin)
			if err != nil {
				return false
			}
			return u.Host == r.Host
		},
	}

	group.GET("/:containerId/logs", func(c *gin.Context) {
		containerID := c.Param("containerId")
		conn, err := upgrader.Upgrade(c.Writer, c.Request, nil)
		if err != nil {
			return
		}
		defer func(conn *websocket.Conn) {
			err := conn.Close()
			if err != nil {
				fmt.Println("error closing websocket connection: ", err.Error())
			}
		}(conn)

		// A container created WITHOUT a TTY has its stdout/stderr multiplexed
		// by the daemon and must be demultiplexed; a TTY stream is raw. No
		// compose file in this repo sets `tty:`, so the multiplexed path is
		// the live one, but inspect rather than assume. An inspect failure is
		// not fatal — fall back to the multiplexed reading, which is what the
		// daemon produces by default.
		tty := false
		if details, inspectErr := provider.ContainerInspect(context.Background(), containerID); inspectErr == nil {
			tty = details.Tty
		}

		/*
		   read the logs from docker using docker SDK. be noticed that the Follow value must set to true.
		*/
		reader, err := provider.ContainerLogs(context.Background(), containerID)
		if err != nil {
			fmt.Println("error reader: ", err.Error())
			return
		}
		defer func(reader io.ReadCloser) {
			err := reader.Close()
			if err != nil {
				fmt.Println("error closing reader: ", err.Error())
			}
		}(reader)

		err = StreamContainerLogLines(reader, tty, func(line []byte) error {
			return conn.WriteMessage(websocket.TextMessage, []byte(base64.StdEncoding.EncodeToString(line)))
		})
		if err != nil && !errors.Is(err, io.EOF) {
			log.Println("Log stream error:", err.Error())
		}
	})
}

// MaxLogLineBytes caps how much of a single over-long log line is reassembled
// before the rest is dropped. bufio's reader hands back long lines in 4 KiB
// fragments; emitting those fragments as separate WebSocket messages splits one
// log line into several, and only the first carries the producer timestamp — so
// the continuations would render with the browser clock. Reassembling needs a
// bound, or a container dumping a binary blob on stdout grows it without limit.
const MaxLogLineBytes = 256 * 1024

// StreamContainerLogLines turns a docker log stream into whole log lines and
// hands each one to emit. It stops and returns the first error from either the
// stream or emit (io.EOF when the stream ends normally).
//
// When tty is false the stream is multiplexed: the daemon frames every chunk
// with an 8-byte header ([stream][0,0,0][size big-endian]). Reading that raw
// leaves the header glued to the front of each line, which defeats every
// ^-anchored producer pattern in gui/web/src/utils/logTime.ts (the RFC3339
// stamp added by ContainerLogsOptions.Timestamps would land at byte 8, not byte
// 0), and the big-endian size field can itself contain 0x0A and split a line
// mid-frame. stdcopy.StdCopy strips the framing; it must NOT be used on a TTY
// stream, which carries no framing at all.
func StreamContainerLogLines(reader io.Reader, tty bool, emit func(line []byte) error) error {
	src := reader
	if !tty {
		pr, pw := io.Pipe()
		defer func() { _ = pr.Close() }()
		go func() {
			_, copyErr := stdcopy.StdCopy(pw, pw, reader)
			_ = pw.CloseWithError(copyErr)
		}()
		src = pr
	}

	rd := bufio.NewReader(src)
	var pending []byte
	for {
		// ReadLine returns a slice into the reader's own buffer, valid only
		// until the next read, and sets isPrefix when the line did not fit.
		chunk, isPrefix, err := rd.ReadLine()
		if err != nil {
			return err
		}
		if isPrefix || len(pending) > 0 {
			if remaining := MaxLogLineBytes - len(pending); remaining > 0 {
				if len(chunk) > remaining {
					chunk = chunk[:remaining]
				}
				pending = append(pending, chunk...)
			}
			if isPrefix {
				continue
			}
			chunk = pending
			pending = nil
		}
		if err := emit(chunk); err != nil {
			return err
		}
	}
}
