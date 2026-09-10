package api

import (
	"bytes"
	"encoding/binary"
	"errors"
	"io"
	"strings"
	"testing"
)

// dockerFrame builds one multiplexed docker log frame: an 8-byte header
// ([stream][0,0,0][size big-endian]) followed by the payload. This is what the
// daemon emits for every container created WITHOUT a TTY — i.e. every container
// in this repo, since no compose file sets `tty:`.
func dockerFrame(stream byte, payload string) []byte {
	header := make([]byte, 8)
	header[0] = stream
	binary.BigEndian.PutUint32(header[4:], uint32(len(payload)))
	return append(header, []byte(payload)...)
}

func collect(t *testing.T, reader io.Reader, tty bool) []string {
	t.Helper()
	var got []string
	err := StreamContainerLogLines(reader, tty, func(line []byte) error {
		got = append(got, string(line))
		return nil
	})
	if err != nil && !errors.Is(err, io.EOF) {
		t.Fatalf("unexpected stream error: %v", err)
	}
	return got
}

func TestStreamContainerLogLinesStripsDockerFraming(t *testing.T) {
	// Without demultiplexing the 8-byte header stays glued to the front of the
	// line, so the RFC3339 stamp lands at byte 8 and the ^-anchored
	// DOCKER_PREFIX_PATTERN in logTime.ts never matches.
	var raw bytes.Buffer
	raw.Write(dockerFrame(1, "2026-05-12T22:02:33.123456789Z [INFO] first\n"))
	raw.Write(dockerFrame(2, "2026-05-12T22:02:34.000000000Z [ERROR] second\n"))

	got := collect(t, &raw, false)

	want := []string{
		"2026-05-12T22:02:33.123456789Z [INFO] first",
		"2026-05-12T22:02:34.000000000Z [ERROR] second",
	}
	if len(got) != len(want) {
		t.Fatalf("got %d lines %q, want %d", len(got), got, len(want))
	}
	for i := range want {
		if got[i] != want[i] {
			t.Errorf("line %d = %q, want %q", i, got[i], want[i])
		}
	}
}

func TestStreamContainerLogLinesHandlesNewlineInsideLengthField(t *testing.T) {
	// A payload of exactly 0x0A0A = 2570 bytes puts 0x0A bytes inside the
	// frame's big-endian length field. Read raw, bufio would split the line
	// there; demultiplexed, the header never reaches the line reader.
	payload := "2026-05-12T22:02:35.000000000Z " + strings.Repeat("x", 0x0A0A-32) + "\n"
	if len(payload) != 0x0A0A {
		t.Fatalf("payload is %d bytes, want %d", len(payload), 0x0A0A)
	}

	var raw bytes.Buffer
	raw.Write(dockerFrame(1, payload))

	got := collect(t, &raw, false)

	if len(got) != 1 {
		t.Fatalf("got %d lines, want 1 (framing length field split the line)", len(got))
	}
	if got[0] != strings.TrimSuffix(payload, "\n") {
		t.Errorf("line was corrupted: got %d bytes, want %d", len(got[0]), len(payload)-1)
	}
}

func TestStreamContainerLogLinesReassemblesLongLines(t *testing.T) {
	// bufio hands back lines longer than its 4 KiB buffer in fragments. Emitted
	// as separate messages, only the first would carry a producer timestamp and
	// the rest would render with the browser clock.
	long := "2026-05-12T22:02:36.000000000Z " + strings.Repeat("y", 20000)
	var raw bytes.Buffer
	raw.Write(dockerFrame(1, long+"\n"))

	got := collect(t, &raw, false)

	if len(got) != 1 {
		t.Fatalf("got %d lines, want 1 reassembled line", len(got))
	}
	if got[0] != long {
		t.Errorf("reassembled line = %d bytes, want %d", len(got[0]), len(long))
	}
}

func TestStreamContainerLogLinesReadsTtyStreamRaw(t *testing.T) {
	// A TTY stream carries no framing; running it through stdcopy would fail.
	raw := strings.NewReader("2026-05-12T22:02:37.000000000Z tty line\nsecond\n")

	got := collect(t, raw, true)

	want := []string{"2026-05-12T22:02:37.000000000Z tty line", "second"}
	if len(got) != len(want) {
		t.Fatalf("got %d lines %q, want %d", len(got), got, len(want))
	}
	for i := range want {
		if got[i] != want[i] {
			t.Errorf("line %d = %q, want %q", i, got[i], want[i])
		}
	}
}

func TestStreamContainerLogLinesStopsOnEmitError(t *testing.T) {
	var raw bytes.Buffer
	raw.Write(dockerFrame(1, "one\n"))
	raw.Write(dockerFrame(1, "two\n"))

	sentinel := errors.New("client gone")
	emitted := 0
	err := StreamContainerLogLines(&raw, false, func([]byte) error {
		emitted++
		return sentinel
	})

	if !errors.Is(err, sentinel) {
		t.Fatalf("err = %v, want %v", err, sentinel)
	}
	if emitted != 1 {
		t.Errorf("emitted %d lines after a write failure, want 1", emitted)
	}
}
