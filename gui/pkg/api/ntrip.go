package api

import (
	"bufio"
	"encoding/base64"
	"net"
	"net/http"
	"strconv"
	"strings"
	"time"

	"github.com/gin-gonic/gin"
)

// NtripStation is one STR (stream) entry from an NTRIP caster sourcetable.
type NtripStation struct {
	Mountpoint string  `json:"mountpoint"`
	Identifier string  `json:"identifier"`
	Format     string  `json:"format"`
	NavSystem  string  `json:"nav_system"`
	Country    string  `json:"country"`
	Lat        float64 `json:"lat"`
	Lon        float64 `json:"lon"`
}

// SourcetableResponse is the response for GET /ntrip/sourcetable.
type SourcetableResponse struct {
	Stations []NtripStation `json:"stations"`
	Total    int            `json:"total"`
}

const (
	ntripDialTimeout = 10 * time.Second
	ntripMaxBytes    = 8 << 20 // 8 MiB — Centipede's table is a few hundred KB
)

// NtripRoutes registers the NTRIP sourcetable proxy. Browsers can't speak raw
// NTRIP, so the GUI asks the backend to pull a caster's sourcetable (the public
// list of mountpoints + their lat/lon) and return it as JSON for the map.
func NtripRoutes(r *gin.RouterGroup) {
	r.GET("/ntrip/sourcetable", getNtripSourcetable)
}

func getNtripSourcetable(c *gin.Context) {
	host := strings.TrimSpace(c.Query("host"))
	if host == "" {
		c.JSON(http.StatusBadRequest, ErrorResponse{Error: "host is required"})
		return
	}
	port := c.DefaultQuery("port", "2101")
	if _, err := strconv.Atoi(port); err != nil {
		c.JSON(http.StatusBadRequest, ErrorResponse{Error: "port must be numeric"})
		return
	}

	raw, err := fetchSourcetable(host, port, c.Query("user"), c.Query("pass"))
	if err != nil {
		c.JSON(http.StatusBadGateway, ErrorResponse{Error: "sourcetable fetch failed: " + err.Error()})
		return
	}
	stations := parseSourcetable(raw)
	c.JSON(http.StatusOK, SourcetableResponse{Stations: stations, Total: len(stations)})
}

// fetchSourcetable opens a TCP connection to the caster and requests its
// sourcetable (NTRIP v1 style: GET / over HTTP/1.0). Optional basic-auth is
// sent for the rare caster that gates its sourcetable.
func fetchSourcetable(host, port, user, pass string) (string, error) {
	conn, err := net.DialTimeout("tcp", net.JoinHostPort(host, port), ntripDialTimeout)
	if err != nil {
		return "", err
	}
	defer conn.Close()
	_ = conn.SetDeadline(time.Now().Add(ntripDialTimeout))

	var req strings.Builder
	req.WriteString("GET / HTTP/1.0\r\n")
	req.WriteString("Host: " + host + "\r\n")
	req.WriteString("User-Agent: NTRIP MowgliNext\r\n")
	if user != "" {
		cred := base64.StdEncoding.EncodeToString([]byte(user + ":" + pass))
		req.WriteString("Authorization: Basic " + cred + "\r\n")
	}
	req.WriteString("Accept: */*\r\n")
	req.WriteString("Connection: close\r\n\r\n")
	if _, err := conn.Write([]byte(req.String())); err != nil {
		return "", err
	}

	var sb strings.Builder
	reader := bufio.NewReader(conn)
	buf := make([]byte, 32<<10)
	for sb.Len() < ntripMaxBytes {
		n, err := reader.Read(buf)
		if n > 0 {
			sb.Write(buf[:n])
		}
		if err != nil {
			break // EOF / timeout / closed — done reading
		}
	}
	return sb.String(), nil
}

// parseSourcetable extracts STR entries from a raw NTRIP sourcetable. Pure
// function (no I/O) so it is unit tested directly.
//
// STR format: STR;mountpoint;identifier;format;format-details;carrier;
//
//	nav-system;network;country;latitude;longitude;...
func parseSourcetable(raw string) []NtripStation {
	stations := make([]NtripStation, 0, 64)
	for _, line := range strings.Split(raw, "\n") {
		line = strings.TrimRight(line, "\r")
		if !strings.HasPrefix(line, "STR;") {
			continue
		}
		f := strings.Split(line, ";")
		if len(f) < 11 {
			continue
		}
		lat, errLat := strconv.ParseFloat(strings.TrimSpace(f[9]), 64)
		lon, errLon := strconv.ParseFloat(strings.TrimSpace(f[10]), 64)
		if errLat != nil || errLon != nil {
			continue // can't place it on a map without coordinates
		}
		stations = append(stations, NtripStation{
			Mountpoint: f[1],
			Identifier: f[2],
			Format:     f[3],
			NavSystem:  f[6],
			Country:    f[8],
			Lat:        lat,
			Lon:        lon,
		})
	}
	return stations
}
