package providers

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net/http"
	"net/url"
	"strconv"
	"strings"
	"time"
)

// ErrIrriSenseUnauthorized: the service rejected the bearer token (401). The
// read-only routes only accept an API token minted under Settings → API tokens
// in IrriSense, never a login session.
var ErrIrriSenseUnauthorized = errors.New("IrriSense rejected the API token (401): mint a read-only API token in IrriSense and paste it here")

// ErrIrriSenseNotFound: the garden is not readable by this token (IrriSense
// answers 404 for both "does not exist" and "belongs to another tenant").
var ErrIrriSenseNotFound = errors.New("IrriSense garden not found (404): check the garden id, or that the token belongs to the same account")

// IrriSenseRateLimitedError is a 429 with the server's Retry-After honoured.
type IrriSenseRateLimitedError struct {
	RetryAfter time.Duration
}

func (e *IrriSenseRateLimitedError) Error() string {
	return fmt.Sprintf("IrriSense rate-limited the request (429), retry after %s", e.RetryAfter)
}

const (
	irriSenseHTTPTimeout      = 10 * time.Second
	irriSenseMaxBodyBytes     = 4 << 20
	irriSenseDefaultRateRetry = 60 * time.Second
)

// irriSenseClient is the thin HTTP layer over the read-only contract
// (irrisense-cloud backend/app/api/routes/readonly.py). Error strings never
// contain the token.
type irriSenseClient struct {
	http *http.Client
}

func newIrriSenseClient(httpClient *http.Client) *irriSenseClient {
	if httpClient == nil {
		httpClient = &http.Client{Timeout: irriSenseHTTPTimeout}
	}
	return &irriSenseClient{http: httpClient}
}

func (c *irriSenseClient) fetchGardens(ctx context.Context, baseURL, token string) ([]IrriSenseGarden, error) {
	var gardens []IrriSenseGarden
	if err := c.get(ctx, irriSenseURL(baseURL, "/api/ha/gardens"), token, &gardens); err != nil {
		return nil, err
	}
	return gardens, nil
}

func (c *irriSenseClient) fetchGarden(ctx context.Context, baseURL, token, gardenID string) (IrriSenseGarden, error) {
	var garden IrriSenseGarden
	path := "/api/ha/gardens/" + url.PathEscape(strings.TrimSpace(gardenID))
	if err := c.get(ctx, irriSenseURL(baseURL, path), token, &garden); err != nil {
		return IrriSenseGarden{}, err
	}
	return garden, nil
}

func irriSenseURL(baseURL, path string) string {
	return strings.TrimRight(strings.TrimSpace(baseURL), "/") + path
}

func (c *irriSenseClient) get(ctx context.Context, endpoint, token string, out any) error {
	req, err := http.NewRequestWithContext(ctx, http.MethodGet, endpoint, nil)
	if err != nil {
		return fmt.Errorf("build IrriSense request: %w", err)
	}
	req.Header.Set("Authorization", "Bearer "+token)
	req.Header.Set("Accept", "application/json")
	req.Header.Set("User-Agent", "mowglinext-gui/irrisense")

	resp, err := c.http.Do(req)
	if err != nil {
		return fmt.Errorf("IrriSense unreachable: %w", err)
	}
	defer resp.Body.Close()

	switch {
	case resp.StatusCode == http.StatusOK:
		if err := json.NewDecoder(io.LimitReader(resp.Body, irriSenseMaxBodyBytes)).Decode(out); err != nil {
			return fmt.Errorf("decode IrriSense response: %w", err)
		}
		return nil
	case resp.StatusCode == http.StatusUnauthorized:
		return ErrIrriSenseUnauthorized
	case resp.StatusCode == http.StatusNotFound:
		return ErrIrriSenseNotFound
	case resp.StatusCode == http.StatusTooManyRequests:
		return &IrriSenseRateLimitedError{RetryAfter: parseRetryAfter(resp.Header.Get("Retry-After"))}
	default:
		return fmt.Errorf("IrriSense answered HTTP %d", resp.StatusCode)
	}
}

// parseRetryAfter reads the delay-seconds form of Retry-After (what
// irrisense-cloud's ratelimit.enforce sends); anything else falls back to a
// conservative minute.
func parseRetryAfter(header string) time.Duration {
	seconds, err := strconv.Atoi(strings.TrimSpace(header))
	if err != nil || seconds <= 0 {
		return irriSenseDefaultRateRetry
	}
	return time.Duration(seconds) * time.Second
}
