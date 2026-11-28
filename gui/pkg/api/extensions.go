package api

import (
	"io"
	"net/http"
	"os"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/updater"
)

// ExtensionsRoutes forwards a closed administrative API to the existing host
// Unix service. Filesystem, UID and Docker/systemd ownership stay on the host.
// @Summary Review optional extensions and manage their separate lifecycle
// @Tags extensions
// @Router /system/extensions/state [get]
func ExtensionsRoutes(r *gin.RouterGroup) {
	socket := os.Getenv("MOWGLI_UPDATER_SOCKET")
	if socket == "" {
		socket = "/run/mowgli-updater/updater.sock"
	}
	registerExtensionsProxy(r, updater.Client(socket))
}
func registerExtensionsProxy(r *gin.RouterGroup, client *http.Client) {
	proxy := func(c *gin.Context) {
		op := c.Param("operation")
		allowed := c.Request.Method == http.MethodGet && op == "state"
		if c.Request.Method == http.MethodPost {
			allowed = op == "review" || op == "apply" || op == "action"
			if !updaterOriginAllowed(c.Request) {
				c.JSON(403, ErrorResponse{Error: "same-origin JSON extension request required"})
				return
			}
		}
		if !allowed || c.Request.URL.RawQuery != "" {
			c.Status(404)
			return
		}
		req, err := http.NewRequestWithContext(c.Request.Context(), c.Request.Method, "http://updater/v1/extensions/"+op, http.MaxBytesReader(c.Writer, c.Request.Body, (4<<20)+16384))
		if err != nil {
			c.JSON(400, ErrorResponse{Error: "invalid extension request"})
			return
		}
		req.Header.Set("Content-Type", "application/json")
		response, err := client.Do(req)
		if err != nil {
			c.JSON(503, ErrorResponse{Error: "Host extension service unavailable; update the host helper before using Extensions"})
			return
		}
		defer response.Body.Close()
		data, err := io.ReadAll(io.LimitReader(response.Body, (2<<20)+1))
		if err != nil || len(data) > 2<<20 {
			c.JSON(502, ErrorResponse{Error: "invalid host extension response"})
			return
		}
		// Older helpers do not have this optional route; keep unrelated features usable.
		if response.StatusCode == 404 {
			c.JSON(503, ErrorResponse{Error: "Host helper does not support Extensions yet"})
			return
		}
		c.Header("Cache-Control", "no-store")
		c.Data(response.StatusCode, "application/json", data)
	}
	// Requests enqueue bounded host jobs and return immediately. Status/review
	// have a short deadline, independent from the background lifecycle job.
	copy := *client
	copy.Timeout = 20 * time.Second
	client = &copy
	r.GET("/system/extensions/:operation", proxy)
	r.POST("/system/extensions/:operation", proxy)
}
