package api

import (
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/gin-gonic/gin"
	"log"
	"net/http"
	"net/http/httputil"
	"net/url"
)

func proxy(dbProvider types.IDBProvider) func(c *gin.Context) {
	return func(c *gin.Context) {
		tileServer, err := dbProvider.Get("system.map.tileServer")
		if err != nil {
			log.Printf("Failed to get tile server config: %v", err)
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: "tile server not configured"})
			return
		}
		remote, err := url.Parse(string(tileServer))
		if err != nil {
			log.Printf("Failed to parse tile server URL: %v", err)
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: "invalid tile server URL"})
			return
		}

		proxy := httputil.NewSingleHostReverseProxy(remote)
		proxy.Director = func(req *http.Request) {
			req.Header = c.Request.Header
			req.Host = remote.Host
			req.URL.Scheme = remote.Scheme
			req.URL.Host = remote.Host
			req.URL.Path = c.Param("proxyPath")
		}
		proxy.ErrorHandler = func(w http.ResponseWriter, r *http.Request, err error) {
			log.Printf("Tile proxy error: %v", err)
			if !c.Writer.Written() {
				c.JSON(http.StatusBadGateway, ErrorResponse{Error: "tile server unavailable"})
			}
		}

		proxy.ServeHTTP(c.Writer, c.Request)
	}
}
func TilesProxy(r *gin.Engine, dbProvider types.IDBProvider) {
	r.Any("/tiles/*proxyPath", proxy(dbProvider))
}
