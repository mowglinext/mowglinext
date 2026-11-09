package api

import (
	"encoding/json"
	"fmt"
	"io"
	"net/http"
	"sort"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
)

func ConfigRoute(r *gin.RouterGroup, db types.IDBProvider) {
	ConfigEnvRoute(r, db)
	ConfigGetKeysRoute(r, db)
	ConfigSetKeysRoute(r, db)
}

// ConfigGetKeysRoute get config from backend
//
// @Summary get config from backend
// @Description get config from backend
// @Tags config
// @Produce  json
// @Param settings body map[string]string true "settings"
// @Success 200 {object} map[string]string
// @Failure 500 {object} ErrorResponse
// @Router /config/keys/get [post]
func ConfigGetKeysRoute(r *gin.RouterGroup, db types.IDBProvider) gin.IRoutes {
	return r.POST("/config/keys/get", func(context *gin.Context) {
		var body gin.H
		err := context.BindJSON(&body)
		if err != nil {
			context.JSON(500, ErrorResponse{
				Error: err.Error(),
			})
			return
		}
		for key, _ := range body {
			get, err := db.Get(key)
			if err != nil {
				continue
			}
			body[key] = string(get)
		}

		context.JSON(200, body)
	})
}

// ConfigSetKeysRoute set config to backend
//
// @Summary set config to backend
// @Description set config to backend
// @Tags config
// @Produce  json
// @Param settings body map[string]string true "settings"
// @Success 200 {object} map[string]string
// @Failure 500 {object} ErrorResponse
// @Router /config/keys/set [post]
func ConfigSetKeysRoute(r *gin.RouterGroup, db types.IDBProvider) gin.IRoutes {
	return r.POST("/config/keys/set", func(context *gin.Context) {
		var body gin.H
		decoder := json.NewDecoder(context.Request.Body)
		if err := decoder.Decode(&body); err != nil || body == nil {
			context.JSON(http.StatusBadRequest, ErrorResponse{Error: "expected a JSON configuration object with string values"})
			return
		}
		// Accept exactly one JSON document. Binding must not acknowledge a valid
		// prefix of an otherwise malformed request, or write before validation ends.
		var extra interface{}
		if err := decoder.Decode(&extra); err != io.EOF {
			context.JSON(http.StatusBadRequest, ErrorResponse{Error: "expected a single JSON configuration object"})
			return
		}

		keys := make([]string, 0, len(body))
		for key, value := range body {
			if _, ok := value.(string); !ok {
				context.JSON(http.StatusBadRequest, ErrorResponse{Error: fmt.Sprintf("key %q: value must be a string", key)})
				return
			}
			keys = append(keys, key)
		}

		// IDBProvider has no batch transaction. Ordering makes a partial failure
		// deterministic; it does not make these independent writes atomic.
		sort.Strings(keys)
		for _, key := range keys {
			if err := db.Set(key, []byte(body[key].(string))); err != nil {
				// Storage errors may contain secrets or values. Expose only the key
				// and the possibility of partial completion, never the raw error.
				context.JSON(http.StatusInternalServerError, ErrorResponse{Error: fmt.Sprintf("failed to store config key %q; earlier keys may already have been stored", key)})
				return
			}
		}
		context.JSON(http.StatusOK, body)
	})
}

// ConfigEnvRoute get config from backend
//
// @Summary get config env from backend
// @Description get config env from backend
// @Tags config
// @Produce  json
// @Produce  json
// @Success 200 {object} GetConfigResponse
// @Failure 500 {object} ErrorResponse
// @Router /config/envs [get]
func ConfigEnvRoute(r *gin.RouterGroup, db types.IDBProvider) gin.IRoutes {
	return r.GET("/config/envs", func(context *gin.Context) {
		// Only advertise a tileUri when the tile proxy is actually mounted
		// (gated on system.map.enabled in api.go). Otherwise the frontend
		// would fetch /tiles/... and 404, breaking the whole map view.
		// An empty tileUri is treated as "no overlay" — the React component
		// then falls back to the base Mapbox style.
		mapEnabled, _ := db.Get("system.map.enabled")
		if string(mapEnabled) != "true" {
			context.JSON(200, GetConfigResponse{TileUri: ""})
			return
		}
		tileUri, err := db.Get("system.map.tileUri")
		if err != nil {
			context.JSON(500, ErrorResponse{
				Error: err.Error(),
			})
			return
		}
		context.JSON(200, GetConfigResponse{
			TileUri: string(tileUri),
		})
	})
}
