package api

import (
	"github.com/cedbossneo/mowglinext/docs"
	"github.com/cedbossneo/mowglinext/pkg/providers"
	"github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/gin-contrib/cors"
	"github.com/gin-contrib/static"
	"github.com/gin-gonic/gin"
	swaggerfiles "github.com/swaggo/files"
	ginSwagger "github.com/swaggo/gin-swagger"
	"log"
)

// gin-swagger middleware
// swagger embed files

func NewAPI(dbProvider types.IDBProvider, dockerProvider types.IDockerProvider, rosProvider types.IRosProvider, firmwareProvider *providers.FirmwareProvider) {
	httpAddr, err := dbProvider.Get("system.api.addr")
	if err != nil {
		log.Fatal(err)
	}

	gin.SetMode(gin.ReleaseMode)
	docs.SwaggerInfo.BasePath = "/api"
	r := gin.Default()
	config := cors.DefaultConfig()
	config.AllowAllOrigins = true
	config.AllowWebSockets = true
	r.Use(cors.New(config))
	webDirectory, err := dbProvider.Get("system.api.webDirectory")
	if err != nil {
		log.Fatal(err)
	}
	r.Use(static.Serve("/", static.LocalFile(string(webDirectory), false)))
	apiGroup := r.Group("/api")
	ConfigRoute(apiGroup, dbProvider)
	SettingsRoutes(apiGroup, dbProvider)
	ContainersRoutes(apiGroup, dockerProvider)
	MowgliNextRoutes(apiGroup, rosProvider)
	SetupRoutes(apiGroup, firmwareProvider)
	SystemRoutes(apiGroup)
	DiagnosticsRoutes(apiGroup, dockerProvider, rosProvider, dbProvider)
	CalibrationRoutes(apiGroup, rosProvider)
	ScheduleRoutes(apiGroup, dbProvider)
	ImportRoutes(apiGroup, rosProvider, dbProvider)
	tileServer, err := dbProvider.Get("system.map.enabled")
	if err != nil {
		log.Fatal(err)
	}
	if string(tileServer) == "true" {
		TilesProxy(r, dbProvider)
	}
	r.GET("/swagger/*any", ginSwagger.WrapHandler(swaggerfiles.Handler))
	r.Run(string(httpAddr))
}
