package api

import (
	"log"

	"github.com/gin-contrib/cors"
	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/docs"
	"github.com/mowglinext/mowglinext/pkg/providers"
	"github.com/mowglinext/mowglinext/pkg/types"
	swaggerfiles "github.com/swaggo/files"
	ginSwagger "github.com/swaggo/gin-swagger"
)

// gin-swagger middleware
// swagger embed files

func NewAPI(dbProvider types.IDBProvider, dockerProvider types.IDockerProvider, rosProvider types.IRosProvider, firmwareProvider *providers.FirmwareProvider, irriSenseProvider *providers.IrriSenseProvider) {
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
	webDir := string(webDirectory)
	registerWebUI(r, webDir)
	apiGroup := r.Group("/api")
	ConfigRoute(apiGroup, dbProvider)
	SettingsRoutes(apiGroup, dbProvider)
	GNSSRoutes(apiGroup, dbProvider, dockerProvider)
	ContainersRoutes(apiGroup, dockerProvider)
	MowgliNextRoutes(apiGroup, rosProvider)
	SetupRoutes(apiGroup, firmwareProvider)
	SystemRoutes(apiGroup)
	DiagnosticsRoutes(apiGroup, dockerProvider, rosProvider, dbProvider)
	RosbagRoutes(apiGroup, dockerProvider)
	WeatherRoutes(apiGroup, dbProvider)
	ParamsRoutes(apiGroup, rosProvider)
	NtripRoutes(apiGroup)
	CalibrationRoutes(apiGroup, rosProvider, dbProvider)
	DriveTuningRoutes(apiGroup, dbProvider, dockerProvider)
	ScheduleRoutes(apiGroup, dbProvider)
	IrriSenseRoutes(apiGroup, irriSenseProvider)
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
