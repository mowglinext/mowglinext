// MowgliNext GUI API
//
// @title MowgliNext GUI API
// @version 1.0
// @description API for the MowgliNext autonomous robot mower GUI
// @host localhost:4200
// @BasePath /api
package main

import (
	"github.com/mowglinext/mowglinext/pkg/api"
	"github.com/mowglinext/mowglinext/pkg/providers"
	"github.com/joho/godotenv"
)

func main() {
	_ = godotenv.Load()

	dbProvider := providers.NewDBProvider()
	dockerProvider := providers.NewDockerProvider()
	rosProvider := providers.NewRosProvider(dbProvider)
	firmwareProvider := providers.NewFirmwareProvider(dbProvider, rosProvider)
	homekitEnabled, err := dbProvider.Get("system.homekit.enabled")
	if err != nil {
		panic(err)
	}
	if string(homekitEnabled) == "true" {
		providers.NewHomeKitProvider(rosProvider, dbProvider)
	}
	mqttEnabled, err := dbProvider.Get("system.mqtt.enabled")
	if err != nil {
		panic(err)
	}
	if string(mqttEnabled) == "true" {
		providers.NewMqttProvider(rosProvider, dbProvider)
	}
	providers.NewSchedulerProvider(rosProvider, dbProvider)
	api.NewAPI(dbProvider, dockerProvider, rosProvider, firmwareProvider)
}
