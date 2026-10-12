// MowgliNext GUI API
//
// @title MowgliNext GUI API
// @version 1.0
// @description API for the MowgliNext autonomous robot mower GUI
// @host localhost:4200
// @BasePath /api
package main

import (
	"log"

	"github.com/joho/godotenv"
	"github.com/mowglinext/mowglinext/pkg/api"
	"github.com/mowglinext/mowglinext/pkg/providers"
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
	irriSenseProvider := providers.NewIrriSenseProvider(dbProvider)
	remoteAccessProvider := providers.NewRemoteAccessProvider(dbProvider, dockerProvider)
	providers.NewSchedulerProvider(rosProvider, dbProvider, irriSenseProvider)
	// Mirrors the schedules onto the same external broker mqtt_bridge_node uses
	// (docs/MQTT_CONTROL.md), gated by the same mqtt_enabled switch; inert when
	// MQTT is off. Deliberately independent of the "system.mqtt.enabled"/
	// NewMqttProvider above -- that is the GUI's own embedded broker for an
	// unrelated purpose, not the robot's documented external MQTT contract.
	api.NewScheduleMqttBridge(dbProvider)
	notificationProvider := providers.NewNotificationProvider(dbProvider)
	if ros, ok := rosProvider.(*providers.RosProvider); ok {
		ros.AttachNotifier(notificationProvider)
	} else {
		log.Printf("notifications: ROS provider %T cannot feed status events", rosProvider)
	}
	fleetProvider := providers.NewFleetProvider(dbProvider, rosProvider)
	fleetCoordinator := providers.NewFleetCoordinator(dbProvider, rosProvider, fleetProvider)
	api.NewAPI(dbProvider, dockerProvider, rosProvider, firmwareProvider, irriSenseProvider, remoteAccessProvider, notificationProvider, fleetProvider, fleetCoordinator)
}
