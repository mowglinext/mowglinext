package providers

import (
	"context"
	"encoding/json"
	"log"
	"os"
	"os/signal"
	"reflect"
	"syscall"
	"time"

	"github.com/brutella/hap/accessory"
	"github.com/cedbossneo/mowglinext/pkg/msgs/mowgli"
	types2 "github.com/cedbossneo/mowglinext/pkg/types"
	mqtt "github.com/mochi-mqtt/server/v2"
	"github.com/mochi-mqtt/server/v2/hooks/auth"
	"github.com/mochi-mqtt/server/v2/listeners"
	"github.com/mochi-mqtt/server/v2/packets"
	"github.com/sirupsen/logrus"
	"golang.org/x/xerrors"
)

type MqttProvider struct {
	rosProvider types2.IRosProvider
	mower       *accessory.Switch
	server      *mqtt.Server
	dbProvider  *DBProvider
	prefix      string
}

func NewMqttProvider(rosProvider types2.IRosProvider, dbProvider *DBProvider) *MqttProvider {
	h := &MqttProvider{}
	h.rosProvider = rosProvider
	h.dbProvider = dbProvider
	h.Init()
	return h
}

func (hc *MqttProvider) Init() {
	hc.prefix = "/gui"
	dbPrefix, err := hc.dbProvider.Get("system.mqtt.prefix")
	if err == nil {
		hc.prefix = string(dbPrefix)
	} else {
		logrus.Error(xerrors.Errorf("Failed to get system.mqtt.prefix: %w", err))
	}
	hc.launchServer()
	hc.subscribeToRos()
	hc.subscribeToMqtt()
}

func (hc *MqttProvider) launchServer() {
	// Create signals channel to run server until interrupted
	sigs := make(chan os.Signal, 1)
	done := make(chan bool, 1)
	signal.Notify(sigs, syscall.SIGINT, syscall.SIGTERM)
	go func() {
		<-sigs
		done <- true
	}()

	// Create the new MQTT Server.
	hc.server = mqtt.New(&mqtt.Options{
		InlineClient: true,
	})

	// Allow all connections.
	_ = hc.server.AddHook(new(auth.AllowHook), nil)

	// Create a TCP listener on a standard port.
	port, err := hc.dbProvider.Get("system.mqtt.host")
	if err != nil {
		log.Fatal(err)
	}
	tcp := listeners.NewTCP("t1", string(port), nil)
	err = hc.server.AddListener(tcp)
	if err != nil {
		log.Fatal(err)
	}

	go func() {
		err := hc.server.Serve()
		if err != nil {
			log.Fatal(err)
		}
	}()
}

func (hc *MqttProvider) subscribeToRos() {
	hc.subscribeToRosTopic("highLevelStatus", "mqtt-mower-logic")
	hc.subscribeToRosTopic("status", "mqtt-mower-status")
	hc.subscribeToRosTopic("pose", "mqtt-pose")
	hc.subscribeToRosTopic("gps", "mqtt-gps")
	hc.subscribeToRosTopic("imu", "mqtt-imu")
	hc.subscribeToRosTopic("ticks", "mqtt-ticks")
	hc.subscribeToRosTopic("wheelOdom", "mqtt-wheel-odom")
	hc.subscribeToRosTopic("map", "mqtt-map")
	hc.subscribeToRosTopic("path", "mqtt-path")
	hc.subscribeToRosTopic("plan", "mqtt-plan")
	hc.subscribeToRosTopic("mowingPath", "mqtt-mowing-path")
}

func (hc *MqttProvider) subscribeToRosTopic(topic string, id string) {
	err := hc.rosProvider.Subscribe(topic, id, 0, func(msg []byte) {
		time.Sleep(500 * time.Millisecond)
		err := hc.server.Publish(hc.prefix+"/"+topic, msg, true, 0)
		if err != nil {
			logrus.Error(xerrors.Errorf("Failed to publish to %s: %w", topic, err))
		}
	})
	if err != nil {
		logrus.Error(xerrors.Errorf("Failed to subscribe to %s: %w", topic, err))
	}
}

func (hc *MqttProvider) subscribeToMqtt() {
	subscribeToMqttCall(hc.server, hc.rosProvider, hc.prefix, "/behavior_tree_node/high_level_control", &mowgli.HighLevelControlReq{}, &mowgli.HighLevelControlRes{})
	subscribeToMqttCall(hc.server, hc.rosProvider, hc.prefix, "/hardware_bridge/emergency_stop", &mowgli.EmergencyStopReq{}, &mowgli.EmergencyStopRes{})
	subscribeToMqttCall(hc.server, hc.rosProvider, hc.prefix, "/hardware_bridge/mower_control", &mowgli.MowerControlReq{}, &mowgli.MowerControlRes{})
	subscribeToMqttCall(hc.server, hc.rosProvider, hc.prefix, "/behavior_tree_node/start_in_area", &mowgli.StartInAreaReq{}, &mowgli.StartInAreaRes{})
}

// subscribeToMqttCall wires an MQTT subscription to a ROS2 service call.
// When a message arrives on prefix+"/call"+service the payload is unmarshalled
// into a new instance of REQ and forwarded to the service via foxglove_bridge.
func subscribeToMqttCall[REQ any, RES any](server *mqtt.Server, rosProvider types2.IRosProvider, prefix, service string, req REQ, res RES) {
	err := server.Subscribe(prefix+"/call"+service, 1, func(cl *mqtt.Client, sub packets.Subscription, pk packets.Packet) {
		logrus.Info("Received " + service)
		var newReq = reflect.New(reflect.TypeOf(req).Elem()).Interface()
		err := json.Unmarshal(pk.Payload, newReq)
		if err != nil {
			logrus.Error(xerrors.Errorf("Failed to unmarshal %s: %w", service, err))
			return
		}
		err = rosProvider.CallService(context.Background(), service, newReq, res)
		if err != nil {
			logrus.Error(xerrors.Errorf("Failed to call %s: %w", service, err))
		}
	})
	if err != nil {
		logrus.Error(xerrors.Errorf("Failed to subscribe to %s: %w", service, err))
	}
}
