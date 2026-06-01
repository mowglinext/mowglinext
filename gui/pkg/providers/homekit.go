package providers

import (
	"context"
	"encoding/json"
	"log"
	"os"
	"os/signal"
	"syscall"

	"github.com/brutella/hap"
	"github.com/brutella/hap/accessory"
	log2 "github.com/brutella/hap/log"
	"github.com/cedbossneo/mowglinext/pkg/msgs/mowgli"
	types2 "github.com/cedbossneo/mowglinext/pkg/types"
)

type HomeKitProvider struct {
	rosProvider types2.IRosProvider
	mower       *accessory.Switch
	db          types2.IDBProvider
}

func NewHomeKitProvider(rosProvider types2.IRosProvider, idbProvider types2.IDBProvider) *HomeKitProvider {
	h := &HomeKitProvider{}
	h.db = idbProvider
	h.rosProvider = rosProvider
	h.Init()
	return h
}

func (hc *HomeKitProvider) Init() {
	// Create the switch accessory.
	as := hc.registerAccessories()
	hc.subscribeToRos()
	hc.launchServer(as)
}

func (hc *HomeKitProvider) registerAccessories() *accessory.A {
	hc.mower = accessory.NewSwitch(accessory.Info{Name: "MowgliNext"})
	hc.mower.Switch.On.OnValueRemoteUpdate(func(on bool) {
		var err error
		if on {
			err = hc.rosProvider.CallService(context.Background(), "/behavior_tree_node/high_level_control", &mowgli.HighLevelControlReq{
				Command: 1,
			}, &mowgli.HighLevelControlRes{})
		} else {
			err = hc.rosProvider.CallService(context.Background(), "/behavior_tree_node/high_level_control", &mowgli.HighLevelControlReq{
				Command: 2,
			}, &mowgli.HighLevelControlRes{})
		}
		if err != nil {
			log.Println(err)
		}
	})
	return hc.mower.A
}

func (hc *HomeKitProvider) launchServer(as *accessory.A) {
	// Store the data in the "./db" directory.
	log2.Debug.Enable()
	// Create the hap server.
	server, err := hap.NewServer(hc.db, as)
	server.Addr = ":8000"
	pinCode, err := hc.db.Get("system.homekit.pincode")
	if err != nil {
		log.Panic(err)
	}
	server.Pin = string(pinCode)
	if err != nil {
		// stop if an error happens
		log.Panic(err)
	}

	// Setup a listener for interrupts and SIGTERM signals
	// to stop the server.
	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt, syscall.SIGTERM)

	ctx, cancel := context.WithCancel(context.Background())
	go func() {
		<-c
		// Stop delivering signals.
		signal.Stop(c)
		// Cancel the context to stop the server.
		cancel()
	}()

	go func() {
		// Run the server.
		server.ListenAndServe(ctx)
	}()
}

func (hc *HomeKitProvider) subscribeToRos() {
	hc.rosProvider.Subscribe("highLevelStatus", "ha-status", 0, func(msg []byte) {
		var status mowgli.HighLevelStatus
		err := json.Unmarshal(msg, &status)
		if err != nil {
			log.Println(err)
			return
		}
		if status.StateName == "MOWING" || status.StateName == "DOCKING" || status.StateName == "UNDOCKING" {
			hc.mower.Switch.On.SetValue(true)
		} else {
			hc.mower.Switch.On.SetValue(false)
		}
	})
}
