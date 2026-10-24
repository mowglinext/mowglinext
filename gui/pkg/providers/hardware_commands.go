package providers

import (
	"context"
	"fmt"

	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
)

// Hardware acknowledgements attest command acceptance, never physical state.
// Share these checks between the standard API and Fleet's local command path.
func CallEmergencyStop(ctx context.Context, ros types.IRosProvider, req *mowgli.EmergencyStopReq) error {
	var res mowgli.EmergencyStopRes
	err := ros.CallService(ctx, "/hardware_bridge/emergency_stop", req, &res, "mowgli_interfaces/srv/EmergencyStop")
	return hardwareAcceptance(err, res.Success, "emergency")
}

func CallMowerControl(ctx context.Context, ros types.IRosProvider, req *mowgli.MowerControlReq) error {
	var res mowgli.MowerControlRes
	err := ros.CallService(ctx, "/hardware_bridge/mower_control", req, &res, "mowgli_interfaces/srv/MowerControl")
	return hardwareAcceptance(err, res.Success, "mow_enabled")
}

func hardwareAcceptance(err error, accepted bool, command string) error {
	if err != nil {
		return err
	}
	if !accepted {
		return fmt.Errorf("%s rejected the command", command)
	}
	return nil
}
