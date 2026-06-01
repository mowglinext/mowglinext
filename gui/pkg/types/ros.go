package types

import "context"

// IRosProvider is the abstraction layer for all ROS2 communication.
// The implementation uses a foxglove WebSocket client connecting to
// foxglove_bridge; callers should never depend on goroslib or any ROS1
// transport.
type IRosProvider interface {
	// CallService calls a ROS2 service via foxglove_bridge.
	// service is the full service name (e.g. "/behavior_tree_node/high_level_control").
	// req is marshalled as the JSON args payload.
	// res, if non-nil, receives the unmarshalled service response values.
	CallService(ctx context.Context, service string, req any, res any, serviceType ...string) error

	// Subscribe registers cb to receive JSON-encoded messages on a logical
	// topic key (e.g. "status", "gps", "pose"). The cb is invoked from a
	// dedicated goroutine for each (topic, id) pair, so individual slow
	// subscribers do not block others. If a message was already received for
	// this topic, cb is called immediately with the last cached message.
	// Multiple callers may register distinct ids for the same logical key.
	// intervalMs throttles delivery to cb: messages arriving faster are
	// coalesced and the latest is delivered at most once per interval (0 =
	// unthrottled). The throttle wait happens on the subscriber's own
	// goroutine, never blocking the publisher.
	Subscribe(topic string, id string, intervalMs int, cb func(msg []byte)) error

	// UnSubscribe removes the subscriber identified by (topic, id) and stops
	// its goroutine. It is a no-op when the subscriber does not exist.
	UnSubscribe(topic string, id string)

	// Publish sends msg to the named ROS2 topic via foxglove_bridge.
	// msgType is the ROS2 message type string (e.g. "geometry_msgs/msg/Twist").
	Publish(topic string, msgType string, msg interface{}) error
}
