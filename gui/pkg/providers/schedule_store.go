package providers

import "sync"

// ScheduleWriteMu is shared by scheduler metadata and HTTP/MQTT mutations.
// Only database read/modify/write belongs inside it, never ROS or network IO.
var ScheduleWriteMu sync.Mutex
