package providers

import (
	"time"
)

// Notification event kinds: the operator-facing toggles. Each event the
// detector emits belongs to exactly one kind; the Message id below it picks
// the wording (notify_messages.go).
const (
	NotifyEventMowStarted    = "mowStarted"
	NotifyEventZoneStarted   = "zoneStarted"
	NotifyEventZoneFinished  = "zoneFinished"
	NotifyEventMowComplete   = "mowComplete"
	NotifyEventMowStopped    = "mowStopped"
	NotifyEventBlocked       = "blocked"
	NotifyEventEmergency     = "emergency"
	NotifyEventBattery       = "battery"
	NotifyEventRain          = "rain"
	NotifyEventWaitingForRtk = "waitingForRtk"
)

// NotifyEventKinds lists every kind in display order.
var NotifyEventKinds = []string{
	NotifyEventMowStarted,
	NotifyEventZoneStarted,
	NotifyEventZoneFinished,
	NotifyEventMowComplete,
	NotifyEventMowStopped,
	NotifyEventBlocked,
	NotifyEventEmergency,
	NotifyEventBattery,
	NotifyEventRain,
	NotifyEventWaitingForRtk,
}

// DefaultNotifyEvents is a fresh install: everything on except the
// "mowing stopped" catch-all, which also fires on a deliberate operator stop.
func DefaultNotifyEvents() map[string]bool {
	events := make(map[string]bool, len(NotifyEventKinds))
	for _, kind := range NotifyEventKinds {
		events[kind] = kind != NotifyEventMowStopped
	}
	return events
}

// Message ids (wording keys). One kind may carry several.
const (
	NotifyMsgMowStarted      = "mowStarted"
	NotifyMsgZoneStarted     = "zoneStarted"
	NotifyMsgZoneFinished    = "zoneFinished"
	NotifyMsgZoneChanged     = "zoneChanged"
	NotifyMsgMowComplete     = "mowComplete"
	NotifyMsgDocked          = "docked"
	NotifyMsgMowStopped      = "mowStopped"
	NotifyMsgDigObstruction  = "digObstruction"
	NotifyMsgNavFailed       = "navFailed"
	NotifyMsgEmergency       = "emergency"
	NotifyMsgBatteryLow      = "batteryLow"
	NotifyMsgBatteryResumed  = "batteryResumed"
	NotifyMsgRainDetected    = "rainDetected"
	NotifyMsgRainResumed     = "rainResumed"
	NotifyMsgRainTimeout     = "rainTimeout"
	NotifyMsgRtkWaiting      = "rtkWaiting"
	NotifyMsgRtkRecovered    = "rtkRecovered"
	NotifyMsgTestNotifcation = "test"
)

// NotifyStatus is the slice of HighLevelStatus the detector reads.
type NotifyStatus struct {
	State           int     `json:"state"`
	StateName       string  `json:"state_name"`
	CurrentArea     int     `json:"current_area"`
	CoveragePercent float32 `json:"coverage_percent"`
	BatteryPercent  float32 `json:"battery_percent"`
	Emergency       bool    `json:"emergency"`
}

// NotifyEvent is one detected, not-yet-rendered notification.
type NotifyEvent struct {
	Kind     string
	Message  string
	Priority int
	// Params feed the message catalogue: area, areaIndex, coverage, next,
	// nextIndex, battery, state.
	Params map[string]string
	At     time.Time
}

// Priorities on ntfy's 1..5 scale (3 = default, 5 = max / bypasses DND).
const (
	notifyPriorityLow     = 2
	notifyPriorityDefault = 3
	notifyPriorityHigh    = 4
	notifyPriorityMax     = 5
)

// notifyRtkDwell is how long WAITING_FOR_RTK must persist before it is worth a
// push: the undock gate routinely waits a few seconds for Fixed under the dock
// canopy, and that is not a problem the operator needs to hear about.
const notifyRtkDwell = 120 * time.Second

// notifyCooldown suppresses a repeat of the same message id: a flapping state
// (Cyclone rediscovery, a bouncing e-stop switch) must not storm the phone.
const notifyCooldown = 60 * time.Second

// blockedStates are the BT's terminal failure states: the mission stopped
// somewhere the operator has to go and look.
var blockedStates = map[string]bool{
	"NAV_TO_DOCK_FAILED":          true,
	"COVERAGE_FAILED_DOCKING":     true,
	"UNDOCK_FAILED":               true,
	"CHARGER_FAILED":              true,
	"CRITICAL_BATTERY_NAV_FAILED": true,
}

// NotifyDetector turns the HighLevelStatus stream into notification events.
// It is a pure state machine: no clock, no I/O — the caller supplies `now`
// and decides what to do with the events. It mirrors the session tracker's
// notion of a session (a recharge pause keeps the session open) so a mid-mow
// recharge does not produce a second "mowing started".
type NotifyDetector struct {
	prevState      string
	prevMowing     bool
	inSession      bool
	rechargePaused bool
	currentArea    int
	areaPeak       float32
	emergency      bool
	rtkSince       time.Time
	rtkNotified    bool
	// lastEventAt is when any event last fired, so the "mowing stopped"
	// catch-all stays quiet when a specific event just explained the stop
	// (dig obstruction, emergency, nav failure).
	lastEventAt time.Time
	lastSent    map[string]time.Time
	areaNames   map[int]string
}

// NewNotifyDetector returns a detector with no session and no area.
func NewNotifyDetector() *NotifyDetector {
	return &NotifyDetector{
		currentArea: -1,
		lastSent:    make(map[string]time.Time),
		areaNames:   make(map[int]string),
	}
}

// SetAreaNames replaces the ROS area index → operator name lookup (from the
// GUI's map poll). Unnamed areas fall back to "Zone N" in the catalogue.
func (d *NotifyDetector) SetAreaNames(names map[int]string) {
	next := make(map[int]string, len(names))
	for k, v := range names {
		next[k] = v
	}
	d.areaNames = next
}

// OnStatus folds one status tick and returns the events it produced, in
// order. wantsKind reports whether a kind is enabled; it only steers the
// zone-finished + zone-started merge (both on → one combined message) and
// never suppresses an event — the caller filters on the same predicate.
func (d *NotifyDetector) OnStatus(st NotifyStatus, now time.Time, wantsKind func(string) bool) []NotifyEvent {
	if wantsKind == nil {
		wantsKind = func(string) bool { return true }
	}
	var events []NotifyEvent
	emit := func(kind, message string, priority int, params map[string]string) {
		if !d.pastCooldown(message, now) {
			return
		}
		d.lastSent[message] = now
		d.lastEventAt = now
		if params == nil {
			params = map[string]string{}
		}
		params["state"] = st.StateName
		params["battery"] = formatPercent(st.BatteryPercent)
		events = append(events, NotifyEvent{Kind: kind, Message: message, Priority: priority, Params: params, At: now})
	}

	prev := d.prevState
	entered := st.StateName != prev
	d.prevState = st.StateName

	// Emergency: rising edge only; a cleared latch is visible on the next mow.
	if st.Emergency && !d.emergency {
		emit(NotifyEventEmergency, NotifyMsgEmergency, notifyPriorityMax, nil)
	}
	d.emergency = st.Emergency

	isMowing := isActiveMowingSessionStatus(st.State, st.StateName)
	wasMowing := d.prevMowing
	d.prevMowing = isMowing

	if isMowing {
		d.onMowingTick(st, wantsKind, emit)
	}

	if entered {
		d.onStateEntered(st, prev, emit)
	}

	d.trackRtkDwell(st, now, entered, emit)

	// Session end: two consecutive non-mowing ticks that are neither a recharge
	// pause nor the handled completion, mirroring the session tracker.
	if d.inSession && !isMowing && !wasMowing && !d.isResumablePause(st) && st.StateName != "MOWING_COMPLETE" {
		d.endSession()
		if d.lastEventAt.IsZero() || now.Sub(d.lastEventAt) >= notifyCooldown {
			emit(NotifyEventMowStopped, NotifyMsgMowStopped, notifyPriorityDefault, nil)
		}
	}
	return events
}

type emitFn func(kind, message string, priority int, params map[string]string)

func (d *NotifyDetector) onMowingTick(st NotifyStatus, wantsKind func(string) bool, emit emitFn) {
	if !d.inSession {
		d.inSession = true
		d.rechargePaused = false
		d.currentArea = -1
		d.areaPeak = 0
		emit(NotifyEventMowStarted, NotifyMsgMowStarted, notifyPriorityDefault, nil)
	} else if d.rechargePaused {
		d.rechargePaused = false
		emit(NotifyEventBattery, NotifyMsgBatteryResumed, notifyPriorityDefault, nil)
	}

	if st.CurrentArea >= 0 && st.CurrentArea != d.currentArea {
		d.onAreaChanged(st.CurrentArea, wantsKind, emit)
	}
	if st.CurrentArea == d.currentArea && st.CoveragePercent > d.areaPeak {
		d.areaPeak = st.CoveragePercent
	}
}

// onAreaChanged reports the previous zone as finished and the new one as
// started. When both kinds are on they merge into ONE push (nobody wants two
// buzzes per zone boundary).
func (d *NotifyDetector) onAreaChanged(next int, wantsKind func(string) bool, emit emitFn) {
	prevArea := d.currentArea
	prevPeak := d.areaPeak
	d.currentArea = next
	d.areaPeak = 0

	nextParams := d.areaParams("next", "nextIndex", next)
	if prevArea < 0 {
		emit(NotifyEventZoneStarted, NotifyMsgZoneStarted, notifyPriorityLow, d.areaParams("area", "areaIndex", next))
		return
	}
	finished := d.areaParams("area", "areaIndex", prevArea)
	finished["coverage"] = formatPercent(prevPeak)
	if wantsKind(NotifyEventZoneFinished) && wantsKind(NotifyEventZoneStarted) {
		for k, v := range nextParams {
			finished[k] = v
		}
		emit(NotifyEventZoneStarted, NotifyMsgZoneChanged, notifyPriorityLow, finished)
		return
	}
	emit(NotifyEventZoneFinished, NotifyMsgZoneFinished, notifyPriorityLow, finished)
	emit(NotifyEventZoneStarted, NotifyMsgZoneStarted, notifyPriorityLow, d.areaParams("area", "areaIndex", next))
}

func (d *NotifyDetector) onStateEntered(st NotifyStatus, prev string, emit emitFn) {
	switch {
	case st.StateName == "MOWING_COMPLETE":
		params := map[string]string{}
		if d.currentArea >= 0 {
			params = d.areaParams("area", "areaIndex", d.currentArea)
			params["coverage"] = formatPercent(maxFloat32(d.areaPeak, st.CoveragePercent))
		}
		d.endSession()
		emit(NotifyEventMowComplete, NotifyMsgMowComplete, notifyPriorityDefault, params)
	case prev == "MOWING_COMPLETE" && (st.StateName == "CHARGING" || st.StateName == "IDLE_DOCKED"):
		emit(NotifyEventMowComplete, NotifyMsgDocked, notifyPriorityLow, nil)
	case st.StateName == "DIG_OBSTRUCTION":
		emit(NotifyEventBlocked, NotifyMsgDigObstruction, notifyPriorityMax, nil)
	case blockedStates[st.StateName]:
		emit(NotifyEventBlocked, NotifyMsgNavFailed, notifyPriorityHigh, nil)
	case st.StateName == "CRITICAL_BATTERY_DOCKING":
		emit(NotifyEventBattery, NotifyMsgBatteryLow, notifyPriorityDefault, nil)
	case st.StateName == "RAIN_DETECTED_DOCKING":
		emit(NotifyEventRain, NotifyMsgRainDetected, notifyPriorityDefault, nil)
	case st.StateName == "RESUMING_AFTER_RAIN":
		emit(NotifyEventRain, NotifyMsgRainResumed, notifyPriorityLow, nil)
	case st.StateName == "RAIN_TIMEOUT":
		emit(NotifyEventRain, NotifyMsgRainTimeout, notifyPriorityDefault, nil)
	}

	// A recharge mid-session keeps the session open and gets its own resume
	// notification. Rain recovery is reported separately by RESUMING_AFTER_RAIN.
	if d.inSession && isRechargeMowingPause(st.StateName) && prev != "MOWING_COMPLETE" {
		d.rechargePaused = true
	}
}

// trackRtkDwell pushes once when WAITING_FOR_RTK has lasted notifyRtkDwell,
// and once more when the fix comes back — only if the wait was reported.
func (d *NotifyDetector) trackRtkDwell(st NotifyStatus, now time.Time, entered bool, emit emitFn) {
	if st.StateName == "WAITING_FOR_RTK" {
		if entered || d.rtkSince.IsZero() {
			d.rtkSince = now
			d.rtkNotified = false
		}
		if !d.rtkNotified && now.Sub(d.rtkSince) >= notifyRtkDwell {
			d.rtkNotified = true
			emit(NotifyEventWaitingForRtk, NotifyMsgRtkWaiting, notifyPriorityDefault, map[string]string{
				"minutes": formatMinutes(now.Sub(d.rtkSince)),
			})
		}
		return
	}
	if d.rtkNotified && entered {
		emit(NotifyEventWaitingForRtk, NotifyMsgRtkRecovered, notifyPriorityLow, nil)
	}
	d.rtkSince = time.Time{}
	d.rtkNotified = false
}

// isResumablePause reports a mid-session state the BT resumes from on its own
// (recharge or rain wait); the session stays open and no stop is reported.
func (d *NotifyDetector) isResumablePause(st NotifyStatus) bool {
	if !d.inSession {
		return false
	}
	return isResumableMowingPause(st.StateName)
}

func (d *NotifyDetector) endSession() {
	d.inSession = false
	d.rechargePaused = false
	d.currentArea = -1
	d.areaPeak = 0
}

func (d *NotifyDetector) pastCooldown(message string, now time.Time) bool {
	last, ok := d.lastSent[message]
	return !ok || now.Sub(last) >= notifyCooldown
}

// areaParams names an area for the catalogue: the operator's name when the
// map has one, else an empty name and the 1-based index for a "Zone N" fallback.
func (d *NotifyDetector) areaParams(nameKey, indexKey string, index int) map[string]string {
	return map[string]string{
		nameKey:  d.areaNames[index],
		indexKey: itoa(index + 1),
	}
}
