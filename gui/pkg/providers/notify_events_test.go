package providers

import (
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

var t0 = time.Date(2026, 9, 15, 10, 0, 0, 0, time.UTC)

func tick(state int, name string, area int, coverage float32) NotifyStatus {
	return NotifyStatus{State: state, StateName: name, CurrentArea: area, CoveragePercent: coverage, BatteryPercent: 80}
}

func allKinds(string) bool { return true }

func messages(events []NotifyEvent) []string {
	out := make([]string, 0, len(events))
	for _, ev := range events {
		out = append(out, ev.Message)
	}
	return out
}

func TestNotifyDetector_MowStartedOnceAcrossRechargePause(t *testing.T) {
	d := NewNotifyDetector()
	now := t0

	got := d.OnStatus(tick(2, "UNDOCKING", -1, 0), now, allKinds)
	assert.Equal(t, []string{NotifyMsgMowStarted}, messages(got))

	now = now.Add(10 * time.Minute)
	got = d.OnStatus(tick(2, "MOWING", 0, 40), now, allKinds)
	assert.Equal(t, []string{NotifyMsgZoneStarted}, messages(got))

	// Low battery → dock → charge → resume must NOT be a second "mowing started".
	now = now.Add(time.Minute)
	got = d.OnStatus(tick(2, "CRITICAL_BATTERY_DOCKING", 0, 40), now, allKinds)
	assert.Equal(t, []string{NotifyMsgBatteryLow}, messages(got))
	now = now.Add(time.Minute)
	got = d.OnStatus(tick(1, "CRITICAL_BATTERY_CHARGING", 0, 40), now, allKinds)
	assert.Empty(t, got)
	now = now.Add(time.Minute)
	got = d.OnStatus(tick(1, "CHARGING", 0, 40), now, allKinds)
	assert.Empty(t, got)
	now = now.Add(30 * time.Minute)
	got = d.OnStatus(tick(2, "RESUMING_UNDOCKING", 0, 40), now, allKinds)
	assert.Equal(t, []string{NotifyMsgBatteryResumed}, messages(got))
}

func TestNotifyDetector_ZoneChangeMergesWhenBothKindsOn(t *testing.T) {
	d := NewNotifyDetector()
	d.SetAreaNames(map[int]string{0: "Front lawn", 2: "Back lawn"})
	now := t0
	d.OnStatus(tick(2, "MOWING", 0, 10), now, allKinds)
	now = now.Add(2 * time.Minute)
	d.OnStatus(tick(2, "MOWING", 0, 97.6), now, allKinds)
	now = now.Add(time.Minute)

	got := d.OnStatus(tick(2, "TRANSIT", 2, 0), now, allKinds)
	require.Len(t, got, 1)
	assert.Equal(t, NotifyEventZoneStarted, got[0].Kind)
	assert.Equal(t, NotifyMsgZoneChanged, got[0].Message)
	assert.Equal(t, "Front lawn", got[0].Params["area"])
	assert.Equal(t, "98 %", got[0].Params["coverage"])
	assert.Equal(t, "Back lawn", got[0].Params["next"])
	assert.Equal(t, "3", got[0].Params["nextIndex"])
}

func TestNotifyDetector_ZoneChangeSplitsWhenOnlyOneKindOn(t *testing.T) {
	d := NewNotifyDetector()
	onlyFinished := func(kind string) bool { return kind != NotifyEventZoneStarted }
	now := t0
	d.OnStatus(tick(2, "MOWING", 0, 10), now, onlyFinished)
	now = now.Add(2 * time.Minute)
	got := d.OnStatus(tick(2, "MOWING", 1, 0), now, onlyFinished)
	assert.Equal(t, []string{NotifyMsgZoneFinished, NotifyMsgZoneStarted}, messages(got))
}

func TestNotifyDetector_MowCompleteThenDocked(t *testing.T) {
	d := NewNotifyDetector()
	now := t0
	d.OnStatus(tick(2, "MOWING", 1, 50), now, allKinds)
	now = now.Add(time.Minute)
	d.OnStatus(tick(2, "MOWING", 1, 100), now, allKinds)
	now = now.Add(time.Minute)

	got := d.OnStatus(tick(2, "MOWING_COMPLETE", 1, 100), now, allKinds)
	require.Equal(t, []string{NotifyMsgMowComplete}, messages(got))
	assert.Equal(t, "2", got[0].Params["areaIndex"])
	assert.Equal(t, "100 %", got[0].Params["coverage"])

	now = now.Add(3 * time.Minute)
	got = d.OnStatus(tick(1, "CHARGING", -1, 0), now, allKinds)
	assert.Equal(t, []string{NotifyMsgDocked}, messages(got))
	// A completed mow never also reports "mowing stopped".
	now = now.Add(time.Minute)
	assert.Empty(t, d.OnStatus(tick(1, "CHARGING", -1, 0), now, allKinds))
	now = now.Add(time.Minute)
	assert.Empty(t, d.OnStatus(tick(1, "IDLE_DOCKED", -1, 0), now, allKinds))
}

func TestNotifyDetector_BlockedSuppressesStoppedCatchAll(t *testing.T) {
	d := NewNotifyDetector()
	now := t0
	d.OnStatus(tick(2, "MOWING", 0, 10), now, allKinds)
	now = now.Add(5 * time.Minute)
	got := d.OnStatus(tick(1, "DIG_OBSTRUCTION", 0, 10), now, allKinds)
	require.Equal(t, []string{NotifyMsgDigObstruction}, messages(got))
	assert.Equal(t, notifyPriorityMax, got[0].Priority)
	now = now.Add(time.Second)
	assert.Empty(t, d.OnStatus(tick(1, "DIG_OBSTRUCTION", 0, 10), now, allKinds), "the hold is already reported")
}

func TestNotifyDetector_OperatorStopReportsStopped(t *testing.T) {
	d := NewNotifyDetector()
	now := t0
	d.OnStatus(tick(2, "MOWING", 0, 10), now, allKinds)
	now = now.Add(5 * time.Minute)
	assert.Empty(t, d.OnStatus(tick(1, "IDLE", 0, 10), now, allKinds), "first non-mowing tick may be a flip")
	now = now.Add(time.Second)
	got := d.OnStatus(tick(1, "IDLE", 0, 10), now, allKinds)
	assert.Equal(t, []string{NotifyMsgMowStopped}, messages(got))
}

func TestNotifyDetector_NavFailureIsBlocked(t *testing.T) {
	d := NewNotifyDetector()
	now := t0
	d.OnStatus(tick(2, "MOWING", 0, 10), now, allKinds)
	now = now.Add(5 * time.Minute)
	got := d.OnStatus(tick(2, "NAV_TO_DOCK_FAILED", 0, 10), now, allKinds)
	require.Equal(t, []string{NotifyMsgNavFailed}, messages(got))
	assert.Equal(t, NotifyEventBlocked, got[0].Kind)
	assert.Equal(t, "NAV_TO_DOCK_FAILED", got[0].Params["state"])
}

func TestNotifyDetector_EmergencyRisingEdgeOnly(t *testing.T) {
	d := NewNotifyDetector()
	now := t0
	st := tick(1, "IDLE", -1, 0)
	st.Emergency = true
	got := d.OnStatus(st, now, allKinds)
	require.Equal(t, []string{NotifyMsgEmergency}, messages(got))
	assert.Equal(t, NotifyEventEmergency, got[0].Kind)
	now = now.Add(time.Second)
	assert.Empty(t, d.OnStatus(st, now, allKinds))
	st.Emergency = false
	now = now.Add(time.Second)
	assert.Empty(t, d.OnStatus(st, now, allKinds))
}

func TestNotifyDetector_RtkWaitNeedsDwell(t *testing.T) {
	d := NewNotifyDetector()
	now := t0
	var gotAll []NotifyEvent
	gotAll = append(gotAll, d.OnStatus(tick(2, "MOWING", 0, 10), now, allKinds)...)
	now = now.Add(time.Minute)
	gotAll = append(gotAll, d.OnStatus(tick(2, "WAITING_FOR_RTK", 0, 10), now, allKinds)...)
	now = now.Add(notifyRtkDwell - time.Second)
	gotAll = append(gotAll, d.OnStatus(tick(2, "WAITING_FOR_RTK", 0, 10), now, allKinds)...)
	now = now.Add(2 * time.Second)
	got := d.OnStatus(tick(2, "WAITING_FOR_RTK", 0, 10), now, allKinds)
	require.Equal(t, []string{NotifyMsgRtkWaiting}, messages(got))
	assert.Equal(t, "2 min", got[0].Params["minutes"])
	gotAll = append(gotAll, got...)
	now = now.Add(time.Second)
	assert.Empty(t, d.OnStatus(tick(2, "WAITING_FOR_RTK", 0, 10), now, allKinds), "reported once")
	// A brief non-autonomous recovery tick after WAITING_FOR_RTK is a status
	// transition inside the same mow, not an operator session boundary.
	now = now.Add(time.Minute)
	got = d.OnStatus(tick(1, "IDLE", 0, 10), now, allKinds)
	assert.Equal(t, []string{NotifyMsgRtkRecovered}, messages(got))
	gotAll = append(gotAll, got...)
	now = now.Add(time.Second)
	gotAll = append(gotAll, d.OnStatus(tick(2, "MOWING", 0, 10), now, allKinds)...)
	assert.Equal(t,
		[]string{NotifyMsgMowStarted, NotifyMsgZoneStarted, NotifyMsgRtkWaiting, NotifyMsgRtkRecovered},
		messages(gotAll),
	)
}

func TestNotifyDetector_ShortRtkWaitIsSilent(t *testing.T) {
	d := NewNotifyDetector()
	now := t0
	var gotAll []NotifyEvent
	gotAll = append(gotAll, d.OnStatus(tick(2, "MOWING", 0, 10), now, allKinds)...)
	now = now.Add(time.Minute)
	gotAll = append(gotAll, d.OnStatus(tick(2, "WAITING_FOR_RTK", 0, 10), now, allKinds)...)
	now = now.Add(20 * time.Second)
	gotAll = append(gotAll, d.OnStatus(tick(1, "IDLE", 0, 10), now, allKinds)...)
	now = now.Add(time.Second)
	gotAll = append(gotAll, d.OnStatus(tick(2, "MOWING", 0, 10), now, allKinds)...)
	assert.Equal(t, []string{NotifyMsgMowStarted, NotifyMsgZoneStarted}, messages(gotAll))
}

func TestNotifyDetector_ExplicitStopThenStartCreatesNewSession(t *testing.T) {
	d := NewNotifyDetector()
	now := t0
	var gotAll []NotifyEvent
	gotAll = append(gotAll, d.OnStatus(tick(2, "MOWING", 0, 10), now, allKinds)...)
	now = now.Add(5 * time.Minute)
	gotAll = append(gotAll, d.OnStatus(tick(1, "IDLE", 0, 10), now, allKinds)...)
	now = now.Add(time.Second)
	gotAll = append(gotAll, d.OnStatus(tick(1, "IDLE", 0, 10), now, allKinds)...)
	now = now.Add(2 * time.Minute)
	gotAll = append(gotAll, d.OnStatus(tick(2, "MOWING", 0, 0), now, allKinds)...)

	assert.Equal(t, []string{
		NotifyMsgMowStarted,
		NotifyMsgZoneStarted,
		NotifyMsgMowStopped,
		NotifyMsgMowStarted,
		NotifyMsgZoneStarted,
	}, messages(gotAll))
}

func TestNotifyDetector_RainEvents(t *testing.T) {
	d := NewNotifyDetector()
	now := t0
	d.OnStatus(tick(2, "MOWING", 0, 10), now, allKinds)
	now = now.Add(time.Minute)
	assert.Equal(t, []string{NotifyMsgRainDetected}, messages(d.OnStatus(tick(2, "RAIN_DETECTED_DOCKING", 0, 10), now, allKinds)))
	now = now.Add(time.Minute)
	assert.Empty(t, d.OnStatus(tick(1, "RAIN_WAITING", 0, 10), now, allKinds))
	now = now.Add(time.Minute)
	assert.Empty(t, d.OnStatus(tick(1, "RAIN_WAITING", 0, 10), now, allKinds))
	now = now.Add(time.Hour)
	assert.Equal(t, []string{NotifyMsgRainTimeout}, messages(d.OnStatus(tick(1, "RAIN_TIMEOUT", 0, 10), now, allKinds)))
}

func TestNotifyDetector_RainPauseResumesSameSession(t *testing.T) {
	d := NewNotifyDetector()
	now := t0
	var gotAll []NotifyEvent
	gotAll = append(gotAll, d.OnStatus(tick(2, "MOWING", 0, 10), now, allKinds)...)
	now = now.Add(time.Minute)
	gotAll = append(gotAll, d.OnStatus(tick(2, "RAIN_DETECTED_DOCKING", 0, 10), now, allKinds)...)
	now = now.Add(time.Minute)
	gotAll = append(gotAll, d.OnStatus(tick(1, "RAIN_WAITING", 0, 10), now, allKinds)...)
	now = now.Add(time.Minute)
	gotAll = append(gotAll, d.OnStatus(tick(1, "RAIN_WAITING", 0, 10), now, allKinds)...)
	now = now.Add(5 * time.Minute)
	gotAll = append(gotAll, d.OnStatus(tick(2, "RESUMING_AFTER_RAIN", 0, 10), now, allKinds)...)
	now = now.Add(time.Second)
	gotAll = append(gotAll, d.OnStatus(tick(2, "MOWING", 0, 10), now, allKinds)...)

	assert.Equal(t, []string{
		NotifyMsgMowStarted,
		NotifyMsgZoneStarted,
		NotifyMsgRainDetected,
		NotifyMsgRainResumed,
	}, messages(gotAll))
}

func TestNotifyDetector_CooldownSuppressesFlapping(t *testing.T) {
	d := NewNotifyDetector()
	now := t0
	st := tick(1, "IDLE", -1, 0)
	st.Emergency = true
	require.Len(t, d.OnStatus(st, now, allKinds), 1)
	st.Emergency = false
	now = now.Add(time.Second)
	d.OnStatus(st, now, allKinds)
	st.Emergency = true
	now = now.Add(time.Second)
	assert.Empty(t, d.OnStatus(st, now, allKinds), "bouncing e-stop inside the cooldown")
	now = now.Add(notifyCooldown)
	st.Emergency = false
	d.OnStatus(st, now, allKinds)
	st.Emergency = true
	now = now.Add(time.Second)
	assert.Len(t, d.OnStatus(st, now, allKinds), 1)
}

func TestRenderNotification_FallsBackToZoneIndexAndLanguage(t *testing.T) {
	ev := NotifyEvent{Kind: NotifyEventZoneStarted, Message: NotifyMsgZoneChanged, Priority: 2, Params: map[string]string{
		"area": "", "areaIndex": "1", "coverage": "98 %", "next": "Potager", "nextIndex": "3",
	}}
	en := RenderNotification("en", "Mowgli", ev)
	assert.Equal(t, "Mowgli", en.Title)
	assert.Equal(t, "Finished Zone 1 (98 % covered). Now mowing Potager.", en.Body)
	assert.Equal(t, []string{"white_check_mark"}, en.Tags)

	fr := RenderNotification("fr", "Mowgli", ev)
	assert.Equal(t, "Zone 1 terminée (98 % couverts). Tonte de Potager en cours.", fr.Body)

	unknown := RenderNotification("de", "Mowgli", ev)
	assert.Equal(t, en.Body, unknown.Body, "unknown language falls back to English")
}

func TestNotifyCatalogue_EveryMessageInEveryLanguage(t *testing.T) {
	for lang, catalogue := range notifyCatalogue {
		for msg := range notifyTags {
			assert.Containsf(t, catalogue, msg, "language %s lacks message %s", lang, msg)
		}
		assert.Contains(t, catalogue, "zoneFallback")
	}
}
