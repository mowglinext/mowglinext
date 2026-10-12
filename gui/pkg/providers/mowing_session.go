package providers

// highLevelStateAutonomous mirrors HighLevelStatus.HIGH_LEVEL_STATE_AUTONOMOUS.
const highLevelStateAutonomous = 2

// isActiveMowingSessionStatus reports statuses that belong to the autonomous
// mowing run. The top-level state is authoritative: state names describe live
// phases (mowing, transit, localization recovery, undocking, and so on) and
// must not each maintain their own incomplete copy of the session definition.
func isActiveMowingSessionStatus(state int, stateName string) bool {
	return state == highLevelStateAutonomous && stateName != "MOWING_COMPLETE"
}

// isResumableMowingPause reports non-autonomous holds from which the behavior
// tree resumes the current mow without a new operator start.
func isResumableMowingPause(stateName string) bool {
	switch stateName {
	case "CHARGING", "CRITICAL_BATTERY_CHARGING", "MANUAL_CHARGING", "RAIN_WAITING":
		return true
	}
	return false
}

func isRechargeMowingPause(stateName string) bool {
	return stateName == "CHARGING" || stateName == "CRITICAL_BATTERY_CHARGING"
}

func isRainMowingPause(stateName string) bool {
	return stateName == "RAIN_WAITING"
}
