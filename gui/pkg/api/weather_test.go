package api

import "testing"

func TestWeatherConditionFromCode(t *testing.T) {
	cases := []struct {
		code      int
		precip    float64
		condition string
		raining   bool
	}{
		{0, 0, "clear", false},
		{1, 0, "partly", false},
		{2, 0, "partly", false},
		{3, 0, "cloudy", false},
		{45, 0, "fog", false},
		{61, 0.2, "rain", true},
		{63, 0, "rain", true}, // code implies rain even without measured precip
		{80, 1.0, "rain", true},
		{71, 0, "snow", false},
		{86, 0, "snow", false},
		{95, 0, "storm", true},
		{99, 0, "storm", true},
		{3, 0.5, "cloudy", true},  // overcast but actively precipitating
		{0, 0.3, "clear", true},   // measured precip overrides clear-code
	}
	for _, tc := range cases {
		gotCond, gotRain := weatherConditionFromCode(tc.code, tc.precip)
		if gotCond != tc.condition || gotRain != tc.raining {
			t.Errorf("code=%d precip=%.1f: got (%q,%v), want (%q,%v)",
				tc.code, tc.precip, gotCond, gotRain, tc.condition, tc.raining)
		}
	}
}
