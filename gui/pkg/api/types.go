package api

type OkResponse struct {
	Ok string `json:"ok,omitempty"`
}
type ErrorResponse struct {
	Error string `json:"error,omitempty"`
}

type SettingsStatusResponse struct {
	OnboardingCompleted bool `json:"onboarding_completed"`
}

type GetSettingsResponse struct {
	Settings map[string]any `json:"settings"`
}

type GetConfigResponse struct {
	TileUri string `json:"tileUri"`
}

type Container struct {
	ID     string            `json:"id"`
	Names  []string          `json:"names"`
	Labels map[string]string `json:"labels"`
	State  string            `json:"state"`
}

type ContainerListResponse struct {
	Containers []Container `json:"containers"`
}
