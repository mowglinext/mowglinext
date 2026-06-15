package api

import (
	"encoding/json"
	"fmt"
	"net/http"
	"os"
	"sync"
	"time"

	"github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/gin-gonic/gin"
	"gopkg.in/yaml.v3"
)

// WeatherResponse is the GUI-facing live weather at the robot's datum.
type WeatherResponse struct {
	Available   bool    `json:"available"`
	TempC       float64 `json:"temp_c"`
	WeatherCode int     `json:"weather_code"`
	Condition   string  `json:"condition"` // clear | partly | cloudy | fog | rain | snow | storm
	IsRaining   bool    `json:"is_raining"`
}

// weatherCache memoizes the upstream call: the open-meteo current conditions
// change on the order of minutes, so a single robot polling every render would
// otherwise hammer the API for no benefit.
type weatherCache struct {
	mu      sync.Mutex
	value   WeatherResponse
	fetched time.Time
}

const weatherTTL = 10 * time.Minute

var weatherClient = &http.Client{Timeout: 6 * time.Second}

// WeatherRoutes registers GET /weather.
func WeatherRoutes(r *gin.RouterGroup, dbProvider types.IDBProvider) {
	cache := &weatherCache{}
	r.GET("/weather", getWeather(dbProvider, cache))
}

func getWeather(dbProvider types.IDBProvider, cache *weatherCache) gin.HandlerFunc {
	return func(c *gin.Context) {
		cache.mu.Lock()
		defer cache.mu.Unlock()

		if !cache.fetched.IsZero() && time.Since(cache.fetched) < weatherTTL {
			c.JSON(http.StatusOK, cache.value)
			return
		}

		lat, lon, ok := readDatumLatLon(dbProvider)
		if !ok {
			c.JSON(http.StatusOK, WeatherResponse{Available: false})
			return
		}

		w, err := fetchWeather(lat, lon)
		if err != nil {
			// Serve the last good value if we have one; otherwise report unavailable.
			if !cache.fetched.IsZero() {
				c.JSON(http.StatusOK, cache.value)
				return
			}
			c.JSON(http.StatusOK, WeatherResponse{Available: false})
			return
		}

		cache.value = w
		cache.fetched = time.Now()
		c.JSON(http.StatusOK, w)
	}
}

// readDatumLatLon loads datum_lat/datum_lon from mowgli_robot.yaml.
func readDatumLatLon(dbProvider types.IDBProvider) (lat, lon float64, ok bool) {
	configFilePath, err := dbProvider.Get("system.mower.yamlConfigFile")
	if err != nil {
		return 0, 0, false
	}
	data, err := os.ReadFile(string(configFilePath))
	if err != nil {
		return 0, 0, false
	}
	var yamlData map[string]interface{}
	if err := yaml.Unmarshal(data, &yamlData); err != nil {
		return 0, 0, false
	}
	lat = extractYAMLFloat(yamlData, "datum_lat")
	lon = extractYAMLFloat(yamlData, "datum_lon")
	if lat == 0 && lon == 0 {
		return 0, 0, false
	}
	return lat, lon, true
}

// fetchWeather queries open-meteo (no API key) for current conditions.
func fetchWeather(lat, lon float64) (WeatherResponse, error) {
	url := fmt.Sprintf(
		"https://api.open-meteo.com/v1/forecast?latitude=%.5f&longitude=%.5f&current=temperature_2m,precipitation,weather_code",
		lat, lon)

	resp, err := weatherClient.Get(url)
	if err != nil {
		return WeatherResponse{}, err
	}
	defer resp.Body.Close()
	if resp.StatusCode != http.StatusOK {
		return WeatherResponse{}, fmt.Errorf("weather api status %d", resp.StatusCode)
	}

	var body struct {
		Current struct {
			Temperature   float64 `json:"temperature_2m"`
			Precipitation float64 `json:"precipitation"`
			WeatherCode   int     `json:"weather_code"`
		} `json:"current"`
	}
	if err := json.NewDecoder(resp.Body).Decode(&body); err != nil {
		return WeatherResponse{}, err
	}

	condition, raining := weatherConditionFromCode(body.Current.WeatherCode, body.Current.Precipitation)
	return WeatherResponse{
		Available:   true,
		TempC:       body.Current.Temperature,
		WeatherCode: body.Current.WeatherCode,
		Condition:   condition,
		IsRaining:   raining,
	}, nil
}

// weatherConditionFromCode maps a WMO weather code (+ measured precipitation) to
// a coarse condition label and a raining flag. Pure function — unit tested.
func weatherConditionFromCode(code int, precipitation float64) (condition string, raining bool) {
	switch {
	case code == 0:
		condition = "clear"
	case code == 1 || code == 2:
		condition = "partly"
	case code == 3:
		condition = "cloudy"
	case code == 45 || code == 48:
		condition = "fog"
	case (code >= 51 && code <= 67) || (code >= 80 && code <= 82):
		condition = "rain"
	case (code >= 71 && code <= 77) || code == 85 || code == 86:
		condition = "snow"
	case code >= 95:
		condition = "storm"
	default:
		condition = "cloudy"
	}
	raining = precipitation > 0 || condition == "rain" || condition == "storm"
	return condition, raining
}
