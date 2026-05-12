package api

import (
	"io"
	"strings"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestSnakeToCamel(t *testing.T) {
	tests := []struct {
		input    string
		expected string
	}{
		{"hello_world", "HelloWorld"},
		{"is_navigation_area", "IsNavigationArea"},
		{"simple", "Simple"},
		{"one_two_three", "OneTwoThree"},
		{"already_Camel", "AlreadyCamel"},
	}

	for _, tt := range tests {
		t.Run(tt.input, func(t *testing.T) {
			result := snakeToCamel(tt.input)
			assert.Equal(t, tt.expected, result)
		})
	}
}

func TestUnmarshalROSMessage(t *testing.T) {
	type TestMsg struct {
		Name             string
		IsNavigationArea bool
		Value            float64
	}

	t.Run("camelCase JSON to PascalCase struct", func(t *testing.T) {
		jsonBody := `{"name": "test-area", "isNavigationArea": true, "value": 3.14}`
		reader := io.NopCloser(strings.NewReader(jsonBody))

		var msg TestMsg
		err := unmarshalROSMessage[*TestMsg](reader, &msg)
		require.NoError(t, err)

		assert.Equal(t, "test-area", msg.Name)
		assert.True(t, msg.IsNavigationArea)
		assert.InDelta(t, 3.14, msg.Value, 0.001)
	})

	// Regression: the GUI sends ROS-style snake_case ("is_navigation_area")
	// in the request body. With a naive case-insensitive matcher (no
	// underscore stripping) this silently dropped the field and every
	// navigation zone saved by the map editor arrived at ROS2 as a
	// mowing zone. Locking in snake_case → PascalCase here.
	t.Run("snake_case JSON to PascalCase struct", func(t *testing.T) {
		jsonBody := `{"name": "nav-zone", "is_navigation_area": true, "value": 0.5}`
		reader := io.NopCloser(strings.NewReader(jsonBody))

		var msg TestMsg
		err := unmarshalROSMessage[*TestMsg](reader, &msg)
		require.NoError(t, err)

		assert.Equal(t, "nav-zone", msg.Name)
		assert.True(t, msg.IsNavigationArea,
			"is_navigation_area must round-trip into IsNavigationArea — "+
				"the GUI sends snake_case and ROS2 expects this flag preserved")
	})

	t.Run("PascalCase JSON to PascalCase struct", func(t *testing.T) {
		jsonBody := `{"Name": "area2", "IsNavigationArea": false, "Value": 1.0}`
		reader := io.NopCloser(strings.NewReader(jsonBody))

		var msg TestMsg
		err := unmarshalROSMessage[*TestMsg](reader, &msg)
		require.NoError(t, err)

		assert.Equal(t, "area2", msg.Name)
		assert.False(t, msg.IsNavigationArea)
	})

	t.Run("invalid JSON", func(t *testing.T) {
		reader := io.NopCloser(strings.NewReader("not json"))

		var msg TestMsg
		err := unmarshalROSMessage[*TestMsg](reader, &msg)
		assert.Error(t, err)
	})

	t.Run("empty JSON object", func(t *testing.T) {
		reader := io.NopCloser(strings.NewReader("{}"))

		var msg TestMsg
		err := unmarshalROSMessage[*TestMsg](reader, &msg)
		require.NoError(t, err)
		assert.Equal(t, "", msg.Name)
		assert.False(t, msg.IsNavigationArea)
		assert.Equal(t, 0.0, msg.Value)
	})
}
