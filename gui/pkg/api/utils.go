package api

import (
	"encoding/json"
	"github.com/mitchellh/mapstructure"
	"io"
	"strings"
	"unicode"
)

func snakeToCamel(in string) string {
	if len(in) == 0 {
		return ""
	}
	tmp := []rune(in)
	tmp[0] = unicode.ToUpper(tmp[0])
	for i := 0; i < len(tmp); i++ {
		if tmp[i] == '_' {
			if i+1 < len(tmp) {
				tmp[i+1] = unicode.ToUpper(tmp[i+1])
				tmp = append(tmp[:i], tmp[i+1:]...)
				i--
			} else {
				tmp = tmp[:i]
			}
		}
	}
	return string(tmp)
}

func unmarshalROSMessage[T any](reader io.ReadCloser, out T) error {
	var m map[string]interface{}
	all, err := io.ReadAll(reader)
	if err != nil {
		return err
	}
	err = json.Unmarshal(all, &m)
	if err != nil {
		return err
	}
	// Normalize both keys: strip underscores and lower-case, so that
	// snake_case JSON ("is_navigation_area") matches PascalCase Go fields
	// ("IsNavigationArea"), camelCase JSON ("isNavigationArea") matches too,
	// and PascalCase JSON ("IsNavigationArea") still matches. Without the
	// underscore strip, snake_case keys silently dropped — that was the
	// regression that made GUI-saved navigation zones come through as
	// mowing zones.
	normalize := func(s string) string {
		return strings.ToLower(strings.ReplaceAll(s, "_", ""))
	}
	decoder, err := mapstructure.NewDecoder(&mapstructure.DecoderConfig{
		Result: out,
		MatchName: func(mapKey, fieldName string) bool {
			return normalize(fieldName) == normalize(mapKey)
		},
	})
	if err != nil {
		return err
	}
	err = decoder.Decode(m)
	if err != nil {
		return err
	}
	return nil
}
