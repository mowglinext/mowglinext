import {observeAsync} from "../utils/async";
import {useCallback, useEffect, useState} from "react";
import {useApi} from "./useApi.ts";
import type {Condition} from "../concept/components/WeatherChip.tsx";

export interface Weather {
  available: boolean;
  temp_c: number;
  weather_code: number;
  condition: Condition;
  is_raining: boolean;
}

/**
 * Live outdoor weather at the robot's GPS datum, fetched from the backend
 * (open-meteo, cached server-side). Polls slowly — conditions move on the
 * order of minutes. Returns null until the first successful fetch, and keeps
 * the last good value if a later poll fails.
 */
export function useWeather(pollMs = 10 * 60 * 1000): Weather | null {
  const guiApi = useApi();
  const [weather, setWeather] = useState<Weather | null>(null);

  const fetchWeather = useCallback(async () => {
    try {
      const res = await guiApi.request<Weather>({path: "/weather", method: "GET", format: "json"});
      if (res.data) setWeather(res.data);
    } catch (error) {
      // Retain the last good weather; expose transport failures in diagnostics.
      console.error('Failed to refresh weather', error);
    }
  }, []);

  useEffect(() => {
    observeAsync(fetchWeather());
    const id = setInterval(() => observeAsync(fetchWeather()), pollMs);
    return () => clearInterval(id);
  }, [fetchWeather, pollMs]);

  return weather;
}
