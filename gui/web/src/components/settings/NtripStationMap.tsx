import {useCallback, useMemo, useState} from "react";
import Map, {Source, Layer, Popup, Marker, NavigationControl} from "react-map-gl/mapbox";
import type {MapMouseEvent} from "react-map-gl/mapbox";
import {useTranslation} from "react-i18next";
import {MAPBOX_TOKEN} from "./ntripProviders.ts";

export interface MapStation {
  mountpoint: string;
  identifier: string;
  lat: number;
  lon: number;
  country?: string;
  format?: string;
  providerId: string;
  providerName: string;
  color: string;
}

interface Props {
  stations: MapStation[];
  selectedMountpoint?: string;
  selectedProviderId?: string;
  center?: { lat: number; lon: number }; // robot / datum
  onSelect: (s: MapStation) => void;
  height?: number;
}

// Renders NTRIP base stations as a single GeoJSON circle layer (GPU-rendered,
// so it scales to the 1000+ markers a full caster sourcetable returns) and
// resolves a click back to the station the operator picked.
export function NtripStationMap({stations, selectedMountpoint, selectedProviderId, center, onSelect, height = 360}: Props) {
  const {t} = useTranslation();
  const [hover, setHover] = useState<{ lon: number; lat: number; text: string } | null>(null);
  const [cursor, setCursor] = useState<"auto" | "pointer">("auto");

  const data = useMemo(
    () => ({
      type: "FeatureCollection" as const,
      features: stations.map((s) => ({
        type: "Feature" as const,
        geometry: { type: "Point" as const, coordinates: [s.lon, s.lat] },
        properties: {
          mountpoint: s.mountpoint,
          providerId: s.providerId,
          providerName: s.providerName,
          country: s.country ?? "",
          color: s.color,
          selected: s.mountpoint === selectedMountpoint && s.providerId === selectedProviderId ? 1 : 0,
        },
      })),
    }),
    [stations, selectedMountpoint, selectedProviderId],
  );

  const initialViewState = useMemo(() => {
    if (center) return { longitude: center.lon, latitude: center.lat, zoom: 7 };
    if (stations.length) {
      const lat = stations.reduce((a, s) => a + s.lat, 0) / stations.length;
      const lon = stations.reduce((a, s) => a + s.lon, 0) / stations.length;
      return { longitude: lon, latitude: lat, zoom: 4 };
    }
    return { longitude: 2.3, latitude: 46.6, zoom: 4 };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [center, stations.length]);

  const handleClick = useCallback(
    (e: MapMouseEvent) => {
      const f = e.features?.[0];
      if (!f) return;
      const p = f.properties as { mountpoint: string; providerId: string };
      const s = stations.find((x) => x.mountpoint === p.mountpoint && x.providerId === p.providerId);
      if (s) onSelect(s);
    },
    [stations, onSelect],
  );

  const handleMove = useCallback((e: MapMouseEvent) => {
    const f = e.features?.[0];
    if (f) {
      const p = f.properties as { mountpoint: string; providerName: string; country: string };
      const [lon, lat] = (f.geometry as unknown as { coordinates: [number, number] }).coordinates;
      setHover({ lon, lat, text: `${p.mountpoint} · ${p.providerName}${p.country ? ` (${p.country})` : ""}` });
      setCursor("pointer");
    } else {
      setHover(null);
      setCursor("auto");
    }
  }, []);

  return (
    <div style={{ height, borderRadius: 12, overflow: "hidden", border: "1px solid rgba(236,255,244,0.1)" }}>
      <Map
        reuseMaps
        mapboxAccessToken={MAPBOX_TOKEN}
        initialViewState={initialViewState}
        mapStyle="mapbox://styles/mapbox/dark-v11"
        style={{ width: "100%", height: "100%" }}
        attributionControl={false}
        interactiveLayerIds={["ntrip-stations"]}
        cursor={cursor}
        onClick={handleClick}
        onMouseMove={handleMove}
        onMouseLeave={() => { setHover(null); setCursor("auto"); }}
      >
        <NavigationControl position="top-right" showCompass={false} />
        <Source id="ntrip" type="geojson" data={data}>
          <Layer
            id="ntrip-stations"
            type="circle"
            paint={{
              "circle-radius": ["case", ["==", ["get", "selected"], 1], 9, 5],
              "circle-color": ["get", "color"],
              "circle-opacity": 0.9,
              "circle-stroke-width": ["case", ["==", ["get", "selected"], 1], 3, 1],
              "circle-stroke-color": ["case", ["==", ["get", "selected"], 1], "#ffffff", "rgba(2,17,13,0.6)"],
            }}
          />
        </Source>
        {center && (
          <Marker longitude={center.lon} latitude={center.lat}>
            <div
              title={t("settingsNtripStationMap.yourRobot")}
              style={{
                width: 14, height: 14, borderRadius: 7, background: "#fff",
                border: "2px solid #02110D", boxShadow: "0 0 0 3px rgba(124,255,178,0.6)",
              }}
            />
          </Marker>
        )}
        {hover && (
          <Popup longitude={hover.lon} latitude={hover.lat} closeButton={false} closeOnClick={false} offset={10}>
            <div style={{ fontSize: 12, color: "#111", fontWeight: 600 }}>{hover.text}</div>
          </Popup>
        )}
      </Map>
    </div>
  );
}
