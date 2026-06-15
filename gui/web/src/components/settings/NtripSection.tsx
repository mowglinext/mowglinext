import {useCallback, useEffect, useMemo, useState} from "react";
import {Alert, Button, Card, Collapse, Input, InputNumber, Select, Space, Spin, Switch, Tag, Typography} from "antd";
import {WifiOutlined, ReloadOutlined, EnvironmentOutlined} from "@ant-design/icons";
import {useApi} from "../../hooks/useApi.ts";
import {useIsMobile} from "../../hooks/useIsMobile.ts";
import {useThemeMode} from "../../theme/ThemeContext.tsx";
import {NTRIP_PROVIDERS, NTRIP_PROVIDER_BY_ID, PUBLIC_SOURCETABLE_PROVIDERS, providerForHost, NtripProvider} from "./ntripProviders.ts";
import {NtripStationMap, MapStation} from "./NtripStationMap.tsx";

const {Text, Paragraph} = Typography;

interface SourcetableStation {
  mountpoint: string;
  identifier: string;
  format: string;
  country: string;
  lat: number;
  lon: number;
}

interface Props {
  values: Record<string, any>;
  onChange: (key: string, value: any) => void;
}

// NtripSection: pick a provider, then a base station (from a map or list). The
// caster host/port and free-network credentials are filled in automatically, so
// the operator normally never types a host or a password.
export const NtripSection: React.FC<Props> = ({values, onChange}) => {
  const guiApi = useApi();
  const {colors} = useThemeMode();
  const isMobile = useIsMobile();

  const ntripEnabled = values.ntrip_enabled ?? true;
  const [providerId, setProviderId] = useState<string>(() => providerForHost(values.ntrip_host)?.id ?? "centipede");
  const [stations, setStations] = useState<MapStation[]>([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const provider = NTRIP_PROVIDER_BY_ID[providerId];

  const center = useMemo(() => {
    const lat = Number(values.datum_lat), lon = Number(values.datum_lon);
    return lat || lon ? {lat, lon} : undefined;
  }, [values.datum_lat, values.datum_lon]);

  const fetchFor = useCallback(async (provs: NtripProvider[]): Promise<MapStation[]> => {
    const all: MapStation[] = [];
    for (const p of provs) {
      const host = p.host || values.ntrip_host;
      if (!host) continue;
      const q = new URLSearchParams({host, port: String(p.port || values.ntrip_port || 2101)});
      if (p.requiresOwnCreds && values.ntrip_user) {
        q.set("user", values.ntrip_user);
        q.set("pass", values.ntrip_password ?? "");
      }
      const res = await guiApi.request<{ stations: SourcetableStation[] }>({
        path: `/ntrip/sourcetable?${q.toString()}`, method: "GET", format: "json",
      });
      for (const s of res.data?.stations ?? []) {
        all.push({...s, providerId: p.id, providerName: p.name, color: p.color});
      }
    }
    return all;
  }, [values.ntrip_host, values.ntrip_port, values.ntrip_user, values.ntrip_password]);

  const loadProvider = useCallback(async (p: NtripProvider) => {
    setLoading(true);
    setError(null);
    try {
      setStations(await fetchFor([p]));
    } catch {
      setError(`Could not load stations from ${p.name}. Check the caster is reachable.`);
      setStations([]);
    } finally {
      setLoading(false);
    }
  }, [fetchFor]);

  const loadAllFree = useCallback(async () => {
    setLoading(true);
    setError(null);
    try {
      setStations(await fetchFor(PUBLIC_SOURCETABLE_PROVIDERS));
    } catch {
      setError("Could not load the free networks.");
    } finally {
      setLoading(false);
    }
  }, [fetchFor]);

  const selectProvider = (p: NtripProvider) => {
    setProviderId(p.id);
    if (p.host) onChange("ntrip_host", p.host);
    onChange("ntrip_port", p.port);
    if (!p.requiresOwnCreds) {
      if (p.defaultUser !== undefined) onChange("ntrip_user", p.defaultUser);
      if (p.defaultPassword !== undefined) onChange("ntrip_password", p.defaultPassword);
    }
    onChange("ntrip_mountpoint", "");
    setStations([]);
    if (p.host && !p.requiresOwnCreds) loadProvider(p);
  };

  const selectStation = (s: MapStation) => {
    const p = NTRIP_PROVIDER_BY_ID[s.providerId];
    if (p) {
      setProviderId(p.id);
      if (p.host) onChange("ntrip_host", p.host);
      onChange("ntrip_port", p.port);
      if (!p.requiresOwnCreds) {
        if (p.defaultUser !== undefined) onChange("ntrip_user", p.defaultUser);
        if (p.defaultPassword !== undefined) onChange("ntrip_password", p.defaultPassword);
      }
    }
    onChange("ntrip_mountpoint", s.mountpoint);
  };

  // Auto-load the initially-selected provider's stations on mount.
  useEffect(() => {
    if (ntripEnabled && provider?.host && !provider.requiresOwnCreds) loadProvider(provider);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const needsManualCaster = provider?.host === "" || provider?.requiresOwnCreds;

  return (
    <Card size="small" style={{marginBottom: 16}}>
      <div style={{display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: 8}}>
        <Text strong style={{fontSize: 14}}><WifiOutlined style={{marginRight: 6}}/>NTRIP RTK Corrections</Text>
        <Switch checked={ntripEnabled} onChange={(v) => onChange("ntrip_enabled", v)}/>
      </div>

      {!ntripEnabled ? (
        <Alert type="info" showIcon message="Without NTRIP, GPS accuracy is ~2–5 m — not enough for autonomous mowing." />
      ) : (
        <>
          <Paragraph type="secondary" style={{marginTop: 0, marginBottom: 12}}>
            Pick a correction network, then choose the nearest base station on the map. Credentials for the free
            networks are filled in for you.
          </Paragraph>

          {/* Provider picker */}
          <div style={{display: "flex", flexWrap: "wrap", gap: 8, marginBottom: 12}}>
            {NTRIP_PROVIDERS.map((p) => {
              const active = p.id === providerId;
              return (
                <button key={p.id} onClick={() => selectProvider(p)} style={{
                  cursor: "pointer", textAlign: "left", borderRadius: 10, padding: "8px 12px",
                  background: active ? "rgba(124,255,178,0.12)" : "rgba(255,255,255,0.04)",
                  border: `1px solid ${active ? "rgba(124,255,178,0.4)" : "rgba(236,255,244,0.1)"}`,
                  minWidth: 150,
                }}>
                  <div style={{display: "flex", alignItems: "center", gap: 6}}>
                    <span style={{width: 10, height: 10, borderRadius: 5, background: p.color}}/>
                    <Text strong style={{fontSize: 13}}>{p.name}</Text>
                  </div>
                  <div style={{fontSize: 11, color: colors.textMuted}}>{p.region}</div>
                </button>
              );
            })}
            <Button size="small" onClick={loadAllFree} icon={<EnvironmentOutlined/>} style={{alignSelf: "center"}}>
              Show all free networks
            </Button>
          </div>

          {provider?.note && (
            <Paragraph type="secondary" style={{fontSize: 12, marginTop: -4}}>{provider.note}</Paragraph>
          )}

          {/* Manual caster fields for custom / paid providers */}
          {needsManualCaster && (
            <Space wrap style={{marginBottom: 12}}>
              <Input addonBefore="Host" style={{width: 240}} value={values.ntrip_host ?? ""}
                     onChange={(e) => onChange("ntrip_host", e.target.value)} placeholder="caster.example.com"/>
              <InputNumber addonBefore="Port" value={values.ntrip_port ?? 2101}
                           onChange={(v) => onChange("ntrip_port", v)}/>
              {provider?.requiresOwnCreds && (
                <>
                  <Input addonBefore="User" value={values.ntrip_user ?? ""}
                         onChange={(e) => onChange("ntrip_user", e.target.value)}/>
                  <Input.Password addonBefore="Pass" value={values.ntrip_password ?? ""}
                                  onChange={(e) => onChange("ntrip_password", e.target.value)}/>
                </>
              )}
              <Button icon={<ReloadOutlined/>} onClick={() => provider && loadProvider(provider)}
                      disabled={!values.ntrip_host}>Load stations</Button>
            </Space>
          )}

          {provider?.userIsEmail && (
            <Input addonBefore="Your email (RTK2go username)" style={{maxWidth: 380, marginBottom: 12}}
                   value={values.ntrip_user ?? ""} onChange={(e) => onChange("ntrip_user", e.target.value)}
                   placeholder="you@example.com"/>
          )}

          {/* Map + selection */}
          {loading ? (
            <div style={{display: "flex", justifyContent: "center", padding: 40}}><Spin/></div>
          ) : error ? (
            <Alert type="warning" showIcon message={error} style={{marginBottom: 12}}/>
          ) : stations.length > 0 ? (
            <>
              <div style={{display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: 8}}>
                <Text type="secondary" style={{fontSize: 12}}>{stations.length} base stations — click the nearest one</Text>
                {values.ntrip_mountpoint && <Tag color="success">Selected: {values.ntrip_mountpoint}</Tag>}
              </div>
              <NtripStationMap stations={stations} selectedMountpoint={values.ntrip_mountpoint}
                               selectedProviderId={providerId} center={center} onSelect={selectStation}
                               height={isMobile ? 280 : 360}/>
              <Select showSearch placeholder="…or search a station by name"
                      style={{width: "100%", marginTop: 8}} value={values.ntrip_mountpoint || undefined}
                      options={stations.map((s) => ({value: s.mountpoint, label: `${s.mountpoint} · ${s.providerName}${s.country ? ` (${s.country})` : ""}`}))}
                      onChange={(mp) => { const s = stations.find((x) => x.mountpoint === mp); if (s) selectStation(s); }}
                      filterOption={(inp, opt) => String(opt?.label ?? "").toLowerCase().includes(inp.toLowerCase())}/>
            </>
          ) : null}

          {/* Advanced raw fields */}
          <Collapse ghost style={{marginTop: 8}} items={[{
            key: "raw",
            label: <Text type="secondary" style={{fontSize: 12}}>Advanced: raw NTRIP fields</Text>,
            children: (
              <Space wrap>
                <Input addonBefore="Host" value={values.ntrip_host ?? ""} onChange={(e) => onChange("ntrip_host", e.target.value)}/>
                <InputNumber addonBefore="Port" value={values.ntrip_port ?? 2101} onChange={(v) => onChange("ntrip_port", v)}/>
                <Input addonBefore="Mountpoint" value={values.ntrip_mountpoint ?? ""} onChange={(e) => onChange("ntrip_mountpoint", e.target.value)}/>
                <Input addonBefore="User" value={values.ntrip_user ?? ""} onChange={(e) => onChange("ntrip_user", e.target.value)}/>
                <Input.Password addonBefore="Pass" value={values.ntrip_password ?? ""} onChange={(e) => onChange("ntrip_password", e.target.value)}/>
              </Space>
            ),
          }]}/>
        </>
      )}
    </Card>
  );
};
