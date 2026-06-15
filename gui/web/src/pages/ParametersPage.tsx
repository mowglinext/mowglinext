import {useCallback, useEffect, useMemo, useState} from "react";
import {Segmented, Input, InputNumber, Switch, Collapse, Spin, Empty, Tag, App, Tooltip} from "antd";
import {SearchOutlined} from "@ant-design/icons";
import {useApi} from "../hooks/useApi.ts";
import {ContentType} from "../api/Api.ts";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {
  paramMeta, ParamTier, TIER_ORDER, TIER_LABEL, TIER_RANK,
} from "../components/settings/paramCatalog.ts";

interface RosParameter {
  name: string;
  value: unknown;
  type?: string;
}

interface ParamsResponse {
  parameters: RosParameter[];
}

// ParamRow renders one editable parameter. Memoised so editing one row does not
// re-render the whole (potentially 300-row) list.
function ParamRow({param, onCommit}: {param: RosParameter; onCommit: (name: string, value: unknown) => Promise<void>}) {
  const {colors} = useThemeMode();
  const meta = paramMeta(param.name);
  const [value, setValue] = useState<unknown>(param.value);
  const [saving, setSaving] = useState(false);

  // Keep local edits in sync if the upstream value changes (e.g. a refetch).
  useEffect(() => { setValue(param.value); }, [param.value]);

  const commit = useCallback(async (next: unknown) => {
    setSaving(true);
    try {
      await onCommit(param.name, next);
    } finally {
      setSaving(false);
    }
  }, [onCommit, param.name]);

  const control = (() => {
    if (typeof param.value === "boolean") {
      return <Switch checked={value as boolean} loading={saving}
                     onChange={(v) => { setValue(v); commit(v); }}/>;
    }
    if (typeof param.value === "number") {
      return <InputNumber value={value as number} disabled={saving} style={{width: 160}}
                          addonAfter={meta.unit}
                          onChange={(v) => setValue(v)}
                          onBlur={() => value !== param.value && commit(value)}
                          onPressEnter={() => value !== param.value && commit(value)}/>;
    }
    if (Array.isArray(param.value)) {
      // Arrays are read-only here — editing vectors safely needs per-element UI.
      return <Tag>{JSON.stringify(param.value)}</Tag>;
    }
    return <Input value={String(value ?? "")} disabled={saving} style={{width: 220}}
                  addonAfter={meta.unit}
                  onChange={(e) => setValue(e.target.value)}
                  onBlur={() => value !== param.value && commit(value)}
                  onPressEnter={() => value !== param.value && commit(value)}/>;
  })();

  return (
    <div style={{
      display: "flex", alignItems: "center", justifyContent: "space-between",
      gap: 16, padding: "10px 0", borderBottom: `1px solid ${colors.borderSubtle}`,
    }}>
      <div style={{minWidth: 0}}>
        <Tooltip title={param.name}>
          <div style={{fontWeight: 600, fontSize: 13}}>{meta.label}</div>
        </Tooltip>
        <div style={{fontSize: 11, color: colors.textMuted, marginTop: 2}}>{meta.description}</div>
      </div>
      <div style={{flexShrink: 0}}>{control}</div>
    </div>
  );
}

export const ParametersPage = () => {
  const guiApi = useApi();
  const {notification} = App.useApp();
  const {colors} = useThemeMode();

  const [params, setParams] = useState<RosParameter[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [tier, setTier] = useState<ParamTier>("basic");
  const [search, setSearch] = useState("");

  const fetchParams = useCallback(async () => {
    setLoading(true);
    setError(null);
    try {
      const res = await guiApi.request<ParamsResponse>({path: "/params", method: "GET", format: "json"});
      setParams((res.data?.parameters ?? []).slice().sort((a, b) => a.name.localeCompare(b.name)));
    } catch (e) {
      setError("Could not load parameters — is the robot connected?");
    } finally {
      setLoading(false);
    }
  }, []);

  useEffect(() => { fetchParams(); }, [fetchParams]);

  const commit = useCallback(async (name: string, value: unknown) => {
    try {
      await guiApi.request({
        path: "/params", method: "POST", format: "json",
        type: ContentType.Json,
        body: {parameters: [{name, value}]},
      });
      setParams((prev) => prev.map((p) => (p.name === name ? {...p, value} : p)));
      notification.success({message: `Updated ${name}`, duration: 2});
    } catch {
      notification.error({message: `Failed to update ${name}`});
      throw new Error("commit failed");
    }
  }, [notification]);

  // Filter by the selected profile (cumulative) and the search box, then group.
  const groups = useMemo(() => {
    const maxRank = TIER_RANK[tier];
    const q = search.trim().toLowerCase();
    const byGroup = new Map<string, RosParameter[]>();
    for (const p of params) {
      const meta = paramMeta(p.name);
      if (TIER_RANK[meta.tier] > maxRank) continue;
      if (q && !p.name.toLowerCase().includes(q) && !meta.label.toLowerCase().includes(q)) continue;
      const list = byGroup.get(meta.group) ?? [];
      list.push(p);
      byGroup.set(meta.group, list);
    }
    return Array.from(byGroup.entries()).sort((a, b) => a[0].localeCompare(b[0]));
  }, [params, tier, search]);

  const shownCount = groups.reduce((n, [, list]) => n + list.length, 0);

  return (
    <div style={{display: "flex", flexDirection: "column", gap: 16, paddingBottom: 16}}>
      <div style={{display: "flex", flexWrap: "wrap", gap: 12, alignItems: "center", justifyContent: "space-between"}}>
        <div>
          <Segmented
            value={tier}
            onChange={(v) => setTier(v as ParamTier)}
            options={TIER_ORDER.map((t) => ({label: TIER_LABEL[t], value: t}))}
          />
          <span style={{marginLeft: 12, fontSize: 12, color: colors.textMuted}}>
            {shownCount} parameter{shownCount !== 1 ? "s" : ""} · live edits apply immediately
          </span>
        </div>
        <Input
          prefix={<SearchOutlined/>}
          placeholder="Search parameters"
          allowClear
          value={search}
          onChange={(e) => setSearch(e.target.value)}
          style={{maxWidth: 280}}
        />
      </div>

      {error && <div style={{color: colors.danger, fontSize: 13}}>{error}</div>}

      {loading ? (
        <div style={{display: "flex", justifyContent: "center", padding: 48}}><Spin/></div>
      ) : groups.length === 0 ? (
        <Empty description={search ? "No parameters match your search" : "No parameters in this profile"}/>
      ) : (
        <Collapse
          defaultActiveKey={groups.map(([g]) => g)}
          items={groups.map(([group, list]) => ({
            key: group,
            label: <span style={{fontWeight: 600}}>{group} <Tag style={{marginLeft: 6}}>{list.length}</Tag></span>,
            children: (
              <div>
                {list.map((p) => <ParamRow key={p.name} param={p} onCommit={commit}/>)}
              </div>
            ),
          }))}
        />
      )}
    </div>
  );
};

export default ParametersPage;
