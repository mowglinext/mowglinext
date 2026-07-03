import {useCallback, useEffect, useMemo, useState} from "react";
import {useTranslation} from "react-i18next";
import {Segmented, Input, InputNumber, Switch, Collapse, Spin, Empty, Tag, Alert, Button, App, Tooltip} from "antd";
import {SearchOutlined, WarningOutlined, InfoCircleOutlined} from "@ant-design/icons";
import {useApi} from "../hooks/useApi.ts";
import {useIsMobile} from "../hooks/useIsMobile.ts";
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

// Profile labels. The catalog keys stay basic/middle/expert; only the display
// strings are localized here. We store i18n KEY strings and resolve them with
// t() at render time (see AppShell.tsx for the same pattern).
const TIER_LABEL_KEY: Record<ParamTier, string> = {
  basic: "parametersPage.tierBasic",
  middle: "parametersPage.tierMiddle",
  expert: "parametersPage.tierExpert",
};

// Parameters that affect physical motion or safety. Editing these live can move
// the robot or change battery/blade behaviour instantly, so they get a danger
// Tag and a confirm-on-change step regardless of tier.
const DANGER_RE = /vel|speed|accel|motor|blade|emergency|current|voltage|pid|limit/i;
const isDangerParam = (name: string): boolean => DANGER_RE.test(name);

// ParamRow renders one editable parameter. Memoised so editing one row does not
// re-render the whole (potentially 300-row) list.
function ParamRow({
  param, onCommit, disabled, isMobile,
}: {
  param: RosParameter;
  onCommit: (name: string, value: unknown) => Promise<void>;
  disabled: boolean;
  isMobile: boolean;
}) {
  const {t} = useTranslation();
  const {colors} = useThemeMode();
  const {modal} = App.useApp();
  const meta = paramMeta(param.name);
  const danger = isDangerParam(param.name);
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

  // For safety/motion params, confirm before applying the live change.
  const guardedCommit = useCallback((next: unknown) => {
    if (!danger) { commit(next); return; }
    modal.confirm({
      title: t("parametersPage.confirmSafetyTitle"),
      icon: <WarningOutlined style={{color: colors.danger}}/>,
      content: (
        <div>
          <p>
            <strong>{t(meta.label)}</strong> {t("parametersPage.confirmSafetyBody")}
          </p>
          <p style={{marginBottom: 0, color: colors.textMuted}}>
            {t("parametersPage.confirmNewValue")} <strong>{String(next)}</strong>{meta.unit ? ` ${meta.unit}` : ""}
          </p>
        </div>
      ),
      okText: t("parametersPage.apply"),
      okButtonProps: {danger: true},
      cancelText: t("parametersPage.cancel"),
      onOk: () => commit(next),
      onCancel: () => setValue(param.value), // revert the visual edit
    });
  }, [danger, commit, modal, colors, meta, param.value, t]);

  const controlWidth = isMobile ? "100%" : undefined;

  const control = (() => {
    if (typeof param.value === "boolean") {
      return <Switch checked={value as boolean} loading={saving} disabled={disabled}
                     onChange={(v) => { setValue(v); guardedCommit(v); }}/>;
    }
    if (typeof param.value === "number") {
      return <InputNumber value={value as number} disabled={saving || disabled}
                          style={{width: controlWidth ?? 160}}
                          addonAfter={meta.unit}
                          onChange={(v) => setValue(v)}
                          onBlur={() => value !== param.value && guardedCommit(value)}
                          onPressEnter={() => value !== param.value && guardedCommit(value)}/>;
    }
    if (Array.isArray(param.value)) {
      // Vectors are read-only here: editing them safely needs a per-element UI
      // (a single text field would let a malformed array reach a live param).
      return (
        <Tooltip title={t("parametersPage.arrayReadOnlyTooltip")}>
          <span style={{
            display: "inline-flex", flexWrap: "wrap", gap: 4,
            maxWidth: isMobile ? "100%" : 320,
            maxHeight: 96, overflowY: "auto",
          }}>
            <Tag icon={<InfoCircleOutlined/>} style={{whiteSpace: "normal", wordBreak: "break-all"}}>
              {JSON.stringify(param.value)}
            </Tag>
          </span>
        </Tooltip>
      );
    }
    return <Input value={String(value ?? "")} disabled={saving || disabled}
                  style={{width: controlWidth ?? 220}}
                  addonAfter={meta.unit}
                  onChange={(e) => setValue(e.target.value)}
                  onBlur={() => value !== param.value && guardedCommit(value)}
                  onPressEnter={() => value !== param.value && guardedCommit(value)}/>;
  })();

  return (
    <div style={{
      display: "flex",
      flexDirection: isMobile ? "column" : "row",
      alignItems: isMobile ? "stretch" : "center",
      justifyContent: "space-between",
      gap: isMobile ? 8 : 16,
      padding: "10px 0", borderBottom: `1px solid ${colors.borderSubtle}`,
    }}>
      <div style={{minWidth: 0}}>
        <Tooltip title={param.name}>
          <div style={{fontWeight: 600, fontSize: 13}}>
            {t(meta.label)}
            {danger && (
              <Tag color="error" icon={<WarningOutlined/>} style={{marginLeft: 8}}>
                {t("parametersPage.safetyTag")}
              </Tag>
            )}
          </div>
        </Tooltip>
        <div style={{fontSize: 11, color: colors.textMuted, marginTop: 2}}>{t(meta.description)}</div>
      </div>
      <div style={{flexShrink: 0, width: isMobile ? "100%" : undefined}}>{control}</div>
    </div>
  );
}

export const ParametersPage = () => {
  const {t} = useTranslation();
  const guiApi = useApi();
  const {notification} = App.useApp();
  const {colors} = useThemeMode();
  const isMobile = useIsMobile();

  const [params, setParams] = useState<RosParameter[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [tier, setTier] = useState<ParamTier>("basic");
  const [search, setSearch] = useState("");
  // One-time acknowledgment that the Avancé/Expert tiers edit live params.
  const [acknowledged, setAcknowledged] = useState(false);

  // The guardrail engages for any tier above "basic" (Avancé or Expert).
  const needsAck = tier !== "basic";
  const editsEnabled = !needsAck || acknowledged;

  const fetchParams = useCallback(async () => {
    setLoading(true);
    setError(null);
    try {
      const res = await guiApi.request<ParamsResponse>({path: "/params", method: "GET", format: "json"});
      setParams((res.data?.parameters ?? []).slice().sort((a, b) => a.name.localeCompare(b.name)));
    } catch (e) {
      setError(t("parametersPage.loadError"));
    } finally {
      setLoading(false);
    }
  }, [t]);

  useEffect(() => { fetchParams(); }, [fetchParams]);

  const commit = useCallback(async (name: string, value: unknown) => {
    try {
      await guiApi.request({
        path: "/params", method: "POST", format: "json",
        type: ContentType.Json,
        body: {parameters: [{name, value}]},
      });
      setParams((prev) => prev.map((p) => (p.name === name ? {...p, value} : p)));
      notification.success({message: t("parametersPage.updateSuccess", {name}), duration: 2});
    } catch {
      notification.error({message: t("parametersPage.updateFailure", {name})});
      throw new Error("commit failed");
    }
  }, [notification, t]);

  // Filter by the selected profile (cumulative) and the search box, then group.
  const groups = useMemo(() => {
    const maxRank = TIER_RANK[tier];
    const q = search.trim().toLowerCase();
    const byGroup = new Map<string, RosParameter[]>();
    for (const p of params) {
      const meta = paramMeta(p.name);
      if (TIER_RANK[meta.tier] > maxRank) continue;
      if (q && !p.name.toLowerCase().includes(q) && !t(meta.label).toLowerCase().includes(q)) continue;
      const list = byGroup.get(meta.group) ?? [];
      list.push(p);
      byGroup.set(meta.group, list);
    }
    return Array.from(byGroup.entries()).sort((a, b) => a[0].localeCompare(b[0]));
  }, [params, tier, search, t]);

  const shownCount = groups.reduce((n, [, list]) => n + list.length, 0);

  return (
    <div style={{display: "flex", flexDirection: "column", gap: 16, paddingBottom: 16}}>
      {/* Persistent reminder that these are LIVE params, not persisted to YAML. */}
      <Alert
        type="warning"
        showIcon
        message={t("parametersPage.liveWarningTitle")}
        description={t("parametersPage.liveWarningDescription")}
      />

      <div style={{display: "flex", flexWrap: "wrap", gap: 12, alignItems: "center", justifyContent: "space-between"}}>
        <div>
          <Segmented
            value={tier}
            onChange={(v) => setTier(v as ParamTier)}
            options={TIER_ORDER.map((tierKey) => ({
              label: TIER_LABEL_KEY[tierKey] ? t(TIER_LABEL_KEY[tierKey]) : t(TIER_LABEL[tierKey]),
              value: tierKey,
            }))}
          />
          <span style={{marginLeft: 12, fontSize: 12, color: colors.textMuted}}>
            {t("parametersPage.shownCount", {count: shownCount})}
          </span>
        </div>
        <Input
          prefix={<SearchOutlined/>}
          placeholder={t("parametersPage.searchPlaceholder")}
          allowClear
          value={search}
          onChange={(e) => setSearch(e.target.value)}
          style={{maxWidth: 280}}
        />
      </div>

      {/* Beginner guardrail: edits in Avancé/Expert require an explicit, one-time
          acknowledgment of the risk before any control is enabled. */}
      {needsAck && !acknowledged && (
        <Alert
          type="error"
          showIcon
          icon={<WarningOutlined/>}
          message={t("parametersPage.tierGuardTitle", {tier: t(TIER_LABEL_KEY[tier])})}
          description={
            <div>
              <p style={{marginTop: 0}}>
                {t("parametersPage.tierGuardDescription")}
              </p>
              <Button danger type="primary" onClick={() => setAcknowledged(true)}>
                {t("parametersPage.understandRisks")}
              </Button>
            </div>
          }
        />
      )}

      {error && <div style={{color: colors.danger, fontSize: 13}}>{error}</div>}

      {loading ? (
        <div style={{display: "flex", justifyContent: "center", padding: 48}}><Spin/></div>
      ) : groups.length === 0 ? (
        <Empty description={search ? t("parametersPage.emptySearch") : t("parametersPage.emptyProfile")}/>
      ) : (
        <Collapse
          defaultActiveKey={groups.map(([g]) => g)}
          items={groups.map(([group, list]) => ({
            key: group,
            label: <span style={{fontWeight: 600}}>{t(`paramGroups.${group}`)} <Tag style={{marginLeft: 6}}>{list.length}</Tag></span>,
            children: (
              <div style={{opacity: editsEnabled ? 1 : 0.55}}>
                {list.map((p) => (
                  <ParamRow key={p.name} param={p} onCommit={commit}
                            disabled={!editsEnabled} isMobile={isMobile}/>
                ))}
              </div>
            ),
          }))}
        />
      )}
    </div>
  );
};

export default ParametersPage;
