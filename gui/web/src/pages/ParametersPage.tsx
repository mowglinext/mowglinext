import {useCallback, useEffect, useMemo, useState} from "react";
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

// Profile labels in French. The catalog keys stay basic/middle/expert; only the
// display strings are localized here.
const TIER_LABEL_FR: Record<ParamTier, string> = {
  basic: "Débutant",
  middle: "Avancé",
  expert: "Expert",
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
      title: "Appliquer ce réglage de sécurité ?",
      icon: <WarningOutlined style={{color: colors.danger}}/>,
      content: (
        <div>
          <p>
            <strong>{meta.label}</strong> influence le mouvement ou la sécurité du robot.
            La nouvelle valeur s'applique <strong>immédiatement</strong> sur le robot en marche.
          </p>
          <p style={{marginBottom: 0, color: colors.textMuted}}>
            Nouvelle valeur : <strong>{String(next)}</strong>{meta.unit ? ` ${meta.unit}` : ""}
          </p>
        </div>
      ),
      okText: "Appliquer",
      okButtonProps: {danger: true},
      cancelText: "Annuler",
      onOk: () => commit(next),
      onCancel: () => setValue(param.value), // revert the visual edit
    });
  }, [danger, commit, modal, colors, meta, param.value]);

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
        <Tooltip title="Les tableaux sont en lecture seule : modifier un vecteur en sécurité nécessite une interface par élément, pas encore disponible ici.">
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
            {meta.label}
            {danger && (
              <Tag color="error" icon={<WarningOutlined/>} style={{marginLeft: 8}}>
                Sécurité
              </Tag>
            )}
          </div>
        </Tooltip>
        <div style={{fontSize: 11, color: colors.textMuted, marginTop: 2}}>{meta.description}</div>
      </div>
      <div style={{flexShrink: 0, width: isMobile ? "100%" : undefined}}>{control}</div>
    </div>
  );
}

export const ParametersPage = () => {
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
      setError("Impossible de charger les paramètres — le robot est-il connecté ?");
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
      notification.success({message: `${name} mis à jour`, duration: 2});
    } catch {
      notification.error({message: `Échec de la mise à jour de ${name}`});
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
      {/* Persistent reminder that these are LIVE params, not persisted to YAML. */}
      <Alert
        type="warning"
        showIcon
        message="Ces réglages s'appliquent en direct et ne sont pas sauvegardés dans le YAML"
        description="Les modifications prennent effet immédiatement sur le robot en marche et sont perdues au prochain redémarrage de ROS2. Pour des valeurs persistantes, utilisez la page Réglages."
      />

      <div style={{display: "flex", flexWrap: "wrap", gap: 12, alignItems: "center", justifyContent: "space-between"}}>
        <div>
          <Segmented
            value={tier}
            onChange={(v) => setTier(v as ParamTier)}
            options={TIER_ORDER.map((t) => ({label: TIER_LABEL_FR[t] ?? TIER_LABEL[t], value: t}))}
          />
          <span style={{marginLeft: 12, fontSize: 12, color: colors.textMuted}}>
            {shownCount} paramètre{shownCount !== 1 ? "s" : ""} · application en direct
          </span>
        </div>
        <Input
          prefix={<SearchOutlined/>}
          placeholder="Rechercher un paramètre…"
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
          message={`Mode ${TIER_LABEL_FR[tier]} — réservé aux utilisateurs avertis`}
          description={
            <div>
              <p style={{marginTop: 0}}>
                Ces paramètres bas-niveau pilotent l'estimation de pose, les gains moteur, les
                seuils de sécurité et plus encore. Une mauvaise valeur peut faire bouger le robot
                de façon inattendue ou dégrader la localisation. Les contrôles restent verrouillés
                tant que vous n'avez pas confirmé.
              </p>
              <Button danger type="primary" onClick={() => setAcknowledged(true)}>
                J'ai compris les risques
              </Button>
            </div>
          }
        />
      )}

      {error && <div style={{color: colors.danger, fontSize: 13}}>{error}</div>}

      {loading ? (
        <div style={{display: "flex", justifyContent: "center", padding: 48}}><Spin/></div>
      ) : groups.length === 0 ? (
        <Empty description={search ? "Aucun paramètre ne correspond à votre recherche" : "Aucun paramètre dans ce profil"}/>
      ) : (
        <Collapse
          defaultActiveKey={groups.map(([g]) => g)}
          items={groups.map(([group, list]) => ({
            key: group,
            label: <span style={{fontWeight: 600}}>{group} <Tag style={{marginLeft: 6}}>{list.length}</Tag></span>,
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
