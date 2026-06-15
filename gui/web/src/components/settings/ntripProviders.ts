// Curated NTRIP correction providers. Selecting one pre-fills the caster host,
// port and (for free networks) the public credentials, so the operator only has
// to pick a station — no manual credential entry. Stations + coordinates are
// fetched live from each caster's sourcetable (GET /api/ntrip/sourcetable).

export interface NtripProvider {
  id: string;
  name: string;
  host: string;            // "" = the user must type a caster host
  port: number;
  defaultUser?: string;
  defaultPassword?: string;
  userIsEmail?: boolean;   // RTK2go: the username is your email address
  requiresOwnCreds: boolean; // SAPOS/paid: no shared login, user supplies it
  color: string;           // map-marker colour (per-provider aggregation)
  region: string;
  note?: string;
}

export const NTRIP_PROVIDERS: NtripProvider[] = [
  {
    id: "centipede",
    name: "Centipede",
    host: "crtk.net",
    port: 2101,
    defaultUser: "centipede",
    defaultPassword: "centipede",
    requiresOwnCreds: false,
    color: "#7CFFB2",
    region: "France / Europe",
    note: "Free community RTK network. Public login — just pick the nearest base.",
  },
  {
    id: "rtk2go",
    name: "RTK2go",
    host: "rtk2go.com",
    port: 2101,
    userIsEmail: true,
    defaultPassword: "none",
    requiresOwnCreds: false,
    color: "#5AB8FF",
    region: "Worldwide",
    note: "Free community casters worldwide. Use your email as the username.",
  },
  {
    id: "sapos",
    name: "SAPOS (DE)",
    host: "",
    port: 2101,
    requiresOwnCreds: true,
    color: "#F3A85C",
    region: "Germany",
    note: "Paid national service — enter your state's caster host and credentials.",
  },
  {
    id: "custom",
    name: "Custom caster",
    host: "",
    port: 2101,
    requiresOwnCreds: false,
    color: "#C9A8FF",
    region: "Any",
    note: "Enter any NTRIP caster host, port and (optional) login.",
  },
];

export const NTRIP_PROVIDER_BY_ID: Record<string, NtripProvider> = Object.fromEntries(
  NTRIP_PROVIDERS.map((p) => [p.id, p]),
);

// Providers whose sourcetable can be fetched without operator-supplied creds —
// these populate the "all free networks" aggregated station map.
export const PUBLIC_SOURCETABLE_PROVIDERS = NTRIP_PROVIDERS.filter(
  (p) => p.host !== "" && !p.requiresOwnCreds,
);

// Shared Mapbox token (same public token the main map uses).
export const MAPBOX_TOKEN =
  (import.meta.env.VITE_MAPBOX_TOKEN as string | undefined) ||
  "pk.eyJ1IjoiY2VkYm9zc25lbyIsImEiOiJjbGxldjB4aDEwOW5vM3BxamkxeWRwb2VoIn0.WOccbQZZyO1qfAgNxnHAnA";

// Infer which provider a host belongs to (so a saved config re-selects it).
export function providerForHost(host?: string): NtripProvider | undefined {
  if (!host) return undefined;
  return NTRIP_PROVIDERS.find((p) => p.host && p.host === host);
}
