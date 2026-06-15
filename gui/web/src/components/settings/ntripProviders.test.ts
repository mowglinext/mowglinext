import {describe, it, expect} from "vitest";
import {NTRIP_PROVIDERS, NTRIP_PROVIDER_BY_ID, PUBLIC_SOURCETABLE_PROVIDERS, providerForHost} from "./ntripProviders.ts";

describe("ntripProviders", () => {
  it("seeds the expected providers", () => {
    const ids = NTRIP_PROVIDERS.map((p) => p.id);
    expect(ids).toEqual(expect.arrayContaining(["centipede", "rtk2go", "sapos", "custom"]));
  });

  it("Centipede ships public credentials so no manual entry is needed", () => {
    const c = NTRIP_PROVIDER_BY_ID["centipede"];
    expect(c.host).toBe("crtk.net");
    expect(c.defaultUser).toBe("centipede");
    expect(c.requiresOwnCreds).toBe(false);
  });

  it("only public, host-bound providers feed the aggregated 'all free networks' map", () => {
    const ids = PUBLIC_SOURCETABLE_PROVIDERS.map((p) => p.id);
    expect(ids).toContain("centipede");
    expect(ids).toContain("rtk2go");
    expect(ids).not.toContain("sapos"); // paid, requires own creds
    expect(ids).not.toContain("custom"); // no fixed host
  });

  it("resolves a saved host back to its provider", () => {
    expect(providerForHost("crtk.net")?.id).toBe("centipede");
    expect(providerForHost("unknown.example")).toBeUndefined();
  });
});
