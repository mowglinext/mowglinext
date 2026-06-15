import {describe, it, expect} from "vitest";
import {paramMeta, paramShortName, TIER_RANK} from "./paramCatalog.ts";

describe("paramCatalog", () => {
  it("extracts the short name regardless of separator", () => {
    expect(paramShortName("fusion_graph_node.node_period_s")).toBe("node_period_s");
    expect(paramShortName("/map_server_node/tool_width")).toBe("tool_width");
    expect(paramShortName("tool_width")).toBe("tool_width");
  });

  it("resolves curated metadata by short name", () => {
    const meta = paramMeta("map_server_node.tool_width");
    expect(meta.tier).toBe("basic");
    expect(meta.group).toBe("Coverage");
    expect(meta.unit).toBe("m");
  });

  it("defaults unknown parameters to the expert tier and Other group", () => {
    const meta = paramMeta("some_node.totally_unknown_param");
    expect(meta.tier).toBe("expert");
    expect(meta.group).toBe("Other");
    expect(meta.label).toBe("totally_unknown_param");
  });

  it("orders tiers basic < middle < expert", () => {
    expect(TIER_RANK.basic).toBeLessThan(TIER_RANK.middle);
    expect(TIER_RANK.middle).toBeLessThan(TIER_RANK.expert);
  });
});
