import {fireEvent, render, screen} from "@testing-library/react";
import {describe, expect, it, vi} from "vitest";
import {ThemeProvider} from "../../theme/ThemeContext.tsx";
import {ActionCluster, type ChargeHold} from "./ActionCluster.tsx";

const actions = () => ({
  onStart: vi.fn(), onPause: vi.fn(), onHome: vi.fn(), onStop: vi.fn(), onRearm: vi.fn(),
  onResume: vi.fn(), onCancelMowing: vi.fn(),
});

function renderCluster(chargeHold: ChargeHold | undefined, handlers = actions()) {
  render(<ThemeProvider><ActionCluster phase="idle" chargeHold={chargeHold} {...handlers}/></ThemeProvider>);
  return handlers;
}

describe("ActionCluster charge holds", () => {
  it("keeps ordinary dock charging as the normal idle/start control", () => {
    renderCluster(undefined);
    expect(screen.getByRole("button", {name: "Start mowing"})).toBeEnabled();
    expect(screen.queryByRole("button", {name: "Resume now"})).not.toBeInTheDocument();
    expect(screen.queryByRole("button", {name: "Cancel mowing"})).not.toBeInTheDocument();
  });

  it.each<ChargeHold>([
    {stateName: "CHARGING", batteryPercent: 55, batteryFullPercent: 92, manualResumePercent: 30, autoResume: true},
    {stateName: "CRITICAL_BATTERY_CHARGING", batteryPercent: 55, batteryFullPercent: 92, manualResumePercent: 30, autoResume: true},
  ])("shows explicit, safe controls for $stateName", (chargeHold) => {
    const handlers = renderCluster(chargeHold);
    fireEvent.click(screen.getByRole("button", {name: "Resume now"}));
    fireEvent.click(screen.getByRole("button", {name: "Cancel mowing"}));
    expect(handlers.onResume).toHaveBeenCalledOnce();
    expect(handlers.onCancelMowing).toHaveBeenCalledOnce();
    expect(screen.getByRole("button", {name: "Emergency stop"})).toBeInTheDocument();
  });

  it("reflects the configured manual-resume floor without replacing BT enforcement", () => {
    renderCluster({stateName: "CHARGING", batteryPercent: 29, batteryFullPercent: 92, manualResumePercent: 30, autoResume: true});
    expect(screen.getByRole("button", {name: "Resume now"})).toBeDisabled();
    expect(screen.getByText(/resumes automatically at 92%/i)).toBeInTheDocument();
    expect(screen.getByText(/Resume is available at 30%/)).toBeInTheDocument();
  });

  it("keeps a manual dock charge hold cancellable without a resume control", () => {
    const handlers = renderCluster({stateName: "MANUAL_CHARGING", batteryPercent: 55, batteryFullPercent: 92, manualResumePercent: 30, autoResume: false});
    expect(screen.queryByRole("button", {name: "Resume now"})).not.toBeInTheDocument();
    fireEvent.click(screen.getByRole("button", {name: "Cancel mowing"}));
    expect(handlers.onCancelMowing).toHaveBeenCalledOnce();
    expect(screen.getByText(/docked manually/i)).toBeInTheDocument();
  });
});
