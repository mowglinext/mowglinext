import { act, renderHook } from "@testing-library/react";
import { beforeEach, describe, expect, it, vi } from "vitest";
import { useResetMowingProgress } from "./useResetMowingProgress.tsx";

const mocks = vi.hoisted(() => ({
  callCreate: vi.fn(),
  confirm: vi.fn<(options: {onOk: () => Promise<void>}) => void>(),
  success: vi.fn(),
  error: vi.fn(),
}));

vi.mock("../../../hooks/useApi.ts", () => ({
  useApi: () => ({
    mowglinext: { callCreate: mocks.callCreate },
  }),
}));

vi.mock("antd", () => ({
  App: {
    useApp: () => ({
      modal: { confirm: mocks.confirm },
      notification: { success: mocks.success, error: mocks.error },
    }),
  },
}));

vi.mock("react-i18next", () => ({
  useTranslation: () => ({ t: (key: string) => key }),
}));

describe("useResetMowingProgress", () => {
  beforeEach(() => {
    vi.clearAllMocks();
    mocks.callCreate.mockResolvedValue({ data: {}, error: undefined });
  });

  it("asks for confirmation before clearing progress", () => {
    const { result } = renderHook(() => useResetMowingProgress());

    act(() => result.current());

    expect(mocks.confirm).toHaveBeenCalledOnce();
    expect(mocks.callCreate).not.toHaveBeenCalled();
    expect(mocks.confirm.mock.calls[0][0]).toMatchObject({
      title: "resetMowingProgress.confirmTitle",
      okText: "resetMowingProgress.confirmAction",
      cancelText: "resetMowingProgress.cancel",
      okType: "danger",
    });
  });

  it("clears resume data without starting a mowing session", async () => {
    const { result } = renderHook(() => useResetMowingProgress());
    act(() => result.current());

    await act(async () => {
      await mocks.confirm.mock.calls[0][0].onOk();
    });

    expect(mocks.callCreate).toHaveBeenCalledOnce();
    expect(mocks.callCreate).toHaveBeenCalledWith("coverage_clear_resume", {});
    expect(mocks.callCreate).not.toHaveBeenCalledWith(
      "high_level_control",
      expect.anything(),
    );
    expect(mocks.success).toHaveBeenCalledWith({
      message: "resetMowingProgress.success",
    });
  });

  it("shows the backend error and keeps the confirmation retryable", async () => {
    mocks.callCreate.mockResolvedValue({
      data: undefined,
      error: { error: "coverage service unavailable" },
    });
    const { result } = renderHook(() => useResetMowingProgress());
    act(() => result.current());

    await expect(mocks.confirm.mock.calls[0][0].onOk()).rejects.toThrow(
      "coverage service unavailable",
    );
    expect(mocks.error).toHaveBeenCalledWith({
      message: "resetMowingProgress.error",
      description: "coverage service unavailable",
    });
    expect(mocks.success).not.toHaveBeenCalled();
  });
});
