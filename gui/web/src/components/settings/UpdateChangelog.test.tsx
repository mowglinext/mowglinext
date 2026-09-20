import {render, screen, waitFor} from "@testing-library/react";
import {afterEach, describe, expect, it, vi} from "vitest";
import {UpdateChangelog, changelogAvailable} from "./UpdateChangelog.tsx";

const installed = "a".repeat(40);
const available = "b".repeat(40);

afterEach(() => vi.unstubAllGlobals());

describe("update changelog", () => {
    it("only applies to two distinct real revisions", () => {
        expect(changelogAvailable(installed, available)).toBe(true);
        expect(changelogAvailable(installed, installed)).toBe(false);
        expect(changelogAvailable(undefined, available)).toBe(false);
        expect(changelogAvailable("dev", available)).toBe(false);
    });

    it("lists features and fixes with their pull requests instead of image digests", async () => {
        const fetchMock = vi.fn().mockResolvedValue({ok: true, json: () => Promise.resolve({
            features: [{scope: "dig", title: "operator-selectable sensitivity", pr: 673}],
            fixes: [{title: "parse stdout only", pr: 677, breaking: true}],
            other: 2, total: 4, truncated: false, url: "https://github.com/owner/repo/compare/x...y",
        })});
        vi.stubGlobal("fetch", fetchMock);
        render(<UpdateChangelog repository="owner/repo" installed={installed} available={available}/>);
        await waitFor(() => expect(screen.getByText("operator-selectable sensitivity")).toBeInTheDocument());
        expect(screen.getByText("New features")).toBeInTheDocument();
        expect(screen.getByText("Fixes")).toBeInTheDocument();
        expect(screen.getByText("Breaking")).toBeInTheDocument();
        expect(screen.getByRole("link", {name: "#673"})).toHaveAttribute("href", "https://github.com/owner/repo/pull/673");
        expect(screen.getByText(/2 maintenance changes/)).toBeInTheDocument();
        expect(String(fetchMock.mock.calls[0][0])).toContain(`installed=${installed}&available=${available}`);
    });

    it("says so when the summary cannot be loaded, and renders nothing without a source range", async () => {
        vi.stubGlobal("fetch", vi.fn().mockResolvedValue({ok: false, status: 502}));
        const {container, rerender} = render(<UpdateChangelog repository="owner/repo" installed={installed} available={available}/>);
        await waitFor(() => expect(screen.getByText(/could not be loaded/)).toBeInTheDocument());
        rerender(<UpdateChangelog repository="owner/repo" installed={undefined} available={available}/>);
        expect(container).toBeEmptyDOMElement();
    });
});
