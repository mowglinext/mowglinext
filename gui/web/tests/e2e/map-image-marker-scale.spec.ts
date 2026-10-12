import {expect, test} from "@playwright/test";

import {MOWER_APPEARANCES} from "../../src/constants/mowerAppearances.ts";

const MAX_PROJECTED_ERROR_PX = 3;

for (const mowerId of ["biltema-rm1000", "generic"] as const) {
    const mowerAlt = mowerId === "generic" ? "Generic mower test image" : "RM1000 mower test image";
    const appearance = MOWER_APPEARANCES[mowerId].mowerImage!;
    test(`${mowerId}: mower and dock images retain their map pose across bearing, pitch, and heading`, async ({page}) => {
        await page.route("https://api.mapbox.com/**", (route) => route.fulfill({status: 200, contentType: "application/json", body: "{}"}));
        await page.goto(`/tests/e2e/fixtures/map-image-marker.html?mower=${mowerId}`);
        await page.waitForFunction(() => Boolean(window.mapImageMarkerTest));
        const image = page.getByAltText(mowerAlt);
        const dockImage = page.locator("img[alt='RM1000 dock test image']");
        const dockForegroundImage = page.locator(".mapboxgl-marker img[style*='clip-path']");
        await expect(image).toBeVisible();
        await expect(dockImage).toBeVisible();
        await expect(dockForegroundImage).toBeVisible();
        await expect(dockForegroundImage).toHaveCSS("clip-path", "polygon(43% 78%, 57% 78%, 68% 83%, 68% 92%, 59% 96%, 41% 96%, 32% 92%, 32% 83%)");
        const dockZIndex = await dockImage.evaluate((element) => Number(element.closest(".mapboxgl-marker")?.style.zIndex));
        const mowerZIndex = await image.evaluate((element) => Number(element.closest(".mapboxgl-marker")?.style.zIndex));
        const dockForegroundZIndex = await dockForegroundImage.evaluate((element) => Number(element.closest(".mapboxgl-marker")?.style.zIndex));
        expect(dockZIndex).toBeLessThan(mowerZIndex);
        expect(dockForegroundZIndex).toBeGreaterThan(mowerZIndex);

        const dockAnchor = await dockImage.evaluate((element) => {
            const marker = element.closest(".mapboxgl-marker")!;
            const imageLeft = Number.parseFloat((element as HTMLImageElement).style.left);
            const imageTop = Number.parseFloat((element as HTMLImageElement).style.top);
            const width = Number.parseFloat(marker.style.width);
            const height = Number.parseFloat(marker.style.height);
            return {x: (imageLeft + width * 0.5) / width, y: (imageTop + height * 0.16) / height};
        });
        expect(dockAnchor.x).toBeCloseTo(0.5);
        expect(dockAnchor.y).toBeCloseTo(0.5);

        const results = await page.evaluate(async ({appearance, mowerAlt}) => {
            const testHarness = window.mapImageMarkerTest!;
            const map = testHarness.map;
            const imageElement = Array.from(document.querySelectorAll<HTMLImageElement>("img")).find((image) => image.alt === mowerAlt)!;
            const sourceAnchor = appearance.poseAnchor;
            const imageLengthM = appearance.visibleLengthM / appearance.visibleLengthFraction;
            const imageWidthM = appearance.visibleWidthM && appearance.visibleWidthFraction
                ? appearance.visibleWidthM / appearance.visibleWidthFraction : imageLengthM;
            const earthRadiusM = 6_378_137;
            const displayOffsetM = appearance.forwardOffsetM ?? 0;
            const center = map.getCenter();
            const states = [
                {bearing: 0, pitch: 0, heading: 0, zoom: 24},
                {bearing: 90, pitch: 0, heading: Math.PI / 2, zoom: 24},
                {bearing: 0, pitch: 50, heading: 0, zoom: 19},
                {bearing: 90, pitch: 50, heading: Math.PI, zoom: 19},
            ];

            const allCorners: Array<{bearing: number; pitch: number; heading: number; error: number}> = [];
            for (const state of states) {
                map.jumpTo({bearing: state.bearing, pitch: state.pitch, zoom: state.zoom});
                await new Promise<void>((resolve) => requestAnimationFrame(() => requestAnimationFrame(() => resolve())));
                testHarness.setHeading(state.heading);
                await new Promise<void>((resolve) => requestAnimationFrame(() => requestAnimationFrame(() => resolve())));
                const renderedBox = imageElement.getBoundingClientRect();
                const mowerCenter = {
                    lng: center.lng + (displayOffsetM * Math.cos(state.heading) / (earthRadiusM * Math.cos(center.lat * Math.PI / 180))) * 180 / Math.PI,
                    lat: center.lat + (displayOffsetM * Math.sin(state.heading) / earthRadiusM) * 180 / Math.PI,
                };
                const source = [
                    {x: 0, y: 0}, {x: 1, y: 0}, {x: 1, y: 1}, {x: 0, y: 1},
                ];
                const predicted = source.map(({x, y}) => {
                    const eastM = (x - sourceAnchor.x) * imageWidthM;
                    const northM = (sourceAnchor.y - y) * imageLengthM;
                    const eastHeadingM = northM * Math.cos(state.heading) + eastM * Math.sin(state.heading);
                    const northHeadingM = northM * Math.sin(state.heading) - eastM * Math.cos(state.heading);
                    const lon = mowerCenter.lng + (eastHeadingM / (earthRadiusM * Math.cos(mowerCenter.lat * Math.PI / 180))) * 180 / Math.PI;
                    const lat = mowerCenter.lat + (northHeadingM / earthRadiusM) * 180 / Math.PI;
                    const point = map.project([lon, lat]);
                    const canvas = map.getCanvas().getBoundingClientRect();
                    return {x: canvas.left + point.x, y: canvas.top + point.y};
                });
                const expectedBox = {
                    left: Math.min(...predicted.map(({x}) => x)),
                    right: Math.max(...predicted.map(({x}) => x)),
                    top: Math.min(...predicted.map(({y}) => y)),
                    bottom: Math.max(...predicted.map(({y}) => y)),
                };
                allCorners.push(...[
                    Math.abs(renderedBox.left - expectedBox.left),
                    Math.abs(renderedBox.right - expectedBox.right),
                    Math.abs(renderedBox.top - expectedBox.top),
                    Math.abs(renderedBox.bottom - expectedBox.bottom),
                ].map((error) => ({
                    bearing: state.bearing,
                    pitch: state.pitch,
                    heading: state.heading,
                    error,
                })));
            }

            return allCorners;
        }, {appearance, mowerAlt});

        for (const point of results) {
            // Small tolerance covers pixel rounding and the local lat/lon approximation
            // used to create sub-meter reference points.
            expect(point.error, `bearing=${point.bearing}, pitch=${point.pitch}, heading=${point.heading}`)
                .toBeLessThan(MAX_PROJECTED_ERROR_PX);
        }

        await page.evaluate(() => {
            window.mapImageMarkerTest!.setHeading(0);
            window.mapImageMarkerTest!.map.jumpTo({bearing: 0, pitch: 0, zoom: 25});
        });
        await page.evaluate(() => new Promise<void>((resolve) => requestAnimationFrame(() => requestAnimationFrame(() => resolve()))));
        await page.screenshot({path: `tests/e2e/.artifacts/docked-${mowerId}-and-station.png`, animations: "disabled"});

        for (const appearanceId of ["generic", "biltema-rm1000"] as const) {
            await page.evaluate((id) => window.mapImageMarkerTest!.setDockAppearance(id), appearanceId);
            await expect(dockImage).toBeVisible();
            const layerZIndexes = await page.evaluate((mowerAlt) => [
                ...Array.from(document.querySelectorAll("img[alt='RM1000 dock test image']")),
                Array.from(document.querySelectorAll<HTMLImageElement>(".mapboxgl-marker img")).find((image) => image.alt === mowerAlt)!,
                document.querySelector(".mapboxgl-marker img[style*='clip-path']")!,
            ].map((element) => Number(element.closest(".mapboxgl-marker")?.style.zIndex)), mowerAlt);
            expect(layerZIndexes[0]).toBeLessThan(layerZIndexes[1]);
            expect(layerZIndexes[1]).toBeLessThan(layerZIndexes[2]);
        }
    });
}
