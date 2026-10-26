import {test, expect} from '@playwright/test';
import {readFile, writeFile} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';

test('component-only boundaries retain state and shared contexts across real Fast Refresh', async ({page}, testInfo) => {
    test.skip(process.env.RUN_REACT_REFRESH_MUTATION !== '1', 'Explicit opt-in: temporarily appends a comment to five component modules and restores their exact bytes.');
    const errors: string[] = [];
    const consoleMessages: {type: string; text: string}[] = [];
    page.on('pageerror', e => errors.push(e.message));
    page.on('console', m => {
        consoleMessages.push({type: m.type(), text: m.text()});
        if (m.type() === 'warning' || m.type() === 'error') console.log(`[browser ${m.type()}] ${m.text()}`);
        if (/Could not Fast Refresh|does not provide an export|Invalid hook call/.test(m.text())) errors.push(m.text());
    });
    await page.route('**/api/system/updater/state', route => route.fulfill({json: {state: {notices: []}, agent: {}}}));
    await page.goto('/tests/e2e/fixtures/react-refresh-exports.html');
    await page.getByRole('button', {name: 'Increment', exact: true}).click();
    await page.getByRole('button', {name: 'Push notice', exact: true}).click();
    await page.getByRole('button', {name: 'Use UTC', exact: true}).click();
    const modules = [
        'src/hooks/NotificationCenterProvider.tsx',
        'src/hooks/TimeFormatProvider.tsx',
        'src/components/BTStateGraph.tsx',
        'src/components/settings/ObstaclesSection.tsx',
        'src/components/settings/UpdateChangelog.tsx',
    ];
    for (const module of modules) {
        const path = fileURLToPath(new URL(`../../${module}`, import.meta.url));
        const original = await readFile(path);
        const root = page.locator('#root');
        const updateCount = async () => Number(await root.getAttribute('data-refresh-updates') ?? 0);
        const waitForUpdate = async (previous: number) => {
            await expect.poll(updateCount).toBeGreaterThan(previous);
            await expect(root).toHaveAttribute('data-refresh-paths', new RegExp(module.replaceAll('.', '\\.')));
        };
        try {
            const before = await updateCount();
            await writeFile(path, Buffer.concat([original, Buffer.from('\n// Fast Refresh boundary characterization\n')]));
            await waitForUpdate(before);
            await expect(page.getByTestId('local-count')).toHaveText('1');
            await expect(page.getByTestId('notice-count')).toHaveText('1');
            await expect(page.getByTestId('zone')).toHaveText('utc');
        } finally {
            const before = await updateCount().catch(() => undefined);
            await writeFile(path, original);
            if (before !== undefined && !page.isClosed()) await waitForUpdate(before);
        }
    }
    await expect(page.getByTestId('local-count')).toHaveText('1');
    await expect(page.getByTestId('notice-count')).toHaveText('1');
    await expect(page.getByTestId('zone')).toHaveText('utc');
    await testInfo.attach('browser-console', {body: JSON.stringify(consoleMessages, null, 2), contentType: 'application/json'});
    expect(errors).toEqual([]);
});
