import {expect, test} from '@playwright/test';
import {installMockBackend} from './mock/mockBackend';
import {SCENARIOS} from './mock/scenarios';
import {ROBOT_URDF} from '../../src/test/robotUrdf';

for (const width of [390, 1440]) {
    test(`numeric sensor units fit and accept edits at ${width}px`, async ({page}) => {
        await page.setViewportSize({width, height: 900});
        await page.addInitScript(() => localStorage.setItem('mowglinext.lang', 'en'));
        const warnings: string[] = [];
        page.on('console', message => {
            if (message.text().includes('[antd: InputNumber]') && /addonAfter|addonBefore/.test(message.text())) warnings.push(message.text());
        });
        await installMockBackend(page, {...SCENARIOS[0],
            topics: {...SCENARIOS[0].topics, robotDescription: {data: ROBOT_URDF}},
            rest: {'/api/settings/yaml': {lidar_enabled: true, lidar_x: 0, lidar_y: 0, lidar_z: .2, lidar_yaw: 0}},
        });
        await page.goto('/#/settings?section=sensors');
        const input = page.getByRole('spinbutton', {name: /LiDAR.*forward/});
        await expect(input).toHaveValue('0.000');
        const geometry = await input.evaluate(element => {
            const compact = element.closest('.ant-space-compact');
            const addon = compact?.querySelector('.ant-space-addon');
            if (!compact || !addon) throw new Error('Numeric compact group missing');
            const group = compact.getBoundingClientRect(), unit = addon.getBoundingClientRect(), numeric = (element.closest('.ant-input-number') ?? element).getBoundingClientRect();
            return {groupWidth: group.width, unitWidth: unit.width, inputWidth: numeric.width, right: unit.right, groupRight: group.right,
                height: unit.height, inputHeight: numeric.height, viewport: window.innerWidth};
        });
        expect(geometry.inputWidth).toBeGreaterThan(30); expect(geometry.unitWidth).toBeGreaterThan(0);
        expect(geometry.right).toBeLessThanOrEqual(geometry.groupRight + 1);
        expect(geometry.groupWidth).toBeLessThan(geometry.viewport);
        expect(Math.abs(geometry.height - geometry.inputHeight)).toBeLessThan(3);
        await input.fill('0.125'); await input.blur(); await expect(input).toHaveValue('0.125');
        expect(warnings).toEqual([]);
    });
}

for (const width of [390, 1440]) {
    test(`fixed/fluid long addons fit a modal and Form.Item saves at ${width}px`, async ({page}) => {
        await page.setViewportSize({width, height: 900});
        const warnings: string[] = [];
        page.on('console', message => {
            if (message.text().includes('[antd: InputNumber]') && /addonAfter|addonBefore/.test(message.text())) warnings.push(message.text());
        });
        await page.goto('/tests/e2e/fixtures/number-addons.html');
        const fixed = page.getByRole('spinbutton', {name: 'Fixed-width datum'});
        await expect(fixed).toHaveValue('0.000');
        const dialog = page.getByRole('dialog', {name: 'Numeric layout fixture'});
        // Input readiness precedes the end of Ant Design's zoom entrance.
        // Measure the final layout without disabling the animation or sleeping.
        await expect(dialog).not.toHaveClass(/ant-zoom-(appear|enter)/);
        await expect(dialog).toHaveCSS('transform', 'none');
        await expect(dialog.getByText('OpenMower latitude', {exact: true})).toBeVisible();
        await expect(dialog.getByText('PWM per metre per second', {exact: true})).toBeVisible();
        for (const name of ['Fixed-width datum', 'Fluid-width gain']) {
            const layout = await page.getByRole('spinbutton', {name}).evaluate(element => {
                const compact = element.closest('.ant-space-compact');
                const unit = compact?.querySelector('.ant-space-addon');
                const numeric = compact?.querySelector('.ant-input-number');
                if (!compact || !unit || !numeric) throw new Error('Missing numeric composition');
                const modal = element.closest('.ant-modal');
                const body = modal?.querySelector('.ant-modal-body');
                if (!modal || !body) throw new Error('Missing modal layout');
                const g = compact.getBoundingClientRect(), a = unit.getBoundingClientRect(), n = numeric.getBoundingClientRect();
                const bodyRect = body.getBoundingClientRect(), bodyStyle = getComputedStyle(body);
                return {width: g.width, right: g.right, left: g.left, viewport: window.innerWidth, inputWidth: n.width,
                    textInputWidth: element.getBoundingClientRect().width, heightDelta: Math.abs(a.height - n.height),
                    topDelta: Math.abs(a.top - n.top), unitRight: a.right, unitWidth: a.width,
                    unitScrollWidth: unit.scrollWidth, unitClientWidth: unit.clientWidth,
                    groupScrollWidth: compact.scrollWidth, groupClientWidth: compact.clientWidth,
                    bodyLeft: bodyRect.left + parseFloat(bodyStyle.paddingLeft), bodyRight: bodyRect.right - parseFloat(bodyStyle.paddingRight),
                    documentWidth: document.documentElement.clientWidth, documentScrollWidth: document.documentElement.scrollWidth,
                    modalTransform: getComputedStyle(modal).transform, entering: /ant-zoom-(appear|enter)/.test(modal.className)};
            });
            expect(layout.entering).toBe(false); expect(layout.modalTransform).toBe('none');
            expect(layout.left).toBeGreaterThanOrEqual(0); expect(layout.right).toBeLessThanOrEqual(layout.viewport);
            expect(layout.left).toBeGreaterThanOrEqual(layout.bodyLeft); expect(layout.right).toBeLessThanOrEqual(layout.bodyRight + 1);
            expect(layout.documentScrollWidth).toBeLessThanOrEqual(layout.documentWidth);
            expect(layout.groupScrollWidth).toBeLessThanOrEqual(layout.groupClientWidth);
            expect(layout.unitWidth).toBeGreaterThan(0); expect(layout.unitScrollWidth).toBeLessThanOrEqual(layout.unitClientWidth);
            expect(layout.textInputWidth).toBeGreaterThan(30); expect(layout.topDelta).toBeLessThan(2);
            expect(layout.inputWidth).toBeGreaterThan(30); expect(layout.heightDelta).toBeLessThan(2);
            expect(layout.unitRight).toBeLessThanOrEqual(layout.right + 1);
            if (name === 'Fixed-width datum') expect(layout.width).toBeCloseTo(280, 0);
        }
        await fixed.fill('2.5'); await fixed.blur(); await page.getByRole('button', {name: 'Save fixture'}).click();
        await expect(page.locator('output')).toHaveText('{"fixed":2.5,"fluid":1}');
        expect(warnings).toEqual([]);
    });
}
