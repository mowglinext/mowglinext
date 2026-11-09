import { act, cleanup, render, screen } from "@testing-library/react";
import { MemoryRouter } from "react-router-dom";
import { afterEach, describe, expect, it, vi } from "vitest";
import { HealthCard } from "./MowgliNextPage.tsx";
import i18n from "../i18n";

vi.mock("../hooks/useWeather.ts", () => ({
    useWeather: () => null,
}));

const data = {
    gps: 100,
    gpsLabel: "RTK Fixed",
    rain: false,
    emergency: false,
    motorTemp: 25,
    firmwareCompatible: false,
    firmwareVersion: "",
    settings: {},
    docked: false,
} as Parameters<typeof HealthCard>[0]["data"];

describe("dashboard firmware inventory by hardware backend", () => {
    it("keeps the existing Mowgli incompatible state and flash action", () => {
        render(
            <MemoryRouter>
                <HealthCard
                    data={data}
                    hardware={{
                        presetsAvailable: true, authority: "confirmed", backend: "mowgli",
                        defaultOverrides: {},
                        parameterRoutes: {},
                        runtimeRouting: "available",
                        robotName: "",
                        loading: false,
                    }}
                    mavros={{
                        state: { connected: false },
                        vehicle: {
                            available_info: 0,
                            autopilot: 0,
                            flight_sw_version: 0,
                            board_version: 0,
                        },
                    }}
                />
            </MemoryRouter>
        );

        expect(
            screen.getByText("Firmware incompatible")
        ).toBeInTheDocument();

        expect(
            screen.getByRole("button", { name: "Flash firmware" })
        ).toBeInTheDocument();
    });

    it("never treats unset STM32 fields as a MAVROS incompatibility", () => {
        render(
            <MemoryRouter>
                <HealthCard
                    data={data}
                    hardware={{
                        presetsAvailable: true, authority: "confirmed", backend: "mavros",
                        defaultOverrides: {},
                        parameterRoutes: {},
                        runtimeRouting: "pending_image",
                        robotName: "",
                        loading: false,
                    }}
                    mavros={{
                        state: { connected: true },
                        vehicle: {
                            available_info: 3,
                            autopilot: 12,
                            flight_sw_version: 0x010203ff,
                            board_version: 0x01000001,
                        },
                    }}
                />
            </MemoryRouter>
        );

        expect(
            screen.queryByText("Firmware incompatible")
        ).not.toBeInTheDocument();

        expect(
            screen.queryByRole("button", { name: "Flash firmware" })
        ).not.toBeInTheDocument();

        expect(
            screen.getByText("FCU connected")
        ).toBeInTheDocument();

        expect(
            screen.getByText(/PX4 · 1.2.3/)
        ).toBeInTheDocument();

        expect(
            screen.getByText(/HW 0x01000001/)
        ).toBeInTheDocument();
    });
});

type HealthProps = Parameters<typeof HealthCard>[0];
const hardware: HealthProps['hardware'] = {
    presetsAvailable: true, authority: 'confirmed', backend: 'mowgli',
    defaultOverrides: {}, parameterRoutes: {}, runtimeRouting: 'available',
    robotName: '', loading: false,
};
const mavros: HealthProps['mavros'] = {
    state: {connected: true},
    vehicle: {available_info: 3, autopilot: 12, flight_sw_version: 0x010203ff, board_version: 0x01000001},
};
const card = (props: HealthProps) => <MemoryRouter><HealthCard {...props}/></MemoryRouter>;
const rows = (container: HTMLElement) => Array.from(container.querySelectorAll<HTMLElement>('.health-card-body > div:first-child > div'));
const expectSameRows = (actual: HTMLElement[], expected: HTMLElement[]) => {
    expect(actual).toHaveLength(expected.length);
    actual.forEach((row, index) => expect(row).toBe(expected[index]));
};
const duplicateKeyCalls = (calls: unknown[][]) => calls.filter(args => args.some(value =>
    typeof value === 'string' && value.includes('Encountered two children with the same key')));

afterEach(async () => {
    cleanup();
    vi.restoreAllMocks();
    await i18n.changeLanguage('en');
});

describe('health row identity', () => {
    it('keeps distinct unknown rain and emergency rows through state, temperature and language changes', async () => {
        // Observe without replacing console.error: every React warning remains visible.
        const errors = vi.spyOn(console, 'error');
        let props: HealthProps = {hardware, mavros, data: {...data, rainKnown: false, emergencyKnown: false}};
        const view = render(card(props));
        const original = rows(view.container);
        expect(original).toHaveLength(5);
        expect(original[1]).not.toBe(original[2]);
        expect(original[1]).toHaveTextContent(i18n.t('telemetry.statusUnknown'));
        expect(original[2]).toHaveTextContent(i18n.t('telemetry.statusUnknown'));
        view.rerender(card(props));
        expect(duplicateKeyCalls(errors.mock.calls)).toEqual([]);
        expectSameRows(rows(view.container), original);

        props = {...props, data: {...props.data, rainKnown: true, rain: true,
            emergencyKnown: true, emergency: true, motorTemp: 60}};
        view.rerender(card(props));
        expectSameRows(rows(view.container), original);
        expect(original[1]).toHaveTextContent(i18n.t('mowgliNextPage.rainDetectedRow'));
        expect(original[2]).toHaveTextContent(i18n.t('mowgliNextPage.alertActive'));
        expect(original[3]).toHaveTextContent(i18n.t('mowgliNextPage.motorTemp', {temp: '60'}));

        await act(async () => {await i18n.changeLanguage('fr');});
        expectSameRows(rows(view.container), original);
        expect(original[1]).toHaveTextContent(i18n.t('mowgliNextPage.rainDetectedRow'));
        expect(original[2]).toHaveTextContent(i18n.t('mowgliNextPage.alertActive'));
        props = {...props, data: {...props.data, rainKnown: false, rain: false,
            emergencyKnown: false, emergency: false, motorTemp: 25}};
        view.rerender(card(props));
        expectSameRows(rows(view.container), original);
        expect(original[1]).toHaveTextContent(i18n.t('telemetry.statusUnknown'));
        expect(original[2]).toHaveTextContent(i18n.t('telemetry.statusUnknown'));
        expect(duplicateKeyCalls(errors.mock.calls)).toEqual([]);
    });

    it('keeps the native firmware row when a preceding optional motor row disappears or its verdict changes', () => {
        const errors = vi.spyOn(console, 'error');
        let props: HealthProps = {hardware, mavros, data: {...data, rainKnown: true, emergencyKnown: true}};
        const view = render(card(props));
        const original = rows(view.container);
        props = {...props, data: {...props.data, motorTemp: NaN, firmwareCompatible: true}};
        view.rerender(card(props));
        expectSameRows(rows(view.container), [original[0], original[1], original[2], original[4]]);
        expect(original[4]).toHaveTextContent(i18n.t('mowgliNextPage.firmwareOk'));
        expect(screen.queryByRole('button', {name: i18n.t('mowgliNextPage.firmwareFlashCta')})).not.toBeInTheDocument();
        props = {...props, data: {...props.data, motorTemp: 30, firmwareCompatible: null}};
        view.rerender(card(props));
        expect(rows(view.container)).toHaveLength(4);
        expectSameRows(rows(view.container).slice(0, 3), original.slice(0, 3));
        expect(original[4]).not.toBeInTheDocument();
        expect(duplicateKeyCalls(errors.mock.calls)).toEqual([]);
    });

    it('keeps MAVROS connection and board identities across connection, optional motor and language changes', async () => {
        const errors = vi.spyOn(console, 'error');
        let props: HealthProps = {hardware: {...hardware, backend: 'mavros'}, mavros,
            data: {...data, rainKnown: true, emergencyKnown: true}};
        const view = render(card(props));
        const original = rows(view.container);
        expect(original).toHaveLength(6);
        props = {...props, mavros: {...mavros, state: {connected: false}}, data: {...props.data, motorTemp: NaN}};
        view.rerender(card(props));
        const expected = [original[0], original[1], original[2], original[4], original[5]];
        expectSameRows(rows(view.container), expected);
        expect(original[4]).toHaveTextContent(i18n.t('mowgliNextPage.mavrosFcuDisconnected'));
        expect(original[5]).toHaveTextContent('HW 0x01000001');
        await act(async () => {await i18n.changeLanguage('fr');});
        expectSameRows(rows(view.container), expected);
        expect(original[4]).toHaveTextContent(i18n.t('mowgliNextPage.mavrosFcuDisconnected'));
        expect(duplicateKeyCalls(errors.mock.calls)).toEqual([]);
    });
});
