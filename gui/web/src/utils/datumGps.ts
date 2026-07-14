import { Api } from "../api/Api.ts";

type GuiApi = Api<unknown>;

/**
 * Error thrown when the set_datum service replied but its message could not
 * be parsed as "lat,lon". Callers should surface a translated fallback toast
 * instead of failing silently.
 */
export class DatumParseError extends Error {
    constructor(public readonly rawMessage: string) {
        super(`Unparseable set_datum response: ${rawMessage}`);
        this.name = "DatumParseError";
    }
}

/**
 * Call the ROS set_datum service and parse the "lat,lon" reply. Shared by the
 * onboarding Datum step and the Settings Positioning section. Throws on API
 * errors and throws DatumParseError when the reply message is unparseable.
 */
export const requestDatumFromGps = async (
    guiApi: GuiApi,
): Promise<{ lat: number; lon: number }> => {
    const res = await guiApi.mowglinext.callCreate("set_datum", {});
    if (res.error) throw new Error((res.error as any).error);
    const msg: string = (res.data as any)?.message ?? "";
    const parts = msg.split(",");
    if (parts.length === 2) {
        const lat = parseFloat(parts[0]);
        const lon = parseFloat(parts[1]);
        if (Number.isFinite(lat) && Number.isFinite(lon)) {
            return { lat, lon };
        }
    }
    throw new DatumParseError(msg);
};
