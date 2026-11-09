/** Public artwork has stable filenames, while the GUI serves /assets as immutable.
 * Tie every source/mask/derivative to the web build so an update cannot reuse
 * cached pixels from a previous build. Keep one URL per asset within a build.
 */
export function artworkUrl(file: string): string {
    const build = import.meta.env.VITE_WEB_BUILD_ID as string | undefined;
    return `/assets/robots/layered/${file}?v=${encodeURIComponent(build || "development")}`;
}
