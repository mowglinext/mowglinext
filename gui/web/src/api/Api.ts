/* eslint-disable */
/* tslint:disable */
// @ts-nocheck
/*
 * ---------------------------------------------------------------
 * ## THIS FILE WAS GENERATED VIA SWAGGER-TYPESCRIPT-API        ##
 * ##                                                           ##
 * ## AUTHOR: acacode                                           ##
 * ## SOURCE: https://github.com/acacode/swagger-typescript-api ##
 * ---------------------------------------------------------------
 */

export enum ProvidersRemoteAccessPhase {
  RemoteAccessDisabled = "disabled",
  RemoteAccessPulling = "pulling",
  RemoteAccessStarting = "starting",
  RemoteAccessRunning = "running",
  RemoteAccessError = "error",
}

export interface ApiBlackboxStatusResponse {
  buffered_bytes?: number;
  buffered_records?: number;
  buffered_seconds?: number;
  capture_id?: string;
  coalesced_triggers?: number;
  completed_snapshots?: number;
  config?: BlackboxConfig;
  dropped_messages?: number;
  effective_memory_bytes?: number;
  effective_pre_seconds?: number;
  history_evictions?: number;
  last_error?: string;
  last_write_seconds?: number;
  memory_pressure?: boolean;
  phase?: string;
  recordings?: BlackboxSnapshot[];
  skipped_trigger_sources?: number;
  topics?: ApiBlackboxTopicStatus[];
  warning?: string;
}

export interface ApiContainer {
  id?: string;
  labels?: Record<string, string>;
  names?: string[];
  state?: string;
}

export interface ApiContainerListResponse {
  containers?: ApiContainer[];
}

export interface ApiErrorResponse {
  error?: string;
}

export interface ApiFleetCoordinationResponse {
  settings?: ProvidersCoordinatorSettings;
  status?: ProvidersCoordinatorStatus;
}

export interface ApiGetConfigResponse {
  tileUri?: string;
}

export interface ApiGetSettingsResponse {
  settings?: Record<string, any>;
}

export interface ApiHardwareBackendResponse {
  backend?: string;
  /**
   * DefaultOverrides are the settings whose default this backend replaces
   * (config/backends/<backend>.yaml). A mower-model preset must not write
   * these: the preset describes the machine, not its electronics.
   */
  default_overrides?: Record<string, any>;
  /**
   * RobotName is robot_name from the installed config (or its default):
   * the header badge shows it beside the backend.
   */
  robot_name?: string;
  supported?: string[];
}

export interface ApiImportDockPose {
  x?: number;
  y?: number;
  yaw_rad?: number;
}

export interface ApiImportOpenMowerRequest {
  apply?: boolean;
  map?: number[];
  om_datum_lat?: number;
  om_datum_lon?: number;
}

export interface ApiImportOpenMowerSummary {
  applied?: boolean;
  areas?: ApiImportedAreaBrief[];
  datum_shift_east_m?: number;
  datum_shift_north_m?: number;
  dock_pose?: ApiImportDockPose;
  /**
   * MnDatumConfigured is true when mowgli_robot.yaml already has a
   * datum_lat/datum_lon set. The GUI uses this to decide whether to
   * offer the "import the datum too" step: on a fresh install (false)
   * the OpenMower datum can be adopted as the robot's datum; when a
   * datum already exists (true) the geometry was reprojected INTO it,
   * so overwriting it would misalign the just-imported map — the GUI
   * then hides the datum-import option.
   */
  mn_datum_configured?: boolean;
  mowing_areas?: number;
  navigation_areas?: number;
  obstacles?: number;
  orphan_obstacles?: number;
  warnings?: string[];
}

export interface ApiImportedAreaBrief {
  approx_area_sqm?: number;
  is_navigation_area?: boolean;
  name?: string;
  obstacles?: number;
  /** "mow" | "nav" */
  type?: string;
  vertices?: number;
}

export interface ApiInstalledComponent {
  architecture?: string;
  built_at?: string;
  component?: string;
  digests?: string[];
  image?: string;
  image_id?: string;
  metadata_available?: boolean;
  name?: string;
  revision?: string;
  state?: string;
  version?: string;
}

export interface ApiIrriSenseErrorResponse {
  code?: string;
  error?: string;
}

export interface ApiIrriSenseGardensResponse {
  gardens?: ProvidersIrriSenseGardenSummary[];
}

export interface ApiIrriSenseSettingsResponse {
  baseUrl?: string;
  dryAfterWateringHours?: number;
  enabled?: boolean;
  gardenId?: string;
  gateScheduler?: boolean;
  maxStaleMinutes?: number;
  tokenMasked?: string;
  tokenSet?: boolean;
  wetDeficitMm?: number;
  zoneIds?: string[];
}

export interface ApiIrriSenseSettingsUpdate {
  baseUrl?: string;
  clearToken?: boolean;
  dryAfterWateringHours?: number;
  enabled?: boolean;
  gardenId?: string;
  gateScheduler?: boolean;
  maxStaleMinutes?: number;
  token?: string;
  wetDeficitMm?: number;
  zoneIds?: string[];
}

export interface ApiNotificationSettingsResponse {
  channel?: string;
  channels?: string[];
  enabled?: boolean;
  eventKinds?: string[];
  events?: Record<string, boolean>;
  language?: string;
  ntfyServer?: string;
  ntfyTokenMasked?: string;
  ntfyTokenSet?: boolean;
  ntfyTopic?: string;
  pushoverAppTokenMasked?: string;
  pushoverAppTokenSet?: boolean;
  pushoverUserKey?: string;
  telegramBotTokenMasked?: string;
  telegramBotTokenSet?: boolean;
  telegramChatId?: string;
  title?: string;
  webhookUrl?: string;
}

export interface ApiNotificationSettingsUpdate {
  channel?: string;
  clearNtfyToken?: boolean;
  clearPushoverAppToken?: boolean;
  clearTelegramBotToken?: boolean;
  enabled?: boolean;
  events?: Record<string, boolean>;
  language?: string;
  ntfyServer?: string;
  ntfyToken?: string;
  ntfyTopic?: string;
  pushoverAppToken?: string;
  pushoverUserKey?: string;
  telegramBotToken?: string;
  telegramChatId?: string;
  title?: string;
  webhookUrl?: string;
}

export interface ApiOkResponse {
  ok?: string;
}

export interface ApiRemoteAccessSettingsResponse {
  authKeyMasked?: string;
  authKeySet?: boolean;
  containerName?: string;
  defaultImage?: string;
  enabled?: boolean;
  hostname?: string;
  image?: string;
  serveHttps?: boolean;
}

export interface ApiRemoteAccessSettingsUpdate {
  authKey?: string;
  clearAuthKey?: boolean;
  enabled?: boolean;
  hostname?: string;
  image?: string;
  serveHttps?: boolean;
}

export interface ApiSchedule {
  /**
   * AreaID is the STABLE map area id (MapArea.id) this schedule mows; 0 means
   * every area (a plain Start). The scheduler resolves it to the current
   * positional index when the schedule fires. AreaName is a display snapshot
   * so the GUI and MQTT consumers can label it, even if the area was removed.
   */
  areaId?: number;
  areaName?: string;
  createdAt?: string;
  /** 0=Sunday .. 6=Saturday */
  daysOfWeek?: number[];
  enabled?: boolean;
  id?: string;
  lastRun?: string;
  /**
   * Written by the scheduler when a due run was skipped (soil wet); the GUI
   * shows them, the API only preserves them across updates.
   */
  lastSkipReason?: string;
  lastSkippedAt?: string;
  /** HH:mm format */
  time?: string;
}

export interface ApiScheduleListResponse {
  schedules?: ApiSchedule[];
}

export interface ApiSettingsStatusResponse {
  onboarding_completed?: boolean;
}

export interface ApiSystemInfo {
  cpuTemperature?: number;
}

export interface ApiUpdateCheck {
  channel?: string;
  checked_at?: string;
  components?: ApiUpdateComponent[];
  last_successful_at?: string;
  notes_url?: string;
  state?: string;
  version?: string;
}

export interface ApiUpdateComponent {
  available_image?: string;
  available_revision?: string;
  custom_image?: boolean;
  digest_reference?: boolean;
  installed_revision?: string;
  name?: string;
  source_relation?: string;
  state?: string;
}

export interface ApiVersionsResponse {
  components?: ApiInstalledComponent[];
  docker_available?: boolean;
  observed_at?: string;
  server?: BuildinfoInfo;
}

export interface ApiAddPeerRequest {
  address?: string;
}

export interface ApiBlackboxTopicStatus {
  last_received_at?: string;
  topic?: string;
}

export interface ApiRegisterPeerRequest {
  api_version?: number;
  id?: string;
  name?: string;
  port?: number;
}

export interface BlackboxConfig {
  cooldown_seconds?: number;
  enabled?: boolean;
  max_disk_bytes?: number;
  max_message_bytes?: number;
  max_snapshots?: number;
  memory_bytes?: number;
  min_free_disk_bytes?: number;
  post_seconds?: number;
  pre_seconds?: number;
}

export interface BlackboxSnapshot {
  actual_post_seconds?: number;
  actual_pre_seconds?: number;
  capture_dropped_messages?: number;
  capture_id?: string;
  config?: BlackboxConfig;
  dropped_messages?: number;
  format_version?: number;
  identifiers?: Record<string, string>;
  interrupted?: boolean;
  kind?: string;
  name?: string;
  reasons?: string[];
  records?: number;
  size?: number;
  triggered_at?: string;
  window_elapsed_seconds?: number;
}

export interface BlackboxStatus {
  buffered_bytes?: number;
  buffered_records?: number;
  buffered_seconds?: number;
  capture_id?: string;
  coalesced_triggers?: number;
  completed_snapshots?: number;
  config?: BlackboxConfig;
  dropped_messages?: number;
  effective_memory_bytes?: number;
  effective_pre_seconds?: number;
  history_evictions?: number;
  last_error?: string;
  last_write_seconds?: number;
  memory_pressure?: boolean;
  phase?: string;
  skipped_trigger_sources?: number;
}

export interface BuildinfoInfo {
  built_at?: string;
  modified?: boolean;
  revision?: string;
  version?: string;
}

export interface GeometryPoint {
  x?: number;
  y?: number;
  z?: number;
}

export interface GeometryPoint32 {
  x?: number;
  y?: number;
  z?: number;
}

export interface GeometryPolygon {
  points?: GeometryPoint32[];
}

export interface GeometryPose {
  orientation?: GeometryQuaternion;
  position?: GeometryPoint;
}

export interface GeometryQuaternion {
  w?: number;
  x?: number;
  y?: number;
  z?: number;
}

export interface MowgliAddMowingAreaReq {
  area?: MowgliMapArea;
  is_navigation_area?: boolean;
}

export interface MowgliMapArea {
  area?: GeometryPolygon;
  has_mow_angle?: boolean;
  has_ring_direction?: boolean;
  has_start_point?: boolean;
  id?: number;
  is_navigation_area?: boolean;
  mow_angle_deg?: number;
  name?: string;
  obstacle_info?: MowgliMapObstacleInfo[];
  obstacles?: GeometryPolygon[];
  proposed_obstacle_info?: MowgliMapObstacleInfo[];
  proposed_obstacles?: GeometryPolygon[];
  ring_direction?: number;
  start_x?: number;
  start_y?: number;
}

export interface MowgliMapObstacleInfo {
  id?: number;
  name?: string;
  pending?: boolean;
  source?: number;
}

export interface MowgliReplaceMapArea {
  area?: MowgliMapArea;
  is_navigation_area?: boolean;
}

export interface MowgliReplaceMapReq {
  areas?: MowgliReplaceMapArea[];
}

export interface MowgliSetDockingPointReq {
  docking_pose?: GeometryPose;
  preserve_position?: boolean;
  use_gps_position?: boolean;
  use_pending_antenna?: boolean;
  yaw_rad?: number;
  yaw_source?: number;
}

export interface ProvidersAddPeerResult {
  peer?: ProvidersFleetPeer;
  reciprocal?: boolean;
  warning?: string;
}

export interface ProvidersCoordinatorSettings {
  completed_ttl_h?: number;
  enabled?: boolean;
  resume_distance_m?: number;
  yield_distance_m?: number;
}

export interface ProvidersCoordinatorStatus {
  completed_areas?: number[];
  enabled?: boolean;
  excluded_areas?: number[];
  last_error?: string;
  last_push_at?: string;
  preferred_start?: number;
  yielded?: boolean;
}

export interface ProvidersFleetPeer {
  /** host:port of the peer's GUI backend */
  address?: string;
  api_version?: number;
  id?: string;
  name?: string;
}

export interface ProvidersFleetRobot {
  address?: string;
  identity?: ProvidersRobotIdentity;
  last_seen?: string;
  online?: boolean;
  self?: boolean;
  topics?: Record<string, number[]>;
}

export interface ProvidersIrriSenseGardenSummary {
  id?: string;
  name?: string;
  zones?: ProvidersIrriSenseZoneSummary[];
}

export interface ProvidersIrriSenseZoneSummary {
  enabled?: boolean;
  id?: string;
  label?: string;
}

export interface ProvidersMapPushPeerResult {
  address?: string;
  error?: string;
  id?: string;
  name?: string;
  ok?: boolean;
}

export interface ProvidersMapPushResult {
  areas?: number;
  peers?: ProvidersMapPushPeerResult[];
}

export interface ProvidersNotifyDeliveryStatus {
  channel?: string;
  configured?: boolean;
  enabled?: boolean;
  failedCount?: number;
  lastError?: string;
  lastErrorAt?: string;
  lastMessage?: string;
  lastSentAt?: string;
  sentCount?: number;
}

export interface ProvidersRemoteAccessStatus {
  /**
   * BackendState mirrors tailscaled: NoState, NeedsLogin, NeedsMachineAuth,
   * Stopped, Starting, Running.
   */
  backendState?: string;
  checkedAt?: string;
  /** ContainerState is Docker's state string, or "absent". */
  containerState?: string;
  dnsName?: string;
  enabled?: boolean;
  error?: string;
  health?: string[];
  hostname?: string;
  httpUrls?: string[];
  httpsUrl?: string;
  /** LoginUrl is set while the node waits for an interactive login. */
  loginUrl?: string;
  magicDnsEnabled?: boolean;
  phase?: ProvidersRemoteAccessPhase;
  tailscaleIps?: string[];
  version?: string;
}

export interface ProvidersRobotIdentity {
  api_version?: number;
  datum_lat?: number;
  datum_lon?: number;
  id?: string;
  name?: string;
  revision?: string;
  version?: string;
}

export interface TypesFirmwareAvailability {
  available?: boolean;
  board?: string;
  fw_version?: string;
  own_release?: boolean;
  panel?: string;
  protocol_version?: number;
  /**
   * Release the manifest came from; OwnRelease is false when this
   * installation's release carries no firmware and the latest stable one
   * was used instead.
   */
  release?: string;
}

export interface TypesFirmwareConfig {
  batChargeCutoffVoltage?: number;
  boardType?: string;
  /**
   * Firmware selection provenance is written alongside the saved config so
   * later mower-model changes can update only fields that still follow model
   * defaults. Empty/unknown values are legacy and are handled conservatively
   * by the GUI.
   */
  boardTypeOrigin?: string;
  bothWheelsLiftEmergencyMillis?: number;
  branch?: string;
  directory?: string;
  disableEmergency?: boolean;
  /**
   * ExpertBuild routes the flash to the compile-from-source path
   * (flashMowgli); the default (false) flashes a prebuilt binary. Kept for
   * backward compatibility — FirmwareSource == "custom" implies it.
   */
  expertBuild?: boolean;
  externalImuAcceleration?: boolean;
  externalImuAngular?: boolean;
  file?: string;
  firmwareSelectionModel?: string;
  /**
   * FirmwareSource is the GUI dropdown selector: "custom" compiles from
   * source (the expert path), "prebuilt" (or empty, for older payloads)
   * flashes the tested prebuilt binary.
   */
  firmwareSource?: string;
  /**
   * FirmwareTarget is an exact PlatformIO/release-manifest environment for
   * boards with multiple firmware variants. Empty preserves legacy routing.
   */
  firmwareTarget?: string;
  firmwareTargetOrigin?: string;
  imuOnboardInclinationThreshold?: number;
  limitVoltage150MA?: number;
  masterJ18?: boolean;
  maxChargeCurrent?: number;
  maxChargeVoltage?: number;
  maxMps?: number;
  oneWheelLiftEmergencyMillis?: number;
  panelType?: string;
  panelTypeOrigin?: string;
  perimeterWire?: boolean;
  playButtonClearEmergencyMillis?: number;
  repository?: string;
  stopButtonEmergencyMillis?: number;
  tickPerM?: number;
  tiltEmergencyMillis?: number;
  version?: string;
  wheelBase?: number;
}

export interface TypesSoilStatus {
  configured?: boolean;
  enabled?: boolean;
  error?: string;
  fetchedAt?: string;
  fresh?: boolean;
  gardenName?: string;
  gateScheduler?: boolean;
  reason?: string;
  unknown?: boolean;
  wet?: boolean;
  zones?: TypesSoilZoneStatus[];
}

export interface TypesSoilZoneStatus {
  deficitMm?: number;
  enabled?: boolean;
  id?: string;
  label?: string;
  lastWateredAt?: string;
  reason?: string;
  selected?: boolean;
  wet?: boolean;
}

export interface UpdatesChangelog {
  features?: UpdatesChangelogEntry[];
  fixes?: UpdatesChangelogEntry[];
  /** Commits that are neither a feature nor a fix (ci, docs, test, chore, ...). */
  other?: number;
  total?: number;
  /**
   * The installed revision was not found in the first page of history, so
   * the lists cover the most recent changes only.
   */
  truncated?: boolean;
  url?: string;
}

export interface UpdatesChangelogEntry {
  breaking?: boolean;
  pr?: number;
  scope?: string;
  title?: string;
}

export type QueryParamsType = Record<string | number, any>;
export type ResponseFormat = keyof Omit<Body, "body" | "bodyUsed">;

export interface FullRequestParams extends Omit<RequestInit, "body"> {
  /** set parameter to `true` for call `securityWorker` for this request */
  secure?: boolean;
  /** request path */
  path: string;
  /** content type of request body */
  type?: ContentType;
  /** query params */
  query?: QueryParamsType;
  /** format of response (i.e. response.json() -> format: "json") */
  format?: ResponseFormat;
  /** request body */
  body?: unknown;
  /** base url */
  baseUrl?: string;
  /** request cancellation token */
  cancelToken?: CancelToken;
}

export type RequestParams = Omit<
  FullRequestParams,
  "body" | "method" | "query" | "path"
>;

export interface ApiConfig<SecurityDataType = unknown> {
  baseUrl?: string;
  baseApiParams?: Omit<RequestParams, "baseUrl" | "cancelToken" | "signal">;
  securityWorker?: (
    securityData: SecurityDataType | null,
  ) => Promise<RequestParams | void> | RequestParams | void;
  customFetch?: typeof fetch;
}

export interface HttpResponse<D extends unknown, E extends unknown = unknown>
  extends Response {
  data: D;
  error: E;
}

type CancelToken = Symbol | string | number;

export enum ContentType {
  Json = "application/json",
  JsonApi = "application/vnd.api+json",
  FormData = "multipart/form-data",
  UrlEncoded = "application/x-www-form-urlencoded",
  Text = "text/plain",
}

export class HttpClient<SecurityDataType = unknown> {
  public baseUrl: string = "//localhost:4200/api";
  private securityData: SecurityDataType | null = null;
  private securityWorker?: ApiConfig<SecurityDataType>["securityWorker"];
  private abortControllers = new Map<CancelToken, AbortController>();
  private customFetch = (...fetchParams: Parameters<typeof fetch>) =>
    fetch(...fetchParams);

  private baseApiParams: RequestParams = {
    credentials: "same-origin",
    headers: {},
    redirect: "follow",
    referrerPolicy: "no-referrer",
  };

  constructor(apiConfig: ApiConfig<SecurityDataType> = {}) {
    Object.assign(this, apiConfig);
  }

  public setSecurityData = (data: SecurityDataType | null) => {
    this.securityData = data;
  };

  protected encodeQueryParam(key: string, value: any) {
    const encodedKey = encodeURIComponent(key);
    return `${encodedKey}=${encodeURIComponent(typeof value === "number" ? value : `${value}`)}`;
  }

  protected addQueryParam(query: QueryParamsType, key: string) {
    return this.encodeQueryParam(key, query[key]);
  }

  protected addArrayQueryParam(query: QueryParamsType, key: string) {
    const value = query[key];
    return value.map((v: any) => this.encodeQueryParam(key, v)).join("&");
  }

  protected toQueryString(rawQuery?: QueryParamsType): string {
    const query = rawQuery || {};
    const keys = Object.keys(query).filter(
      (key) => "undefined" !== typeof query[key],
    );
    return keys
      .map((key) =>
        Array.isArray(query[key])
          ? this.addArrayQueryParam(query, key)
          : this.addQueryParam(query, key),
      )
      .join("&");
  }

  protected addQueryParams(rawQuery?: QueryParamsType): string {
    const queryString = this.toQueryString(rawQuery);
    return queryString ? `?${queryString}` : "";
  }

  private contentFormatters: Record<ContentType, (input: any) => any> = {
    [ContentType.Json]: (input: any) =>
      input !== null && (typeof input === "object" || typeof input === "string")
        ? JSON.stringify(input)
        : input,
    [ContentType.JsonApi]: (input: any) =>
      input !== null && (typeof input === "object" || typeof input === "string")
        ? JSON.stringify(input)
        : input,
    [ContentType.Text]: (input: any) =>
      input !== null && typeof input !== "string"
        ? JSON.stringify(input)
        : input,
    [ContentType.FormData]: (input: any) => {
      if (input instanceof FormData) {
        return input;
      }

      return Object.keys(input || {}).reduce((formData, key) => {
        const property = input[key];
        formData.append(
          key,
          property instanceof Blob
            ? property
            : typeof property === "object" && property !== null
              ? JSON.stringify(property)
              : `${property}`,
        );
        return formData;
      }, new FormData());
    },
    [ContentType.UrlEncoded]: (input: any) => this.toQueryString(input),
  };

  protected mergeRequestParams(
    params1: RequestParams,
    params2?: RequestParams,
  ): RequestParams {
    return {
      ...this.baseApiParams,
      ...params1,
      ...(params2 || {}),
      headers: {
        ...(this.baseApiParams.headers || {}),
        ...(params1.headers || {}),
        ...((params2 && params2.headers) || {}),
      },
    };
  }

  protected createAbortSignal = (
    cancelToken: CancelToken,
  ): AbortSignal | undefined => {
    if (this.abortControllers.has(cancelToken)) {
      const abortController = this.abortControllers.get(cancelToken);
      if (abortController) {
        return abortController.signal;
      }
      return void 0;
    }

    const abortController = new AbortController();
    this.abortControllers.set(cancelToken, abortController);
    return abortController.signal;
  };

  public abortRequest = (cancelToken: CancelToken) => {
    const abortController = this.abortControllers.get(cancelToken);

    if (abortController) {
      abortController.abort();
      this.abortControllers.delete(cancelToken);
    }
  };

  public request = async <T = any, E = any>({
    body,
    secure,
    path,
    type,
    query,
    format,
    baseUrl,
    cancelToken,
    ...params
  }: FullRequestParams): Promise<HttpResponse<T, E>> => {
    const secureParams =
      ((typeof secure === "boolean" ? secure : this.baseApiParams.secure) &&
        this.securityWorker &&
        (await this.securityWorker(this.securityData))) ||
      {};
    const requestParams = this.mergeRequestParams(params, secureParams);
    const queryString = query && this.toQueryString(query);
    const payloadFormatter = this.contentFormatters[type || ContentType.Json];
    const responseFormat = format || requestParams.format;

    return this.customFetch(
      `${baseUrl || this.baseUrl || ""}${path}${queryString ? `?${queryString}` : ""}`,
      {
        ...requestParams,
        headers: {
          ...(requestParams.headers || {}),
          ...(type && type !== ContentType.FormData
            ? { "Content-Type": type }
            : {}),
        },
        signal:
          (cancelToken
            ? this.createAbortSignal(cancelToken)
            : requestParams.signal) || null,
        body:
          typeof body === "undefined" || body === null
            ? null
            : payloadFormatter(body),
      },
    ).then(async (response) => {
      const r = response as HttpResponse<T, E>;
      r.data = null as unknown as T;
      r.error = null as unknown as E;

      const responseToParse = responseFormat ? response.clone() : response;
      const data = !responseFormat
        ? r
        : await responseToParse[responseFormat]()
            .then((data) => {
              if (r.ok) {
                r.data = data;
              } else {
                r.error = data;
              }
              return r;
            })
            .catch((e) => {
              r.error = e;
              return r;
            });

      if (cancelToken) {
        this.abortControllers.delete(cancelToken);
      }

      if (!response.ok) throw data;
      return data;
    });
  };
}

/**
 * @title MowgliNext GUI API
 * @version 1.0
 * @baseUrl //localhost:4200/api
 * @contact
 *
 * API for the MowgliNext autonomous robot mower GUI
 */
export class Api<
  SecurityDataType extends unknown,
> extends HttpClient<SecurityDataType> {
  config = {
    /**
     * @description get config env from backend
     *
     * @tags config
     * @name EnvsList
     * @summary get config env from backend
     * @request GET:/config/envs
     */
    envsList: (params: RequestParams = {}) =>
      this.request<ApiGetConfigResponse, ApiErrorResponse>({
        path: `/config/envs`,
        method: "GET",
        format: "json",
        ...params,
      }),

    /**
     * @description get config from backend
     *
     * @tags config
     * @name KeysGetCreate
     * @summary get config from backend
     * @request POST:/config/keys/get
     */
    keysGetCreate: (
      settings: Record<string, string>,
      params: RequestParams = {},
    ) =>
      this.request<Record<string, string>, ApiErrorResponse>({
        path: `/config/keys/get`,
        method: "POST",
        body: settings,
        type: ContentType.Json,
        format: "json",
        ...params,
      }),

    /**
     * @description set config to backend
     *
     * @tags config
     * @name KeysSetCreate
     * @summary set config to backend
     * @request POST:/config/keys/set
     */
    keysSetCreate: (
      settings: Record<string, string>,
      params: RequestParams = {},
    ) =>
      this.request<Record<string, string>, ApiErrorResponse>({
        path: `/config/keys/set`,
        method: "POST",
        body: settings,
        type: ContentType.Json,
        format: "json",
        ...params,
      }),
  };
  containers = {
    /**
     * @description list all containers
     *
     * @tags containers
     * @name ContainersList
     * @summary list all containers
     * @request GET:/containers
     */
    containersList: (params: RequestParams = {}) =>
      this.request<ApiContainerListResponse, ApiErrorResponse>({
        path: `/containers`,
        method: "GET",
        format: "json",
        ...params,
      }),

    /**
     * @description get container logs
     *
     * @tags containers
     * @name LogsList
     * @summary get container logs
     * @request GET:/containers/{containerId}/logs
     */
    logsList: (containerId: string, params: RequestParams = {}) =>
      this.request<any, any>({
        path: `/containers/${containerId}/logs`,
        method: "GET",
        ...params,
      }),

    /**
     * @description execute a command on a container
     *
     * @tags containers
     * @name ContainersCreate
     * @summary execute a command on a container
     * @request POST:/containers/{containerId}/{command}
     */
    containersCreate: (
      containerId: string,
      command: string,
      params: RequestParams = {},
    ) =>
      this.request<ApiOkResponse, ApiErrorResponse>({
        path: `/containers/${containerId}/${command}`,
        method: "POST",
        format: "json",
        ...params,
      }),
  };
  fleet = {
    /**
     * No description
     *
     * @tags fleet
     * @name CoordinationList
     * @summary coordinated mowing settings and status
     * @request GET:/fleet/coordination
     */
    coordinationList: (params: RequestParams = {}) =>
      this.request<ApiFleetCoordinationResponse, any>({
        path: `/fleet/coordination`,
        method: "GET",
        format: "json",
        ...params,
      }),

    /**
     * No description
     *
     * @tags fleet
     * @name CoordinationUpdate
     * @summary update coordinated mowing settings
     * @request PUT:/fleet/coordination
     */
    coordinationUpdate: (
      body: ProvidersCoordinatorSettings,
      params: RequestParams = {},
    ) =>
      this.request<ApiFleetCoordinationResponse, ApiErrorResponse>({
        path: `/fleet/coordination`,
        method: "PUT",
        body: body,
        type: ContentType.Json,
        format: "json",
        ...params,
      }),

    /**
     * No description
     *
     * @tags fleet
     * @name CoordinationResetCreate
     * @summary fleet-wide start fresh
     * @request POST:/fleet/coordination/reset
     */
    coordinationResetCreate: (params: RequestParams = {}) =>
      this.request<ApiOkResponse, ApiErrorResponse>({
        path: `/fleet/coordination/reset`,
        method: "POST",
        format: "json",
        ...params,
      }),

    /**
     * No description
     *
     * @tags fleet
     * @name IdentityList
     * @summary this robot's fleet identity
     * @request GET:/fleet/identity
     */
    identityList: (params: RequestParams = {}) =>
      this.request<ProvidersRobotIdentity, ApiErrorResponse>({
        path: `/fleet/identity`,
        method: "GET",
        format: "json",
        ...params,
      }),

    /**
     * No description
     *
     * @tags fleet
     * @name MapPushCreate
     * @summary push this robot's map to the fleet
     * @request POST:/fleet/map/push
     */
    mapPushCreate: (params: RequestParams = {}) =>
      this.request<ProvidersMapPushResult, ApiErrorResponse>({
        path: `/fleet/map/push`,
        method: "POST",
        format: "json",
        ...params,
      }),

    /**
     * No description
     *
     * @tags fleet
     * @name PeersCreate
     * @summary add a fleet peer
     * @request POST:/fleet/peers
     */
    peersCreate: (body: ApiAddPeerRequest, params: RequestParams = {}) =>
      this.request<ProvidersAddPeerResult, ApiErrorResponse>({
        path: `/fleet/peers`,
        method: "POST",
        body: body,
        type: ContentType.Json,
        format: "json",
        ...params,
      }),

    /**
     * No description
     *
     * @tags fleet
     * @name PeersRegisterCreate
     * @summary reverse-register a peer
     * @request POST:/fleet/peers/register
     */
    peersRegisterCreate: (
      body: ApiRegisterPeerRequest,
      params: RequestParams = {},
    ) =>
      this.request<ProvidersFleetPeer, ApiErrorResponse>({
        path: `/fleet/peers/register`,
        method: "POST",
        body: body,
        type: ContentType.Json,
        format: "json",
        ...params,
      }),

    /**
     * No description
     *
     * @tags fleet
     * @name PeersUnregisterCreate
     * @summary a peer asks to be forgotten
     * @request POST:/fleet/peers/unregister
     */
    peersUnregisterCreate: (params: RequestParams = {}) =>
      this.request<ApiOkResponse, any>({
        path: `/fleet/peers/unregister`,
        method: "POST",
        type: ContentType.Json,
        format: "json",
        ...params,
      }),

    /**
     * No description
     *
     * @tags fleet
     * @name PeersDelete
     * @summary remove a fleet peer
     * @request DELETE:/fleet/peers/{id}
     */
    peersDelete: (id: string, params: RequestParams = {}) =>
      this.request<ApiOkResponse, ApiErrorResponse>({
        path: `/fleet/peers/${id}`,
        method: "DELETE",
        format: "json",
        ...params,
      }),

    /**
     * No description
     *
     * @tags fleet
     * @name RobotsList
     * @summary fleet snapshot
     * @request GET:/fleet/robots
     */
    robotsList: (params: RequestParams = {}) =>
      this.request<ProvidersFleetRobot[], ApiErrorResponse>({
        path: `/fleet/robots`,
        method: "GET",
        format: "json",
        ...params,
      }),

    /**
     * No description
     *
     * @tags fleet
     * @name RobotsCallCreate
     * @summary send a command to a fleet robot
     * @request POST:/fleet/robots/{id}/call/{command}
     */
    robotsCallCreate: (
      id: string,
      command: string,
      params: RequestParams = {},
    ) =>
      this.request<ApiOkResponse, ApiErrorResponse>({
        path: `/fleet/robots/${id}/call/${command}`,
        method: "POST",
        type: ContentType.Json,
        format: "json",
        ...params,
      }),
  };
  import = {
    /**
     * @description Parse a user-supplied OpenMower map.json, translate it into MowgliNext's coordinate frame, and return a summary for confirmation. Setting `apply=true` runs the live write path (areas + dock pose). See docs/IMPORT_OPENMOWER_MAP.md.
     *
     * @tags import
     * @name OpenmowerCreate
     * @summary import an OpenMower map.json (preview-only by default)
     * @request POST:/import/openmower
     */
    openmowerCreate: (
      body: ApiImportOpenMowerRequest,
      params: RequestParams = {},
    ) =>
      this.request<ApiImportOpenMowerSummary, ApiErrorResponse>({
        path: `/import/openmower`,
        method: "POST",
        body: body,
        type: ContentType.Json,
        format: "json",
        ...params,
      }),
  };
  irrisense = {
    /**
     * @description gardens readable by the stored token, for the picker and "test connection"
     *
     * @tags irrisense
     * @name GardensList
     * @summary list IrriSense gardens
     * @request GET:/irrisense/gardens
     */
    gardensList: (params: RequestParams = {}) =>
      this.request<ApiIrriSenseGardensResponse, ApiIrriSenseErrorResponse>({
        path: `/irrisense/gardens`,
        method: "GET",
        format: "json",
        ...params,
      }),

    /**
     * No description
     *
     * @tags irrisense
     * @name SettingsList
     * @summary IrriSense settings
     * @request GET:/irrisense/settings
     */
    settingsList: (params: RequestParams = {}) =>
      this.request<ApiIrriSenseSettingsResponse, any>({
        path: `/irrisense/settings`,
        method: "GET",
        format: "json",
        ...params,
      }),

    /**
     * No description
     *
     * @tags irrisense
     * @name SettingsUpdate
     * @summary update IrriSense settings
     * @request PUT:/irrisense/settings
     */
    settingsUpdate: (
      settings: ApiIrriSenseSettingsUpdate,
      params: RequestParams = {},
    ) =>
      this.request<ApiIrriSenseSettingsResponse, ApiErrorResponse>({
        path: `/irrisense/settings`,
        method: "PUT",
        body: settings,
        type: ContentType.Json,
        format: "json",
        ...params,
      }),

    /**
     * @description cached wet/dry/unknown verdict the scheduler gate reads
     *
     * @tags irrisense
     * @name StatusList
     * @summary IrriSense soil status
     * @request GET:/irrisense/status
     */
    statusList: (params: RequestParams = {}) =>
      this.request<TypesSoilStatus, any>({
        path: `/irrisense/status`,
        method: "GET",
        format: "json",
        ...params,
      }),
  };
  mowglinext = {
    /**
     * @description call a service
     *
     * @tags mowglinext
     * @name CallCreate
     * @summary call a service
     * @request POST:/mowglinext/call/{command}
     */
    callCreate: (
      command: string,
      CallReq: Record<string, any>,
      params: RequestParams = {},
    ) =>
      this.request<ApiOkResponse, ApiErrorResponse>({
        path: `/mowglinext/call/${command}`,
        method: "POST",
        body: CallReq,
        type: ContentType.Json,
        format: "json",
        ...params,
      }),

    /**
     * @description clear the map and insert all provided areas in a single transaction
     *
     * @tags mowglinext
     * @name PutMowglinext
     * @summary Delete the current map and replace all areas
     * @request PUT:/mowglinext/map
     */
    putMowglinext: (CallReq: MowgliReplaceMapReq, params: RequestParams = {}) =>
      this.request<ApiOkResponse, ApiErrorResponse>({
        path: `/mowglinext/map`,
        method: "PUT",
        body: CallReq,
        type: ContentType.Json,
        format: "json",
        ...params,
      }),

    /**
     * @description clear the map
     *
     * @tags mowglinext
     * @name DeleteMowglinext
     * @summary clear the map
     * @request DELETE:/mowglinext/map
     */
    deleteMowglinext: (params: RequestParams = {}) =>
      this.request<ApiOkResponse, ApiErrorResponse>({
        path: `/mowglinext/map`,
        method: "DELETE",
        type: ContentType.Json,
        format: "json",
        ...params,
      }),

    /**
     * @description add a map area
     *
     * @tags mowglinext
     * @name MapAreaAddCreate
     * @summary add a map area
     * @request POST:/mowglinext/map/area/add
     */
    mapAreaAddCreate: (
      CallReq: MowgliAddMowingAreaReq,
      params: RequestParams = {},
    ) =>
      this.request<ApiOkResponse, ApiErrorResponse>({
        path: `/mowglinext/map/area/add`,
        method: "POST",
        body: CallReq,
        type: ContentType.Json,
        format: "json",
        ...params,
      }),

    /**
     * @description set the docking point
     *
     * @tags mowglinext
     * @name MapDockingCreate
     * @summary set the docking point
     * @request POST:/mowglinext/map/docking
     */
    mapDockingCreate: (
      CallReq: MowgliSetDockingPointReq,
      params: RequestParams = {},
    ) =>
      this.request<ApiOkResponse, ApiErrorResponse>({
        path: `/mowglinext/map/docking`,
        method: "POST",
        body: CallReq,
        type: ContentType.Json,
        format: "json",
        ...params,
      }),

    /**
     * @description multiplexed topic subscription
     *
     * @tags mowglinext
     * @name MultiplexList
     * @summary multiplexed topic subscription
     * @request GET:/mowglinext/multiplex
     */
    multiplexList: (params: RequestParams = {}) =>
      this.request<any, any>({
        path: `/mowglinext/multiplex`,
        method: "GET",
        ...params,
      }),

    /**
     * @description publish to a topic
     *
     * @tags mowglinext
     * @name PublishDetail
     * @summary publish to a topic
     * @request GET:/mowglinext/publish/{topic}
     */
    publishDetail: (topic: string, params: RequestParams = {}) =>
      this.request<any, any>({
        path: `/mowglinext/publish/${topic}`,
        method: "GET",
        ...params,
      }),

    /**
     * @description subscribe to a topic
     *
     * @tags mowglinext
     * @name SubscribeDetail
     * @summary subscribe to a topic
     * @request GET:/mowglinext/subscribe/{topic}
     */
    subscribeDetail: (topic: string, params: RequestParams = {}) =>
      this.request<any, any>({
        path: `/mowglinext/subscribe/${topic}`,
        method: "GET",
        ...params,
      }),
  };
  notifications = {
    /**
     * No description
     *
     * @tags notifications
     * @name SettingsList
     * @summary notification settings
     * @request GET:/notifications/settings
     */
    settingsList: (params: RequestParams = {}) =>
      this.request<ApiNotificationSettingsResponse, any>({
        path: `/notifications/settings`,
        method: "GET",
        format: "json",
        ...params,
      }),

    /**
     * No description
     *
     * @tags notifications
     * @name SettingsUpdate
     * @summary update notification settings
     * @request PUT:/notifications/settings
     */
    settingsUpdate: (
      settings: ApiNotificationSettingsUpdate,
      params: RequestParams = {},
    ) =>
      this.request<ApiNotificationSettingsResponse, ApiErrorResponse>({
        path: `/notifications/settings`,
        method: "PUT",
        body: settings,
        type: ContentType.Json,
        format: "json",
        ...params,
      }),

    /**
     * No description
     *
     * @tags notifications
     * @name StatusList
     * @summary notification delivery status
     * @request GET:/notifications/status
     */
    statusList: (params: RequestParams = {}) =>
      this.request<ProvidersNotifyDeliveryStatus, any>({
        path: `/notifications/status`,
        method: "GET",
        format: "json",
        ...params,
      }),

    /**
     * No description
     *
     * @tags notifications
     * @name TestCreate
     * @summary send a test notification
     * @request POST:/notifications/test
     */
    testCreate: (params: RequestParams = {}) =>
      this.request<ApiOkResponse, ApiErrorResponse>({
        path: `/notifications/test`,
        method: "POST",
        format: "json",
        ...params,
      }),
  };
  remoteAccess = {
    /**
     * No description
     *
     * @tags remote-access
     * @name ApplyCreate
     * @summary Retry applying remote access settings
     * @request POST:/remote-access/apply
     */
    applyCreate: (params: RequestParams = {}) =>
      this.request<ApiOkResponse, any>({
        path: `/remote-access/apply`,
        method: "POST",
        format: "json",
        ...params,
      }),

    /**
     * @description the node key is discarded; the sidecar restarts and logs in again (interactively or with the stored auth key)
     *
     * @tags remote-access
     * @name LogoutCreate
     * @summary Log the robot out of the tailnet
     * @request POST:/remote-access/logout
     */
    logoutCreate: (params: RequestParams = {}) =>
      this.request<ApiOkResponse, ApiErrorResponse>({
        path: `/remote-access/logout`,
        method: "POST",
        format: "json",
        ...params,
      }),

    /**
     * No description
     *
     * @tags remote-access
     * @name SettingsList
     * @summary Remote access settings
     * @request GET:/remote-access/settings
     */
    settingsList: (params: RequestParams = {}) =>
      this.request<ApiRemoteAccessSettingsResponse, any>({
        path: `/remote-access/settings`,
        method: "GET",
        format: "json",
        ...params,
      }),

    /**
     * No description
     *
     * @tags remote-access
     * @name SettingsUpdate
     * @summary update remote access settings
     * @request PUT:/remote-access/settings
     */
    settingsUpdate: (
      settings: ApiRemoteAccessSettingsUpdate,
      params: RequestParams = {},
    ) =>
      this.request<ApiRemoteAccessSettingsResponse, ApiErrorResponse>({
        path: `/remote-access/settings`,
        method: "PUT",
        body: settings,
        type: ContentType.Json,
        format: "json",
        ...params,
      }),

    /**
     * @description container phase, tailscaled login state, login URL while waiting, reachable URLs once connected
     *
     * @tags remote-access
     * @name StatusList
     * @summary Remote access status
     * @request GET:/remote-access/status
     */
    statusList: (params: RequestParams = {}) =>
      this.request<ProvidersRemoteAccessStatus, any>({
        path: `/remote-access/status`,
        method: "GET",
        format: "json",
        ...params,
      }),
  };
  schedules = {
    /**
     * @description list all mowing schedules
     *
     * @tags schedules
     * @name SchedulesList
     * @summary list all schedules
     * @request GET:/schedules
     */
    schedulesList: (params: RequestParams = {}) =>
      this.request<ApiScheduleListResponse, any>({
        path: `/schedules`,
        method: "GET",
        format: "json",
        ...params,
      }),

    /**
     * @description create a new mowing schedule
     *
     * @tags schedules
     * @name SchedulesCreate
     * @summary create a schedule
     * @request POST:/schedules
     */
    schedulesCreate: (schedule: ApiSchedule, params: RequestParams = {}) =>
      this.request<ApiSchedule, ApiErrorResponse>({
        path: `/schedules`,
        method: "POST",
        body: schedule,
        type: ContentType.Json,
        format: "json",
        ...params,
      }),

    /**
     * @description update an existing mowing schedule
     *
     * @tags schedules
     * @name SchedulesUpdate
     * @summary update a schedule
     * @request PUT:/schedules/{id}
     */
    schedulesUpdate: (
      id: string,
      schedule: ApiSchedule,
      params: RequestParams = {},
    ) =>
      this.request<ApiSchedule, ApiErrorResponse>({
        path: `/schedules/${id}`,
        method: "PUT",
        body: schedule,
        type: ContentType.Json,
        format: "json",
        ...params,
      }),

    /**
     * @description delete a mowing schedule
     *
     * @tags schedules
     * @name SchedulesDelete
     * @summary delete a schedule
     * @request DELETE:/schedules/{id}
     */
    schedulesDelete: (id: string, params: RequestParams = {}) =>
      this.request<ApiOkResponse, ApiErrorResponse>({
        path: `/schedules/${id}`,
        method: "DELETE",
        format: "json",
        ...params,
      }),
  };
  settings = {
    /**
     * @description returns a JSON object with the settings
     *
     * @tags settings
     * @name SettingsList
     * @summary returns a JSON object with the settings
     * @request GET:/settings
     */
    settingsList: (params: RequestParams = {}) =>
      this.request<ApiGetSettingsResponse, ApiErrorResponse>({
        path: `/settings`,
        method: "GET",
        format: "json",
        ...params,
      }),

    /**
     * @description saves the settings to the mower_config.sh file
     *
     * @tags settings
     * @name SettingsCreate
     * @summary saves the settings to the mower_config.sh file
     * @request POST:/settings
     */
    settingsCreate: (
      settings: Record<string, any>,
      params: RequestParams = {},
    ) =>
      this.request<ApiOkResponse, ApiErrorResponse>({
        path: `/settings`,
        method: "POST",
        body: settings,
        type: ContentType.Json,
        format: "json",
        ...params,
      }),

    /**
     * @description HARDWARE_BACKEND from the runtime env (default mowgli)
     *
     * @tags settings
     * @name HardwareBackendList
     * @summary returns the active hardware backend
     * @request GET:/settings/hardware-backend
     */
    hardwareBackendList: (params: RequestParams = {}) =>
      this.request<ApiHardwareBackendResponse, any>({
        path: `/settings/hardware-backend`,
        method: "GET",
        format: "json",
        ...params,
      }),

    /**
     * @description returns the JSON Schema for mower configuration parameters
     *
     * @tags settings
     * @name SchemaList
     * @summary returns the mower config JSON Schema
     * @request GET:/settings/schema
     */
    schemaList: (params: RequestParams = {}) =>
      this.request<Record<string, any>, ApiErrorResponse>({
        path: `/settings/schema`,
        method: "GET",
        format: "json",
        ...params,
      }),

    /**
     * @description returns whether onboarding has been completed
     *
     * @tags settings
     * @name StatusList
     * @summary get settings onboarding status
     * @request GET:/settings/status
     */
    statusList: (params: RequestParams = {}) =>
      this.request<ApiSettingsStatusResponse, any>({
        path: `/settings/status`,
        method: "GET",
        format: "json",
        ...params,
      }),

    /**
     * @description marks onboarding as completed so the wizard is not shown again
     *
     * @tags settings
     * @name StatusCreate
     * @summary mark onboarding as completed
     * @request POST:/settings/status
     */
    statusCreate: (params: RequestParams = {}) =>
      this.request<ApiOkResponse, ApiErrorResponse>({
        path: `/settings/status`,
        method: "POST",
        format: "json",
        ...params,
      }),

    /**
     * @description returns the current YAML mower configuration values as a flat key-value map
     *
     * @tags settings
     * @name YamlList
     * @summary returns the current YAML mower configuration
     * @request GET:/settings/yaml
     */
    yamlList: (params: RequestParams = {}) =>
      this.request<Record<string, any>, ApiErrorResponse>({
        path: `/settings/yaml`,
        method: "GET",
        format: "json",
        ...params,
      }),

    /**
     * @description saves the mower configuration as YAML to mowgli_robot.yaml
     *
     * @tags settings
     * @name YamlCreate
     * @summary saves the mower configuration as YAML
     * @request POST:/settings/yaml
     */
    yamlCreate: (settings: Record<string, any>, params: RequestParams = {}) =>
      this.request<ApiOkResponse, ApiErrorResponse>({
        path: `/settings/yaml`,
        method: "POST",
        body: settings,
        type: ContentType.Json,
        format: "json",
        ...params,
      }),

    /**
     * @description returns a flat key-value map of default values (the reset-to-default source)
     *
     * @tags settings
     * @name YamlDefaultsList
     * @summary returns the schema default value for every known parameter
     * @request GET:/settings/yaml/defaults
     */
    yamlDefaultsList: (params: RequestParams = {}) =>
      this.request<Record<string, any>, ApiErrorResponse>({
        path: `/settings/yaml/defaults`,
        method: "GET",
        format: "json",
        ...params,
      }),
  };
  setup = {
    /**
     * @description Version and protocol of the prebuilt firmware that /setup/flashBoard would install for the saved board selection, taken from this installation's release (or the latest stable one when it carries none).
     *
     * @tags setup
     * @name FirmwareAvailableList
     * @summary prebuilt firmware available for the saved board
     * @request GET:/setup/firmware/available
     */
    firmwareAvailableList: (params: RequestParams = {}) =>
      this.request<TypesFirmwareAvailability, ApiErrorResponse>({
        path: `/setup/firmware/available`,
        method: "GET",
        format: "json",
        ...params,
      }),

    /**
     * @description flash the mower board with the given config
     *
     * @tags setup
     * @name FlashBoardCreate
     * @summary flash the mower board with the given config
     * @request POST:/setup/flashBoard
     */
    flashBoardCreate: (
      settings: TypesFirmwareConfig,
      params: RequestParams = {},
    ) =>
      this.request<ApiOkResponse, ApiErrorResponse>({
        path: `/setup/flashBoard`,
        method: "POST",
        body: settings,
        type: ContentType.Json,
        ...params,
      }),
  };
  system = {
    /**
     * @description get system info such as CPU temperature
     *
     * @tags system
     * @name InfoList
     * @summary get system info
     * @request GET:/system/info
     */
    infoList: (params: RequestParams = {}) =>
      this.request<ApiSystemInfo, any>({
        path: `/system/info`,
        method: "GET",
        format: "json",
        ...params,
      }),

    /**
     * @description reboots the Raspberry Pi
     *
     * @tags system
     * @name RebootCreate
     * @summary reboot the system
     * @request POST:/system/reboot
     */
    rebootCreate: (params: RequestParams = {}) =>
      this.request<ApiOkResponse, ApiErrorResponse>({
        path: `/system/reboot`,
        method: "POST",
        format: "json",
        ...params,
      }),

    /**
     * @description shuts down the Raspberry Pi
     *
     * @tags system
     * @name ShutdownCreate
     * @summary shutdown the system
     * @request POST:/system/shutdown
     */
    shutdownCreate: (params: RequestParams = {}) =>
      this.request<ApiOkResponse, ApiErrorResponse>({
        path: `/system/shutdown`,
        method: "POST",
        format: "json",
        ...params,
      }),

    /**
     * No description
     *
     * @tags updates
     * @name UpdaterStateList
     * @summary Read cached host updater status
     * @request GET:/system/updater/state
     */
    updaterStateList: (params: RequestParams = {}) =>
      this.request<Record<string, any>, ApiErrorResponse>({
        path: `/system/updater/state`,
        method: "GET",
        format: "json",
        ...params,
      }),

    /**
     * @description Same-origin JSON and X-Mowgli-Update: 1 are required. See docs/UPDATES.md for the operation contract.
     *
     * @tags updates
     * @name UpdaterCreate
     * @summary Review or control a coordinated software update
     * @request POST:/system/updater/{operation}
     */
    updaterCreate: (
      operation:
        | "policy"
        | "check"
        | "plan"
        | "apply"
        | "rollback"
        | "recover"
        | "notice"
        | "agent-update",
      request: Record<string, any>,
      params: RequestParams = {},
    ) =>
      this.request<Record<string, any>, ApiErrorResponse>({
        path: `/system/updater/${operation}`,
        method: "POST",
        body: request,
        type: ContentType.Json,
        format: "json",
        ...params,
      }),

    /**
     * No description
     *
     * @tags system
     * @name UpdatesList
     * @summary Check available software versions
     * @request GET:/system/updates
     */
    updatesList: (
      query: {
        /** Comparison channel */
        channel: "stable" | "dev";
        /** Check remote metadata (otherwise cached result) */
        check?: boolean;
      },
      params: RequestParams = {},
    ) =>
      this.request<ApiUpdateCheck, any>({
        path: `/system/updates`,
        method: "GET",
        query: query,
        format: "json",
        ...params,
      }),

    /**
     * @description Features and fixes read from the commit subjects between the installed and the candidate revision.
     *
     * @tags system
     * @name UpdatesChangelogList
     * @summary What changed between two source revisions
     * @request GET:/system/updates/changelog
     */
    updatesChangelogList: (
      query: {
        /** owner/name */
        repository: string;
        /** Installed 40-hex revision */
        installed: string;
        /** Candidate 40-hex revision */
        available: string;
      },
      params: RequestParams = {},
    ) =>
      this.request<UpdatesChangelog, ApiErrorResponse>({
        path: `/system/updates/changelog`,
        method: "GET",
        query: query,
        format: "json",
        ...params,
      }),

    /**
     * No description
     *
     * @tags system
     * @name VersionsList
     * @summary Installed software versions
     * @request GET:/system/versions
     */
    versionsList: (params: RequestParams = {}) =>
      this.request<ApiVersionsResponse, any>({
        path: `/system/versions`,
        method: "GET",
        format: "json",
        ...params,
      }),
  };
  tools = {
    /**
     * No description
     *
     * @tags diagnostics
     * @name BlackboxConfigUpdate
     * @summary Configure passive blackbox (complete configuration)
     * @request PUT:/tools/blackbox/config
     */
    blackboxConfigUpdate: (
      config: BlackboxConfig,
      params: RequestParams = {},
    ) =>
      this.request<BlackboxStatus, any>({
        path: `/tools/blackbox/config`,
        method: "PUT",
        body: config,
        type: ContentType.Json,
        ...params,
      }),

    /**
     * No description
     *
     * @tags diagnostics
     * @name BlackboxDownloadDetail
     * @summary Download a completed timestamped blackbox timeline
     * @request GET:/tools/blackbox/download/{name}
     */
    blackboxDownloadDetail: (name: string, params: RequestParams = {}) =>
      this.request<File, any>({
        path: `/tools/blackbox/download/${name}`,
        method: "GET",
        ...params,
      }),

    /**
     * No description
     *
     * @tags diagnostics
     * @name BlackboxSaveCreate
     * @summary Save available pre-event history and collect post-event window
     * @request POST:/tools/blackbox/save
     */
    blackboxSaveCreate: (params: RequestParams = {}) =>
      this.request<Record<string, any>, any>({
        path: `/tools/blackbox/save`,
        method: "POST",
        ...params,
      }),

    /**
     * No description
     *
     * @tags diagnostics
     * @name BlackboxStatusList
     * @summary Passive blackbox status and completed recordings
     * @request GET:/tools/blackbox/status
     */
    blackboxStatusList: (params: RequestParams = {}) =>
      this.request<ApiBlackboxStatusResponse, any>({
        path: `/tools/blackbox/status`,
        method: "GET",
        format: "json",
        ...params,
      }),

    /**
     * No description
     *
     * @tags diagnostics
     * @name BlackboxDelete
     * @summary Delete a completed blackbox recording
     * @request DELETE:/tools/blackbox/{name}
     */
    blackboxDelete: (name: string, params: RequestParams = {}) =>
      this.request<ApiOkResponse, any>({
        path: `/tools/blackbox/${name}`,
        method: "DELETE",
        ...params,
      }),
  };
}
