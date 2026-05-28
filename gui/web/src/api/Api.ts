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

export interface ApiGetConfigResponse {
  tileUri?: string;
}

export interface ApiGetSettingsResponse {
  settings?: Record<string, any>;
}

export interface ApiOkResponse {
  ok?: string;
}

export interface ApiSchedule {
  area?: number;
  createdAt?: string;
  /** 0=Sunday .. 6=Saturday */
  daysOfWeek?: number[];
  enabled?: boolean;
  id?: string;
  lastRun?: string;
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
  is_navigation_area?: boolean;
  name?: string;
  obstacles?: GeometryPolygon[];
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
}

export interface TypesFirmwareConfig {
  batChargeCutoffVoltage?: number;
  boardType?: string;
  bothWheelsLiftEmergencyMillis?: number;
  branch?: string;
  debugType?: string;
  directory?: string;
  disableEmergency?: boolean;
  externalImuAcceleration?: boolean;
  externalImuAngular?: boolean;
  file?: string;
  imuOnboardInclinationThreshold?: number;
  limitVoltage150MA?: number;
  masterJ18?: boolean;
  maxChargeCurrent?: number;
  maxChargeVoltage?: number;
  maxMps?: number;
  oneWheelLiftEmergencyMillis?: number;
  panelType?: string;
  perimeterWire?: boolean;
  playButtonClearEmergencyMillis?: number;
  repository?: string;
  stopButtonEmergencyMillis?: number;
  tickPerM?: number;
  tiltEmergencyMillis?: number;
  version?: string;
  wheelBase?: number;
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
  };
  setup = {
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
  };
}
