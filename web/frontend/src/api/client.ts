// Same-origin by default: the Vite dev server proxies /api -> :8000 (vite.config.ts)
// and nginx does the same in the container (web/frontend/nginx.conf). Override with
// VITE_API_URL only when the backend lives on a different host.
const API: string =
  (import.meta as unknown as { env?: Record<string, string> }).env?.VITE_API_URL ?? "";

export class ApiError extends Error {
  status: number;
  constructor(status: number, detail: string) {
    super(detail);
    this.status = status;
  }
}

function authHeaders(): Record<string, string> {
  const token = localStorage.getItem("token");
  return token ? { Authorization: `Bearer ${token}` } : {};
}

async function req<T>(method: string, path: string, body?: unknown): Promise<T> {
  const res = await fetch(API + path, {
    method,
    headers: { "Content-Type": "application/json", ...authHeaders() },
    body: body === undefined ? undefined : JSON.stringify(body),
  });
  if (res.status === 204) return undefined as T;
  const data = await res.json().catch(() => ({}));
  if (!res.ok) throw new ApiError(res.status, (data as { detail?: string }).detail ?? res.statusText);
  return data as T;
}

export interface Run {
  id: number; title: string; routeId: number; description: string;
  date: string; duration: number; length: number; status: string;
  sourceFile: string; createdAt: string; updatedAt: string;
}

export interface Route {
  id: number; name: string; startPoint: string; endPoint: string;
  stops: string[]; polyline: number[][]; startHeight: number;
  endHeight: number; totalAscent: number; createdAt: string; updatedAt: string;
}

export interface Frame {
  t: number;
  u: number;
  v_model: number;
  v_gnss: number | null;
  v_wheels: number;
  s: number;
  x: number;
  y: number;
  z: number;
  trust: number;
  slip: number;
  heading?: number;
}

export const runsApi = {
  list: () => req<Run[]>("GET", "/api/runs"),
  get: (id: number) => req<Run>("GET", `/api/runs/${id}`),
  create: (b: object) => req<Run>("POST", "/api/runs", b),
  update: (id: number, b: object) => req<Run>("PUT", `/api/runs/${id}`, b),
  remove: (id: number) => req<void>("DELETE", `/api/runs/${id}`),
  telemetry: (id: string, from?: number, to?: number, step?: number) =>
    req<Frame[]>("GET", `/api/runs/${id}/telemetry?from=${from ?? 0}&to=${to ?? 1e9}&step=${step ?? 1}`),
  events: (id: string) => req<any[]>("GET", `/api/runs/${id}/events`),
  report: (id: string) => req<any>("GET", `/api/runs/${id}/report`),
};

export const routesApi = {
  list: () => req<Route[]>("GET", "/api/routes"),
  get: (id: number) => req<Route>("GET", `/api/routes/${id}`),
  create: (b: object) => req<Route>("POST", "/api/routes", b),
  update: (id: number, b: object) => req<Route>("PUT", `/api/routes/${id}`, b),
  remove: (id: number) => req<void>("DELETE", `/api/routes/${id}`),
};

export const authApi = {
  login: (body: { username: string; password: string }) =>
    req<{ token: string; role: string }>("POST", "/api/auth/login", body),
  register: (body: { username: string; password: string }) =>
    req<{ token: string; role: string }>("POST", "/api/auth/register", body),
};