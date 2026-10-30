import type { Defaults, LiveFrame, LiveStatus, RoutesResponse, SimResult, SketchRequest, Tuning } from './types.js';

/** Talks to the Java backend that runs the real PurePursuitFollower. */

async function postJson<T>(url: string, body: unknown): Promise<T> {
  const response = await fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  const data = (await response.json()) as T & { error?: string };
  if (!response.ok || data.error) {
    throw new Error(data.error ?? `${url} failed (${response.status})`);
  }
  return data;
}

export async function fetchRoutes(): Promise<RoutesResponse> {
  const response = await fetch('/api/routes');
  if (!response.ok) {
    throw new Error(`couldn't load routes (${response.status})`);
  }
  return (await response.json()) as RoutesResponse;
}

export async function fetchDefaults(): Promise<Defaults> {
  const response = await fetch('/api/defaults');
  if (!response.ok) {
    throw new Error(`couldn't load tuning defaults (${response.status})`);
  }
  return (await response.json()) as Defaults;
}

export function simulateRoute(route: string, holonomic: boolean, tuning: Tuning): Promise<SimResult> {
  return postJson<SimResult>('/api/simulate', { route, holonomic, ...tuning });
}

export function simulateSketch(sketch: SketchRequest, holonomic: boolean, tuning: Tuning): Promise<SimResult> {
  return postJson<SimResult>('/api/simulate', { sketch, holonomic, ...tuning });
}

export function connectLive(host: string): Promise<unknown> {
  return postJson('/api/live/connect', { host });
}

export function disconnectLive(): Promise<unknown> {
  return postJson('/api/live/disconnect', {});
}

/** Subscribes to live robot frames and connection status. Returns a function that unsubscribes. */
export function subscribeLive(
  onFrame: (frame: LiveFrame) => void,
  onStatus: (status: LiveStatus) => void,
): () => void {
  const events = new EventSource('/api/live/events');
  events.addEventListener('frame', (e) => onFrame(JSON.parse((e as MessageEvent<string>).data) as LiveFrame));
  events.addEventListener('status', (e) => onStatus(JSON.parse((e as MessageEvent<string>).data) as LiveStatus));
  return () => events.close();
}
