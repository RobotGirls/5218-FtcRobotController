import type { Frame, LiveFrame, Pose, Recording, RouteInfo, SimResult, Vec } from './types.js';

/** Records live robot frames so a match can be replayed on the timeline afterwards. */
export class Recorder {
  private frames: (LiveFrame & { t: number })[] = [];
  private start = 0;
  private route = '';
  active = false;

  begin(): void {
    this.frames = [];
    this.route = '';
    this.start = performance.now();
    this.active = true;
  }

  add(frame: LiveFrame): void {
    if (!this.active || !frame.pose) return;
    if (frame.route) this.route = frame.route;
    this.frames.push({ ...frame, t: (performance.now() - this.start) / 1000 });
  }

  get seconds(): number {
    return this.active ? (performance.now() - this.start) / 1000 : 0;
  }

  get count(): number {
    return this.frames.length;
  }

  /** Stops recording; null if nothing usable came in. */
  finish(): Recording | null {
    this.active = false;
    if (this.frames.length < 2) return null;
    return { version: 1, route: this.route, recordedAt: new Date().toISOString(), frames: this.frames };
  }
}

const DT = 0.02;

/**
 * Turns a recording into the same shape as a simulation so the timeline, graphs and inspector
 * all just work. Frames arrive unevenly over wifi, so they're resampled every 20 ms.
 */
export function recordingToSim(recording: Recording, routes: RouteInfo[]): SimResult {
  const src = recording.frames.filter((f) => f.pose);
  const t0 = src[0].t;
  const duration = src[src.length - 1].t - t0;
  const planned = routes.find((r) => r.name === recording.route);
  const start = src[0].pose as Pose;

  const paths: Vec[][] = [];
  const pathIds = new Map<string, number>();
  const statuses: string[] = [];
  const statusIds = new Map<string, number>();
  const frames: Frame[] = [];
  let maxOffPath = 0;
  let contactFrames = 0;
  let j = 0;

  for (let k = 0; k * DT <= duration + 1e-9; k++) {
    const t = k * DT;
    while (j + 1 < src.length && src[j + 1].t - t0 <= t) j++;
    const a = src[j];
    const b = src[Math.min(j + 1, src.length - 1)];
    const span = Math.max(1e-6, b.t - a.t);
    const frac = Math.max(0, Math.min(1, (t - (a.t - t0)) / span));
    const pa = a.pose as Pose;
    const pb = b.pose as Pose;
    let dh = pb[2] - pa[2];
    dh = Math.atan2(Math.sin(dh), Math.cos(dh));
    const pose: Pose = [pa[0] + (pb[0] - pa[0]) * frac, pa[1] + (pb[1] - pa[1]) * frac, pa[2] + dh * frac];

    let pathIndex = -1;
    if (a.path.length >= 2) {
      const key = JSON.stringify(a.path);
      if (!pathIds.has(key)) {
        pathIds.set(key, paths.length);
        paths.push(a.path);
      }
      pathIndex = pathIds.get(key) as number;
    }
    if (!statusIds.has(a.status)) {
      statusIds.set(a.status, statuses.length);
      statuses.push(a.status);
    }
    if (a.clipped) {
      maxOffPath = Math.max(maxOffPath, Math.hypot(pose[0] - a.clipped[0], pose[1] - a.clipped[1]));
    }
    if (a.colliding) contactFrames++;

    frames.push([
      t, pose[0], pose[1], pose[2],
      a.follow?.[0] ?? null, a.follow?.[1] ?? null, a.follow?.[2] ?? null,
      a.pointTo?.[0] ?? null, a.pointTo?.[1] ?? null, a.pointTo?.[2] ?? null,
      a.clipped?.[0] ?? null, a.clipped?.[1] ?? null,
      a.powers?.[0] ?? 0, a.powers?.[1] ?? 0, a.powers?.[2] ?? 0,
      pathIndex, 0, statusIds.get(a.status) as number, a.colliding ? 1 : 0, 0, 0,
    ]);
  }

  // speed over a 0.2 s window; frame-to-frame differences are too jumpy over wifi
  const window = 5;
  for (let k = 0; k < frames.length; k++) {
    const a = frames[Math.max(0, k - window)];
    const b = frames[Math.min(frames.length - 1, k + window)];
    const span = (b[0] as number) - (a[0] as number);
    if (span <= 0) continue;
    let dh = (b[3] as number) - (a[3] as number);
    dh = Math.atan2(Math.sin(dh), Math.cos(dh));
    frames[k][19] = Math.hypot((b[1] as number) - (a[1] as number), (b[2] as number) - (a[2] as number)) / span;
    frames[k][20] = dh / span;
  }

  const name = recording.route || 'Unknown route';
  const route: RouteInfo = {
    name,
    description: `Recorded ${new Date(recording.recordedAt).toLocaleString()} · ${duration.toFixed(1)} s`,
    alliance: planned?.alliance ?? 'NONE',
    group: 'Recordings',
    requiresHolonomic: false,
    start,
    steps: [{ type: 'follow', name: 'recorded run', reversed: false, endHeading: null, fixedHeading: null,
      speed: 0, lookahead: 0, points: [] }],
  };
  const end = frames[frames.length - 1];
  return {
    route,
    planned,
    holonomic: false,
    dt: DT,
    summary: {
      duration,
      settled: duration,
      finished: true,
      maxOffPath,
      centerContact: contactFrames * DT,
      timedOut: [],
      end: [end[1] as number, end[2] as number, end[3] as number],
    },
    paths,
    statuses,
    frames,
  };
}

export function parseRecording(text: string): Recording {
  const data = JSON.parse(text) as Partial<Recording>;
  if (data.version !== 1 || !Array.isArray(data.frames) || data.frames.filter((f) => f?.pose).length < 2) {
    throw new Error("That file isn't a FieldViewer recording");
  }
  return data as Recording;
}

export function download(name: string, blob: Blob): void {
  const url = URL.createObjectURL(blob);
  const link = document.createElement('a');
  link.href = url;
  link.download = name;
  document.body.append(link);
  link.click();
  link.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

export function slug(text: string): string {
  return text.toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/^-|-$/g, '') || 'fieldviewer';
}
