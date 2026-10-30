import {
  connectLive, disconnectLive, fetchDefaults, fetchRoutes, simulateRoute, simulateSketch, subscribeLive,
} from './api.js';
import { emptySeries, TelemetryChart, type Series } from './chart.js';
import { FieldView, type Ghost, type Layers, type Scene } from './field.js';
import { download, parseRecording, Recorder, recordingToSim, slug } from './recording.js';
import { freePoint, snapPoint, type SnapGuide } from './snap.js';
import { TuningPanel } from './tuning.js';
import {
  F,
  type Frame,
  type LiveFrame,
  type LiveStatus,
  type Pose,
  type Recording,
  type RouteInfo,
  type SimResult,
  type Step,
  type Vec,
} from './types.js';

// ---- state ------------------------------------------------------------------------------------

type Mode = 'sim' | 'live';
const SKETCH = '__sketch__';

/** A waypoint in the path editor; speed / lookahead override the path defaults when set. */
interface SketchPoint {
  x: number;
  y: number;
  speed: number | null;
  lookahead: number | null;
}
const REC = 'rec:';

const state = {
  mode: 'sim' as Mode,
  holonomic: false,
  routes: [] as RouteInfo[],
  selected: '',
  sim: null as SimResult | null,
  simTrail: [] as Vec[],
  series: emptySeries(),
  t: 0,
  playing: false,
  speed: 1,
  request: 0,
  sketch: {
    name: '',
    points: [] as SketchPoint[],
    reversed: false,
    speed: 0.6,
    lookahead: 16,
    /** degrees, or null when not set */
    startHeading: null as number | null,
    endHeading: null as number | null,
    fixedHeading: null as number | null,
  },
  selectedPoint: -1,
  history: { past: [] as SketchPoint[][], future: [] as SketchPoint[][] },
  drag: -1,
  hover: -1,
  mouse: null as Vec | null,
  measuring: false,
  ruler: null as [Vec, Vec] | null,
  snap: loadPref('snap', true),
  snapGuides: [] as SnapGuide[],
  snapPreview: null as Vec | null,
  follow: false,
  ghost: null as Ghost | null,
  graphs: loadPref('graphs', true),
  layers: { lookahead: true, trail: true, route: true, keepOut: true, grid: true } as Layers,
  recordings: new Map<string, Recording>(),
  live: {
    status: null as LiveStatus | null,
    frame: null as LiveFrame | null,
    prev: null as Pose | null,
    latest: null as Pose | null,
    at: 0,
    interval: 50,
    trail: [] as Vec[],
    route: '',
    series: emptySeries(),
    start: 0,
    speed: 0,
  },
};

let field: FieldView;
let chart: TelemetryChart;
let tuning: TuningPanel;
const recorder = new Recorder();
let dirty = true;
let backendDown = false;
let lastTick = performance.now();
let sketchTimer = 0;
let tuningTimer = 0;
/** What a mouse drag on the field is doing right now. */
let gesture: { kind: 'pan' | 'ruler' | 'point'; x: number; y: number; moved: boolean } | null = null;

const $ = <T extends HTMLElement = HTMLElement>(id: string): T => document.getElementById(id) as T;
const canvas = $<HTMLCanvasElement>('field');
const wrap = $('canvas-wrap');

// ---- startup ----------------------------------------------------------------------------------

async function init(): Promise<void> {
  // the page itself never scrolls (only its panels do); undo it if focus ever tries
  window.addEventListener('scroll', () => window.scrollTo(0, 0));
  if (navigator.userAgent.includes('Electron') && navigator.userAgent.includes('Macintosh')) {
    document.body.classList.add('electron-mac');
  }
  for (;;) {
    try {
      const [data, defaults] = await Promise.all([fetchRoutes(), fetchDefaults()]);
      state.routes = data.routes;
      field = new FieldView(canvas, data.field);
      field.onChange = invalidate;
      tuning = new TuningPanel($('tuning'), defaults, onTuningChanged);
      break;
    } catch {
      $('backend-banner').hidden = false;
      await sleep(1000);
    }
  }
  $('backend-banner').hidden = true;

  chart = new TelemetryChart($<HTMLCanvasElement>('chart'));
  chart.onSeek = (t) => {
    setPlaying(false);
    seek(t);
  };
  setGraphs(state.graphs);
  updateTuningBadge();

  buildRouteList();
  wireControls();
  wireToolbar();
  wireCanvas();
  wireKeys();
  subscribeLive(onLiveFrame, onLiveStatus);
  void selectRoute(state.routes[0]?.name ?? SKETCH);
  requestAnimationFrame(tick);
}

// ---- route list -------------------------------------------------------------------------------

function buildRouteList(): void {
  const list = $('route-list');
  const query = $<HTMLInputElement>('search').value.trim().toLowerCase();
  list.replaceChildren();
  let group = '';
  const addGroup = (name: string) => {
    if (name !== group) {
      group = name;
      list.append(el('div', 'route-group', name));
    }
  };

  for (const route of state.routes) {
    if (query && !`${route.name} ${route.description}`.toLowerCase().includes(query)) continue;
    addGroup(route.group);
    const item = routeItem(route.name, route.alliance, route.name.replace(/^TEST /, ''), describeSteps(route.steps));
    item.append(route.requiresHolonomic ? el('span', 'tag', 'mecanum') : el('span'));
    list.append(item);
  }
  for (const [id, recording] of state.recordings) {
    const name = recording.route || 'Unknown route';
    if (query && !name.toLowerCase().includes(query)) continue;
    addGroup('Recordings');
    const seconds = recording.frames[recording.frames.length - 1].t - recording.frames[0].t;
    const planned = state.routes.find((r) => r.name === recording.route);
    const item = routeItem(id, planned?.alliance ?? 'NONE', name,
      `${new Date(recording.recordedAt).toLocaleTimeString([], { hour: 'numeric', minute: '2-digit' })} · ${seconds.toFixed(1)} s`);
    item.append(el('span', 'rec', 'REC'));
    list.append(item);
  }
  if (!list.children.length) {
    list.append(el('div', 'route-group', 'No matching routes'));
  }
}

function routeItem(id: string, alliance: string, name: string, meta: string): HTMLElement {
  const item = el('button', 'route-item');
  item.dataset.route = id;
  item.classList.toggle('active', id === state.selected);
  item.append(el('span', `dot ${alliance}`));
  const text = el('span');
  text.append(el('span', 'name', name), el('span', 'meta', meta));
  item.append(text);
  item.addEventListener('click', () => void selectRoute(id));
  return item;
}

function describeSteps(steps: Step[]): string {
  const paths = steps.filter((s) => s.type === 'follow').length;
  const markers = steps.filter((s) => s.type === 'marker').length;
  const turns = steps.filter((s) => s.type === 'turn').length;
  const parts = [];
  if (paths) parts.push(`${paths} path${paths > 1 ? 's' : ''}`);
  if (turns) parts.push(`${turns} turn${turns > 1 ? 's' : ''}`);
  if (markers) parts.push(`${markers} marker${markers > 1 ? 's' : ''}`);
  return parts.join(' · ');
}

async function selectRoute(name: string): Promise<void> {
  if (state.mode === 'live') {
    setMode('sim');
  }
  state.selected = name;
  const sketching = name === SKETCH;
  document.querySelectorAll<HTMLElement>('.route-item').forEach((item) =>
    item.classList.toggle('active', item.dataset.route === name));
  $('new-sketch').classList.toggle('active', sketching);
  $('sketch-card').hidden = !sketching;
  wrap.classList.toggle('sketching', sketching);
  updateSketchHint();

  if (name.startsWith(REC)) {
    const recording = state.recordings.get(name);
    if (recording) {
      state.request++;
      loadSim(recordingToSim(recording, state.routes));
    }
  } else if (sketching) {
    state.request++;
    loadSim(null);
    showRouteCard(null);
    await runSketch();
  } else {
    const route = state.routes.find((r) => r.name === name);
    showRouteCard(route ?? null);
    await runSim(() => simulateRoute(name, state.holonomic, tuning.tuning()));
  }
}

// ---- simulation -------------------------------------------------------------------------------

async function runSim(request: () => Promise<SimResult>): Promise<void> {
  const id = ++state.request;
  try {
    const result = await request();
    if (id !== state.request) {
      return; // a newer request already went out
    }
    loadSim(result);
  } catch (e) {
    if (id !== state.request) {
      return;
    }
    if (e instanceof TypeError) {
      void onBackendLost(); // fetch itself failed: the backend isn't running
    } else {
      toast((e as Error).message, true);
    }
  }
}

/** Reruns whatever is selected (after a tuning or drivetrain change), keeping the playhead. */
async function rerun(): Promise<void> {
  if (state.selected.startsWith(REC)) return;
  const t = state.t;
  const wasPlaying = state.playing;
  if (state.selected === SKETCH) {
    await runSketch();
  } else {
    await runSim(() => simulateRoute(state.selected, state.holonomic, tuning.tuning()));
  }
  if (state.sim) {
    seek(Math.min(t, duration()));
    setPlaying(wasPlaying || state.t < duration());
  }
}

/**
 * The Java backend stopped answering (the Electron app restarts it). Stop showing the old
 * run, wait for it to come back, then run whatever is selected again.
 */
async function onBackendLost(): Promise<void> {
  if (backendDown) {
    return;
  }
  backendDown = true;
  $('backend-banner').hidden = false;
  loadSim(null);
  while (!(await healthy())) {
    await sleep(1000);
  }
  backendDown = false;
  $('backend-banner').hidden = true;
  toast('Simulator reconnected');
  void selectRoute(state.selected);
}

async function healthy(): Promise<boolean> {
  try {
    return (await fetch('/api/health', { cache: 'no-store' })).ok;
  } catch {
    return false;
  }
}

function loadSim(result: SimResult | null): void {
  state.sim = result;
  state.simTrail = result ? result.frames.map((f) => [f[F.X] as number, f[F.Y] as number] as Vec) : [];
  state.series = result ? seriesFromFrames(result.frames) : emptySeries();
  state.t = 0;
  buildSegments();
  showResults();
  if (result || state.selected === SKETCH) {
    showRouteCard(state.selected === SKETCH ? null : result?.route ?? null);
  }
  $('sk-java').textContent = result?.java ?? '';
  setPlaying(Boolean(result));
  invalidate();
}

function seriesFromFrames(frames: Frame[]): Series {
  const series = emptySeries();
  for (const f of frames) {
    series.t.push(f[F.T] as number);
    series.fwd.push(f[F.FWD] as number);
    series.left.push(f[F.LEFT] as number);
    series.turn.push(f[F.TURN] as number);
    series.speed.push((f[F.SPEED] as number | null) ?? 0);
  }
  return series;
}

function duration(): number {
  const frames = state.sim?.frames;
  return frames && frames.length ? (frames[frames.length - 1][F.T] as number) : 0;
}

function frameIndex(): number {
  const sim = state.sim;
  if (!sim) return 0;
  return Math.max(0, Math.min(sim.frames.length - 1, Math.floor(state.t / sim.dt + 1e-6)));
}

function setPlaying(playing: boolean): void {
  if (playing && state.sim && state.t >= duration() - 1e-6) {
    state.t = 0; // play again from the start
  }
  state.playing = playing && Boolean(state.sim);
  $('play').classList.toggle('playing', state.playing);
  lastTick = performance.now();
}

function seek(t: number): void {
  state.t = Math.max(0, Math.min(duration(), t));
  invalidate();
}

function buildSegments(): void {
  const container = $('segments');
  container.replaceChildren();
  const sim = state.sim;
  if (!sim) return;
  const steps = sim.route.steps;
  let start = 0;
  for (let i = 1; i <= sim.frames.length; i++) {
    const step = sim.frames[start][F.STEP] as number;
    if (i < sim.frames.length && sim.frames[i][F.STEP] === step) continue;
    const segment = el('div', step >= steps.length ? 'coast' : steps[step].type);
    segment.style.flexGrow = String(i - start);
    segment.dataset.step = String(step);
    segment.title = step >= steps.length ? 'coasting to a stop' : stepLabel(steps[step]);
    const startTime = sim.frames[start][F.T] as number;
    segment.addEventListener('click', () => {
      setPlaying(false);
      seek(startTime);
    });
    container.append(segment);
    start = i;
  }
}

// ---- tuning -----------------------------------------------------------------------------------

function onTuningChanged(): void {
  updateTuningBadge();
  invalidate();
  window.clearTimeout(tuningTimer);
  tuningTimer = window.setTimeout(() => void rerun(), 200);
}

function updateTuningBadge(): void {
  const count = tuning.changedCount();
  const badge = $('tuning-badge');
  badge.hidden = count === 0;
  badge.textContent = `${count} changed`;
}

// ---- sketch -----------------------------------------------------------------------------------

async function runSketch(): Promise<void> {
  updateSketchHint();
  updateUndoButtons();
  renderEditor();
  if (state.sketch.points.length < 2) {
    state.request++;
    loadSim(null);
    showResults();
    return;
  }
  const sk = state.sketch;
  const radians = (deg: number | null) => (deg === null ? null : (deg * Math.PI) / 180);
  await runSim(() => simulateSketch({
    name: sk.name,
    points: sk.points.map((p) => [p.x, p.y, p.speed, p.lookahead]),
    reversed: sk.reversed,
    moveSpeed: sk.speed,
    followDistance: sk.lookahead,
    startHeading: radians(sk.startHeading),
    endHeading: radians(sk.endHeading),
    fixedHeading: radians(sk.fixedHeading),
  }, state.holonomic, tuning.tuning()));
}

function scheduleSketch(delay = 120): void {
  window.clearTimeout(sketchTimer);
  sketchTimer = window.setTimeout(() => void runSketch(), delay);
}

/** Call before changing the sketch so it can be undone. */
function remember(): void {
  state.history.past.push(clonePoints(state.sketch.points));
  if (state.history.past.length > 100) state.history.past.shift();
  state.history.future = [];
  updateUndoButtons();
}

function undo(): void {
  const previous = state.history.past.pop();
  if (!previous) return;
  state.history.future.push(state.sketch.points);
  state.sketch.points = previous;
  state.selectedPoint = Math.min(state.selectedPoint, previous.length - 1);
  void runSketch();
}

function redo(): void {
  const next = state.history.future.pop();
  if (!next) return;
  state.history.past.push(state.sketch.points);
  state.sketch.points = next;
  state.selectedPoint = Math.min(state.selectedPoint, next.length - 1);
  void runSketch();
}

function updateUndoButtons(): void {
  $<HTMLButtonElement>('sk-undo').disabled = !state.history.past.length;
  $<HTMLButtonElement>('sk-redo').disabled = !state.history.future.length;
}

/** Starts a sketch from an existing route's points so it can be tweaked. */
function editAsSketch(route: RouteInfo): void {
  const follows = route.steps.filter((s): s is Extract<Step, { type: 'follow' }> => s.type === 'follow');
  remember();
  const sk = state.sketch;
  sk.speed = follows[0]?.speed ?? 0.6;
  sk.lookahead = follows[0]?.lookahead ?? 16;
  // each leg's speed / lookahead becomes a per-point setting where it differs from the first leg
  sk.points = [{ x: route.start[0], y: route.start[1], speed: null, lookahead: null },
    ...follows.flatMap((leg) => leg.points.map((p) => ({
      x: p[0], y: p[1],
      speed: leg.speed === sk.speed ? null : leg.speed,
      lookahead: leg.lookahead === sk.lookahead ? null : leg.lookahead,
    })))];
  sk.name = `${route.name} copy`;
  sk.reversed = follows[0]?.reversed ?? false;
  sk.startHeading = Math.round((route.start[2] * 180) / Math.PI);
  const last = follows[follows.length - 1];
  sk.endHeading = last?.endHeading == null ? null : Math.round((last.endHeading * 180) / Math.PI);
  sk.fixedHeading = follows[0]?.fixedHeading == null ? null : Math.round((follows[0].fixedHeading * 180) / Math.PI);
  state.selectedPoint = -1;
  syncPathSettings();
  if (follows.length > 1 || route.steps.some((s) => s.type !== 'follow')) {
    toast('The editor makes one path: legs were joined and markers / turns dropped');
  }
  void selectRoute(SKETCH);
}

function updateSketchHint(): void {
  const n = state.sketch.points.length;
  if (state.measuring) {
    return;
  }
  if (state.selected !== SKETCH || state.mode !== 'sim' || n >= 2) {
    $('hint').hidden = true;
    return;
  }
  showHint(n === 0 ? 'Click the field to place the start' : 'Click to add a waypoint');
}

// ---- live -------------------------------------------------------------------------------------

function onLiveStatus(status: LiveStatus): void {
  state.live.status = status;
  const pill = $('live-pill');
  const receiving = status.running && status.text.startsWith('Receiving');
  pill.classList.toggle('live', receiving);
  pill.classList.toggle('connecting', status.running && !receiving);
  pill.querySelector('span')!.textContent = receiving
    ? `Robot live · ${status.text.replace('Receiving ', '').replace(' frames/s', ' fps')}`
    : status.running ? 'Waiting for robot' : 'Robot offline';
  $('connect').textContent = status.running ? 'Disconnect' : 'Connect';
  $('live-status').textContent = status.text;
  $('live-badge').hidden = !(state.mode === 'live' && receiving);
}

function onLiveFrame(frame: LiveFrame): void {
  const live = state.live;
  recorder.add(frame);
  if (frame.route !== live.route) {
    live.route = frame.route;
    live.trail = [];
    if (state.mode === 'live') {
      showRouteCard(state.routes.find((r) => r.name === frame.route) ?? null, frame.route);
    }
  }
  live.frame = frame;
  if (frame.pose) {
    const now = performance.now();
    // a jump of more than two feet means the OpMode restarted, so start a fresh trail
    if (live.latest && Math.hypot(live.latest[0] - frame.pose[0], live.latest[1] - frame.pose[1]) > 24) {
      live.trail = [];
      live.latest = null;
    }
    if (live.latest && live.at > 0) {
      const dt = (now - live.at) / 1000;
      const moved = Math.hypot(frame.pose[0] - live.latest[0], frame.pose[1] - live.latest[1]);
      live.speed = dt > 0 ? live.speed * 0.6 + (moved / dt) * 0.4 : live.speed;
    }
    live.prev = live.latest ?? frame.pose;
    live.latest = frame.pose;
    if (live.at > 0) {
      live.interval = Math.max(10, Math.min(250, now - live.at));
    }
    live.at = now;
    const last = live.trail[live.trail.length - 1];
    if (!last || Math.hypot(last[0] - frame.pose[0], last[1] - frame.pose[1]) >= 0.75) {
      live.trail.push([frame.pose[0], frame.pose[1]]);
      if (live.trail.length > 4000) live.trail.shift();
    }

    // rolling graphs of the last 15 seconds
    if (!live.start) live.start = now;
    const s = live.series;
    s.t.push((now - live.start) / 1000);
    s.fwd.push(frame.powers?.[0] ?? 0);
    s.left.push(frame.powers?.[1] ?? 0);
    s.turn.push(frame.powers?.[2] ?? 0);
    s.speed.push(live.speed);
    while (s.t.length && s.t[0] < s.t[s.t.length - 1] - 20) {
      s.t.shift(); s.fwd.shift(); s.left.shift(); s.turn.shift(); s.speed.shift();
    }
  }
  invalidate();
}

function toggleRecording(): void {
  const button = $('record');
  if (!recorder.active) {
    recorder.begin();
    button.classList.add('recording');
    return;
  }
  button.classList.remove('recording');
  button.querySelector('span')!.textContent = 'Record run';
  const recording = recorder.finish();
  if (!recording) {
    toast('Nothing recorded: no frames came in from the robot', true);
    return;
  }
  addRecording(recording);
  toast('Saved under Recordings in the sidebar');
}

function addRecording(recording: Recording): string {
  const id = `${REC}${state.recordings.size + 1}`;
  state.recordings.set(id, recording);
  buildRouteList();
  return id;
}

function setMode(mode: Mode): void {
  state.mode = mode;
  document.body.classList.toggle('mode-live', mode === 'live');
  document.querySelectorAll<HTMLButtonElement>('#mode button').forEach((b) =>
    b.classList.toggle('active', b.dataset.mode === mode));
  const live = mode === 'live';
  $('timeline').hidden = live;
  $('live-card').hidden = !live;
  $('result-card').hidden = live;
  $('sketch-card').hidden = live || state.selected !== SKETCH;
  wrap.classList.toggle('sketching', !live && state.selected === SKETCH);
  if (live) {
    setPlaying(false);
    showRouteCard(state.routes.find((r) => r.name === state.live.route) ?? null, state.live.route || 'Waiting for robot');
  } else if (state.sim) {
    showRouteCard(state.selected === SKETCH ? null : state.sim.route);
  }
  if (state.live.status) onLiveStatus(state.live.status);
  updateSketchHint();
  invalidate();
}

// ---- drawing loop -----------------------------------------------------------------------------

function invalidate(): void {
  dirty = true;
}

function tick(now: number): void {
  const dt = Math.min(0.1, (now - lastTick) / 1000);
  lastTick = now;
  if (state.mode === 'sim' && state.playing) {
    state.t += dt * state.speed;
    if (state.t >= duration()) {
      state.t = duration();
      setPlaying(false);
    }
    dirty = true;
  }
  if (recorder.active) {
    $('record').querySelector('span')!.textContent = `Stop · ${recorder.seconds.toFixed(1)} s · ${recorder.count} frames`;
  }
  const liveMoving = state.mode === 'live' && state.live.latest !== null && now - state.live.at < 500;
  if (dirty || liveMoving) {
    dirty = false;
    const scene = state.mode === 'sim' ? simScene() : liveScene(now);
    if (state.follow && scene.pose) {
      field.lookAt([scene.pose[0], scene.pose[1]]);
    }
    field.draw(scene);
    setText('tb-zoom', `${Math.round(field.zoom * 100)}%`);
    drawChart(now);
  }
  requestAnimationFrame(tick);
}

function drawChart(now: number): void {
  if (!state.graphs) return;
  if (state.mode === 'sim') {
    chart.draw(state.series, state.sim ? state.t : null, [0, Math.max(1, duration())], Boolean(state.sim));
  } else {
    const end = Math.max(15, state.live.start ? (now - state.live.start) / 1000 : 0);
    chart.draw(state.live.series, null, [end - 15, end], false);
  }
}

function baseScene(): Scene {
  return {
    route: null, path: [], trail: [], pose: null, follow: null, pointTo: null, clipped: null,
    colliding: false, sketch: null, sketchActive: -1, sketchSelected: -1, mouse: state.snapPreview ?? state.mouse, ruler: state.ruler,
    snapGuides: state.snapGuides, snapPreview: state.snapPreview, ghost: state.ghost, robotSize: tuning.robotSize(), layers: state.layers,
  };
}

function simScene(): Scene {
  const scene = baseScene();
  const sketching = state.selected === SKETCH;
  if (sketching) {
    scene.sketch = sketchVecs();
    scene.sketchActive = state.drag >= 0 ? state.drag : state.hover;
    scene.sketchSelected = state.selectedPoint;
  }
  const sim = state.sim;
  if (!sim) {
    scene.route = sketching ? null : state.routes.find((r) => r.name === state.selected) ?? null;
    updateInspector(null, null, backendDown ? 'Waiting for the simulator…' : '—', -1);
    setText('time', '—');
    return scene;
  }

  const i = frameIndex();
  const frame = sim.frames[i];
  const next = sim.frames[Math.min(i + 1, sim.frames.length - 1)];
  const frac = Math.max(0, Math.min(1, (state.t - (frame[F.T] as number)) / sim.dt));
  scene.route = sketching ? null : sim.planned ?? sim.route;
  scene.pose = lerpPose(framePose(frame), framePose(next), frac);
  scene.follow = frame[F.FX] === null ? null : [frame[F.FX] as number, frame[F.FY] as number, frame[F.FR] as number];
  scene.pointTo = frame[F.PX] === null ? null : [frame[F.PX] as number, frame[F.PY] as number, frame[F.PR] as number];
  scene.clipped = frame[F.CX] === null ? null : [frame[F.CX] as number, frame[F.CY] as number];
  scene.path = (frame[F.PATH] as number) >= 0 ? sim.paths[frame[F.PATH] as number] : [];
  scene.trail = state.simTrail.slice(0, i + 1);
  scene.trail.push([scene.pose[0], scene.pose[1]]);
  scene.colliding = frame[F.COLLIDE] === 1;

  updateInspector(scene.pose, [frame[F.FWD] as number, frame[F.LEFT] as number, frame[F.TURN] as number],
    sim.statuses[frame[F.STATUS] as number] ?? '', frame[F.STEP] as number);
  setText('time', `${state.t.toFixed(2)} / ${duration().toFixed(2)} s`);
  const scrub = $<HTMLInputElement>('scrub');
  if (document.activeElement !== scrub) {
    scrub.value = String(duration() > 0 ? Math.round((state.t / duration()) * 1000) : 0);
  }
  document.querySelectorAll<HTMLElement>('#segments div').forEach((s) =>
    s.classList.toggle('active', Number(s.dataset.step) === frame[F.STEP]));
  return scene;
}

function liveScene(now: number): Scene {
  const scene = baseScene();
  const live = state.live;
  const frame = live.frame;
  scene.route = state.routes.find((r) => r.name === live.route) ?? null;
  scene.trail = live.trail;
  if (frame) {
    if (live.prev && live.latest) {
      scene.pose = lerpPose(live.prev, live.latest, Math.min(1, (now - live.at) / live.interval));
      scene.trail = [...live.trail, [scene.pose[0], scene.pose[1]]];
    }
    scene.follow = frame.follow;
    scene.pointTo = frame.pointTo;
    scene.clipped = frame.clipped;
    scene.path = frame.path;
    scene.colliding = frame.colliding;
  }
  updateInspector(scene.pose, frame?.powers ?? null, frame?.status ?? 'No data yet', -1);
  return scene;
}

// ---- inspector --------------------------------------------------------------------------------

function showRouteCard(route: RouteInfo | null, fallbackName?: string): void {
  const sketching = state.mode === 'sim' && state.selected === SKETCH;
  const recording = state.mode === 'sim' && state.selected.startsWith(REC);
  const alliance = $('route-alliance');
  alliance.className = `chip ${sketching ? 'SKETCH' : recording ? 'RED' : route?.alliance ?? ''}`;
  alliance.textContent = sketching ? 'SKETCH' : recording ? 'REC' : route ? (route.alliance === 'NONE' ? 'TEST' : route.alliance) : 'LIVE';
  setText('route-name', sketching ? state.sketch.name.trim() || 'Sketch' : route?.name ?? fallbackName ?? '—');
  setText('route-desc', sketching ? 'Your own path. Copy it as Java when it drives the way you want.'
    : route?.description ?? '');

  $('edit-sketch').hidden = !route || sketching || recording || state.mode === 'live';
  $('save-recording').hidden = !recording;
  $('route-actions').hidden = $('edit-sketch').hidden && $('save-recording').hidden;

  const steps = $('steps');
  steps.replaceChildren();
  const list = sketching ? state.sim?.route.steps ?? [] : route?.steps ?? [];
  list.forEach((step, index) => {
    const li = el('li', step.type);
    li.dataset.index = String(index);
    li.append(el('span', 'num', String(index + 1)), el('span', 'label', stepLabel(step)),
      el('span', 'detail', stepDetail(step)));
    li.addEventListener('click', () => jumpToStep(index));
    steps.append(li);
  });
}

function stepLabel(step: Step): string {
  switch (step.type) {
    case 'follow': return step.name;
    case 'marker': return step.name;
    case 'turn': return `turn to ${Math.round((step.heading * 180) / Math.PI)}°`;
  }
}

function stepDetail(step: Step): string {
  switch (step.type) {
    case 'follow': {
      if (!step.points.length) return '';
      const parts = [`${step.points.length} pt`];
      if (step.reversed) parts.push('reverse');
      if (step.fixedHeading !== null) parts.push('strafe');
      if (step.endHeading !== null) parts.push(`→ ${Math.round((step.endHeading * 180) / Math.PI)}°`);
      return parts.join(' · ');
    }
    case 'marker': return `${step.seconds.toFixed(1)} s`;
    case 'turn': return 'in place';
  }
}

function jumpToStep(index: number): void {
  const sim = state.sim;
  if (state.mode !== 'sim' || !sim) return;
  const frame = sim.frames.find((f) => f[F.STEP] === index);
  if (frame) {
    seek(frame[F.T] as number);
  }
}

function updateInspector(pose: Pose | null, powers: [number, number, number] | null, status: string, step: number): void {
  setText('stat-x', pose ? pose[0].toFixed(1) : '—');
  setText('stat-y', pose ? pose[1].toFixed(1) : '—');
  setText('stat-h', pose ? `${((pose[2] * 180) / Math.PI).toFixed(1)}°` : '—');
  const ids = ['fwd', 'left', 'turn'] as const;
  ids.forEach((id, i) => {
    const v = powers ? powers[i] : 0;
    const bar = $(`bar-${id}`);
    const width = Math.min(50, Math.abs(v) * 50);
    bar.style.left = v >= 0 ? '50%' : `${50 - width}%`;
    bar.style.width = `${width}%`;
    setText(`val-${id}`, v.toFixed(2));
  });
  setText('status', status);
  document.querySelectorAll<HTMLElement>('#steps li').forEach((li) => {
    const index = Number(li.dataset.index);
    li.classList.toggle('active', step >= 0 && index === step);
    li.classList.toggle('done', step >= 0 && index < step);
  });
}

function showResults(): void {
  const summary = state.sim?.summary;
  const set = (id: string, text: string, level: '' | 'good' | 'warn' | 'bad') => {
    const node = $(id);
    node.textContent = text;
    node.className = `mono ${level}`;
  };
  $<HTMLButtonElement>('pin').disabled = !state.sim;
  if (!summary) {
    ['res-time', 'res-off', 'res-contact', 'res-timeouts'].forEach((id) => set(id, '—', ''));
    return;
  }
  set('res-time', `${summary.duration.toFixed(2)} s`, '');
  set('res-off', `${summary.maxOffPath.toFixed(1)} in`, summary.maxOffPath < 3 ? 'good' : summary.maxOffPath < 8 ? 'warn' : 'bad');
  set('res-contact', summary.centerContact > 0 ? `${summary.centerContact.toFixed(2)} s` : 'none',
    summary.centerContact > 0 ? 'bad' : 'good');
  set('res-timeouts', summary.timedOut.length ? summary.timedOut.join(', ') : 'none', summary.timedOut.length ? 'bad' : 'good');
}

function pinRun(): void {
  const sim = state.sim;
  if (!sim) return;
  const changed = tuning.changedCount();
  const name = state.selected === SKETCH ? 'sketch' : sim.route.name.replace(/^TEST /, '');
  state.ghost = {
    trail: state.simTrail,
    label: `${name} · ${sim.holonomic ? 'mecanum' : 'tank'}${changed ? ` · ${changed} tuned` : ''} · ${sim.summary.duration.toFixed(2)} s`,
  };
  $('unpin').hidden = false;
  toast('Pinned. Change the tuning or drivetrain to compare');
  invalidate();
}

// ---- controls ---------------------------------------------------------------------------------

function wireControls(): void {
  $<HTMLInputElement>('search').addEventListener('input', buildRouteList);

  document.querySelectorAll<HTMLButtonElement>('#mode button').forEach((b) =>
    b.addEventListener('click', () => setMode(b.dataset.mode as Mode)));

  document.querySelectorAll<HTMLButtonElement>('#drive button').forEach((b) =>
    b.addEventListener('click', () => {
      state.holonomic = b.dataset.drive === 'mecanum';
      document.querySelectorAll('#drive button').forEach((x) => x.classList.toggle('active', x === b));
      void rerun();
    }));

  document.querySelectorAll<HTMLButtonElement>('#speed button').forEach((b) =>
    b.addEventListener('click', () => {
      state.speed = Number(b.dataset.speed);
      document.querySelectorAll('#speed button').forEach((x) => x.classList.toggle('active', x === b));
    }));

  $('play').addEventListener('click', () => setPlaying(!state.playing));
  $('restart').addEventListener('click', () => {
    seek(0);
    setPlaying(true);
  });
  const scrub = $<HTMLInputElement>('scrub');
  scrub.addEventListener('input', () => {
    setPlaying(false);
    seek((Number(scrub.value) / 1000) * duration());
  });
  $('graphs-toggle').addEventListener('click', () => setGraphs(!state.graphs));

  $('pin').addEventListener('click', pinRun);
  $('unpin').addEventListener('click', () => {
    state.ghost = null;
    $('unpin').hidden = true;
    invalidate();
  });
  $('edit-sketch').addEventListener('click', () => {
    const route = state.routes.find((r) => r.name === state.selected);
    if (route) editAsSketch(route);
  });
  $('save-recording').addEventListener('click', () => {
    const recording = state.recordings.get(state.selected);
    if (recording) {
      const stamp = recording.recordedAt.slice(0, 16).replace(/[:T]/g, '-');
      download(`${slug(recording.route || 'recording')}-${stamp}.json`,
        new Blob([JSON.stringify(recording)], { type: 'application/json' }));
    }
  });
  $('open-recording').addEventListener('click', () => $('recording-file').click());
  $<HTMLInputElement>('recording-file').addEventListener('change', async (e) => {
    const input = e.target as HTMLInputElement;
    const file = input.files?.[0];
    input.value = '';
    if (!file) return;
    try {
      const id = addRecording(parseRecording(await file.text()));
      void selectRoute(id);
    } catch (err) {
      toast((err as Error).message, true);
    }
  });

  $('new-sketch').addEventListener('click', () => void selectRoute(SKETCH));
  wireEditor();
  $<HTMLInputElement>('sk-reversed').addEventListener('change', (e) => {
    state.sketch.reversed = (e.target as HTMLInputElement).checked;
    scheduleSketch(0);
  });
  $<HTMLInputElement>('sk-speed').addEventListener('input', (e) => {
    state.sketch.speed = Number((e.target as HTMLInputElement).value);
    setText('sk-speed-val', state.sketch.speed.toFixed(2));
    scheduleSketch();
  });
  $<HTMLInputElement>('sk-look').addEventListener('input', (e) => {
    state.sketch.lookahead = Number((e.target as HTMLInputElement).value);
    setText('sk-look-val', `${state.sketch.lookahead} in`);
    scheduleSketch();
  });
  $('sk-undo').addEventListener('click', undo);
  $('sk-redo').addEventListener('click', redo);
  $('sk-clear').addEventListener('click', () => {
    if (!state.sketch.points.length) return;
    remember();
    state.sketch.points = [];
    state.selectedPoint = -1;
    void runSketch();
  });
  $('sk-copy').addEventListener('click', () => {
    const java = state.sim?.java;
    if (!java) {
      toast('Add at least two points first', true);
      return;
    }
    void copy(java, 'Copied the route as Java');
  });

  $('tune-copy').addEventListener('click', () => void copy(tuning.java(), 'Copied the tuning as Java'));
  $('tune-reset').addEventListener('click', () => tuning.reset());

  $('connect').addEventListener('click', async () => {
    try {
      if (state.live.status?.running) {
        await disconnectLive();
      } else {
        state.live.trail = [];
        await connectLive($<HTMLInputElement>('host').value.trim() || '192.168.43.1');
      }
    } catch (e) {
      toast((e as Error).message, true);
    }
  });
  $('clear-trail').addEventListener('click', () => {
    state.live.trail = [];
    invalidate();
  });
  $('record').addEventListener('click', toggleRecording);

  document.querySelectorAll<HTMLInputElement>('[data-layer]').forEach((input) =>
    input.addEventListener('change', () => {
      state.layers[input.dataset.layer as keyof Layers] = input.checked;
      invalidate();
    }));

  $('help-btn').addEventListener('click', () => toggleHelp(true));
  $('help-close').addEventListener('click', () => toggleHelp(false));
  $('help').addEventListener('click', (e) => {
    if (e.target === $('help')) toggleHelp(false);
  });
}

function wireToolbar(): void {
  $('tb-zoom-in').addEventListener('click', () => zoomBy(1.4));
  $('tb-zoom-out').addEventListener('click', () => zoomBy(1 / 1.4));
  $('tb-fit').addEventListener('click', fitView);
  $('tb-rotate').addEventListener('click', () => {
    field.rotateView();
    invalidate();
  });
  $('tb-follow').addEventListener('click', () => setFollow(!state.follow));
  $('tb-measure').addEventListener('click', () => setMeasuring(!state.measuring));
  $('tb-snap').addEventListener('click', () => setSnap(!state.snap));
  $('tb-snap').classList.toggle('active', state.snap);
  $('tb-shot').addEventListener('click', screenshot);
}

function zoomBy(factor: number): void {
  field.zoomBy(factor);
  invalidate();
}

function fitView(): void {
  setFollow(false);
  field.fitField();
  invalidate();
}

function setFollow(on: boolean): void {
  state.follow = on;
  $('tb-follow').classList.toggle('active', on);
  if (on && field.zoom < 2) {
    field.zoom = 2.5; // following only makes sense zoomed in
  }
  invalidate();
}

function setMeasuring(on: boolean): void {
  state.measuring = on;
  if (!on) state.ruler = null;
  $('tb-measure').classList.toggle('active', on);
  wrap.classList.toggle('measuring', on);
  if (on) {
    showHint('Drag on the field to measure. Esc to stop.');
  } else {
    $('hint').hidden = true;
    updateSketchHint();
  }
  invalidate();
}

function setSnap(on: boolean): void {
  state.snap = on;
  savePref('snap', on);
  $('tb-snap').classList.toggle('active', on);
  if (isSketching() || state.measuring) toast(on ? 'Snapping on (hold Option/Alt to place freely)' : 'Snapping off');
  invalidate();
}

function setGraphs(on: boolean): void {
  state.graphs = on;
  savePref('graphs', on);
  $('graphs').hidden = !on;
  $('graphs-toggle').classList.toggle('active', on);
  invalidate();
}

function toggleHelp(show: boolean): void {
  $('help').hidden = !show;
}

function screenshot(): void {
  const name = state.mode === 'live' ? state.live.route || 'live'
    : state.selected === SKETCH ? 'sketch' : state.sim?.route.name ?? 'field';
  canvas.toBlob((blob) => {
    if (blob) {
      download(`${slug(name)}.png`, blob);
      toast('Saved a picture of the field');
    }
  }, 'image/png');
}

function wireCanvas(): void {
  const coord = $('coord');

  canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    field.zoomAt(e.clientX, e.clientY, Math.exp(-e.deltaY * 0.0015));
    invalidate();
  }, { passive: false });

  canvas.addEventListener('pointerdown', (e) => {
    if (e.button === 2) {
      // right-click deletes a sketch point
      if (isSketching()) {
        const hit = field.pick(sketchVecs(), e.clientX, e.clientY);
        if (hit >= 0) {
          deletePoint(hit);
        }
      }
      return;
    }
    canvas.setPointerCapture(e.pointerId);
    const point = field.toField(e.clientX, e.clientY);
    if (state.measuring && e.button === 0) {
      if (point) {
        const start = snapAt(point, null, e.altKey);
        state.ruler = [start, start];
        gesture = { kind: 'ruler', x: e.clientX, y: e.clientY, moved: false };
      }
      return;
    }
    if (isSketching() && e.button === 0 && !e.shiftKey) {
      const hit = field.pick(sketchVecs(), e.clientX, e.clientY);
      const mid = hit < 0 ? field.pickMidpoint(sketchVecs(), e.clientX, e.clientY) : -1;
      if (hit >= 0) {
        remember();
        selectPoint(hit);
        state.drag = hit;
        gesture = { kind: 'point', x: e.clientX, y: e.clientY, moved: false };
      } else if (mid >= 0) {
        // insert halfway along the segment and keep dragging the new point
        remember();
        const a = state.sketch.points[mid];
        const b = state.sketch.points[mid + 1];
        state.sketch.points.splice(mid + 1, 0, { x: round((a.x + b.x) / 2), y: round((a.y + b.y) / 2), speed: null, lookahead: null });
        selectPoint(mid + 1);
        state.drag = mid + 1;
        gesture = { kind: 'point', x: e.clientX, y: e.clientY, moved: true };
        renderEditor();
      } else if (point) {
        remember();
        const [x, y] = snapAt(point, 'new', e.altKey);
        state.snapPreview = null;
        state.sketch.points.push({ x, y, speed: null, lookahead: null });
        selectPoint(state.sketch.points.length - 1);
        void runSketch();
      }
      return;
    }
    gesture = { kind: 'pan', x: e.clientX, y: e.clientY, moved: false };
  });

  canvas.addEventListener('pointermove', (e) => {
    state.mouse = field.toField(e.clientX, e.clientY);
    state.snapGuides = [];
    state.snapPreview = null;
    let readout = state.mouse;

    if (gesture) {
      const dx = e.clientX - gesture.x;
      const dy = e.clientY - gesture.y;
      if (Math.hypot(dx, dy) > 3) gesture.moved = true;
      if (gesture.kind === 'pan' && gesture.moved) {
        if (state.follow) setFollow(false);
        field.panBy(dx, dy);
        gesture.x = e.clientX;
        gesture.y = e.clientY;
        wrap.classList.add('panning');
      } else if (gesture.kind === 'ruler' && state.ruler && state.mouse) {
        readout = snapAt(state.mouse, null, e.altKey);
        state.ruler = [state.ruler[0], readout];
      } else if (gesture.kind === 'point' && state.drag >= 0 && state.mouse) {
        const [x, y] = readout = snapAt(state.mouse, state.drag, e.altKey);
        state.sketch.points[state.drag] = { ...state.sketch.points[state.drag], x, y };
        syncEditorValues();
        scheduleSketch();
      }
    } else if (isSketching() && !state.measuring) {
      state.hover = field.pick(sketchVecs(), e.clientX, e.clientY);
      const overHandle = state.hover < 0 && field.pickMidpoint(sketchVecs(), e.clientX, e.clientY) >= 0;
      wrap.classList.toggle('grab', state.hover >= 0 || overHandle);
      if (state.mouse && state.hover < 0 && !overHandle && !e.shiftKey) {
        // show where a click would put the next point
        state.snapPreview = readout = snapAt(state.mouse, 'new', e.altKey);
      }
    } else if (state.measuring && state.mouse) {
      readout = snapAt(state.mouse, null, e.altKey);
      state.snapPreview = null;
    }

    if (readout) {
      const rect = wrap.getBoundingClientRect();
      coord.hidden = false;
      coord.style.left = `${e.clientX - rect.left}px`;
      coord.style.top = `${e.clientY - rect.top}px`;
      coord.textContent = `x ${trimInches(readout[0])}   y ${trimInches(readout[1])}`;
    } else {
      coord.hidden = true;
    }
    invalidate();
  });

  const endGesture = () => {
    if (!gesture) return;
    if (gesture.kind === 'ruler' && !gesture.moved) {
      state.ruler = null; // a click without dragging clears the ruler
    }
    state.snapGuides = [];
    if (gesture.kind === 'point') {
      if (!gesture.moved) state.history.past.pop(); // just a click on a point, nothing changed
      state.drag = -1;
      scheduleSketch(0);
    }
    gesture = null;
    wrap.classList.remove('panning', 'grabbing');
    updateUndoButtons();
    invalidate();
  };
  canvas.addEventListener('pointerup', endGesture);
  canvas.addEventListener('pointercancel', endGesture);

  canvas.addEventListener('pointerleave', () => {
    state.mouse = null;
    state.hover = -1;
    state.snapPreview = null;
    if (!gesture) state.snapGuides = [];
    coord.hidden = true;
    invalidate();
  });

  canvas.addEventListener('dblclick', () => {
    if (!isSketching() && !state.measuring) fitView();
  });
  canvas.addEventListener('contextmenu', (e) => e.preventDefault());
}

function wireKeys(): void {
  window.addEventListener('keydown', (e) => {
    const target = e.target as HTMLElement;
    const typing = target.tagName === 'INPUT' && !['range', 'checkbox'].includes((target as HTMLInputElement).type);
    if (typing) return;
    const mod = e.metaKey || e.ctrlKey;

    if (mod && (e.key === 'z' || e.key === 'Z')) {
      if (isSketching()) {
        e.preventDefault();
        if (e.shiftKey) redo();
        else undo();
      }
      return;
    }
    if (mod && e.key === 'y') {
      if (isSketching()) {
        e.preventDefault();
        redo();
      }
      return;
    }
    if (mod || e.altKey) return;

    switch (e.key) {
      case ' ':
        e.preventDefault();
        setPlaying(!state.playing);
        break;
      case 'r': case 'R':
        seek(0);
        setPlaying(true);
        break;
      case 'ArrowRight': case 'ArrowLeft': case 'ArrowUp': case 'ArrowDown': {
        if (isSketching() && state.selectedPoint >= 0) {
          e.preventDefault();
          const d = e.shiftKey ? 6 : 1;
          const [dx, dy] = { ArrowRight: [d, 0], ArrowLeft: [-d, 0], ArrowUp: [0, d], ArrowDown: [0, -d] }[e.key] as Vec;
          nudgePoint(state.selectedPoint, dx, dy, !e.repeat);
          break;
        }
        if (e.key === 'ArrowUp' || e.key === 'ArrowDown') break;
        e.preventDefault();
        setPlaying(false);
        const step = (state.sim?.dt ?? 0.02) * (e.shiftKey ? 10 : 1);
        seek(state.t + (e.key === 'ArrowRight' ? step : -step));
        break;
      }
      case '+': case '=':
        zoomBy(1.4);
        break;
      case '-': case '_':
        zoomBy(1 / 1.4);
        break;
      case '0':
        fitView();
        break;
      case 'o': case 'O':
        field.rotateView();
        invalidate();
        break;
      case 'f': case 'F':
        setFollow(!state.follow);
        break;
      case 'm': case 'M':
        setMeasuring(!state.measuring);
        break;
      case 's': case 'S':
        setSnap(!state.snap);
        break;
      case 'g': case 'G':
        setGraphs(!state.graphs);
        break;
      case 'p': case 'P':
        if (state.mode === 'sim') pinRun();
        break;
      case '?':
        toggleHelp($('help').hidden);
        break;
      case 'Escape':
        if (!$('help').hidden) toggleHelp(false);
        else if (state.measuring) setMeasuring(false);
        else if (state.selectedPoint >= 0) selectPoint(-1);
        break;
      case 'Backspace': case 'Delete':
        if (isSketching() && state.sketch.points.length) {
          e.preventDefault();
          deletePoint(state.selectedPoint >= 0 ? state.selectedPoint : state.sketch.points.length - 1);
        }
        break;
      default:
        break;
    }
  });
}

// ---- path editor ------------------------------------------------------------------------------

function sketchVecs(): Vec[] {
  return state.sketch.points.map((p) => [p.x, p.y] as Vec);
}

function clonePoints(points: SketchPoint[]): SketchPoint[] {
  return points.map((p) => ({ ...p }));
}

function selectPoint(index: number): void {
  const changed = state.selectedPoint !== index;
  state.selectedPoint = index;
  if (changed) renderEditor();
  if (index >= 0) {
    $('sk-list').querySelector<HTMLElement>(`[data-index="${index}"]`)?.scrollIntoView({ block: 'nearest' });
  }
  invalidate();
}

function deletePoint(index: number): void {
  if (index < 0 || index >= state.sketch.points.length) return;
  remember();
  state.sketch.points.splice(index, 1);
  state.hover = -1;
  state.selectedPoint = state.sketch.points.length ? Math.min(index, state.sketch.points.length - 1) : -1;
  void runSketch();
}

/** Moves a point by an offset; rememberIt is false for held-down keys so one undo reverts the whole nudge. */
function nudgePoint(index: number, dx: number, dy: number, rememberIt: boolean): void {
  if (rememberIt) remember();
  const p = state.sketch.points[index];
  state.sketch.points[index] = { ...p, x: clampField(round(p.x + dx)), y: clampField(round(p.y + dy)) };
  syncEditorValues();
  scheduleSketch();
  invalidate();
}

function movePoint(index: number, to: number): void {
  const points = state.sketch.points;
  if (index < 0 || to < 0 || to >= points.length) return;
  remember();
  const [point] = points.splice(index, 1);
  points.splice(to, 0, point);
  state.selectedPoint = to;
  void runSketch();
}

function insertAfter(index: number): void {
  const points = state.sketch.points;
  const a = points[index];
  if (!a) return;
  let x: number;
  let y: number;
  if (index + 1 < points.length) {
    x = (a.x + points[index + 1].x) / 2;
    y = (a.y + points[index + 1].y) / 2;
  } else {
    // past the end: continue a foot in the same direction (or towards the center)
    const prev = points[index - 1];
    const dx = prev ? a.x - prev.x : -a.x;
    const dy = prev ? a.y - prev.y : -a.y;
    const length = Math.hypot(dx, dy) || 1;
    x = a.x + (dx / length) * 12;
    y = a.y + (dy / length) * 12;
  }
  remember();
  points.splice(index + 1, 0, { x: clampField(round(x)), y: clampField(round(y)), speed: null, lookahead: null });
  state.selectedPoint = index + 1;
  void runSketch();
}

/** Rebuilds the waypoint list and the selected point's settings. */
function renderEditor(): void {
  const list = $('sk-list');
  const points = state.sketch.points;
  setText('sk-count', points.length ? `${points.length} pt` : '');
  list.replaceChildren();
  if (!points.length) {
    list.append(el('div', 'wp-empty', 'No points yet. Click the field to place the start.'));
  }
  points.forEach((p, i) => {
    const row = el('div', `wp-row${i === state.selectedPoint ? ' selected' : ''}`);
    row.dataset.index = String(i);
    row.append(el('span', 'wp-badge', i === 0 ? 'S' : String(i)));
    for (const axis of ['x', 'y'] as const) {
      const label = el('label', 'wp-field', axis.toUpperCase());
      const input = document.createElement('input');
      input.type = 'number';
      input.step = '0.5';
      input.min = '-72';
      input.max = '72';
      input.value = String(p[axis]);
      input.dataset.axis = axis;
      input.addEventListener('focus', () => selectPoint(i));
      input.addEventListener('change', () => {
        const value = Number(input.value);
        if (!Number.isFinite(value)) return;
        remember();
        state.sketch.points[i] = { ...state.sketch.points[i], [axis]: clampField(value) };
        void runSketch();
      });
      label.append(input);
      row.append(label);
    }
    row.append(p.speed !== null || p.lookahead !== null ? el('span', 'wp-flag') : el('span'));
    const del = el('button', 'wp-del', '×') as HTMLButtonElement;
    del.title = 'Delete this point';
    del.addEventListener('click', (e) => {
      e.stopPropagation();
      deletePoint(i);
    });
    row.append(del);
    row.addEventListener('click', (e) => {
      if ((e.target as HTMLElement).tagName !== 'INPUT') selectPoint(i);
    });
    list.append(row);
  });
  renderPointDetail();
}

/** Updates the numbers in the list without rebuilding it (so a focused field keeps focus). */
function syncEditorValues(): void {
  $('sk-list').querySelectorAll<HTMLElement>('.wp-row').forEach((row) => {
    const p = state.sketch.points[Number(row.dataset.index)];
    if (!p) return;
    row.querySelectorAll<HTMLInputElement>('input').forEach((input) => {
      if (document.activeElement !== input) input.value = String(p[input.dataset.axis as 'x' | 'y']);
    });
  });
  renderPointDetail();
}

function renderPointDetail(): void {
  // when the start heading isn't set, show what it will be
  input('sk-start').placeholder = `auto ${autoStartHeading()}`;
  const i = state.selectedPoint;
  const p = state.sketch.points[i];
  $('sk-point').hidden = !p;
  if (!p) return;
  setText('sk-point-title', i === 0 ? 'Start' : `Point ${i}`);
  setText('sk-point-sub', `(${p.x}, ${p.y})`);
  const isStart = i === 0;
  const speedOn = $<HTMLInputElement>('sk-pt-speed-on');
  const lookOn = $<HTMLInputElement>('sk-pt-look-on');
  const speed = $<HTMLInputElement>('sk-pt-speed');
  const look = $<HTMLInputElement>('sk-pt-look');
  // the start isn't the end of any segment, so it has no speed / lookahead of its own
  for (const input of [speedOn, lookOn]) input.disabled = isStart;
  speedOn.checked = p.speed !== null;
  lookOn.checked = p.lookahead !== null;
  speed.disabled = p.speed === null;
  look.disabled = p.lookahead === null;
  speed.value = String(p.speed ?? state.sketch.speed);
  look.value = String(p.lookahead ?? state.sketch.lookahead);
  setText('sk-pt-speed-val', (p.speed ?? state.sketch.speed).toFixed(2));
  setText('sk-pt-look-val', `${p.lookahead ?? state.sketch.lookahead} in`);
  $<HTMLButtonElement>('sk-up').disabled = i <= 0;
  $<HTMLButtonElement>('sk-down').disabled = i >= state.sketch.points.length - 1;
}

/** Puts the path settings controls in line with state.sketch (after "Edit as sketch"). */
function syncPathSettings(): void {
  const sk = state.sketch;
  $<HTMLInputElement>('sk-name').value = sk.name;
  $<HTMLInputElement>('sk-reversed').checked = sk.reversed;
  $<HTMLInputElement>('sk-speed').value = String(sk.speed);
  $<HTMLInputElement>('sk-look').value = String(sk.lookahead);
  setText('sk-speed-val', sk.speed.toFixed(2));
  setText('sk-look-val', `${sk.lookahead} in`);
  for (const key of ['start', 'end', 'fixed'] as const) {
    const value = sk[`${key}Heading`];
    $<HTMLInputElement>(`sk-${key}-on`).checked = value !== null;
    const input = $<HTMLInputElement>(`sk-${key}`);
    input.disabled = value === null;
    if (value !== null) input.value = String(value);
  }
}

function wireEditor(): void {
  $<HTMLInputElement>('sk-name').addEventListener('input', (e) => {
    state.sketch.name = (e.target as HTMLInputElement).value;
    scheduleSketch(400);
  });

  const wireOverride = (key: 'speed' | 'lookahead', toggleId: string, sliderId: string) => {
    $<HTMLInputElement>(toggleId).addEventListener('change', (e) => {
      const i = state.selectedPoint;
      if (i < 0) return;
      remember();
      const on = (e.target as HTMLInputElement).checked;
      state.sketch.points[i] = { ...state.sketch.points[i], [key]: on ? Number($<HTMLInputElement>(sliderId).value) : null };
      renderEditor();
      scheduleSketch(0);
    });
    $<HTMLInputElement>(sliderId).addEventListener('input', (e) => {
      const i = state.selectedPoint;
      if (i < 0) return;
      state.sketch.points[i] = { ...state.sketch.points[i], [key]: Number((e.target as HTMLInputElement).value) };
      renderPointDetail();
      scheduleSketch();
    });
    $<HTMLInputElement>(sliderId).addEventListener('pointerdown', () => remember());
  };
  wireOverride('speed', 'sk-pt-speed-on', 'sk-pt-speed');
  wireOverride('lookahead', 'sk-pt-look-on', 'sk-pt-look');

  $('sk-up').addEventListener('click', () => movePoint(state.selectedPoint, state.selectedPoint - 1));
  $('sk-down').addEventListener('click', () => movePoint(state.selectedPoint, state.selectedPoint + 1));
  $('sk-insert').addEventListener('click', () => insertAfter(state.selectedPoint));
  $('sk-delete').addEventListener('click', () => deletePoint(state.selectedPoint));

  for (const key of ['start', 'end', 'fixed'] as const) {
    const field = `${key}Heading` as const;
    const toggle = $<HTMLInputElement>(`sk-${key}-on`);
    const input = $<HTMLInputElement>(`sk-${key}`);
    toggle.addEventListener('change', () => {
      input.disabled = !toggle.checked;
      if (toggle.checked && input.value === '') {
        input.value = String(key === 'start' ? autoStartHeading() : 0);
      }
      state.sketch[field] = toggle.checked ? Number(input.value) || 0 : null;
      if (key === 'fixed' && toggle.checked && !state.holonomic) {
        toast('Strafing needs a mecanum drive, so this path will simulate as mecanum');
      }
      scheduleSketch(0);
    });
    input.addEventListener('change', () => {
      if (!toggle.checked) return;
      state.sketch[field] = Number(input.value) || 0;
      scheduleSketch(0);
    });
  }
  input('sk-start').disabled = true;
  input('sk-end').disabled = true;
  input('sk-fixed').disabled = true;
}

/** What the start heading is when it isn't set: facing the first waypoint. */
function autoStartHeading(): number {
  const [a, b] = state.sketch.points;
  if (!a || !b) return 0;
  let heading = (Math.atan2(b.y - a.y, b.x - a.x) * 180) / Math.PI + (state.sketch.reversed ? 180 : 0);
  heading = ((heading + 540) % 360) - 180;
  return Math.round(heading);
}

function input(id: string): HTMLInputElement {
  return $<HTMLInputElement>(id);
}

function round(v: number): number {
  return Math.round(v * 2) / 2;
}

function clampField(v: number): number {
  return Math.max(-72, Math.min(72, v));
}

// ---- helpers ----------------------------------------------------------------------------------

function isSketching(): boolean {
  return state.mode === 'sim' && state.selected === SKETCH;
}

/**
 * Where a point placed at the mouse goes, and shows what it snapped to. index is the sketch point
 * being dragged, 'new' for the next point to add, or null for the ruler. Snap off or Option/Alt
 * held places it freely.
 */
function snapAt(raw: Vec, index: number | 'new' | null, free: boolean): Vec {
  if (free || !state.snap) {
    state.snapGuides = [];
    return freePoint(raw);
  }
  const points = isSketching() ? sketchVecs() : [];
  let neighbors: Vec[] = [];
  let others = points;
  if (index === 'new') {
    neighbors = points.slice(-1);
    others = points.slice(0, -1);
  } else if (index !== null) {
    neighbors = [points[index - 1], points[index + 1]].filter((p): p is Vec => p !== undefined);
    others = points.filter((_, i) => Math.abs(i - index) > 1);
  }
  const result = snapPoint(raw, {
    reach: Math.min(3, 9 / field.ppi),
    neighbors,
    others,
    robotSize: tuning.robotSize(),
  });
  state.snapGuides = result.guides;
  return result.point;
}

function trimInches(v: number): string {
  return Number.isInteger(v) ? String(v) : v.toFixed(1);
}

function framePose(frame: Frame): Pose {
  return [frame[F.X] as number, frame[F.Y] as number, frame[F.H] as number];
}

function lerpPose(a: Pose, b: Pose, t: number): Pose {
  let dh = b[2] - a[2];
  dh = Math.atan2(Math.sin(dh), Math.cos(dh));
  return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + dh * t];
}

async function copy(text: string, message: string): Promise<void> {
  try {
    await navigator.clipboard.writeText(text);
    toast(message);
  } catch {
    toast("Couldn't reach the clipboard", true);
  }
}

function el(tag: string, className = '', text = ''): HTMLElement {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text) node.textContent = text;
  return node;
}

function setText(id: string, text: string): void {
  const node = $(id);
  if (node.textContent !== text) node.textContent = text;
}

function sleep(ms: number): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function loadPref(key: string, fallback: boolean): boolean {
  try {
    const value = localStorage.getItem(`fieldviewer.${key}`);
    return value === null ? fallback : value === 'true';
  } catch {
    return fallback;
  }
}

function savePref(key: string, value: boolean): void {
  try {
    localStorage.setItem(`fieldviewer.${key}`, String(value));
  } catch {
    // preferences just won't stick
  }
}

function showHint(text: string): void {
  const hint = $('hint');
  hint.textContent = text;
  hint.hidden = false;
}

let toastTimer = 0;
function toast(text: string, error = false): void {
  const node = $('toast');
  node.textContent = text;
  node.classList.toggle('error', error);
  node.classList.add('show');
  window.clearTimeout(toastTimer);
  toastTimer = window.setTimeout(() => node.classList.remove('show'), 2600);
}

void init();
