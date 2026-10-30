import type { SnapGuide } from './snap.js';
import type { FieldInfo, Pose, RouteInfo, Target, Vec } from './types.js';

/** Which overlays to draw. */
export interface Layers {
  lookahead: boolean;
  trail: boolean;
  route: boolean;
  keepOut: boolean;
  grid: boolean;
}

/** A frozen trail to compare against (the "Pin run" button). */
export interface Ghost {
  trail: Vec[];
  label: string;
}

/** Everything drawn in one frame. */
export interface Scene {
  route: RouteInfo | null;
  path: Vec[];
  trail: Vec[];
  pose: Pose | null;
  follow: Target | null;
  pointTo: Target | null;
  clipped: Vec | null;
  colliding: boolean;
  sketch: Vec[] | null;
  /** Point under the mouse or being dragged. */
  sketchActive: number;
  /** Point selected in the path editor. */
  sketchSelected: number;
  mouse: Vec | null;
  ruler: [Vec, Vec] | null;
  /** What the point being placed or dragged snapped to. */
  snapGuides: SnapGuide[];
  /** Where a click would add the next point. */
  snapPreview: Vec | null;
  ghost: Ghost | null;
  robotSize: [number, number];
  layers: Layers;
}

const COLORS = {
  background: '#05070a',
  path: '#22d3ee',
  follow: '#22d3ee',
  pointTo: '#f8fafc',
  trailOld: [245, 158, 11] as const,
  trailNew: [253, 224, 71] as const,
  robot: '#22d3ee',
  robotHit: '#f87171',
  keepOut: 'rgba(248, 113, 113, 0.55)',
  sketch: '#f472b6',
  ruler: '#a3e635',
  snap: 'rgba(190, 242, 100, 0.9)',
  ghost: 'rgba(226, 232, 240, 0.55)',
  BLUE: '#38bdf8',
  RED: '#fb7185',
  NONE: '#cbd5e1',
};

const FONT = 'Inter, ui-sans-serif, system-ui, -apple-system, "Segoe UI", sans-serif';
const MIN_ZOOM = 1;
const MAX_ZOOM = 10;

/**
 * Draws the field on a canvas and converts between screen and field coordinates.
 *
 * The camera has a zoom, a center (in field inches) and a rotation in quarter turns. Every
 * field-to-screen conversion goes through {@link toScreen}, so overlays, hit testing and
 * the mouse readout all agree however the view is zoomed or turned.
 */
export class FieldView {
  private readonly ctx: CanvasRenderingContext2D;
  private readonly image = new Image();
  private imageReady = false;
  private width = 0;
  private height = 0;
  private fit = 0;

  zoom = 1;
  center: Vec = [0, 0];
  quarterTurns = 0;

  /** Called when the field image finishes loading or the canvas is resized. */
  onChange: () => void = () => {};

  constructor(private readonly canvas: HTMLCanvasElement, private readonly field: FieldInfo) {
    const ctx = canvas.getContext('2d');
    if (!ctx) {
      throw new Error('canvas 2d not supported');
    }
    this.ctx = ctx;
    this.image.onload = () => {
      this.imageReady = true;
      this.onChange();
    };
    this.image.src = '/field-dark.png';
    new ResizeObserver(() => {
      this.resize();
      this.onChange();
    }).observe(canvas.parentElement ?? canvas);
    this.resize();
  }

  // ---- camera ---------------------------------------------------------------------------------

  /** Screen pixels per field inch. */
  get ppi(): number {
    return (this.fit / (2 * this.field.half)) * this.zoom;
  }

  private get viewAngle(): number {
    return (this.quarterTurns * Math.PI) / 2;
  }

  toScreen(x: number, y: number): Vec {
    const [rx, ry] = rotate(x - this.center[0], y - this.center[1], this.quarterTurns);
    return [this.width / 2 + rx * this.ppi, this.height / 2 - ry * this.ppi];
  }

  /** Field point under a screen position (canvas pixels), whether or not it's on the field. */
  private screenToField(sx: number, sy: number): Vec {
    const [dx, dy] = rotate((sx - this.width / 2) / this.ppi, -(sy - this.height / 2) / this.ppi, -this.quarterTurns);
    return [dx + this.center[0], dy + this.center[1]];
  }

  private local(clientX: number, clientY: number): Vec {
    const rect = this.canvas.getBoundingClientRect();
    return [clientX - rect.left, clientY - rect.top];
  }

  /** Field coordinates under a mouse event, or null when outside the field. */
  toField(clientX: number, clientY: number): Vec | null {
    const [sx, sy] = this.local(clientX, clientY);
    const p = this.screenToField(sx, sy);
    const half = this.field.half;
    return Math.abs(p[0]) <= half && Math.abs(p[1]) <= half ? p : null;
  }

  /** Zooms by a factor, keeping the field point under the mouse where it is. */
  zoomAt(clientX: number, clientY: number, factor: number): void {
    const [sx, sy] = this.local(clientX, clientY);
    const before = this.screenToField(sx, sy);
    this.zoom = clamp(this.zoom * factor, MIN_ZOOM, MAX_ZOOM);
    const after = this.screenToField(sx, sy);
    this.center = [this.center[0] + before[0] - after[0], this.center[1] + before[1] - after[1]];
    this.clampCenter();
  }

  /** Zooms around the middle of the view. */
  zoomBy(factor: number): void {
    const rect = this.canvas.getBoundingClientRect();
    this.zoomAt(rect.left + this.width / 2, rect.top + this.height / 2, factor);
  }

  /** Moves the view by a mouse drag (screen pixels). */
  panBy(dxPixels: number, dyPixels: number): void {
    const [dx, dy] = rotate(dxPixels / this.ppi, -dyPixels / this.ppi, -this.quarterTurns);
    this.center = [this.center[0] - dx, this.center[1] - dy];
    this.clampCenter();
  }

  lookAt(point: Vec): void {
    this.center = [point[0], point[1]];
    this.clampCenter();
  }

  fitField(): void {
    this.zoom = 1;
    this.center = [0, 0];
  }

  rotateView(): void {
    this.quarterTurns = (this.quarterTurns + 1) % 4;
  }

  private clampCenter(): void {
    // at 1x the field is centered; zoomed in you can pan until an edge reaches the middle
    const limit = this.zoom <= 1.0001 ? 0 : this.field.half;
    this.center = [clamp(this.center[0], -limit, limit), clamp(this.center[1], -limit, limit)];
  }

  /** Index of the point within reach of the mouse, or -1. */
  pick(points: Vec[], clientX: number, clientY: number, reachPx = 12): number {
    const [mx, my] = this.local(clientX, clientY);
    let best = -1;
    let bestDistance = reachPx;
    points.forEach(([x, y], i) => {
      const [sx, sy] = this.toScreen(x, y);
      const d = Math.hypot(sx - mx, sy - my);
      if (d <= bestDistance) {
        best = i;
        bestDistance = d;
      }
    });
    return best;
  }

  private resize(): void {
    const parent = this.canvas.parentElement ?? this.canvas;
    this.width = parent.clientWidth;
    this.height = parent.clientHeight;
    const dpr = window.devicePixelRatio || 1;
    this.canvas.style.width = `${this.width}px`;
    this.canvas.style.height = `${this.height}px`;
    this.canvas.width = Math.round(this.width * dpr);
    this.canvas.height = Math.round(this.height * dpr);
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.fit = Math.max(100, Math.min(this.width, this.height) - 32);
  }

  // ---- drawing --------------------------------------------------------------------------------

  draw(scene: Scene): void {
    const { ctx } = this;
    ctx.clearRect(0, 0, this.width, this.height);
    this.drawField(scene.layers.grid);
    if (scene.layers.keepOut) {
      this.drawKeepOut();
    }
    if (scene.layers.route && scene.route) {
      this.drawRoute(scene.route, scene.robotSize);
    }
    if (scene.ghost) {
      this.drawGhost(scene.ghost);
    }
    if (scene.path.length >= 2) {
      this.polyline(scene.path, COLORS.path, Math.max(2.5, this.ppi * 0.55), 14);
    }
    if (scene.layers.trail) {
      this.drawTrail(scene.trail);
    }
    if (scene.sketch) {
      this.drawSketch(scene.sketch, scene.sketchActive, scene.sketchSelected);
    }
    this.drawSnap(scene.snapGuides, scene.snapPreview);
    if (scene.pose) {
      if (scene.layers.lookahead) {
        this.drawLookahead(scene);
      }
      this.drawRobot(scene.pose, scene.robotSize, scene.colliding);
    }
    if (scene.ruler) {
      this.drawRuler(scene.ruler[0], scene.ruler[1]);
    }
    if (scene.mouse) {
      this.drawCrosshair(scene.mouse);
    }
    this.drawCompass();
  }

  /** Runs fn with the canvas set up to draw in field inches (y up), honoring the camera. */
  private inFieldSpace(fn: () => void): void {
    const { ctx } = this;
    ctx.save();
    ctx.translate(this.width / 2, this.height / 2);
    ctx.scale(this.ppi, -this.ppi);
    ctx.rotate(this.viewAngle);
    ctx.translate(-this.center[0], -this.center[1]);
    fn();
    ctx.restore();
  }

  private drawField(grid: boolean): void {
    const { ctx } = this;
    const half = this.field.half;
    const corner = 10 / this.ppi;

    this.inFieldSpace(() => {
      ctx.save();
      ctx.shadowColor = 'rgba(34, 211, 238, 0.10)';
      ctx.shadowBlur = 40;
      ctx.fillStyle = COLORS.background;
      ctx.beginPath();
      ctx.roundRect(-half, -half, 2 * half, 2 * half, corner);
      ctx.fill();
      ctx.restore();

      ctx.save();
      ctx.beginPath();
      ctx.roundRect(-half, -half, 2 * half, 2 * half, corner);
      ctx.clip();
      if (this.imageReady) {
        ctx.save();
        ctx.globalAlpha = 0.92;
        ctx.scale(1, -1); // images are drawn top-down, the field is y-up
        ctx.drawImage(this.image, -half, -half, 2 * half, 2 * half);
        ctx.restore();
      }

      // soft alliance glow at each end: blue at +y, red at -y
      const blue = ctx.createLinearGradient(0, half, 0, half * 0.3);
      blue.addColorStop(0, 'rgba(56, 189, 248, 0.10)');
      blue.addColorStop(1, 'rgba(56, 189, 248, 0)');
      ctx.fillStyle = blue;
      ctx.fillRect(-half, half * 0.3, 2 * half, half * 0.7);
      const red = ctx.createLinearGradient(0, -half, 0, -half * 0.3);
      red.addColorStop(0, 'rgba(251, 113, 133, 0.10)');
      red.addColorStop(1, 'rgba(251, 113, 133, 0)');
      ctx.fillStyle = red;
      ctx.fillRect(-half, -half, 2 * half, half * 0.7);

      if (grid) {
        // finer grid as you zoom in: 6in normally, 1in when close
        const step = this.ppi > 22 ? 1 : this.ppi > 9 ? 3 : 6;
        ctx.lineWidth = 1 / this.ppi;
        for (let v = -half; v <= half + 1e-6; v += step) {
          const tile = Math.abs(v % 24) < 1e-6;
          const six = Math.abs(v % 6) < 1e-6;
          ctx.strokeStyle = tile ? 'rgba(255, 255, 255, 0.11)' : six ? 'rgba(255, 255, 255, 0.045)' : 'rgba(255, 255, 255, 0.025)';
          ctx.beginPath();
          ctx.moveTo(v, -half);
          ctx.lineTo(v, half);
          ctx.moveTo(-half, v);
          ctx.lineTo(half, v);
          ctx.stroke();
        }
      }
      ctx.restore();

      ctx.strokeStyle = 'rgba(255, 255, 255, 0.10)';
      ctx.lineWidth = 1 / this.ppi;
      ctx.beginPath();
      ctx.roundRect(-half, -half, 2 * half, 2 * half, corner);
      ctx.stroke();
    });
  }

  private drawKeepOut(): void {
    const { ctx } = this;
    const [minX, minY, maxX, maxY] = this.field.center;
    const corners: Vec[] = [[minX, minY], [maxX, minY], [maxX, maxY], [minX, maxY]];
    ctx.save();
    ctx.beginPath();
    corners.forEach(([x, y], i) => {
      const [sx, sy] = this.toScreen(x, y);
      if (i === 0) ctx.moveTo(sx, sy);
      else ctx.lineTo(sx, sy);
    });
    ctx.closePath();
    ctx.fillStyle = 'rgba(248, 113, 113, 0.04)';
    ctx.fill();
    ctx.setLineDash([6, 6]);
    ctx.strokeStyle = COLORS.keepOut;
    ctx.lineWidth = 1.25;
    ctx.stroke();
    ctx.restore();
  }

  private drawRoute(route: RouteInfo, robotSize: [number, number]): void {
    const { ctx } = this;
    const color = COLORS[route.alliance];
    let at: Vec = [route.start[0], route.start[1]];
    let waypoint = 0;

    this.footprint(route.start, robotSize, color, 0.5, [4, 4]);
    for (const step of route.steps) {
      if (step.type === 'follow') {
        const points: Vec[] = [at, ...step.points];
        ctx.save();
        ctx.setLineDash([Math.max(4, this.ppi * 2.2), Math.max(3, this.ppi * 1.8)]);
        ctx.globalAlpha = 0.6;
        this.polyline(points, color, Math.max(1.25, this.ppi * 0.28), 0);
        ctx.restore();
        for (const p of step.points) {
          waypoint += 1;
          this.waypoint(p, color, String(waypoint), false);
        }
        at = points[points.length - 1];
      } else if (step.type === 'marker') {
        this.tag(at, `${step.name} · ${step.seconds.toFixed(1)}s`, '#fbbf24');
      } else {
        this.tag(at, `turn ${Math.round((step.heading * 180) / Math.PI)}°`, '#a78bfa');
      }
    }
  }

  private drawGhost(ghost: Ghost): void {
    if (ghost.trail.length < 2) return;
    const { ctx } = this;
    ctx.save();
    ctx.setLineDash([2, 5]);
    this.polyline(ghost.trail, COLORS.ghost, 2, 0);
    ctx.restore();
    const end = ghost.trail[ghost.trail.length - 1];
    this.tag(end, `pinned · ${ghost.label}`, '#e2e8f0');
  }

  private drawSketch(points: Vec[], active: number, selected: number): void {
    if (points.length >= 2) {
      this.polyline(points, COLORS.sketch, Math.max(2, this.ppi * 0.4), 10);
    }
    // "+" handles halfway along each segment: click or drag one to insert a point there
    for (const mid of midpoints(points)) {
      this.insertHandle(mid);
    }
    points.forEach((p, i) => this.waypoint(p, COLORS.sketch, i === 0 ? 'S' : String(i), i === active, i === selected));
  }

  /** Dashed lines to whatever the point snapped to, and a ghost of the next point to add. */
  private drawSnap(guides: SnapGuide[], preview: Vec | null): void {
    const { ctx } = this;
    ctx.save();
    ctx.strokeStyle = COLORS.snap;
    ctx.lineWidth = 1.5;
    ctx.setLineDash([6, 4]);
    for (const guide of guides) {
      const [ax, ay] = this.toScreen(guide.a[0], guide.a[1]);
      const [bx, by] = this.toScreen(guide.b[0], guide.b[1]);
      ctx.beginPath();
      ctx.moveTo(ax, ay);
      ctx.lineTo(bx, by);
      ctx.stroke();
    }
    ctx.setLineDash([]);
    if (preview) {
      const [x, y] = this.toScreen(preview[0], preview[1]);
      ctx.strokeStyle = 'rgba(244, 114, 182, 0.8)';
      ctx.fillStyle = 'rgba(244, 114, 182, 0.25)';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(x, y, 6, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    }
    ctx.restore();
    // labels sit under the snapped point, one per kind
    let offset = 24;
    const labels = new Set<string>();
    for (const guide of guides) {
      if (!guide.label || labels.has(guide.label)) continue;
      labels.add(guide.label);
      const [x, y] = this.toScreen(guide.at[0], guide.at[1]);
      this.pill(x, y + offset, guide.label, COLORS.snap);
      offset += 24;
    }
  }

  private insertHandle(p: Vec): void {
    const { ctx } = this;
    const [x, y] = this.toScreen(p[0], p[1]);
    const r = 6;
    ctx.save();
    ctx.fillStyle = 'rgba(11, 16, 22, 0.9)';
    ctx.strokeStyle = 'rgba(244, 114, 182, 0.7)';
    ctx.lineWidth = 1.25;
    ctx.beginPath();
    ctx.arc(x, y, r, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.strokeStyle = COLORS.sketch;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(x - 3, y);
    ctx.lineTo(x + 3, y);
    ctx.moveTo(x, y - 3);
    ctx.lineTo(x, y + 3);
    ctx.stroke();
    ctx.restore();
  }

  /** Index i of the segment (points[i] to points[i + 1]) whose "+" handle is under the mouse, or -1. */
  pickMidpoint(points: Vec[], clientX: number, clientY: number): number {
    return this.pick(midpoints(points), clientX, clientY, 9);
  }

  /** Where the robot has been: a line that fades in from amber to yellow. */
  private drawTrail(trail: Vec[]): void {
    if (trail.length < 2) {
      return;
    }
    const { ctx } = this;
    ctx.save();
    ctx.lineCap = 'round';
    ctx.lineWidth = Math.max(2, this.ppi * 0.45);
    const n = trail.length;
    let [px, py] = this.toScreen(trail[0][0], trail[0][1]);
    for (let i = 1; i < n; i++) {
      const t = i / (n - 1);
      const [r0, g0, b0] = COLORS.trailOld;
      const [r1, g1, b1] = COLORS.trailNew;
      const [sx, sy] = this.toScreen(trail[i][0], trail[i][1]);
      ctx.strokeStyle = `rgba(${lerp(r0, r1, t)}, ${lerp(g0, g1, t)}, ${lerp(b0, b1, t)}, ${0.15 + 0.85 * t})`;
      ctx.beginPath();
      ctx.moveTo(px, py);
      ctx.lineTo(sx, sy);
      ctx.stroke();
      px = sx;
      py = sy;
    }
    ctx.restore();
  }

  private drawLookahead(scene: Scene): void {
    const { ctx } = this;
    const pose = scene.pose as Pose;
    const [rx, ry] = this.toScreen(pose[0], pose[1]);

    if (scene.follow && scene.follow[2] > 0) {
      const r = scene.follow[2] * this.ppi;
      ctx.save();
      ctx.fillStyle = 'rgba(34, 211, 238, 0.06)';
      ctx.beginPath();
      ctx.arc(rx, ry, r, 0, Math.PI * 2);
      ctx.fill();
      ctx.setLineDash([5, 5]);
      ctx.strokeStyle = 'rgba(34, 211, 238, 0.55)';
      ctx.lineWidth = 1.25;
      ctx.stroke();
      ctx.restore();
    }
    if (scene.clipped) {
      this.dot(scene.clipped, 3, 'rgba(148, 163, 184, 0.9)');
    }
    if (scene.pointTo) {
      const [px, py] = this.toScreen(scene.pointTo[0], scene.pointTo[1]);
      ctx.save();
      ctx.strokeStyle = 'rgba(248, 250, 252, 0.35)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(rx, ry);
      ctx.lineTo(px, py);
      ctx.stroke();
      ctx.strokeStyle = COLORS.pointTo;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(px, py, Math.max(7, this.ppi * 1.9), 0, Math.PI * 2);
      ctx.stroke();
      ctx.restore();
    }
    if (scene.follow) {
      const [fx, fy] = this.toScreen(scene.follow[0], scene.follow[1]);
      ctx.save();
      ctx.strokeStyle = COLORS.follow;
      ctx.lineWidth = 1.75;
      ctx.shadowColor = COLORS.follow;
      ctx.shadowBlur = 8;
      ctx.beginPath();
      ctx.moveTo(rx, ry);
      ctx.lineTo(fx, fy);
      ctx.stroke();
      ctx.fillStyle = COLORS.follow;
      ctx.beginPath();
      ctx.arc(fx, fy, Math.max(4, this.ppi * 1.1), 0, Math.PI * 2);
      ctx.fill();
      ctx.shadowBlur = 0;
      ctx.strokeStyle = '#05070a';
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.restore();
    }
  }

  private drawRobot(pose: Pose, robotSize: [number, number], colliding: boolean): void {
    const { ctx } = this;
    const [length, width] = robotSize;
    const ppi = this.ppi;
    const l = (length / 2) * ppi;
    const w = (width / 2) * ppi;
    const color = colliding ? COLORS.robotHit : COLORS.robot;
    const [sx, sy] = this.toScreen(pose[0], pose[1]);

    ctx.save();
    ctx.translate(sx, sy);
    ctx.rotate(-(pose[2] + this.viewAngle));

    // body
    ctx.shadowColor = color;
    ctx.shadowBlur = 16;
    const body = ctx.createLinearGradient(-l, 0, l, 0);
    body.addColorStop(0, hexAlpha(color, 0.10));
    body.addColorStop(1, hexAlpha(color, 0.24));
    ctx.fillStyle = body;
    ctx.beginPath();
    ctx.roundRect(-l, -w, 2 * l, 2 * w, ppi * 2.2);
    ctx.fill();
    ctx.shadowBlur = 10;
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.shadowBlur = 0;

    // wheels
    ctx.fillStyle = hexAlpha(color, 0.55);
    const wl = length * 0.24 * ppi;
    const ww = width * 0.13 * ppi;
    for (const ix of [-1, 1]) {
      for (const iy of [-1, 1]) {
        const cx = ix * (l - wl / 2 - ppi * 1.2);
        const cy = iy * (w - ww / 2 - ppi * 0.9);
        ctx.beginPath();
        ctx.roundRect(cx - wl / 2, cy - ww / 2, wl, ww, ww / 3);
        ctx.fill();
      }
    }

    // front bumper and heading arrow, so it's obvious which way it faces
    ctx.strokeStyle = '#f8fafc';
    ctx.lineWidth = 2.5;
    ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(l - ppi * 0.6, -w * 0.55);
    ctx.lineTo(l - ppi * 0.6, w * 0.55);
    ctx.stroke();
    ctx.fillStyle = '#f8fafc';
    ctx.beginPath();
    ctx.moveTo(l * 0.62, 0);
    ctx.lineTo(l * 0.05, -w * 0.32);
    ctx.lineTo(l * 0.18, 0);
    ctx.lineTo(l * 0.05, w * 0.32);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  private drawRuler(a: Vec, b: Vec): void {
    const { ctx } = this;
    const [ax, ay] = this.toScreen(a[0], a[1]);
    const [bx, by] = this.toScreen(b[0], b[1]);
    const dx = b[0] - a[0];
    const dy = b[1] - a[1];
    const distance = Math.hypot(dx, dy);
    const angle = (Math.atan2(dy, dx) * 180) / Math.PI;

    ctx.save();
    ctx.strokeStyle = COLORS.ruler;
    ctx.fillStyle = COLORS.ruler;
    ctx.lineWidth = 2;
    ctx.shadowColor = COLORS.ruler;
    ctx.shadowBlur = 8;
    ctx.beginPath();
    ctx.moveTo(ax, ay);
    ctx.lineTo(bx, by);
    ctx.stroke();
    ctx.shadowBlur = 0;
    for (const [x, y] of [[ax, ay], [bx, by]]) {
      ctx.beginPath();
      ctx.arc(x, y, 4, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.restore();

    const label = `${distance.toFixed(1)} in   ${angle.toFixed(1)}°   Δx ${dx.toFixed(1)}  Δy ${dy.toFixed(1)}`;
    this.pill((ax + bx) / 2, (ay + by) / 2 - 18, label, COLORS.ruler);
  }

  private drawCrosshair(mouse: Vec): void {
    const { ctx } = this;
    const half = this.field.half;
    const segments: [Vec, Vec][] = [
      [[mouse[0], -half], [mouse[0], half]],
      [[-half, mouse[1]], [half, mouse[1]]],
    ];
    ctx.save();
    ctx.strokeStyle = 'rgba(148, 163, 184, 0.25)';
    ctx.lineWidth = 1;
    ctx.setLineDash([3, 4]);
    ctx.beginPath();
    for (const [p, q] of segments) {
      const [x1, y1] = this.toScreen(p[0], p[1]);
      const [x2, y2] = this.toScreen(q[0], q[1]);
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
    }
    ctx.stroke();
    ctx.restore();
  }

  /** Little +x / +y axes in the corner so you can tell which way the field is turned. */
  private drawCompass(): void {
    const { ctx } = this;
    const ox = 34;
    const oy = this.height - 34;
    const length = 13;
    const axes: [Vec, string, string][] = [
      [[1, 0], '+X', '#fb7185'],
      [[0, 1], '+Y', '#4ade80'],
    ];
    ctx.save();
    ctx.fillStyle = 'rgba(13, 17, 23, 0.8)';
    ctx.beginPath();
    ctx.arc(ox, oy, 30, 0, Math.PI * 2);
    ctx.fill();
    ctx.font = `700 10px ${FONT}`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    for (const [[x, y], label, color] of axes) {
      const [rx, ry] = rotate(x, y, this.quarterTurns);
      const ex = ox + rx * length;
      const ey = oy - ry * length;
      ctx.strokeStyle = color;
      ctx.fillStyle = color;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(ox, oy);
      ctx.lineTo(ex, ey);
      ctx.stroke();
      ctx.fillText(label, ox + rx * (length + 9), oy - ry * (length + 9));
    }
    ctx.restore();
  }

  // ---- shapes ---------------------------------------------------------------------------------

  private polyline(points: Vec[], color: string, width: number, glow: number): void {
    const { ctx } = this;
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    if (glow > 0) {
      ctx.shadowColor = color;
      ctx.shadowBlur = glow;
    }
    ctx.beginPath();
    points.forEach(([x, y], i) => {
      const [sx, sy] = this.toScreen(x, y);
      if (i === 0) ctx.moveTo(sx, sy);
      else ctx.lineTo(sx, sy);
    });
    ctx.stroke();
    ctx.restore();
  }

  private waypoint(p: Vec, color: string, label: string, active: boolean, selected = false): void {
    const { ctx } = this;
    const r = Math.min(16, Math.max(8.5, this.ppi * 1.9)) * (active || selected ? 1.25 : 1);
    const [x, y] = this.toScreen(p[0], p[1]);
    ctx.save();
    ctx.fillStyle = selected ? color : '#0b1016';
    ctx.strokeStyle = selected ? '#f8fafc' : color;
    ctx.lineWidth = active || selected ? 2.5 : 1.5;
    if (active || selected) {
      ctx.shadowColor = color;
      ctx.shadowBlur = 14;
    }
    ctx.beginPath();
    ctx.arc(x, y, r, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.shadowBlur = 0;
    ctx.fillStyle = selected ? '#0b1016' : color;
    ctx.font = `600 ${Math.round(r * 1.05)}px ${FONT}`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(label, x, y + 0.5);
    ctx.restore();
  }

  private tag(at: Vec, text: string, color: string): void {
    const [x, y] = this.toScreen(at[0], at[1]);
    this.pill(x + 14 + this.measure(text) / 2, y - 22, text, color);
  }

  private measure(text: string): number {
    this.ctx.save();
    this.ctx.font = `600 11px ${FONT}`;
    const w = this.ctx.measureText(text).width + 14;
    this.ctx.restore();
    return w;
  }

  /** A rounded label centered on (x, y). */
  private pill(x: number, y: number, text: string, color: string): void {
    const { ctx } = this;
    const w = this.measure(text);
    ctx.save();
    ctx.font = `600 11px ${FONT}`;
    ctx.fillStyle = 'rgba(11, 16, 22, 0.92)';
    ctx.strokeStyle = color;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.roundRect(x - w / 2, y - 10, w, 20, 10);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = color;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, x, y + 0.5);
    ctx.restore();
  }

  private footprint(pose: Pose, robotSize: [number, number], color: string, alpha: number, dash: number[]): void {
    const { ctx } = this;
    const [length, width] = robotSize;
    const [sx, sy] = this.toScreen(pose[0], pose[1]);
    ctx.save();
    ctx.translate(sx, sy);
    ctx.rotate(-(pose[2] + this.viewAngle));
    ctx.globalAlpha = alpha;
    ctx.setLineDash(dash);
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.25;
    ctx.beginPath();
    ctx.roundRect((-length / 2) * this.ppi, (-width / 2) * this.ppi, length * this.ppi, width * this.ppi, this.ppi * 2.2);
    ctx.stroke();
    ctx.restore();
  }

  private dot(p: Vec, r: number, color: string): void {
    const [x, y] = this.toScreen(p[0], p[1]);
    this.ctx.fillStyle = color;
    this.ctx.beginPath();
    this.ctx.arc(x, y, r, 0, Math.PI * 2);
    this.ctx.fill();
  }
}

/** Halfway point of each segment. */
function midpoints(points: Vec[]): Vec[] {
  const mids: Vec[] = [];
  for (let i = 0; i + 1 < points.length; i++) {
    mids.push([(points[i][0] + points[i + 1][0]) / 2, (points[i][1] + points[i + 1][1]) / 2]);
  }
  return mids;
}

/** Rotates a vector by a number of quarter turns counter-clockwise. */
function rotate(x: number, y: number, quarterTurns: number): Vec {
  switch (((quarterTurns % 4) + 4) % 4) {
    case 1: return [-y, x];
    case 2: return [-x, -y];
    case 3: return [y, -x];
    default: return [x, y];
  }
}

function clamp(v: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, v));
}

function lerp(a: number, b: number, t: number): number {
  return Math.round(a + (b - a) * t);
}

function hexAlpha(hex: string, alpha: number): string {
  const n = parseInt(hex.slice(1), 16);
  return `rgba(${(n >> 16) & 255}, ${(n >> 8) & 255}, ${n & 255}, ${alpha})`;
}
