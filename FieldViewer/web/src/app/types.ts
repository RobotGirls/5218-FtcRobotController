// Shapes of the JSON the Java backend (WebServer.java) sends. Field units are inches and
// radians in Road Runner's frame: origin at the field center, +x right, +y up.

export type Vec = [number, number];
export type Pose = [number, number, number];
/** x, y, lookahead radius */
export type Target = [number, number, number];
export type Alliance = 'RED' | 'BLUE' | 'NONE';

export interface FollowStep {
  type: 'follow';
  name: string;
  reversed: boolean;
  endHeading: number | null;
  fixedHeading: number | null;
  speed: number;
  lookahead: number;
  points: Vec[];
}

export interface MarkerStep {
  type: 'marker';
  name: string;
  seconds: number;
}

export interface TurnStep {
  type: 'turn';
  heading: number;
}

export type Step = FollowStep | MarkerStep | TurnStep;

export interface RouteInfo {
  name: string;
  description: string;
  alliance: Alliance;
  group: string;
  requiresHolonomic: boolean;
  start: Pose;
  steps: Step[];
}

export interface FieldInfo {
  half: number;
  /** minX, minY, maxX, maxY of the central structure */
  center: [number, number, number, number];
  /** robot length, width */
  robot: [number, number];
}

export interface RoutesResponse {
  routes: RouteInfo[];
  field: FieldInfo;
}

/**
 * One simulation timeline entry:
 * [t, x, y, h, fx, fy, fr, px, py, pr, cx, cy, fwd, left, turn, path, step, status, colliding,
 *  speed (in/s), turnRate (rad/s)]
 */
export type Frame = (number | null)[];

export const F = {
  T: 0, X: 1, Y: 2, H: 3,
  FX: 4, FY: 5, FR: 6,
  PX: 7, PY: 8, PR: 9,
  CX: 10, CY: 11,
  FWD: 12, LEFT: 13, TURN: 14,
  PATH: 15, STEP: 16, STATUS: 17, COLLIDE: 18,
  SPEED: 19, TURN_RATE: 20,
} as const;

export interface SimSummary {
  duration: number;
  settled: number;
  finished: boolean;
  maxOffPath: number;
  centerContact: number;
  timedOut: string[];
  end: Pose;
}

export interface SimResult {
  route: RouteInfo;
  /** For a replayed recording: the route the robot was running, to draw underneath. */
  planned?: RouteInfo;
  holonomic: boolean;
  dt: number;
  summary: SimSummary;
  paths: Vec[][];
  statuses: string[];
  frames: Frame[];
  java?: string;
  error?: string;
}

export interface LiveFrame {
  seq: number;
  route: string;
  status: string;
  pose: Pose | null;
  follow: Target | null;
  pointTo: Target | null;
  clipped: Vec | null;
  powers: [number, number, number] | null;
  path: Vec[];
  colliding: boolean;
}

export interface LiveStatus {
  running: boolean;
  host: string;
  text: string;
}

export interface SketchRequest {
  name: string;
  /** [x, y, speed override or null, lookahead override or null] */
  points: [number, number, number | null, number | null][];
  reversed: boolean;
  moveSpeed: number;
  followDistance: number;
  /** field headings in radians, or null when not set */
  startHeading: number | null;
  endHeading: number | null;
  fixedHeading: number | null;
}

/** Default tuning, read from the Java Params classes. */
export interface Defaults {
  follower: Record<string, number>;
  drive: Record<string, number>;
  robot: [number, number];
}

/** Tuning changes from the defaults that the simulator should use. */
export interface Tuning {
  follower: Record<string, number>;
  drive: Record<string, number>;
  robot: [number, number] | null;
}

/** A live run saved from the robot, replayable on the timeline. */
export interface Recording {
  version: 1;
  route: string;
  recordedAt: string;
  frames: (LiveFrame & { t: number })[];
}
