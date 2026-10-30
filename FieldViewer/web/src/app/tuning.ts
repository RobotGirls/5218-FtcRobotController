import type { Defaults, Tuning } from './types.js';

/**
 * The tuning panel: sliders for PurePursuitFollower.Params (what the robot uses), the
 * simulated drive (so the sim behaves like your robot), and the robot's size. Only values that
 * differ from the Java defaults are sent to the simulator, and they're remembered between runs.
 */

type Group = 'follower' | 'drive' | 'robot';

interface Spec {
  group: Group;
  key: string;
  label: string;
  min: number;
  max: number;
  step: number;
  unit: string;
  /** stored in radians, shown in degrees */
  degrees?: boolean;
  hint: string;
}

const SPECS: Spec[] = [
  { group: 'follower', key: 'forwardSlipSeconds', label: 'Forward slip', min: 0, max: 0.4, step: 0.005, unit: 's',
    hint: 'How far the robot coasts per in/s. Raise if it overshoots the end, lower if it stops short.' },
  { group: 'follower', key: 'strafeSlipSeconds', label: 'Strafe slip', min: 0, max: 0.4, step: 0.005, unit: 's',
    hint: 'Same as forward slip, sideways (mecanum).' },
  { group: 'follower', key: 'turnSlipSeconds', label: 'Turn slip', min: 0, max: 0.4, step: 0.005, unit: 's',
    hint: 'Raise if turns overshoot the target heading.' },
  { group: 'follower', key: 'decelDistance', label: 'Decel distance', min: 2, max: 36, step: 0.5, unit: 'in',
    hint: 'Distance over which the robot slows down for the last point.' },
  { group: 'follower', key: 'minMovePower', label: 'Min move power', min: 0, max: 0.4, step: 0.01, unit: '',
    hint: 'Least power that still moves the robot. Raise if it stalls just before the end.' },
  { group: 'follower', key: 'minTurnPower', label: 'Min turn power', min: 0, max: 0.4, step: 0.01, unit: '',
    hint: 'Least power that still turns the robot.' },
  { group: 'follower', key: 'turnDecelRadians', label: 'Turn decel', min: 5, max: 120, step: 1, unit: '°', degrees: true,
    hint: 'Heading error at which turning uses full power.' },
  { group: 'follower', key: 'finalApproachExtra', label: 'Final approach', min: 0, max: 24, step: 0.5, unit: 'in',
    hint: 'Start aiming straight at the last point this much earlier.' },
  { group: 'follower', key: 'positionTolerance', label: 'Position tolerance', min: 0.25, max: 4, step: 0.25, unit: 'in',
    hint: 'How close counts as arrived.' },
  { group: 'follower', key: 'headingTolerance', label: 'Heading tolerance', min: 0.5, max: 15, step: 0.5, unit: '°', degrees: true,
    hint: 'How close counts as facing the end heading.' },
  { group: 'drive', key: 'maxForwardVel', label: 'Top speed', min: 10, max: 90, step: 1, unit: 'in/s',
    hint: 'Measure with the Road Runner MaxVelocityTuner.' },
  { group: 'drive', key: 'maxStrafeVel', label: 'Top strafe speed', min: 10, max: 90, step: 1, unit: 'in/s',
    hint: 'Mecanum only.' },
  { group: 'drive', key: 'maxAngVel', label: 'Top turn speed', min: 60, max: 720, step: 10, unit: '°/s', degrees: true,
    hint: 'How fast the robot spins at full power.' },
  { group: 'drive', key: 'timeConstant', label: 'Response lag', min: 0.02, max: 0.5, step: 0.01, unit: 's',
    hint: 'How long the robot takes to speed up / coast down (heavier robots are slower).' },
  { group: 'drive', key: 'staticFrictionPower', label: 'Static friction', min: 0, max: 0.3, step: 0.01, unit: '',
    hint: 'Power below which the simulated robot does not move.' },
  { group: 'robot', key: 'length', label: 'Length', min: 8, max: 18, step: 0.5, unit: 'in',
    hint: 'Front to back. Used for drawing and center structure contact.' },
  { group: 'robot', key: 'width', label: 'Width', min: 8, max: 18, step: 0.5, unit: 'in',
    hint: 'Side to side.' },
];

const GROUP_TITLES: Record<Group, string> = {
  follower: 'Follower (robot code)',
  drive: 'Simulated robot',
  robot: 'Robot size',
};

const STORAGE_KEY = 'fieldviewer.tuning.v1';

export class TuningPanel {
  private readonly values: Record<Group, Record<string, number>>;
  private readonly inputs = new Map<string, { range: HTMLInputElement; number: HTMLInputElement; row: HTMLElement }>();

  constructor(private readonly container: HTMLElement, private readonly defaults: Defaults,
              private readonly onChange: () => void) {
    this.values = {
      follower: { ...defaults.follower },
      drive: { ...defaults.drive },
      robot: { length: defaults.robot[0], width: defaults.robot[1] },
    };
    this.load();
    this.build();
  }

  /** Only the values that differ from the Java defaults. */
  tuning(): Tuning {
    const changed = (group: 'follower' | 'drive') => {
      const out: Record<string, number> = {};
      for (const [key, value] of Object.entries(this.values[group])) {
        if (!same(value, this.defaults[group][key])) out[key] = value;
      }
      return out;
    };
    const robot: [number, number] = [this.values.robot.length, this.values.robot.width];
    const robotChanged = !same(robot[0], this.defaults.robot[0]) || !same(robot[1], this.defaults.robot[1]);
    return { follower: changed('follower'), drive: changed('drive'), robot: robotChanged ? robot : null };
  }

  robotSize(): [number, number] {
    return [this.values.robot.length, this.values.robot.width];
  }

  changedCount(): number {
    const t = this.tuning();
    return Object.keys(t.follower).length + Object.keys(t.drive).length + (t.robot ? 1 : 0);
  }

  reset(): void {
    this.values.follower = { ...this.defaults.follower };
    this.values.drive = { ...this.defaults.drive };
    this.values.robot = { length: this.defaults.robot[0], width: this.defaults.robot[1] };
    this.save();
    this.refresh();
    this.onChange();
  }

  /** Lines to paste into an OpMode (or copy into FTC Dashboard) so the robot matches the sim. */
  java(): string {
    const t = this.tuning();
    const lines = ['// Pure pursuit tuning from the FieldViewer. Paste into an OpMode\'s init, or set the',
      '// same values live in FTC Dashboard under PurePursuitConfig > FOLLOWER.'];
    const follower = SPECS.filter((s) => s.group === 'follower' && s.key in t.follower);
    if (!follower.length) {
      lines.push('// (no follower changes from the defaults)');
    }
    for (const spec of follower) {
      const value = t.follower[spec.key];
      const literal = spec.degrees ? `Math.toRadians(${trim(toDegrees(value))})` : trim(value);
      lines.push(`PurePursuitConfig.FOLLOWER.${spec.key} = ${literal};`);
    }
    const sim = SPECS.filter((s) => (s.group === 'drive' && s.key in t.drive) || (s.group === 'robot' && t.robot));
    if (sim.length) {
      lines.push('', '// Simulator only (describe your robot; not used on the robot):');
      for (const spec of sim) {
        const value = this.values[spec.group][spec.key];
        lines.push(`//   ${spec.label}: ${trim(spec.degrees ? toDegrees(value) : value)} ${spec.unit}`.trimEnd());
      }
    }
    return lines.join('\n');
  }

  private build(): void {
    this.container.replaceChildren();
    for (const group of ['follower', 'drive', 'robot'] as Group[]) {
      const title = document.createElement('div');
      title.className = 'tune-group';
      title.textContent = GROUP_TITLES[group];
      this.container.append(title);
      for (const spec of SPECS.filter((s) => s.group === group)) {
        if (group !== 'robot' && !(spec.key in this.defaults[group])) continue;
        this.container.append(this.row(spec));
      }
    }
    this.refresh();
  }

  private row(spec: Spec): HTMLElement {
    const row = document.createElement('label');
    row.className = 'tune-row';
    row.title = spec.hint;
    const name = document.createElement('span');
    name.className = 'tune-name';
    name.textContent = spec.label;
    const range = document.createElement('input');
    range.type = 'range';
    range.min = String(spec.min);
    range.max = String(spec.max);
    range.step = String(spec.step);
    const number = document.createElement('input');
    number.type = 'number';
    number.className = 'tune-number mono';
    number.step = String(spec.step);
    const unit = document.createElement('span');
    unit.className = 'tune-unit';
    unit.textContent = spec.unit;

    const set = (shown: number) => {
      if (!Number.isFinite(shown)) return;
      this.values[spec.group][spec.key] = spec.degrees ? (shown * Math.PI) / 180 : shown;
      this.save();
      this.refresh();
      this.onChange();
    };
    range.addEventListener('input', () => set(Number(range.value)));
    number.addEventListener('change', () => set(Number(number.value)));

    row.append(name, range, number, unit);
    this.inputs.set(`${spec.group}.${spec.key}`, { range, number, row });
    return row;
  }

  private refresh(): void {
    for (const spec of SPECS) {
      const input = this.inputs.get(`${spec.group}.${spec.key}`);
      if (!input) continue;
      const value = this.values[spec.group][spec.key];
      const fallback = spec.group === 'robot'
        ? this.defaults.robot[spec.key === 'length' ? 0 : 1]
        : this.defaults[spec.group][spec.key];
      const shown = spec.degrees ? toDegrees(value) : value;
      input.range.value = String(shown);
      if (document.activeElement !== input.number) {
        input.number.value = trim(shown);
      }
      input.row.classList.toggle('changed', !same(value, fallback));
    }
  }

  private load(): void {
    try {
      const saved = JSON.parse(localStorage.getItem(STORAGE_KEY) ?? 'null') as Tuning | null;
      if (!saved) return;
      for (const group of ['follower', 'drive'] as const) {
        for (const [key, value] of Object.entries(saved[group] ?? {})) {
          if (key in this.values[group] && typeof value === 'number') this.values[group][key] = value;
        }
      }
      if (Array.isArray(saved.robot)) {
        this.values.robot = { length: saved.robot[0], width: saved.robot[1] };
      }
    } catch {
      // storage unavailable or corrupt; start from the defaults
    }
  }

  private save(): void {
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(this.tuning()));
    } catch {
      // not fatal, the values just won't be remembered
    }
  }
}

function same(a: number, b: number | undefined): boolean {
  return b !== undefined && Math.abs(a - b) < 1e-9;
}

function toDegrees(radians: number): number {
  return Math.round(((radians * 180) / Math.PI) * 1000) / 1000;
}

function trim(value: number): string {
  return String(Math.round(value * 10000) / 10000);
}
