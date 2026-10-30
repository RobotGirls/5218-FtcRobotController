# FieldViewer

Desktop app for watching pure pursuit on this season's (BioBuzz) field: simulate any route,
sketch new ones, or watch the real robot live over wifi.

- **UI:** TypeScript + Electron (`web/`).
- **Simulation and robot link:** a small Java backend (`src/main/java`). It runs the exact same
  `PurePursuitFollower` as the robot, so what you see is what the robot will do.

## Run it

Needs Node.js 18+ (plus the Java that comes with Android Studio).

```
cd FieldViewer/web
npm install        # first time only
npm start
```

`npm start` builds the UI, builds and starts the Java backend with Gradle, and opens the
window. The first launch takes a little longer while Gradle and Electron download.

Prefer a browser? Run `./gradlew :FieldViewer:run` from the repo root and it opens
http://localhost:5218 (run `npm run build` in `web/` once first).

## Simulate

- Pick a route on the left and **Tank** or **Mecanum**; it plays straight away.
- Scrub the timeline. The colored blocks are the route's steps (cyan paths, amber markers,
  purple turns); click one to jump to it.
- **Space** play/pause, **R** restart, **← / →** step one robot loop (Shift for ten).
- The inspector shows pose, drive powers, the current step, and the result: time, worst
  distance off the path, and any time spent overlapping the center structure (dashed red box).

On the field: cyan line = the path being followed, cyan dot = follow point (what the robot
drives toward), white ring = point the robot faces, dashed circle = lookahead, amber line =
where the robot has been.

## Path editor

Click **Sketch a path** (or **Edit as sketch** on any route to start from its points). It
re-simulates as you edit.

- **On the field:** click to add a point, drag to move one, click or drag a **+** (halfway along
  each segment) to insert a point, right-click to delete. Arrow keys nudge the selected point
  1 in (Shift: 6 in); Delete removes it; ⌘/Ctrl-Z undoes.
- **Snapping** (magnet button, **S**): while you place or drag a point it snaps when it gets
  close to lining up with the points next to it (or any other point), a 45° line from its
  neighbors, the robot touching a wall, or the tile grid (seams every 24 in, centers between).
  Dashed lines show what it snapped to; otherwise it rounds to a whole inch. Hold
  **Option/Alt** to place freely. Typed values and arrow-key nudges are never snapped.
- **Waypoint list:** type exact X / Y for any point. Click a row or a point to select it.
- **Selected point:** give the segment ending there its own speed or lookahead (marked with an
  amber dot), move it up / down the path, insert a point after it, or delete it.
- **Path settings:** name, drive backwards, default speed and lookahead, a fixed start heading,
  a heading to turn to at the end, and (mecanum) strafing while facing one direction.

**Copy Java** gives you the finished `Route` to paste into `BioBuzzRoutes` / `TestRoutes` (add
it to `Routes.ALL` so the robot's Path Tester and this viewer list it).

## Viewing tools

The toolbar on the field (top-left):

- **Zoom**: scroll at the cursor, or the + / − buttons. **Fit** (or double-click) resets.
- **Drag** to pan (Shift-drag while sketching).
- **Rotate** turns the field 90° to match where you stand. The compass bottom-left shows +X / +Y.
- **Follow** keeps the robot centered.
- **Measure**: drag a ruler for distance, angle and ΔX / ΔY in inches. Its ends snap too.
- **Camera** saves a PNG of the field.

Under the field, **graphs** show drive power and speed over time. Click them to jump to that
moment; press **G** to hide them. Press **?** for every keyboard shortcut.

## Tuning

Open **Tuning** in the right panel and drag the sliders. The simulation reruns immediately.

- **Follower** values are what the robot uses. **Copy as Java** gives you
  `PurePursuitConfig.FOLLOWER...` lines to paste into an OpMode, or set the same values live in
  FTC Dashboard.
- **Simulated robot** (top speed, response lag, friction) makes the simulator behave like your
  robot. Measure top speed with Road Runner's MaxVelocityTuner.
- **Robot size** is used for drawing and center structure contact.

Changed values are marked and remembered. **Pin run** freezes the current trail as a ghost, so
you can change tuning (or Tank / Mecanum) and compare.

## Recording matches

In **Live**, press **Record run** before the auto starts and **Stop** after. The run shows up
under **Recordings** in the sidebar and replays on the timeline with graphs and results.
**Save recording…** writes a `.json` file; **Open recording…** loads one back.

## Live

1. Connect the laptop to the robot's wifi.
2. Switch to **Live**, enter the robot address (Control Hub `192.168.43.1`, phone RC
   `192.168.49.1`), and **Connect**.
3. Run any `PP` OpMode. You'll see the Road Runner pose, the path, and the lookahead.

The backend says hello to the robot on UDP port 11116 and the robot streams frames back, so no
laptop IP is hard coded on the robot. Nothing is sent when no viewer is connected; set
`PurePursuitConfig.STREAM_TO_FIELD_VIEWER = false` to switch it off completely.

## Layout

| Path | What |
| --- | --- |
| `web/src/app/` | UI: `main.ts` (app state, controls), `field.ts` (field renderer and camera), `chart.ts` (graphs), `tuning.ts`, `recording.ts`, `api.ts`, `types.ts` |
| `web/src/electron/main.ts` | Electron window; starts the Java backend |
| `web/static/` | `index.html`, `styles.css`, loading screen |
| `src/main/java/.../WebServer.java` | Backend API on `localhost:5218` (routes, simulate, live stream) |
| `src/main/java/.../FieldRenderer.java` | Dark field image, and headless PNG snapshots |

Headless pictures (no UI):

```
./gradlew :FieldViewer:run --args="--snapshot all build/snapshots"
```
