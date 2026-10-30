# CLAUDE.md

FTC team 5218's robot code for the 2026–27 season (BioBuzz). It is built on Road Runner 1.0.1
for localization and a pure pursuit follower ported from Gluten Free's code. **FieldViewer** is a
desktop app that simulates routes and shows the robot live.

## Set up on a new computer

1. **Clone and switch branch.** The pure pursuit work is on `pure-pursuit-migration`:
   ```
   git clone https://github.com/RobotGirls/5218-FtcRobotController.git
   cd 5218-FtcRobotController
   git checkout pure-pursuit-migration
   ```
2. **Java 17 or newer.** Gradle 9.1 needs it. Android Studio's bundled JDK works. Check with
   `java -version`. If that fails, set `JAVA_HOME` to the JDK, e.g. on macOS
   `/Applications/Android Studio.app/Contents/jbr/Contents/Home`.
3. **Node.js 18 or newer**, for FieldViewer only. Check with `node -v`.
4. **Android Studio and the Android SDK**, only to build and deploy to the robot. The
   `PurePursuit` and `FieldViewer` modules build and test without the Android SDK.

## Common commands

All run from the repo root unless noted.

| What | Command |
| --- | --- |
| Run FieldViewer (desktop app) | `cd FieldViewer/web && npm install && npm start` |
| FieldViewer in a browser instead | `cd FieldViewer/web && npm run build`, then `./gradlew :FieldViewer:run` (opens http://localhost:5218) |
| Typecheck the FieldViewer UI | `cd FieldViewer/web && npm run typecheck` |
| Pure pursuit unit tests | `./gradlew :PurePursuit:test` |
| Headless route pictures | `./gradlew :FieldViewer:run --args="--snapshot all build/snapshots"` |
| Deploy to the robot | Android Studio, run the `TeamCode` configuration |

`npm start` builds the UI, builds the Java backend (`gradlew :FieldViewer:installDist`), starts it
on `localhost:5218`, and opens the Electron window. The first launch takes a while because Gradle
and Electron download. The backend log is `FieldViewer/build/fieldviewer-backend.log`. Quitting
the window also stops the backend.

## Layout

| Path | What |
| --- | --- |
| `PurePursuit/` | Plain Java library (no Android), shared by the robot and FieldViewer. The follower, paths, routes, a drive simulator, and the routes themselves in `purepursuit/paths/` (`BioBuzzRoutes`, `TestRoutes`, `Routes.ALL`). README has the tuning guide. |
| `TeamCode/.../purepursuit/ftc/` | Robot side: `PurePursuitConfig` (`@Config`; drive type and follower tuning), `PurePursuitDrive` (runs routes on `TankDrive` / `MecanumDrive` / `Comp5218MecanumDrive`). |
| `TeamCode/.../purepursuitautos/` | `PP` OpModes: park autos for both alliances, Garden + Shooter, Path Tester (dpad picks a route), FieldViewer localization test. |
| `TeamCode/.../biobuzzautos/` | The original Road Runner autos. Left untouched; keep them working. |
| `FieldViewer/src/main/java/` | Backend on `localhost:5218`: route simulation API, path editor sketches → Java, UDP link to the robot, SSE live stream. |
| `FieldViewer/web/` | TypeScript UI (`src/app/`, plain `tsc` ES modules, canvas), Electron shell (`src/electron/main.ts`), `static/` HTML/CSS. |
| `MeepMeepTesting/` | Old MeepMeep visualizer. FieldViewer reads the field image from its resources. |

## How the pieces connect

- **One follower everywhere.** FieldViewer runs the exact same `PurePursuitFollower` as the robot,
  so a change in `PurePursuit/` changes both. Run `./gradlew :PurePursuit:test` after any change.
- **Adding a route.** Write it in `BioBuzzRoutes` / `TestRoutes`, or build it in FieldViewer's path
  editor and use **Copy Java**, then add it to `Routes.ALL`. It then shows up in both FieldViewer
  and the robot's Path Tester.
- **Live view.** The robot listens on UDP 11116. FieldViewer sends `FIELDVIEWER_HELLO`, and the
  robot streams `ViewerFrame` text records back (`FieldViewerLink`). Control Hub address
  `192.168.43.1`. Turn off with `PurePursuitConfig.STREAM_TO_FIELD_VIEWER = false`.
- **Units and frame.** Inches and radians in the Road Runner field frame: origin at field center,
  ±72 in, the center structure keep-out is ±25.5 in.

## Conventions

- `PurePursuit` and `FieldViewer` Java target Java 8 (same as the robot). Don't use newer
  language features there.
- Match the surrounding style: short doc comments that say why, no comment noise.
- FieldViewer UI: no frameworks or bundler, just `tsc`. Electron is the only runtime
  dependency. Colors and theme are in `static/styles.css`.
- Commit only when asked. Branch off `master` for new work; don't commit straight to `master`.

## Status and caveats

- Nothing has been run on real robot hardware yet; the sim tuning (`DriveSimulator.Params`)
  is a guess until measured.
- BLUE/RED Garden routes clip the center structure for about 3 s with an 18 in robot.
- The path editor builds one continuous path. Markers and turns are dropped when you use
  **Edit as sketch** on a multi-step route.
- FieldViewer recordings are only kept for the session unless saved with **Save recording…**.
