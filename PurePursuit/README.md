# Pure Pursuit (Road Runner localization)

Autonomous driving for team 5218 that uses **Road Runner for localization** (the Pinpoint /
drive encoder localizers in TeamCode) and **pure pursuit for path following**. The follower is
ported from Gluten Free's `MovementEssentials` (`followCurve` / `gunToPosition` / `pointAngle`,
https://github.com/nwatx/Gluten-Free-Code-Reuploaded).

This module is plain Java so the robot and the [FieldViewer](../FieldViewer/README.md) run the
exact same code.

| Where | What |
| --- | --- |
| `PurePursuit/.../purepursuit/` | `PurePursuitFollower`, `PurePursuitPath`, `CurvePoint`, `Route`, simulator, FieldViewer link |
| `PurePursuit/.../purepursuit/paths/` | `BioBuzzRoutes` (this season's autos), `TestRoutes`, `Routes.ALL` |
| `TeamCode/.../purepursuit/ftc/` | `PurePursuitDrive` (Road Runner Actions), `PurePursuitConfig` (Dashboard tuning) |
| `TeamCode/.../purepursuitautos/` | `PP ...` OpModes, `PP Path Tester`, `PP Localization Test` |

## Writing a route

```java
public static final Route BLUE_GARDEN_PARK = Route.builder(
                "BLUE Garden Park", Alliance.BLUE, new Pose2d(-66, 34, 0))
        .follow(PurePursuitPath.builder().name("across the field")
                .to(34, 34)
                .build())
        .follow(PurePursuitPath.builder().name("back into zone")
                .moveSpeed(0.5)
                .to(34, 60)
                .reversed()          // back leads, like the tank auto's lineToY did
                .build())
        .marker("shoot", 3.5)        // OpMode maps "shoot" to a mechanism Action
        .build();
```

- Coordinates are Road Runner's: inches, field center is (0, 0), +x right / +y up on
  `biobuzz-field.png`, heading 0 faces +x.
- The robot's current position is automatically the first point of every path.
- Per point: `moveSpeed`, `turnSpeed`, `followDistance` (lookahead), `pointLength`, `slowDownTurn`.
- Per path: `reversed()` / `followAngle(rad)`, `endHeading(rad)`, `fixedHeading(rad)` (mecanum
  strafing only), `timeout(s)`.
- Add new routes to `Routes.ALL` so they show up in the FieldViewer and `PP Path Tester`.

To run a route, extend `PurePursuitAutoBase` and return it from `route()`. Create mechanisms in
`initMechanisms()`, bind each marker name to the Action it runs in `mechanisms()`, and stop the
motors in `stopMechanisms()` (see `BioBuzzMechAuto`):

```java
@Override
protected MechanismBindings<Action> mechanisms() {
    return new MechanismBindings<Action>()
            .register(BioBuzzRoutes.SHOOT, launcher::launcherForward)
            .register(BioBuzzRoutes.INTAKE, intake::intakeIn);
}
```

Every marker step gets a fresh Action, so a route can shoot twice. A marker with nothing bound
just waits, and init shows a warning for it. The simulator only uses each marker's estimated
seconds, so keep them close to how long the real Action takes.

Road Runner style steps: `linearHeading(from, to)` turns evenly along a strafe like
`strafeToLinearHeading`, `Route.turn(rad)` turns by an amount like `turn()`, and
`parallelMarker(name, s)` runs a mechanism while the robot keeps driving like `ParallelAction`.

## Robot setup and tuning

1. `PurePursuitConfig.DRIVE_TYPE` picks the drive class and localizer: `TANK` (starter bot,
   Pinpoint), `MECANUM` (the default), or `COMP_MECANUM`. Tune the localizer with the existing
   Road Runner tuning OpModes first.
2. Run `PP Localization Test` with the FieldViewer in Live mode and check the pose matches the field.
3. Run `PP Auto Tuner` from the `TEST Tuning Course` start, (-48, -48) facing +x, with the
   bottom-left lane clear (`AutoTuner`). It measures static friction, top speeds, response lag
   and coast with short bursts sized to the free space, then drives the course and corrects
   `forwardSlipSeconds`, `strafeSlipSeconds`, `turnSlipSeconds`, `minMovePower` and
   `minTurnPower` until the robot stops where the coded legs end. The results are saved on the
   Control Hub (`/sdcard/FIRST/purepursuit_tuning.properties`, `TuningFile`) and every pure
   pursuit OpMode loads them when it starts; init telemetry shows which tuning is in use. In
   Live mode the robot sends them to the FieldViewer, which copies them into its Tuning panel
   once so the simulator matches the robot. Press B during the tuner's init to delete them and
   go back to the defaults, or set `PurePursuitConfig.USE_SAVED_TUNING = false`. The file only
   lives on that Control Hub; copy the printed lines into `PurePursuitConfig` to keep them in git.
4. To tune by hand instead: `TEST Straight 48in` from `PP Path Tester`. If it stops short or
   overshoots, tune `forwardSlipSeconds` and `decelDistance` in Dashboard. If it stalls just
   before the end, raise `minMovePower`. `TEST Turn In Place` for `turnSlipSeconds` /
   `minTurnPower`, then the loop / S-curve / U-turn.

## Tests

`./gradlew :PurePursuit:test` checks the geometry, the follower, the viewer protocol, and
simulates every route on tank and mecanum (finishes, no timeouts, stays on the field, test
routes stay clear of the center structure).
