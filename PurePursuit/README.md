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

To run a route, extend `PurePursuitAutoBase` and return it from `route()`; override `markers()`
for mechanism Actions (see `PPBlueGardenShooter`).

## Robot setup and tuning

1. `PurePursuitConfig.DRIVE_TYPE` picks the drive class and localizer: `TANK` (starter bot,
   Pinpoint, the default), `MECANUM`, or `COMP_MECANUM`. Tune the localizer with the existing
   Road Runner tuning OpModes first.
2. Run `PP Localization Test` with the FieldViewer in Live mode and check the pose matches the field.
3. Run `TEST Straight 48in` from `PP Path Tester`. If it stops short or overshoots, tune
   `forwardSlipSeconds` and `decelDistance` in Dashboard (`PurePursuitConfig.FOLLOWER`). If it
   stalls just before the end, raise `minMovePower`.
4. `TEST Turn In Place` for `turnSlipSeconds` / `minTurnPower`, then the loop / S-curve / U-turn.

## Tests

`./gradlew :PurePursuit:test` checks the geometry, the follower, the viewer protocol, and
simulates every route on tank and mecanum (finishes, no timeouts, stays on the field, test
routes stay clear of the center structure).
