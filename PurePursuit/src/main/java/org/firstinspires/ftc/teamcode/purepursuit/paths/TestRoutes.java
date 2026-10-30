package org.firstinspires.ftc.teamcode.purepursuit.paths;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.purepursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.Route.Alliance;

/**
 * Paths for tuning and checking pure pursuit. They all stay in the outer ring of tiles so they
 * can be run on a full BioBuzz field without hitting the central structure.
 */
public final class TestRoutes {
    private TestRoutes() {}

    public static final Route STRAIGHT = Route.builder(
                    "TEST Straight 48in", Alliance.NONE, new Pose2d(-24, -48, 0))
            .description("Two tiles forward. Check the robot stops on the line, then tune slip.")
            .follow(PurePursuitPath.builder().name("forward 48")
                    .to(24, -48)
                    .build())
            .build();

    public static final Route STRAIGHT_24 = Route.builder(
                    "TEST Straight 24in", Alliance.NONE, new Pose2d(-12, -48, 0))
            .description("One tile straight forward, then stop. The simplest auto to try first.")
            .follow(PurePursuitPath.builder().name("forward 24")
                    .to(12, -48)
                    .build())
            .build();

    public static final Route OUT_AND_BACK = Route.builder(
                    "TEST Out and Back", Alliance.NONE, new Pose2d(-48, -40, Math.toRadians(90)))
            .description("Drive up the left side and back up to the start. Repeat to check drift.")
            .follow(PurePursuitPath.builder().name("out")
                    .to(-48, 40)
                    .build())
            .follow(PurePursuitPath.builder().name("back")
                    .to(-48, -40)
                    .reversed()
                    .build())
            .build();

    public static final Route FIELD_LOOP = Route.builder(
                    "TEST Field Loop", Alliance.NONE, new Pose2d(-48, -48, 0))
            .description("A lap around the outer tiles with four 90 degree corners.")
            .follow(PurePursuitPath.builder().name("lap")
                    .to(48, -48)
                    .to(48, 48)
                    .to(-48, 48)
                    .to(-48, -48)
                    .endHeading(0)
                    .build())
            .build();

    public static final Route U_TURN = Route.builder(
                    "TEST U-Turn", Alliance.NONE, new Pose2d(56, -48, Math.toRadians(90)))
            .description("Up the right side, a tight 180, and back down.")
            .follow(PurePursuitPath.builder().name("hairpin")
                    .followDistance(10)
                    .to(56, 40)
                    .to(40, 40)
                    .to(40, -40)
                    .build())
            .build();

    public static final Route S_CURVE = sCurve();

    public static final Route STRAFE_SQUARE = Route.builder(
                    "TEST Strafe Square (mecanum)", Alliance.NONE, new Pose2d(36, -56, 0))
            .description("Mecanum only: a 20in square while always facing +x.")
            .follow(PurePursuitPath.builder().name("square")
                    .moveSpeed(0.5)
                    .followDistance(8)
                    .to(56, -56)
                    .to(56, -36)
                    .to(36, -36)
                    .to(36, -56)
                    .fixedHeading(0)
                    .build())
            .build();

    public static final Route TURN_IN_PLACE = Route.builder(
                    "TEST Turn In Place", Alliance.NONE, new Pose2d(-48, 48, 0))
            .description("Turns to 90, 180, -90 and back to 0. Checks heading slip.")
            .turnTo(Math.toRadians(90))
            .turnTo(Math.toRadians(180))
            .turnTo(Math.toRadians(-90))
            .turnTo(0)
            .build();

    /** A sine wave along the bottom of the field, below the central structure. */
    private static Route sCurve() {
        PurePursuitPath.Builder path = PurePursuitPath.builder().name("sine")
                .moveSpeed(0.5)
                .followDistance(10);
        for (int x = -48; x <= 56; x += 8) {
            path.to(x, -48 + 8 * Math.sin((x + 56) * Math.PI / 28));
        }
        return Route.builder("TEST S-Curve", Alliance.NONE, new Pose2d(-56, -48, 0))
                .description("Weaves along the bottom tiles. Shows how lookahead cuts corners.")
                .follow(path.build())
                .build();
    }
}
