package org.firstinspires.ftc.teamcode.purepursuit.paths;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.purepursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.Route.Alliance;

/**
 * BioBuzz autonomous routes. These are the same start poses and moves as the Road Runner autos
 * in teamcode/biobuzzautos (tested on the starter bot), rewritten as pure pursuit paths.
 *
 * Those autos drive a tank drive with heading 0, so a lineToX towards -x was really driving
 * backwards; those legs use .reversed() here so the robot doesn't spin around first.
 */
public final class BioBuzzRoutes {
    private BioBuzzRoutes() {}

    /** Marker name the shooter auto maps to its launch Action. */
    public static final String SHOOT = "shoot";
    /** Marker name the mecanum autos map to running the intake while they drive. */
    public static final String INTAKE = "intake";


    public static final Route BLUE_GARDEN_PARK = Route.builder(
                    "BLUE Garden Park", Alliance.BLUE, new Pose2d(-66, 34, 0))
            .description("Drive above the center structure, then back into the blue zone.")
            .follow(PurePursuitPath.builder().name("across the field")
                    .to(34, 34)
                    .build())
            .follow(PurePursuitPath.builder().name("back into zone")
                    .moveSpeed(0.5)
                    .to(34, 60)
                    .reversed()
                    .build())
            .build();

    public static final Route BLUE_CORNER_PARK = Route.builder(
                    "BLUE Corner Park", Alliance.BLUE, new Pose2d(66, 60, 0))
            .description("Back out of the corner into the blue zone.")
            .follow(PurePursuitPath.builder().name("to park")
                    .to(48, 60)
                    .reversed()
                    .build())
            .build();



    public static final Route RED_FLOWER_PARK = Route.builder(
                    "RED Flower Park", Alliance.RED, new Pose2d(10, -60, 0))
            .description("Back up along the red wall into the red zone.")
            .follow(PurePursuitPath.builder().name("to park")
                    .to(-30, -60)
                    .reversed()
                    .build())
            .build();

    public static final Route RED_GARDEN_PARK = Route.builder(
                    "RED Garden Park", Alliance.RED, new Pose2d(66, -34, 0))
            .description("Back up below the center structure, then drive into the red zone.")
            .follow(PurePursuitPath.builder().name("across the field")
                    .to(-34, -34)
                    .reversed()
                    .build())
            .follow(PurePursuitPath.builder().name("into zone")
                    .moveSpeed(0.5)
                    .to(-34, -60)
                    .build())
            .build();

    public static final Route RED_CORNER_PARK = Route.builder(
                    "RED Corner Park", Alliance.RED, new Pose2d(-60, -66, 0))
            .description("Drive out of the corner into the red zone.")
            .follow(PurePursuitPath.builder().name("to park")
                    .to(-40, -66)
                    .build())
            .build();

    public static final Route BLUE_GARDEN_SHOOTER = Route.builder(
                    "BLUE Garden Park + Shooter", Alliance.BLUE, new Pose2d(-66, 34, 0))
            .description("BLUE Garden Park, then spin up the launcher and feed artifacts.")
            .follow(PurePursuitPath.builder().name("across the field")
                    .to(34, 34)
                    .build())
            .follow(PurePursuitPath.builder().name("back into zone")
                    .moveSpeed(0.5)
                    .to(34, 60)
                    .reversed()
                    .build())
            .marker(SHOOT, 3.5)
            .build();

    // Mecanum autos from issue-30-autonomousBiobuzz (biobuzzautos/*Mech*Auto), written to drive
    // like the MeepMeep preview: every leg starts where the last one ended. strafeToLinearHeading
    // turns evenly along the leg, strafeTo keeps the heading, turn() turns by an amount the same
    // way Road Runner does, and the intake runs alongside the pickup leg like the ParallelAction.

    public static final Route BLUE_MECH_SHORT = Route.builder(
                    "BLUE Mech Short", Alliance.BLUE, new Pose2d(-10, 60, 0))
            .description("Shoot, pick up at the far wall, shoot again, then leave the zone.")
            .waitSeconds(2)
            .follow(strafeToLinearHeading("to shoot", -30, 10, 0, 0))
            .waitSeconds(3)
            .marker(SHOOT, 2)
            .parallelMarker(INTAKE, 2)
            .follow(strafeToLinearHeading("to pickup", 55, 17, 0, 5))
            .follow(strafeToLinearHeading("into pickup", 60, 17, 5, 45))
            .waitSeconds(3)
            .follow(strafeToLinearHeading("to shoot 2", 35, 10, 45, 0))
            .waitSeconds(3)
            .marker(SHOOT, 2)
            .follow(strafeToLinearHeading("out of zone", 40, 56, 0, 0))
            .build();

    public static final Route RED_MECH_SHORT = Route.builder(
                    "RED Mech Short", Alliance.RED, new Pose2d(56, -34, 0))
            .description("Drive to shoot (launcher off for now), pick up, then park.")
            .waitSeconds(3)
            .follow(strafeToLinearHeading("to shoot", 38, -16, 0, 0))
            .waitSeconds(3)
            // launcher is commented out in the Road Runner version, so no SHOOT markers yet
            .parallelMarker(INTAKE, 2)
            .follow(strafeToLinearHeading("to pickup", -55, -20, 0, 0))
            .waitSeconds(3)
            .follow(strafeTo("to park", -34, -60, 0))
            .build();

    public static final Route BLUE_MECH_CORNER = Route.builder(
                    "BLUE Mech Corner", Alliance.BLUE, new Pose2d(56, 60, 0))
            .description("Along the blue wall to shoot, pick up, then shoot from the middle.")
            .waitSeconds(2)
            .follow(strafeToLinearHeading("to shoot", -52, 56, 0, 0))
            .turn(Math.toRadians(-90))
            .waitSeconds(3)
            .marker(SHOOT, 2)
            .parallelMarker(INTAKE, 2)
            .follow(strafeToLinearHeading("to pickup", -20, 56, -90, 0))
            .turn(Math.toRadians(90))
            .waitSeconds(3)
            .follow(strafeToLinearHeading("to shoot 2", 35, 10, 90, 0))
            .marker(SHOOT, 2)
            .build();

    public static final Route BLUE_MECH_FLOWER = Route.builder(
                    "BLUE Mech Flower", Alliance.BLUE, new Pose2d(0, 60, 0))
            .description("Shoot from the left, pick up by the wall, shoot from the right.")
            .waitSeconds(2)
            .follow(strafeToLinearHeading("to shoot", -52, 15, 0, 0))
            .waitSeconds(3)
            .marker(SHOOT, 2)
            .parallelMarker(INTAKE, 2)
            .follow(strafeTo("to pickup", -20, 56, 0))
            .turn(Math.toRadians(90))
            .waitSeconds(3)
            .follow(strafeToLinearHeading("to shoot 2", 35, 60, 90, 0))
            .marker(SHOOT, 2)
            .build();

    // starts at (0, -60) like the MeepMeep preview; the OpMode on issue-30 says (0, 60)
    public static final Route RED_MECH_FLOWER = Route.builder(
                    "RED Mech Flower", Alliance.RED, new Pose2d(0, -60, 0))
            .description("Shoot from the right, pick up by the red wall, shoot from the left.")
            .waitSeconds(2)
            .follow(strafeToLinearHeading("to shoot", 45, -16, 0, 0))
            .turn(Math.toRadians(180))
            .waitSeconds(3)
            .marker(SHOOT, 2)
            .parallelMarker(INTAKE, 2)
            .follow(strafeTo("to pickup", 25, -56, 180))
            .turn(Math.toRadians(90))
            .waitSeconds(3)
            .follow(strafeToLinearHeading("to shoot 2", -35, -60, 270, 0))
            .marker(SHOOT, 2)
            .build();

    public static final Route RED_MECH_CORNER = Route.builder(
                    "RED Mech Corner", Alliance.RED, new Pose2d(-60, -60, 0))
            .description("Along the red wall, up to shoot, pick up, then shoot from the left.")
            .waitSeconds(2)
            .follow(strafeToLinearHeading("along the wall", 38, -52, 0, 0))
            .follow(strafeToLinearHeading("to shoot", 38, -15, 0, 180))
            .waitSeconds(3)
            .marker(SHOOT, 2)
            .parallelMarker(INTAKE, 2)
            .follow(strafeTo("to pickup", 20, -55, 180))
            .turn(Math.toRadians(90))
            .waitSeconds(3)
            .follow(strafeToLinearHeading("to shoot 2", -34, -56, 270, 0))
            .marker(SHOOT, 2)
            .build();

    /** Road Runner's strafeTo: drive straight to (x, y) keeping the heading (degrees). */
    private static PurePursuitPath strafeTo(String name, double x, double y, double headingDegrees) {
        return PurePursuitPath.builder().name(name)
                .to(x, y)
                .fixedHeading(Math.toRadians(headingDegrees))
                .build();
    }

    /** Road Runner's strafeToLinearHeading: turn evenly from one heading to another (degrees). */
    private static PurePursuitPath strafeToLinearHeading(String name, double x, double y,
                                                         double fromDegrees, double toDegrees) {
        return PurePursuitPath.builder().name(name)
                .to(x, y)
                .linearHeading(Math.toRadians(fromDegrees), Math.toRadians(toDegrees))
                .build();
    }

    public static final Route BLUE_GARDEN_PARK_COPY = Route.builder(
                    "BLUE Garden Park copy", Alliance.NONE, new Pose2d(-66, 34, Math.toRadians(0)))
            .follow(PurePursuitPath.builder().name("blue garden park copy")
                    .to(34, 34)
                    .moveSpeed(0.5)
                    .to(34, 60)
                    .build())
            .build();

    public static final Route BLUE_SHOOTER_STARTER_BOT_COPY = Route.builder(
                    "BLUE Shooter starter bot copy", Alliance.NONE, new Pose2d(-63, 12, Math.toRadians(180)))
            .follow(PurePursuitPath.builder().name("blue shooter starter bot copy")
                    .to(-39, 12)
                    .reversed()
                    .build())
            .marker(SHOOT, 3.5)
            .build();

    public static final Route BLUE_FLOWER_PARK = Route.builder(
                    "BLUE Flower Park", Alliance.NONE, new Pose2d(-10, 60, Math.toRadians(0)))
            .follow(PurePursuitPath.builder().name("blue flower park")
                    .to(2, 48)
                    .to(36, 56)
                    .build())
            .build();







}