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

    public static final Route BLUE_FLOWER_PARK = Route.builder(
                    "BLUE Flower Park", Alliance.BLUE, new Pose2d(-10, 60, 0))
            .description("Drive forward along the blue wall into the blue zone.")
            .follow(PurePursuitPath.builder().name("to park")
                    .to(30, 60)
                    .build())
            .build();

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
                    .to(40, 60)
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
}
