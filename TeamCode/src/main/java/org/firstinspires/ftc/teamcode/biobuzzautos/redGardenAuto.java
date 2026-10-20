
package org.firstinspires.ftc.teamcode.biobuzzautos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TankDrive;

import java.lang.Math;

@Autonomous(name = "RED Garden Auto Park")
public class redGardenAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Starting position and heading match MeepMeep
        Pose2d initialPose =
                new Pose2d(66, -34, Math.toRadians(0));

        TankDrive drive = new TankDrive(hardwareMap, initialPose);

        // Reproduce the MeepMeep trajectory
        TrajectoryActionBuilder toPark =
                drive.actionBuilder(initialPose)
                        .lineToX(-34)
                        .turn(Math.toRadians(-90))
                        .lineToY(-60);

        // Build the complete movement
        Action parkTrajectory = toPark.build();

        telemetry.addLine("Blue Garden Auto Park initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        Actions.runBlocking(
                new SequentialAction(
                        parkTrajectory
                )
        );
    }
}
//tested&works