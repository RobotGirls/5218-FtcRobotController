package org.firstinspires.ftc.teamcode.test;


// RR-specific imports

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.TankDrive;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
//import org.firstinspires.ftc.teamcode.mechanismCode.IntakeRoadRunner;
//import org.firstinspires.ftc.teamcode.mechanismCode.ShooterRoadRunner;
//import org.firstinspires.ftc.teamcode.mechanismCode.TransferRoadRunner;

//@Config
@Autonomous(name = "ILT Blue")
public class ILTBlueAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(-52, -46, Math.toRadians(-130));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);



        TrajectoryActionBuilder toShoot = drive.actionBuilder(initialPose)
                .setReversed(true)
                .splineTo(new Vector2d(-8,-8),Math.toRadians(45));
        TrajectoryActionBuilder intakeBalls = toShoot.endTrajectory().fresh()
                .turn(Math.toRadians(25))
                .splineTo(new Vector2d(-12,-52),Math.toRadians(-90));
        TrajectoryActionBuilder backToShoot = intakeBalls.endTrajectory().fresh()
                .setReversed(true)
                .splineTo(new Vector2d(-8,-8),Math.toRadians(45));
        Action outOfZone = backToShoot.endTrajectory().fresh()
                .turn(Math.toRadians(90))
                .lineToX(2)
                .build();


        Action firstTraj = toShoot.build();
        Action secondTraj = intakeBalls.build();
        Action thirdTraj = backToShoot.build();


        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Robot position: ", drive.updatePoseEstimate());
            telemetry.update();
        }
        waitForStart();
        if (isStopRequested()) return;

        // IN RUNTIME
        // running the action sequence!
        Actions.runBlocking(
                new SequentialAction(
                        firstTraj,
                        new ParallelAction(
//                                shooter.shootArtifact(),
//                                intake.intakeArtifact(),
//                                transfer.intakeArtifact()
                        ),

                        new ParallelAction(
                                secondTraj
//                                intake.intakeArtifact(),
//                                transfer.intakeArtifact()
                        ),

                        new ParallelAction(
                                thirdTraj
                                // shooter.shootArtifact()
                        ),
                        new ParallelAction(
//                                shooter.shootArtifact(),
//                                transfer.intakeArtifact(),
//                                intake.intakeArtifact()
                        ),
                        outOfZone

                )
        );

    }
}

