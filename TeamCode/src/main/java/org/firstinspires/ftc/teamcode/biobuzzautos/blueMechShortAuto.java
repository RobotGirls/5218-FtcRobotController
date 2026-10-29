package org.firstinspires.ftc.teamcode.biobuzzautos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "BLUE Mecanum Auto")
public class blueMechShortAuto extends LinearOpMode{

    @Override
    public void runOpMode(){
        Launcher launcher = new Launcher(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Pose2d initialPose = new Pose2d(-10,60,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder toShoot = drive.actionBuilder(initialPose)
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-30,10),Math.toRadians(0))
                .waitSeconds(3);

        TrajectoryActionBuilder toPickUp = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(55,17),Math.toRadians(5))
                .strafeToLinearHeading(new Vector2d(60,17),Math.toRadians(45))
                .waitSeconds(3);


        TrajectoryActionBuilder toShoot2 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(35,10),Math.toRadians(0))
                .waitSeconds(3);

        Action outOfZone = toShoot2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(40,56),Math.toRadians(0))
                .build();


        Action firstTraj = toShoot.build();
        Action secondTraj = toPickUp.build();
        Action thirdTraj = toShoot2.build();


        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        firstTraj,
                        launcher.launcherForward(),
                        new ParallelAction(
                                secondTraj,
                                intake.intakeIn()
                        ),
                        thirdTraj,
                        launcher.launcherForward(),

                        outOfZone
                )
        );
    }
    }


//yet to be tested