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

@Autonomous(name = "RED Mecanum Auto")
public class redMechShortAuto extends LinearOpMode{

    @Override
    public void runOpMode(){
        org.firstinspires.ftc.teamcode.biobuzzautos.Launcher launcher = new org.firstinspires.ftc.teamcode.biobuzzautos.Launcher(hardwareMap);
        org.firstinspires.ftc.teamcode.biobuzzautos.Intake intake = new org.firstinspires.ftc.teamcode.biobuzzautos.Intake(hardwareMap);
        Pose2d initialPose = new Pose2d(56,-34,0);
        Pose2d shootPose   = new Pose2d(38, -16, 0);
        Pose2d pickupPose  = new Pose2d(-55, -20, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder toShoot = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(38,-16),Math.toRadians(0))
                .waitSeconds(3);

        TrajectoryActionBuilder toPickUp = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-55,-20),Math.toRadians(0))
                .waitSeconds(3);


        TrajectoryActionBuilder toShoot2 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-34,-60));


        Action outOfZone = toShoot2.endTrajectory().fresh()
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
