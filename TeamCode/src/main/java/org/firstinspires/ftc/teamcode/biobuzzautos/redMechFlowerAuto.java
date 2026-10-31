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
import org.firstinspires.ftc.teamcode.biobuzzautos.Intake;
import org.firstinspires.ftc.teamcode.biobuzzautos.Launcher;

@Autonomous(name = "RED mechanum Flower Auto")
public class redMechFlowerAuto extends LinearOpMode {

    @Override
    public void runOpMode(){
        Launcher launcher = new Launcher(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Pose2d initialPose = new Pose2d(0,60,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder toShoot = drive.actionBuilder(initialPose)
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(45,-16),Math.toRadians(0))
                .turn(Math.toRadians(180))
                .waitSeconds(3);

        TrajectoryActionBuilder toPickUp = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(25,-56))
                .turn(Math.toRadians(90))
                .waitSeconds(3);


        TrajectoryActionBuilder toShoot2 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-35,-60),Math.toRadians(0));


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




//yet to be tested
