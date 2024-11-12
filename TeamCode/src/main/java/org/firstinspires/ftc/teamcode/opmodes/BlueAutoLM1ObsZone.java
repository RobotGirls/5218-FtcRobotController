package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "BlueObservationAuto")
public class BlueAutoLM1ObsZone extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        TrajectoryActionBuilder toSubmersible = drive.actionBuilder(initialPose)
                //.turn(Math.toRadians(45))
                .strafeTo(new Vector2d(-20,29))
                .waitSeconds(2);
        Action toSubmersibleTraj = toSubmersible.build();

        Action toObservation = toSubmersible.fresh()
                .lineToY(-5)
                .waitSeconds(1)
                .setTangent(Math.toRadians(0))
                .lineToX(25)
                .waitSeconds(2)
              //  .setTangent(0)
                .build();


        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Robot position: ", drive.updatePoseEstimate());
        }
        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                       toSubmersibleTraj,
                       toObservation
                )
        );



    }
}
