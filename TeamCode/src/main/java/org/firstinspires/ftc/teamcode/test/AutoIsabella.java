package org.firstinspires.ftc.teamcode.test;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.Comp5218MecanumDrive;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "straight test")

public class AutoIsabella extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {



        Pose2d initialPose = new Pose2d(new Vector2d(60, 10), Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder toLaunchZone = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(20,10),Math.toRadians(180))
                .waitSeconds(1.5);

        Action toLaunchZone2 = toLaunchZone.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(38,35),Math.toRadians(90))
                .build();



        Action firstTraj = toLaunchZone.build();


//        while (!isStarted() && !isStopRequested()) {
//            drive.updatePoseEstimate();
//            telemetry.addData("Robot position: ", drive.pose);
//            telemetry.update();
//        }
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Robot position: ", drive.updatePoseEstimate());
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        firstTraj,
                        toLaunchZone2





                )

        );

    }




}








