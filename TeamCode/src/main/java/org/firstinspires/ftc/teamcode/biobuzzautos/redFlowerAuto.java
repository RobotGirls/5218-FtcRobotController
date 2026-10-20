package org.firstinspires.ftc.teamcode.biobuzzautos;

import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TankDrive;


import java.lang.Math;
@Autonomous(name = "RED Flower Auto Park")
public class redFlowerAuto extends LinearOpMode{
//    private DcMotorEx intake;
//    private DcMotorEx flywheel;
//    private Servo flap;

    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(10, -60, Math.toRadians(0));
        TankDrive drive = new TankDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder toPark = drive.actionBuilder(initialPose)
                .lineToX(-30);


        Action outOfZone = toPark.endTrajectory().fresh()
                .waitSeconds(1)
                .build();

        Action firstTraj = toPark.build();


        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(



                new SequentialAction(
                        firstTraj,



                        outOfZone

                )
        );

    }
}
//tested&works
