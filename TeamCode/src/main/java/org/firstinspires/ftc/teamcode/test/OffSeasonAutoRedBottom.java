package org.firstinspires.ftc.teamcode.test;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.lang.Math;

@Autonomous(name = "ElyseILTREDSIDEcloseshooter")
public class OffSeasonAutoRedBottom extends LinearOpMode {

    private DcMotorEx intake;
    private DcMotorEx flywheel;
    private Servo flap;

    @Override
    public void runOpMode() {

        flywheel = hardwareMap.get(DcMotorEx.class, "FlyWheelMotor");
        flap = hardwareMap.get(Servo.class, "flapServo");
        intake = hardwareMap.get(DcMotorEx.class, "IntakeMotor");

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
//FIXME CHANGE VALUES TO MATCH CURRENT AUTO
        // (-50, -50, 45°) → (-50, 50, -45°)
        Pose2d initialPose = new Pose2d(-50, 50, Math.toRadians(-45)
        );

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // (-12.7, -12, 270°) → (-12.7, 12, 90°)
        TrajectoryActionBuilder toShoot = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(54, 6), Math.toRadians(-205))
                .waitSeconds(1);

        // turn(25°) → turn(-25°)
        // (-12, -58, 270°) → (-12, 58, 90°)
        TrajectoryActionBuilder firstintakeBalls = toShoot.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(36, 25), Math.toRadians(-270))
                .waitSeconds(1);

        // (-12.5, -13.2, 45°) → (-12.5, 13.2, -45°)
        TrajectoryActionBuilder pickupfirstintakeBalls = firstintakeBalls.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(36, 45), Math.toRadians(-270));

        TrajectoryActionBuilder backToShoot = pickupfirstintakeBalls.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(54, 6), Math.toRadians(-205))
                .waitSeconds(1);

        TrajectoryActionBuilder secondintakeBalls = backToShoot.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(12, 28), Math.toRadians(-270))
                .waitSeconds(1);
        TrajectoryActionBuilder pickupsecondintakeBalls = secondintakeBalls.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(12, 45), Math.toRadians(-270));
        TrajectoryActionBuilder backToShoot2 = pickupsecondintakeBalls.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(54, 6), Math.toRadians(-205))
                .waitSeconds(1);


        // turn(90°) → turn(-90°)
        // lineToX unchanged
        Action outOfZone = backToShoot2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(30, -34), Math.toRadians(-180))
                .build();

        Action firstTraj = toShoot.build();
        Action secondTraj = firstintakeBalls.build();
        Action thirdTraj = pickupfirstintakeBalls.build();
        Action fourthTraj = backToShoot.build();
        Action fifthTraj = secondintakeBalls.build();
        Action sixthTraj = pickupsecondintakeBalls.build();
        Action seventhTraj = backToShoot2.build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(

                        flywheelOn(),

                        new SequentialAction(
                                firstTraj,
                                flapThreeTimes(),

                                new ParallelAction(
                                        secondTraj,
                                        flywheelOn()
                                ),

                                thirdTraj,
                                flapThreeTimes(),

                                outOfZone,
                                flywheelOff()
                        )
                )
        );
    }

    private Action intakeForSeconds(double seconds) {
        return new Action() {

            ElapsedTime timer = new ElapsedTime();
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {
                    intake.setPower(-0.8);
                    timer.reset();
                    initialized = true;
                }

                if (timer.seconds() >= seconds) {
                    intake.setPower(0);
                    return true;
                }

                return false;
            }
        };
    }

    private Action flywheelOn() {
        return packet -> {
            flywheel.setPower(-0.6);
            return false;
        };
    }

    private Action flywheelOff() {
        return packet -> {
            flywheel.setPower(0);
            return true;
        };
    }

    private Action flapThreeTimes() {
        return new Action() {

            int hitCount = 0;
            boolean flapOut = false;

            ElapsedTime timer = new ElapsedTime();
            double lastToggleTime = 0;

            final double FLAP_OUT = 0.65;
            final double FLAP_IN = 0.35;
            final double TOGGLE_TIME = 0.25;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                double now = timer.seconds();

                if (lastToggleTime == 0) {
                    flap.setPosition(FLAP_IN);
                    lastToggleTime = now;
                }

                if (now - lastToggleTime >= TOGGLE_TIME) {
                    flapOut = !flapOut;
                    flap.setPosition(flapOut ? FLAP_OUT : FLAP_IN);
                    lastToggleTime = now;

                    if (!flapOut) hitCount++;
                }

                return hitCount >= 3;
            }
        };
    }
}



