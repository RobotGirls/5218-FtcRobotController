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
public class ElyseILTredsidecloseshooter extends LinearOpMode {

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

        // (-50, -50, 45°) → (-50, 50, -45°)
        Pose2d initialPose = new Pose2d(
                -50,
                50,
                Math.toRadians(-45)
        );

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // (-12.7, -12, 270°) → (-12.7, 12, 90°)
        TrajectoryActionBuilder toShoot = drive.actionBuilder(initialPose)
                .setReversed(true)
                .splineTo(
                        new Vector2d(-12.7, 12),
                        Math.toRadians(90)
                );

        // turn(25°) → turn(-25°)
        // (-12, -58, 270°) → (-12, 58, 90°)
        TrajectoryActionBuilder intakeBalls = toShoot.endTrajectory().fresh()
                .turn(Math.toRadians(-25))
                .splineTo(
                        new Vector2d(-12, 58),
                        Math.toRadians(90)
                );

        // (-12.5, -13.2, 45°) → (-12.5, 13.2, -45°)
        TrajectoryActionBuilder backToShoot = intakeBalls.endTrajectory().fresh()
                .setReversed(true)
                .splineTo(
                        new Vector2d(-12.5, 13.2),
                        Math.toRadians(-45)
                );

        // turn(90°) → turn(-90°)
        // lineToX unchanged
        Action outOfZone = backToShoot.endTrajectory().fresh()
                .turn(Math.toRadians(-90))
                .lineToX(2)
                .build();

        Action firstTraj = toShoot.build();
        Action secondTraj = intakeBalls.build();
        Action thirdTraj = backToShoot.build();

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
                                        intakeForSeconds(5.0)
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
