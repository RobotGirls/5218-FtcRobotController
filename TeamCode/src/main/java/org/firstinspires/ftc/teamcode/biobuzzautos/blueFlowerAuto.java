package org.firstinspires.ftc.teamcode.biobuzzautos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TankDrive;

@Autonomous(name = "BLUE Flower Auto Park2")
public class blueFlowerAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        Launcher launcher = new Launcher(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        Pose2d initialPose = new Pose2d(-10,60,0);
        TankDrive drive = new TankDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder toPark = drive.actionBuilder(initialPose)
                .lineToX(30);

        Action outOfZone = toPark.endTrajectory().fresh()
                .waitSeconds(1)
                .build();

        Action firstTraj = toPark.build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        firstTraj,
                        launcher.launcherForward(),
                        outOfZone
                )
        );
    }

    public class Launcher {
        private final DcMotorEx launcher;
        private final ElapsedTime timer;

        public Launcher(HardwareMap hardwareMap) {
            launcher = hardwareMap.get(DcMotorEx.class, "launcher");
            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            launcher.setDirection(DcMotorSimple.Direction.FORWARD);

            timer = new ElapsedTime();
        }

        public class LauncherForward implements Action {
            // move the motor in the direction that launches the ball
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    launcher.setPower(-0.6);
                    initialized = true;
                    timer.reset();
                }
                double timerValueShooter = timer.milliseconds();
                telemetry.addData("Shooter timer", timerValueShooter);
                telemetry.update();
                if (timerValueShooter < 2000) {
                    return true;
                } else {
                    launcher.setPower(0);
                    return false;
                }
            }
        }

        public Action launcherForward() {
            return new LauncherForward();
        }

        public class LauncherBackwards implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    launcher.setPower(0.8);
                    initialized = true;
                    timer.reset();
                }
                double timerValueShooter = timer.milliseconds();
                telemetry.addData("Shooter timer", timerValueShooter);
                telemetry.update();
                if (timerValueShooter < 2000) {
                    return true;
                } else {
                    launcher.setPower(0);
                    return false;
                }
            }
        }

        public Action launcherBackwards() {
            return new LauncherBackwards();
        }
    }

    public class Intake {
        private final DcMotorEx intakeMotor;
        private final ElapsedTime timer1;

        public Intake(HardwareMap hardwareMap) {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            timer1 = new ElapsedTime();
        }

        public class IntakeIn implements Action {
            // move the motor in the direction that moves the ball into the robot;
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    intakeMotor.setPower(-0.8);
                    initialized = true;
                    timer1.reset();
                }
                double timerValue = timer1.milliseconds();
                telemetry.addData("Intake Timer", timerValue);
                telemetry.update();
                if (timerValue < 2000) {
                    return true;
                } else {
                    intakeMotor.setPower(0);
                    return false;
                }
            }
        }

        public Action intakeIn() {
            return new IntakeIn();
        }

        public class IntakeOut implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    intakeMotor.setPower(0.8);
                    initialized = true;
                    timer1.reset();
                }
                double timerValue = timer1.milliseconds();
                telemetry.addData("Intake Timer", timerValue);
                telemetry.update();
                if (timerValue < 2000) {
                    return true;
                } else {
                    intakeMotor.setPower(0);
                    return false;
                }
            }
        }

        public Action intakeOut() {
            return new IntakeOut();
        }
    }
}

//tested&works
