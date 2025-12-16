package org.firstinspires.ftc.teamcode.test;

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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "BlueBottomAuto")
public class BlueBottomAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Launcher launcher = new Launcher(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Flap flap = new Flap(hardwareMap);

        Pose2d initialPose = new Pose2d(60, -20, Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder toLaunchZone = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-16, -35), Math.toRadians(235))
                .waitSeconds(1.5);

        TrajectoryActionBuilder toArtifact = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-11, -25), Math.toRadians(-90));

        TrajectoryActionBuilder toIntake = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-11, -49), Math.toRadians(-90))
                .waitSeconds(1.5);

        TrajectoryActionBuilder toLaunchZone2 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-34, -34), Math.toRadians(230))
                .waitSeconds(1.5);

        Action toPark = toLaunchZone2.endTrajectory().fresh()
                .strafeTo(new Vector2d(60, 5))
                .build();

        Action firstTraj = toLaunchZone.build();
        Action secondTraj = toArtifact.build();
        Action thirdTraj = toIntake.build();
        Action fourthTraj = toLaunchZone2.build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        firstTraj,

                 //       flap.closeFlap(),

                 //       launcher.launcherForward(),

                //        new ParallelAction(
//                                intake.intakeIn(),
//                                launcher.launcherForward();
                //        ),

                //        flap.openFlap(),

                        secondTraj,
                        thirdTraj,

                        fourthTraj,

                 //       flap.closeFlap(),

                        toPark
                )
        );
    }

    /* ===================== FLAP (SERVO) ===================== */
    public class Flap {
        private Servo flap;

        public Flap(HardwareMap hardwareMap) {
            flap = hardwareMap.get(Servo.class, "Flap");
        }

        public class CloseFlap implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                flap.setPosition(0.55);
                return false;
            }
        }

        public Action closeFlap() {
            return new CloseFlap();
        }

        public class OpenFlap implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                flap.setPosition(1.0);
                return false;
            }
        }

        public Action openFlap() {
            return new OpenFlap();
        }
    }

    /* ===================== LAUNCHER ===================== */
    public class Launcher {
        private DcMotorEx launcher;
        private ElapsedTime timer = new ElapsedTime();

        public Launcher(HardwareMap hardwareMap) {
            launcher = hardwareMap.get(DcMotorEx.class, "FlywheelMotor");
            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LauncherForward implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    launcher.setPower(-0.6);
                    timer.reset();
                    initialized = true;
                }

                if (timer.milliseconds() < 4000) {
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
    }

    /* ===================== INTAKE ===================== */
    public class Intake {
        private DcMotorEx intakeMotor;
        private ElapsedTime timer = new ElapsedTime();

        public Intake(HardwareMap hardwareMap) {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class IntakeIn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeMotor.setPower(-0.8);
                    timer.reset();
                    initialized = true;
                }

                if (timer.milliseconds() < 4000) {
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
    }
}


