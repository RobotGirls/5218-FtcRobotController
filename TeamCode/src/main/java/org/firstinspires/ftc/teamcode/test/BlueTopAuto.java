package org.firstinspires.ftc.teamcode.test;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Comp5218MecanumDrive;

@Disabled
@Autonomous(name = "BlueTopAuto")
public class BlueTopAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Launcher launcher = new Launcher(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        Comp5218MecanumDrive drive = new Comp5218MecanumDrive(hardwareMap, initialPose);
        TrajectoryActionBuilder toLaunchingPosition = drive.actionBuilder(initialPose)
                .lineToY(15)
                .waitSeconds(2);
        Action toLaunchingPositionTraj = toLaunchingPosition.build();

        Action toSpikeMark = toLaunchingPosition.fresh()
                .turn(Math.toRadians(315))
                .lineToY(47)
                .waitSeconds(2.5)
                .build();
        Pose2d SpikeMarkEndPose = new Pose2d(-49, 47, Math.toRadians(90)); // Example
        Action toLaunchingPosition2 = drive.actionBuilder(SpikeMarkEndPose)

                .turn(Math.toRadians(150))
                .lineToY(25)
                .turn(Math.toRadians(255))
                .waitSeconds(2)

                .build();
        Pose2d LaunchingPosition2EndPose = new Pose2d(-49, 25, Math.toRadians(135));
        Action toParking = drive.actionBuilder(LaunchingPosition2EndPose)
                .lineToY(-34)
                .turn(Math.toRadians(45))

                .build();
        Pose2d  ParkingEndPose = new Pose2d(-49,-34,Math.toRadians(180));






        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Robot position: ", drive.updatePoseEstimate());
        }
        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        toLaunchingPositionTraj,
                        launcher.launcherForward(),
                        toSpikeMark,
                        intake.intakeIn(),
                        toLaunchingPosition2,
                        launcher.launcherForward(),
                        toParking

                        // For Reference:
                        //  toSubmersibleTraj,
                        //  lift.liftDown(),
                        //  claw.openClaw(),
                        //  toObservation,
                        //  claw.closeClaw(),
                        // lift.liftUp(),
                        // toSample,
                        //lift.liftDown(),
                        //  claw.openClaw(),
                        //  toHangSpecimen

                )
        );
    }

    public class Launcher {
        private DcMotorEx launcher;

        public Launcher(HardwareMap hardwareMap) {
            launcher = hardwareMap.get(DcMotorEx.class, "launcherMotor");
            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LauncherForward implements Action {
            // move the motor in the direction that launches the ball
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    launcher.setPower(0.8);
                    initialized = true;
                    sleep(300);

                }


                return true;

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
                    launcher.setPower(-0.8);
                    initialized = true;
                }

                return true;
            }
        }
        public Action launcherBackwards() {
            return new LauncherBackwards();
        }

    }

    public class Intake {
        private DcMotorEx intakeMotor;


        public Intake(HardwareMap hardwareMap) {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        }
        public class IntakeIn implements Action {
            // move the motor in the direction that moves the ball into the robot;
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    intakeMotor.setPower(0.8);
                    initialized = true;
                    sleep(300);

                }


                return true;

            }
        }
        public Action intakeIn() {
            return new Intake.IntakeIn();
        }

        public class IntakeOut implements Action {
            private boolean initialized = false;


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeMotor.setPower(-0.8);
                    initialized = true;
                }

                return true;
            }
        }
        public Action intakeOut() {
            return new Intake.IntakeOut();
        }


    }
}

