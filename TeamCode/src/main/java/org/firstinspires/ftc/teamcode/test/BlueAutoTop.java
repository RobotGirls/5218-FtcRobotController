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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Comp5218MecanumDrive;

@Disabled
@Autonomous(name = "BlueObservationAuto")
public class BlueAutoTop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        Comp5218MecanumDrive drive = new Comp5218MecanumDrive(hardwareMap, initialPose);
        TrajectoryActionBuilder toSubmersible = drive.actionBuilder(initialPose)
                //.turn(Math.toRadians(45))
                .strafeTo(new Vector2d(-20, 29))
                .waitSeconds(2);
        Action toSubmersibleTraj = toSubmersible.build();

//        Action toObservation = toSubmersible.fresh()
//                .waitSeconds(2)
//                .lineToY(5)
//                .waitSeconds(1)
//                .setTangent(Math.toRadians(0))
//                .lineToX(30)
//                .waitSeconds(2)
//                //  .setTangent(0)
//                .build();
//
//        Pose2d observationEndPose = new Pose2d(30, 5, Math.toRadians(0)); // Example
//        Action toSample = drive.actionBuilder(observationEndPose)
//                .waitSeconds(2)
//                .lineToY(10)
//                .lineToX(40)
//                .build();
        Action toObservation = toSubmersible.fresh()
                .waitSeconds(2)
                .strafeTo(new Vector2d(35, -5))
                .lineToY(10)
                .lineToX(5)
                .lineToY(-20)
                .lineToY(10)
                .waitSeconds(2)
                .lineToY(-10)

                //  .setTangent(0)
                .build();

        Pose2d observationEndPose = new Pose2d(5, 10, Math.toRadians(0)); // Example
        Action toSample = drive.actionBuilder(observationEndPose)
                .waitSeconds(2)
                .strafeTo(new Vector2d(-35, 29))
                .build();
        Pose2d sampleEndPose = new Pose2d(5, 10, Math.toRadians(0)); // Example
        Action toHangSpecimen = drive.actionBuilder(sampleEndPose)
                .waitSeconds(2)
                .lineToY(5)
                .waitSeconds(1)
                .setTangent(Math.toRadians(0))
                .lineToX(30)
                .waitSeconds(2)
                //  .setTangent(0)
                .build();


        Actions.runBlocking(claw.closeClaw());


        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Robot position: ", drive.updatePoseEstimate());
        }
        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        lift.liftUp(),
                        toSubmersibleTraj,
                        lift.liftDown(),
                        claw.openClaw(),
                        toObservation,
                        claw.closeClaw(),
                        lift.liftUp(),
                        toSample,
                        lift.liftDown(),
                        claw.openClaw(),
                        toHangSpecimen

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
