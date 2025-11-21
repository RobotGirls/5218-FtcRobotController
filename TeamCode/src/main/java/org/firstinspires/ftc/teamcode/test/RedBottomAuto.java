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
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.Comp5218MecanumDrive;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "RedBottomAuto")

public class RedBottomAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Launcher launcher = new Launcher(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        Pose2d initialPose = new Pose2d(60, 14, Math.toRadians(180));

        // takes the hardware and tuning inputs from mecanum drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder toLaunchZone = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-22,25),Math.toRadians(145))
                .waitSeconds(1.5);

        Action toPark = toLaunchZone.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(26,-20),Math.toRadians(90))
                .build();



//      //TrajectoryActionBuilder toArtifact = drive.actionBuilder(new Pose2d(-22,25,Math.toRadians(225)))
//       //      .turn(Math.toRadians(-136))
//      //       .strafeTo(new Vector2d(-30,47))
//               .strafeTo(new Vector2d(-14,47));



//       Action toLaunchZone2 = toArtifact.endTrajectory().fresh()
//               .strafeTo(new Vector2d(-40,20))
//               .turn(Math.toRadians(-36))
//
//            .build();
//
//
//
//       Pose2d LaunchZone2EndPose = new Pose2d(38,-22,Math.toRadians(269));
//       Action toParking = drive.actionBuilder(LaunchZone2EndPose)
//               .strafeTo(new Vector2d(38,-22))
//               .turn(Math.toRadians(45))
//                    .build();

//        Pose2d ParkingEndPose = new Pose2d(38,-22,Math.toRadians(314));
//
//        Action toLaunchZoneTraj = toLaunchZone.build();

//        Action toLaunchZone1=toLaunchZone.endTrajectory().fresh()
//                .build();









        Action firstTraj = toLaunchZone.build();


        //if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("Robot position: ", drive.updatePoseEstimate());
            telemetry.update();
        }
        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        firstTraj,
                        launcher.launcherForward(),
                        new ParallelAction(
                                intake.intakeIn(),
                                launcher.launcherForward()
                        ),
                        toPark




                )

        );

        //if (isStopRequested()) return;
    }

    public class Launcher {
        private DcMotorEx launcher;
        private ElapsedTime timer;

        public Launcher(HardwareMap hardwareMap) {
            launcher = hardwareMap.get(DcMotorEx.class, "FlywheelMotor");
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
            private ElapsedTime timer1;

            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    timer1.reset();
                    if (timer1.milliseconds() < 2000) {
                        launcher.setPower(0.8);

                    } else {
                        launcher.setPower(0);
                        timer1.reset();
                        initialized = true;
                    }
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

            private ElapsedTime timer1;


            public Intake(HardwareMap hardwareMap) {
                intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
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
                    telemetry.addData("Intake Timer",timerValue);
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
                public Action intakeOut() {
                    return new IntakeOut();
                }


            }
        }


    }







