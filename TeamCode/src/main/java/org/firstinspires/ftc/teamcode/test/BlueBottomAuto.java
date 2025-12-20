//package org.firstinspires.ftc.teamcode.test;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//<<<<<<< HEAD
////import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
////import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//=======
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//>>>>>>> 444ca3099c83078ecb3db68e14ff2546acbda136
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//<<<<<<< HEAD
//import com.qualcomm.robotcore.util.ElapsedTime;
//
////import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
////import org.firstinspires.ftc.teamcode.Comp5218MecanumDrive;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
//@Autonomous(name = "BlueBottomAuto")
//
//public class BlueBottomAuto extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Launcher launcher = new Launcher(hardwareMap);
//        Intake intake = new Intake(hardwareMap);
//
//        Pose2d initialPose = new Pose2d(60, 14, Math.toRadians(180));
//
//        // takes the hardware and tuning inputs from mecanum drive
//        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
//
//        TrajectoryActionBuilder toLaunchZone = drive.actionBuilder(initialPose)
//                .strafeToLinearHeading(new Vector2d(-22,25),Math.toRadians(145))
//                .waitSeconds(1.5);
//
//        Action toPark = toLaunchZone.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(26,-20),Math.toRadians(90))
//                .build();
//
//
//
////      //TrajectoryActionBuilder toArtifact = drive.actionBuilder(new Pose2d(-22,25,Math.toRadians(225)))
////       //      .turn(Math.toRadians(-136))
////      //       .strafeTo(new Vector2d(-30,47))
////               .strafeTo(new Vector2d(-14,47));
//
//
//
////       Action toLaunchZone2 = toArtifact.endTrajectory().fresh()
////               .strafeTo(new Vector2d(-40,20))
////               .turn(Math.toRadians(-36))
////
////            .build();
////
////
////
////       Pose2d LaunchZone2EndPose = new Pose2d(38,-22,Math.toRadians(269));
////       Action toParking = drive.actionBuilder(LaunchZone2EndPose)
////               .strafeTo(new Vector2d(38,-22))
////               .turn(Math.toRadians(45))
////                    .build();
//
////        Pose2d ParkingEndPose = new Pose2d(38,-22,Math.toRadians(314));
////
////        Action toLaunchZoneTraj = toLaunchZone.build();
//
////        Action toLaunchZone1=toLaunchZone.endTrajectory().fresh()
////                .build();
//
//
//
//
//
//
//
//
//
//        Action firstTraj = toLaunchZone.build();
//
//
//        //if (isStopRequested()) return;
//
//        while (!isStopRequested() && opModeIsActive()) {
//            telemetry.addData("Robot position: ", drive.updatePoseEstimate());
//            telemetry.update();
//        }
//=======
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
//@Autonomous(name = "BlueBottomAuto")
//public class BlueBottomAuto extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        Launcher launcher = new Launcher(hardwareMap);
//        Intake intake = new Intake(hardwareMap);
//        Flap flap = new Flap(hardwareMap);
//
//        Pose2d initialPose = new Pose2d(60, -20, Math.toRadians(180));
//
//        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
//
//        TrajectoryActionBuilder toLaunchZone = drive.actionBuilder(initialPose)
//                .strafeToLinearHeading(new Vector2d(-16, -35), Math.toRadians(235))
//                .waitSeconds(1.5);
//
//        TrajectoryActionBuilder toArtifact = drive.actionBuilder(initialPose)
//                .strafeToLinearHeading(new Vector2d(-11, -25), Math.toRadians(-90));
//
//        TrajectoryActionBuilder toIntake = drive.actionBuilder(initialPose)
//                .strafeToLinearHeading(new Vector2d(-11, -49), Math.toRadians(-90))
//                .waitSeconds(1.5);
//
//        TrajectoryActionBuilder toLaunchZone2 = drive.actionBuilder(initialPose)
//                .strafeToLinearHeading(new Vector2d(-34, -34), Math.toRadians(230))
//                .waitSeconds(1.5);
//
//        Action toPark = toLaunchZone2.endTrajectory().fresh()
//                .strafeTo(new Vector2d(60, 5))
//                .build();
//
//        Action firstTraj = toLaunchZone.build();
//        Action secondTraj = toArtifact.build();
//        Action thirdTraj = toIntake.build();
//        Action fourthTraj = toLaunchZone2.build();
//
//>>>>>>> 444ca3099c83078ecb3db68e14ff2546acbda136
//        waitForStart();
//        if (isStopRequested()) return;
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        firstTraj,
//<<<<<<< HEAD
//                        launcher.launcherForward(),
//                        new ParallelAction(
//                                intake.intakeIn(),
//                                launcher.launcherForward()
//                        ),
//                        toPark
//
//
//
//
//                )
//
//        );
//
//        //if (isStopRequested()) return;
//    }
//
//    public class Launcher {
//        private DcMotorEx launcher;
//        private ElapsedTime timer;
//=======
//
//                 //       flap.closeFlap(),
//
//                 //       launcher.launcherForward(),
//
//                //        new ParallelAction(
////                                intake.intakeIn(),
////                                launcher.launcherForward();
//                //        ),
//
//                //        flap.openFlap(),
//
//                        secondTraj,
//                        thirdTraj,
//
//                        fourthTraj,
//
//                 //       flap.closeFlap(),
//
//                        toPark
//                )
//        );
//    }
//
//    /* ===================== FLAP (SERVO) ===================== */
//    public class Flap {
//        private Servo flap;
//
//        public Flap(HardwareMap hardwareMap) {
//            flap = hardwareMap.get(Servo.class, "Flap");
//        }
//
//        public class CloseFlap implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                flap.setPosition(0.55);
//                return false;
//            }
//        }
//
//        public Action closeFlap() {
//            return new CloseFlap();
//        }
//
//        public class OpenFlap implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                flap.setPosition(1.0);
//                return false;
//            }
//        }
//
//        public Action openFlap() {
//            return new OpenFlap();
//        }
//    }
//
//    /* ===================== LAUNCHER ===================== */
//    public class Launcher {
//        private DcMotorEx launcher;
//        private ElapsedTime timer = new ElapsedTime();
//>>>>>>> 444ca3099c83078ecb3db68e14ff2546acbda136
//
//        public Launcher(HardwareMap hardwareMap) {
//            launcher = hardwareMap.get(DcMotorEx.class, "FlywheelMotor");
//            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            launcher.setDirection(DcMotorSimple.Direction.FORWARD);
//<<<<<<< HEAD
//
//            timer = new ElapsedTime();
//        }
//
//        public class LauncherForward implements Action {
//            // move the motor in the direction that launches the ball
//            private boolean initialized = false;
//
//            // actions are formatted via telemetry packets as below
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                // powers on motor, if it is not on
//                if (!initialized) {
//                    launcher.setPower(-0.6);
//                    initialized = true;
//                    timer.reset();
//                }
//                double timerValueShooter = timer.milliseconds();
//                telemetry.addData("Shooter timer", timerValueShooter);
//                telemetry.update();
//                if (timerValueShooter < 2000) {
//=======
//        }
//
//        public class LauncherForward implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    launcher.setPower(-0.6);
//                    timer.reset();
//                    initialized = true;
//                }
//
//                if (timer.milliseconds() < 4000) {
//>>>>>>> 444ca3099c83078ecb3db68e14ff2546acbda136
//                    return true;
//                } else {
//                    launcher.setPower(0);
//                    return false;
//                }
//            }
//        }
//
//        public Action launcherForward() {
//            return new LauncherForward();
//        }
//<<<<<<< HEAD
//
//        public class LauncherBackwards implements Action {
//            private boolean initialized = false;
//            private ElapsedTime timer1;
//
//            public boolean run(@NonNull TelemetryPacket packet) {
//                // powers on motor, if it is not on
//                if (!initialized) {
//                    timer1.reset();
//                    if (timer1.milliseconds() < 2000) {
//                        launcher.setPower(0.8);
//
//                    } else {
//                        launcher.setPower(0);
//                        timer1.reset();
//                        initialized = true;
//                    }
//                }
//                return true;
//            }
//        }
//
//        public Action launcherBackwards() {
//            return new LauncherBackwards();
//        }
//
//    }
//
//    public class Intake {
//        private DcMotorEx intakeMotor;
//
//        private ElapsedTime timer1;
//
//=======
//    }
//
//    /* ===================== INTAKE ===================== */
//    public class Intake {
//        private DcMotorEx intakeMotor;
//        private ElapsedTime timer = new ElapsedTime();
//>>>>>>> 444ca3099c83078ecb3db68e14ff2546acbda136
//
//        public Intake(HardwareMap hardwareMap) {
//            intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
//            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//<<<<<<< HEAD
//            timer1 = new ElapsedTime();
//
//        }
//        public class IntakeIn implements Action {
//            // move the motor in the direction that moves the ball into the robot;
//            private boolean initialized = false;
//
//            // actions are formatted via telemetry packets as below
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                // powers on motor, if it is not on
//                if (!initialized) {
//                    intakeMotor.setPower(-0.8);
//                    initialized = true;
//                    timer1.reset();
//                }
//                double timerValue = timer1.milliseconds();
//                telemetry.addData("Intake Timer",timerValue);
//                telemetry.update();
//                if (timerValue < 2000) {
//=======
//        }
//
//        public class IntakeIn implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    intakeMotor.setPower(-0.8);
//                    timer.reset();
//                    initialized = true;
//                }
//
//                if (timer.milliseconds() < 4000) {
//>>>>>>> 444ca3099c83078ecb3db68e14ff2546acbda136
//                    return true;
//                } else {
//                    intakeMotor.setPower(0);
//                    return false;
//                }
//<<<<<<< HEAD
//
//            }
//        }
//        public Action intakeIn() {
//            return new IntakeIn();
//        }
//
//        public class IntakeOut implements Action {
//            private boolean initialized = false;
//
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                // powers on motor, if it is not on
//                if (!initialized) {
//                    intakeMotor.setPower(0.8);
//                    initialized = true;
//                    timer1.reset();
//                }
//                double timerValue = timer1.milliseconds();
//                telemetry.addData("Intake Timer", timerValue);
//                telemetry.update();
//                if (timerValue < 2000) {
//                    return true;
//                } else {
//                    intakeMotor.setPower(0);
//                    return false;
//                }
//
//            }
//            public Action intakeOut() {
//                return new IntakeOut();
//            }
//
//
//        }
//    }
//
//
//}
//
//
//
//
//
//
//
//=======
//            }
//        }
//
//        public Action intakeIn() {
//            return new IntakeIn();
//        }
//    }
//}
//
//
//>>>>>>> 444ca3099c83078ecb3db68e14ff2546acbda136
