    //package org.firstinspires.ftc.teamcode.test;
    //
    //import androidx.annotation.NonNull;
    //
    //import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
    //import com.acmerobotics.roadrunner.Action;
    //import com.acmerobotics.roadrunner.Pose2d;
    //import com.acmerobotics.roadrunner.SequentialAction;
    //import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
    ////import com.acmerobotics.roadrunner.Vector2d;
    //import com.acmerobotics.roadrunner.Vector2d;
    //import com.acmerobotics.roadrunner.ftc.Actions;
    //import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    //import com.qualcomm.robotcore.eventloop.opmode.Disabled;
    //import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    //import com.qualcomm.robotcore.hardware.DcMotor;
    //import com.qualcomm.robotcore.hardware.DcMotorEx;
    //import com.qualcomm.robotcore.hardware.DcMotorSimple;
    //import com.qualcomm.robotcore.hardware.HardwareMap;
    //import com.qualcomm.robotcore.util.ElapsedTime;
    //
    //import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
    //import org.firstinspires.ftc.teamcode.Comp5218MecanumDrive;
    //import org.firstinspires.ftc.teamcode.MecanumDrive;
    //
    //@Autonomous(name = "RedBottomAuto3")
    //
    //<<<<<<< HEAD
    //@Autonomous(name = "RedBottomAuto")
    //
    //=======
    //>>>>>>> 2fa159b3b0e33c595f96928614b638217317a4eb
    //public class RedBottomAuto extends LinearOpMode {
    //    @Override
    //    public void runOpMode() throws InterruptedException {
    //        Launcher launcher = new Launcher(hardwareMap);
    //        Intake intake = new Intake(hardwareMap);
    //
    //<<<<<<< HEAD
    //        Pose2d initialPose = new Pose2d(62, 10, Math.toRadians(90));
    //
    //        Comp5218MecanumDrive drive = new Comp5218MecanumDrive(hardwareMap, initialPose);
    //
    //        //action builds from the frive steps passed to it
    //        TrajectoryActionBuilder toLaunchZone = drive.actionBuilder(initialPose)
    //                .turn(Math.toRadians(180))
    //                .lineToY(25)
    //                .strafeTo(new Vector2d(-22, 25))
    //                .turn(Math.toRadians(45));
    //
    //        TrajectoryActionBuilder toArtifact = drive.actionBuilder(new Pose2d(-22, 25, Math.toRadians(225)))
    //                .turn(Math.toRadians(-136))
    //                .strafeTo(new Vector2d(-30, 47))
    //                .strafeTo(new Vector2d(-14, 47));
    //
    //
    //        Action toLaunchZone2 = toArtifact.endTrajectory().fresh()
    //                .strafeTo(new Vector2d(-40, 20))
    //                .turn(Math.toRadians(-36))
    //                .waitSeconds(1.5)
    //
    //                .build();
    //
    //
    //        Pose2d LaunchZone2EndPose = new Pose2d(38, -22, Math.toRadians(269));
    //        Action toParking = drive.actionBuilder(LaunchZone2EndPose)
    //                .strafeTo(new Vector2d(38, -22))
    //                .turn(Math.toRadians(45))
    //                .build();
    //=======
    //        Pose2d initialPose = new Pose2d(62, 10, Math.toRadians(180));
    //
    //       // Comp5218MecanumDrive drive = new Comp5218MecanumDrive(hardwareMap, initialPose);
    //         MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
    //
    //        TrajectoryActionBuilder toLaunchZone = drive.actionBuilder(initialPose)
    //                .strafeToLinearHeading(new Vector2d(-22,25),Math.toRadians(145));
    //>>>>>>> 2fa159b3b0e33c595f96928614b638217317a4eb
    //
    //
    //        Pose2d ParkingEndPose = new Pose2d(38, -22, Math.toRadians(314));
    //
    //<<<<<<< HEAD
    //        Action toLaunchZoneTraj = toLaunchZone.build();
    //        Action toArtifactPath = toArtifact.build();
    //
    //=======
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
////        Action nextTraj = toLaunchZone.endTrajectory().fresh()
////                .turn(Math.toRadians(45))
////                .build();
//
//
//        //if (isStopRequested()) return;
//
//        while (!isStopRequested() && opModeIsActive()) {
//            telemetry.addData("Robot position: ", drive.updatePoseEstimate());
//            telemetry.update();
//        }
//>>>>>>> 2fa159b3b0e33c595f96928614b638217317a4eb
//        waitForStart();
//
//        while (!isStopRequested() && opModeIsActive()) {
//            telemetry.addData("Robot position: ", drive.updatePoseEstimate());
//            telemetry.update();
//        }
//        //Running Sequence
//        Actions.runBlocking(
//                new SequentialAction(
//<<<<<<< HEAD
//
//
//=======
//                        firstTraj
//                        //nextTraj
//>>>>>>> 2fa159b3b0e33c595f96928614b638217317a4eb
//                )
//
//        );
//<<<<<<< HEAD
//
//        public class Launcher {
//            private DcMotorEx launcher;
//
//            public Launcher(HardwareMap hardwareMap) {
//                launcher = hardwareMap.get(DcMotorEx.class, "launcherMotor");
//                launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//                launcher.setDirection(DcMotorSimple.Direction.FORWARD);
//            }
//=======
//
//        //if (isStopRequested()) return;
//    }
//
//    public class Launcher {
//        private DcMotorEx launcher;
//        private ElapsedTime timer = new ElapsedTime();
//
//        public Launcher(HardwareMap hardwareMap) {
//            launcher = hardwareMap.get(DcMotorEx.class, "FlywheelMotor");
//            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            launcher.setDirection(DcMotorSimple.Direction.FORWARD);
//        }
//>>>>>>> 2fa159b3b0e33c595f96928614b638217317a4eb
//
//            public class LauncherForward implements Action {
//                // move the motor in the direction that launches the ball
//                private boolean initialized = false;
//
//<<<<<<< HEAD
//                // actions are formatted via telemetry packets as below
//                @Override
//                public boolean run(@NonNull TelemetryPacket packet) {
//                    // powers on motor, if it is not on
//                    if (!initialized) {
//                        launcher.setPower(0.8);
//                        initialized = true;
//                        sleep(300);
//
//                    }
//
//
//                    return true;
//
//                }
//=======
//            // actions are formatted via telemetry packets as below
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                // powers on motor, if it is not on
//                if (!initialized) {
//                    timer.reset();
//                    if (timer.milliseconds() < 2000) {
//                        launcher.setPower(0.8);
//                    } else {
//                        launcher.setPower(0);
//                        timer.reset();
//                        initialized = true;
//                    }
//                }
//                return true;
//>>>>>>> 2fa159b3b0e33c595f96928614b638217317a4eb
//            }
//
//            public Action launcherForward() {
//                return new LauncherForward();
//            }
//
//
//<<<<<<< HEAD
//            public class LauncherBackwards implements Action {
//                private boolean initialized = false;
//
//
//                @Override
//                public boolean run(@NonNull TelemetryPacket packet) {
//                    if (!initialized) {
//                        launcher.setPower(-0.8);
//                        initialized = true;
//                    }
//
//                    return true;
//                }
//=======
//        public class LauncherBackwards implements Action {
//            private boolean initialized = false;
//            private ElapsedTime timer1;
//
//            public boolean run(@NonNull TelemetryPacket packet) {
//                // powers on motor, if it is not on
//                if (!initialized) {
//                    timer1.reset();
//                    if (timer1.milliseconds() < 2000) {
//                        launcher.setPower(-0.8);
//                    } else {
//                        launcher.setPower(0);
//                        timer1.reset();
//                        initialized = true;
//                    }
//                }
//                return true;
//>>>>>>> 2fa159b3b0e33c595f96928614b638217317a4eb
//            }
//
//<<<<<<< HEAD
//            public Action launcherBackwards() {
//                return new LauncherBackwards();
//            }
//=======
//    }
//
//    public class Intake {
//        private DcMotorEx intakeMotor;
//
//        private ElapsedTime timer1 = new ElapsedTime();
//
//
//        public Intake(HardwareMap hardwareMap) {
//            intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
//
//            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//>>>>>>> 2fa159b3b0e33c595f96928614b638217317a4eb
//
//        }
//
//<<<<<<< HEAD
//        public class Intake {
//            private DcMotorEx intakeMotor;
//
//
//            public Intake(HardwareMap hardwareMap) {
//                intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
//
//                intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//                intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//
//            }
//
//            public class IntakeIn implements Action {
//                // move the motor in the direction that moves the ball into the robot;
//                private boolean initialized = false;
//
//                // actions are formatted via telemetry packets as below
//                @Override
//                public boolean run(@NonNull TelemetryPacket packet) {
//                    // powers on motor, if it is not on
//                    if (!initialized) {
//                        intakeMotor.setPower(0.8);
//                        initialized = true;
//                        sleep(300);
//
//                    }
//
//
//                    return true;
//
//                }
//=======
//            // actions are formatted via telemetry packets as below
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                // powers on motor, if it is not on
//                if (!initialized) {
//                    timer1.reset();
//                    if (timer1.milliseconds() < 2000) {
//                        intakeMotor.setPower(0.8);
//                    } else {
//                        intakeMotor.setPower(0);
//                        timer1.reset();
//                        initialized = true;
//                    }
//                }
//
//                return true;
//
//>>>>>>> 2fa159b3b0e33c595f96928614b638217317a4eb
//            }
//
//            public Action intakeIn() {
//                return new Intake.IntakeIn();
//            }
//
//            public class IntakeOut implements Action {
//                private boolean initialized = false;
//
//
//<<<<<<< HEAD
//                @Override
//                public boolean run(@NonNull TelemetryPacket packet) {
//                    if (!initialized) {
//                        intakeMotor.setPower(-0.8);
//                        initialized = true;
//                    }
//
//                    return true;
//=======
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    timer1.reset();
//                    if (timer1.milliseconds() < 2000) {
//                        intakeMotor.setPower(-0.8);
//                    } else {
//                        intakeMotor.setPower(0);
//                        timer1.reset();
//                        initialized = true;
//                    }
//>>>>>>> 2fa159b3b0e33c595f96928614b638217317a4eb
//                }
//            }
//
//            public Action intakeOut() {
//                return new Intake.IntakeOut();
//            }
//
//
//        }
//    }
//}
