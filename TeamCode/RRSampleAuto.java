package org.firstinspires.ftc.teamcode.opmodes;

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

import org.firstinspires.ftc.teamcode.Comp5218MecanumDrive;

@Autonomous(name = "RRSampleAuto")
public class RRSampleAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-8, 65, 300);
        Comp5218MecanumDrive drive = new Comp5218MecanumDrive(hardwareMap, initialPose);

        Lift lift = new Lift(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        WRClaw wrClaw = new WRClaw(hardwareMap);
        HangLift hangLift = new HangLift(hardwareMap);

        // Trajectories
        TrajectoryActionBuilder toSub = drive.actionBuilder(initialPose).lineToY(44);
        TrajectoryActionBuilder closerSub = toSub.endTrajectory().lineToY(40);
        TrajectoryActionBuilder fromSub = closerSub.endTrajectory().lineToY(55);

        // Correct the chaining of the toSample trajectory, and use .build() to convert TrajectoryActionBuilder to Action
        TrajectoryActionBuilder toSample = fromSub.endTrajectory().strafeTo(new Vector2d(-42, 8))
                .lineToY(60)
                .lineToY(8)
                .lineToX(-52)
                .lineToY(60)
                .lineToY(35)

                .turn(3.2)
                .lineToY(58);
        //close claw
        //lift the lift up

        TrajectoryActionBuilder toHangS = toSample.endTrajectory().strafeTo(new Vector2d(-8, 44))
                .turn(-3.2)
                .lineToY(35);



        //lift down
        TrajectoryActionBuilder closerSub2 = toHangS.endTrajectory().lineToY(50);

        //claw open and lift up
        TrajectoryActionBuilder HP = closerSub2.endTrajectory().strafeTo(new Vector2d(-48, 50))
                .turn(3.2)
                .lineToY(60);
        // close claw
        //lift up
        TrajectoryActionBuilder toSub2 = HP.endTrajectory().strafeTo(new Vector2d(8, 44))
                .strafeTo(new Vector2d(-8, 44))
                .turn(-3.2)
                .lineToY(40);

        //loower lift

        TrajectoryActionBuilder prepPark = toSub2.endTrajectory()
                .lineToY(55);
        //open claw
        //lower lift to the bottom and park? or push another samaple








        // Actions
        Action hangLiftUp = hangLift.hangLiftUp();
        Action liftAndOpenClaw = new SequentialAction(lift.liftUp(), claw.openClaw());
        Action moveBackAndLowerLift = new SequentialAction(lift.lowerLift());
        Action liftDownAndOpenClaw = new SequentialAction( claw.openClaw(),lift.lowerLift2());

        Action toHP = toSample.build(); // Ensure this uses the correct build method
        Action waitAndCloseClaw = new SequentialAction(new WaitAction(1000), claw.closeClaw());

        //Actions.runBlocking(claw.closeClaw);


        // Running Actions
        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        hangLiftUp,
                        new ParallelAction(
                                toSub.build(), // Ensure toSub is built as an Action
                                lift.liftUp()
                        ),
                        closerSub.build(),
                        moveBackAndLowerLift,
                        fromSub.build(),
                        liftDownAndOpenClaw,
                        new SequentialAction(
                                toSample.build(),  // Executes the trajectory toSample as an Action
                                waitAndCloseClaw

                        ),
                        new ParallelAction(
                                lift.liftUp(),
                                toHangS.build()
                        ),
                        moveBackAndLowerLift,
                        closerSub2.build(),
                        liftDownAndOpenClaw,
                        HP.build(),
                        waitAndCloseClaw,

                        new ParallelAction(
                                lift.liftUp(),
                                toSub2.build()
                        ),
                        moveBackAndLowerLift,
                        prepPark.build(),
                        liftDownAndOpenClaw



                )
        );
    }

    // Other classes (Lift, Claw, WRClaw, HangLift, etc.) remain unchanged

    // WaitAction to implement a delay
    public class WaitAction implements Action {
        private long waitTime;
        private long startTime;

        public WaitAction(long waitTime) {
            this.waitTime = waitTime;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (startTime == 0) {
                startTime = System.currentTimeMillis();
            }
            return System.currentTimeMillis() - startTime < waitTime;
        }
    }

    //    public class Lift {
//        private DcMotorEx lift;
//
//        public Lift(HardwareMap hardwareMap) {
//            lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
//            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            lift.setDirection(DcMotorSimple.Direction.FORWARD);
//            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//
//        // Lift up method
//        public Action liftUp() {
//            return new Action() {
//                private boolean initialized = false;
//
//                @Override
//                public boolean run(@NonNull TelemetryPacket packet) {
//                    if (!initialized) {
//                        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                        lift.setPower(-1); // Start moving the lift up
//                        initialized = true;
//                    }
//
//                    // Current lift position
//                    double liftPos = lift.getCurrentPosition();
//                    packet.put("liftPos", liftPos);
//                    telemetry.addData("lift Encoder", liftPos);
//                    telemetry.update();
//
//
//                    // Threshold for stopping (prevent overshooting)
//                    if (liftPos <= -6000) {
//                        lift.setPower(0); // Stop lift movement
//                        return false; // Stop the action
//                    }
//
//                    return true;
//                }
//            };
//        }
//
//        // Lower lift method
//        public Action lowerLift() {
//            return new Action() {
//                private boolean initialized = false;
//
//                @Override
//                public boolean run(@NonNull TelemetryPacket packet) {
//                    if (!initialized) {
////                        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                        lift.setPower(-1); // Start lowering the lift
//                        initialized = true;
//                    }
//
//                    // Current lift position
//                    double liftPos = lift.getCurrentPosition();
//                    packet.put("liftPos", liftPos);
//                    telemetry.addData("lift Encoder", liftPos);
//                    telemetry.update();
//
//
//                    // Threshold for stopping (prevent overshooting)
//                    if (liftPos >= -3000) {
//                        lift.setPower(0); // Stop lift movement
//                        return false; // Stop the action
//                    }
//
//                    return true;
//                }
//            };
//        }
//
//        // Another lower lift method (lower to another position)
//        public Action lowerLift2() {
//            return new Action() {
//                private boolean initialized = false;
//
//                @Override
//                public boolean run(@NonNull TelemetryPacket packet) {
//                    if (!initialized) {
////                        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                        lift.setPower(1); // Start lowering the lift
//                        initialized = true;
//                    }
//
//                    // Current lift position
//                    double liftPos = lift.getCurrentPosition();
//                    packet.put("liftPos", liftPos);
//                    telemetry.addData("lift Encoder", liftPos);
//                    telemetry.update();
//
//
//                    // Threshold for stopping (prevent overshooting)
//                    if (liftPos >= -3500) {
//                        lift.setPower(0); // Stop lift movement
//                        return false; // Stop the action
//                    }
//
//                    return true;
//                }
//            };
//        }
//    }
    public class Lift {
        private DcMotorEx lift;
        private boolean encoderReset = false;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
            // Reset encoder once during initialization
            resetEncoder();
        }

        private void resetEncoder() {
            if (!encoderReset) {
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                encoderReset = true;
            }
        }

        // Lift up method
        public Action liftUp() {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        resetEncoder();
                        lift.setPower(-1); // Start moving the lift up
                        initialized = true;
                    }

                    // Current lift position
                    double liftPos = lift.getCurrentPosition();
                    packet.put("liftPos", liftPos);
                    telemetry.addData("lift Encoder", liftPos);
                    telemetry.update();

                    // Threshold for stopping (prevent overshooting)
                    if (liftPos <= -6000) {
                        lift.setPower(0); // Stop lift movement
                        return false; // Stop the action
                    }

                    return true;
                }
            };
        }

        // Lower lift method
        public Action lowerLift() {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        resetEncoder();
                        lift.setPower(1); // Start lowering the lift
                        initialized = true;
                    }

                    // Current lift position
                    double liftPos = lift.getCurrentPosition();
                    packet.put("liftPos", liftPos);
                    telemetry.addData("lift Encoder", liftPos);
                    telemetry.update();

                    // Threshold for stopping (prevent overshooting)
                    if (liftPos >= -5000) {
                        lift.setPower(0); // Stop lift movement
                        return false; // Stop the action
                    }

                    return true;
                }
            };
        }

        // Another lower lift method (lower to another position)
        public Action lowerLift2() {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        resetEncoder();
                        lift.setPower(1); // Start lowering the lift
                        initialized = true;
                    }

                    // Current lift position
                    double liftPos = lift.getCurrentPosition();
                    packet.put("liftPos", liftPos);
                    telemetry.addData("lift Encoder", liftPos);
                    telemetry.update();

                    // Threshold for stopping (prevent overshooting)
                    if (liftPos >= -3000) {
                        lift.setPower(0); // Stop lift movement
                        return false; // Stop the action
                    }

                    return true;
                }
            };
        }
    }


    // Claw mechanism class
    public class Claw {
        private Servo miniClawServo;

        public Claw(HardwareMap hardwareMap) {
            miniClawServo = hardwareMap.get(Servo.class, "miniClawServo");
            initminiClawServo(); // Initialize the servo to the down position

        }
        public void initminiClawServo() {
            miniClawServo.setPosition(0.75); // Set to the down position
        }

        public Action openClaw() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    miniClawServo.setPosition(0.3);
                    return false;
                }
            };
        }

        public Action closeClaw() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    miniClawServo.setPosition(0.75);
                    return false;
                }
            };
        }
    }

//    // WRClaw mechanism class
//    public class WRClaw {
//        public  Action downWRClaw;
//        private Servo wheelieRotationServo;
//
//        public WRClaw(HardwareMap hardwareMap) {
//            wheelieRotationServo = hardwareMap.get(Servo.class, "wheelieRotationServo");
//        }
//
//        public Action upWRClaw() {
//            return new Action() {
//                @Override
//                public boolean run(@NonNull TelemetryPacket packet) {
//                    wheelieRotationServo.setPosition(0.6);
//                    return false;
//                }
//            };
//        }
//
//        public Action downWRClaw() {
//            return new Action() {
//                @Override
//                public boolean run(@NonNull TelemetryPacket packet) {
//                    wheelieRotationServo.setPosition(1);
//                    return false;
//                }
//            };
//        }
//    }

    // WRClaw mechanism class
    public class WRClaw {
        private Servo wheelieRotationServo;

        public WRClaw(HardwareMap hardwareMap) {
            wheelieRotationServo = hardwareMap.get(Servo.class, "wheelieRotationServo");
            initWRServo(); // Initialize the servo to the down position
        }

        // Initializes the WR servo to the down position
        public void initWRServo() {
            wheelieRotationServo.setPosition(0.6); // Set to the down position
        }

        public Action upWRClaw() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    wheelieRotationServo.setPosition(0.05);
                    return false;
                }
            };
        }

        public Action downWRClaw() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    wheelieRotationServo.setPosition(0.6);
                    return false;
                }
            };
        }
    }

    // HangLift mechanism class
    public class HangLift {
        private DcMotorEx hangLeftMotor;
        private DcMotorEx hangRightMotor;

        public HangLift(HardwareMap hardwareMap) {
            hangLeftMotor = hardwareMap.get(DcMotorEx.class, "hangLeftMotor");
            hangLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hangLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            hangRightMotor = hardwareMap.get(DcMotorEx.class, "hangRightMotor");
            hangRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hangRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            hangLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hangLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hangRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hangRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public Action hangLiftUp() {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        hangLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        hangRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        hangLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        hangRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        sleep(200);
                        hangLeftMotor.setPower(-1);
                        hangRightMotor.setPower(-1);
                        initialized = true;
                    }

                    double rightPos = hangRightMotor.getCurrentPosition();
                    double leftPos = hangLeftMotor.getCurrentPosition();
                    packet.put("hangLeftPos", leftPos);
                    packet.put("hangRightPos", rightPos);
                    telemetry.addData("Left Encoder", leftPos);
                    telemetry.addData("Right Encoder", rightPos);
                    // telemetry.addData("Right Encoder", rightPos);

                    telemetry.update();

                    if (leftPos <= -3200 && rightPos <= -3200) {
                        hangLeftMotor.setPower(0);
                        hangRightMotor.setPower(0);
                        return false; // Stop the action
                    }
                    return true;
                }
            };
        }
    }
}
