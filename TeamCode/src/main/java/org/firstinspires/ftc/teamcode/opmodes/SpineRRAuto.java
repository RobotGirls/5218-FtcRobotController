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

@Autonomous(name = "RRSpecimenWC")
public class SpineRRAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-8, 65, 300);
        Comp5218MecanumDrive drive = new Comp5218MecanumDrive(hardwareMap, initialPose);

      //  Lift lift = new Lift(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        ArmServo armServo = new ArmServo(hardwareMap);

        WRClaw wrClaw = new WRClaw(hardwareMap);
        HangLift hangLift = new HangLift(hardwareMap);
        TwoLift twoLift = new TwoLift(hardwareMap);


        // Start trajectory
        TrajectoryActionBuilder toSub = drive.actionBuilder(initialPose)
                .lineToY(36);
        telemetry.addData("Current Trajectory", "Running toSub");
        telemetry.update();

        TrajectoryActionBuilder awaySub = toSub.endTrajectory().fresh()
                .lineToY(45);
        telemetry.addData("Current Trajectory", "closerSub");
        telemetry.update();






        TrajectoryActionBuilder toSample = awaySub.endTrajectory().fresh()
                .strafeTo(new Vector2d(-35,35))
                .strafeTo(new Vector2d(-35,10))
                .strafeTo(new Vector2d(-45,10))
                .strafeTo(new Vector2d(-45,58));

        TrajectoryActionBuilder grabSpecimen = toSample.endTrajectory().fresh()
                .splineTo(new Vector2d(0,48),300)
                .lineToY(36);

        TrajectoryActionBuilder toHangS = grabSpecimen.endTrajectory().fresh()
                .lineToY(45);



        //lift down
        TrajectoryActionBuilder grabSpecimenTwo = toHangS.endTrajectory().fresh()
                .splineTo(new Vector2d(-40,58),-300);


//
//        //loower lift
        TrajectoryActionBuilder hangSpecimenTwo =grabSpecimenTwo.endTrajectory().fresh()
                .strafeTo(new Vector2d(-2,35));


        TrajectoryActionBuilder grabSpecimenThree = hangSpecimenTwo.endTrajectory().fresh()
                .strafeTo(new Vector2d(-2,45));


        TrajectoryActionBuilder prepPark = grabSpecimenThree.endTrajectory().fresh()
                .splineTo(new Vector2d(-40,60),-300);







        // Actions
          Action TwoLiftUp = new SequentialAction (twoLift.twoLiftUp());
//        Action liftAndOpenClaw = new SequentialAction(lift.liftUp(), claw.openClaw());
//        Action moveBackAndLowerLift = new SequentialAction(lift.lowerLift());
//        Action liftDownAndOpenClaw = new SequentialAction( claw.openClaw(),lift.lowerLift2());
//        Action liftDownAndOpenClaw1 = new SequentialAction( claw.openClaw(),lift.lowerLift2());

        Action toHP = toSample.build(); // Ensure this uses the correct build method
        Action waitAndCloseClaw = new SequentialAction( claw.closeClaw());

        //Actions.runBlocking(claw.closeClaw);


        // Running Actions
        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(// Starts the lift-up sequence

                        new ParallelAction(

                                toSub.build(),
                                twoLift.twoLiftUp()
                        ),
                        new ParallelAction(
                                twoLift.TwoLiftDown(),
                                new WaitAction(500),
                                claw.openClaw()
                        ),
                        awaySub.build(),
                        new SequentialAction(
                                toSample.build(),
                                armServo.armBack(),
                                new WaitAction(500),
                                claw.closeClaw(),
                                twoLift.twoLiftUp1()

                        ),
                        grabSpecimen.build(),
                        armServo.armFront(),

                        new ParallelAction(
                                twoLift.TwoLiftDown1(),
                                new WaitAction(500),
                                claw.openClaw(),
                                toHangS.build()
                        ),
                        new ParallelAction(
                                grabSpecimenTwo.build(),
                                armServo.armBack(),
                                new WaitAction(500),
                                claw.closeClaw(),
                                twoLift.twoLiftUp2()
                        ),
                        new ParallelAction(
                                hangSpecimenTwo.build(),
                                armServo.armFront()
                        ),
                        new ParallelAction(
                                twoLift.TwoLiftDown2(),
                                new WaitAction(500),
                                claw.openClaw()
                        ),
                        grabSpecimenThree.build(),
                        prepPark.build() // Executes park preparation


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
//    public class Lift {
//        private DcMotorEx lift;
//        private boolean encoderReset = false;
//
//        public Lift(HardwareMap hardwareMap) {
//            lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
//            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            lift.setDirection(DcMotorSimple.Direction.FORWARD);
//            // Reset encoder once during initialization
//            resetEncoder();
//        }
//
//        private void resetEncoder() {
//            if (!encoderReset) {
//                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                encoderReset = true;
//            }
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
//                        resetEncoder();
//                        lift.setPower(-2); // Start moving the lift up
//                        initialized = true;
//                    }
//
//                    // Current lift position
//                    double liftPos = lift.getCurrentPosition();
//                    packet.put("liftPos", liftPos);
//                    telemetry.addData("lift Encoder", liftPos);
//                    telemetry.update();
//
//                    // Threshold for stopping (prevent overshooting)
//                    if (liftPos <= -6700) {
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
//                        resetEncoder();
//                        lift.setPower(2); // Start lowering the lift
//                        initialized = true;
//                    }
//
//                    // Current lift position
//                    double liftPos = lift.getCurrentPosition();
//                    packet.put("liftPos", liftPos);
//                    telemetry.addData("lift Encoder", liftPos);
//                    telemetry.update();
//
//                    // Threshold for stopping (prevent overshooting)
//                    if (liftPos >= -6270) {
//                        lift.setPower(0); // Stop lift movement
//                        return false; // Stop the action
//                    }
//
//                    return true;
//                }
//            };
//        }
//        public Action lowerLift3() {
//            return new Action() {
//                private boolean initialized = false;
//
//                @Override
//                public boolean run(@NonNull TelemetryPacket packet) {
//                    if (!initialized) {
//                        resetEncoder();
//                        lift.setPower(2); // Start lowering the lift
//                        initialized = true;
//                    }
//
//                    // Current lift position
//                    double liftPos = lift.getCurrentPosition();
//                    packet.put("liftPos", liftPos);
//                    telemetry.addData("lift Encoder", liftPos);
//                    telemetry.update();
//
//                    // Threshold for stopping (prevent overshooting)
//                    if (liftPos >= -6300) {
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
//                        resetEncoder();
//                        lift.setPower(2); // Start lowering the lift
//                        initialized = true;
//                    }
//
//                    // Current lift position
//                    double liftPos = lift.getCurrentPosition();
//                    packet.put("liftPos", liftPos);
//                    telemetry.addData("lift Encoder", liftPos);
//                    telemetry.update();
//
//                    // Threshold for stopping (prevent overshooting)
//                    if (liftPos >= -1740) {
//                        lift.setPower(0); // Stop lift movement
//                        return false; // Stop the action
//                    }
//
//                    return true;
//                }
//            };
//        }
//    }
//

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
                    miniClawServo.setPosition(0.80);
                    return false;
                }
            };
        }
    }
    public class horizantalLiftServo {
        private Servo horizantalLiftServo;

        public horizantalLiftServo(HardwareMap hardwareMap) {
            horizantalLiftServo = hardwareMap.get(Servo.class, "horizontalLiftClawServo");
            inithorizantalLiftServo(); // Initialize the servo to the down position

        }

        public void inithorizantalLiftServo() {
            horizantalLiftServo.setPosition(0.6);
        }

        public Action horizantalliftOut() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    horizantalLiftServo.setPosition(0.3);
                    return false;
                }
            };
        }

        public Action horizantalLiftServoIn() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    horizantalLiftServo.setPosition(0.80);
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
    // armServo mechanism class
    public class ArmServo {
        private Servo armServo;

        public ArmServo(HardwareMap hardwareMap) {
            armServo = hardwareMap.get(Servo.class, "armServo");
            initArmServo(); // Initialize the servo to the down position
        }

        // Initializes the initArmServo to the down position
        public void initArmServo() {
            armServo.setPosition(0.07); // Set to the down position
        }

        public Action armFront() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    armServo.setPosition(0.07);
                    return false;  // Action completed
                }
            };
        }

        public Action armBack() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    armServo.setPosition(0.7);
                    return false;  // Action completed
                }
            };
        }
    }

    // HangLift mechanism class
    public class TwoLift {
        private DcMotorEx liftLeftMotor;
        private DcMotorEx liftRightMotor;

        public TwoLift(HardwareMap hardwareMap) {
            liftRightMotor = hardwareMap.get(DcMotorEx.class, "liftRightMotor");
            liftRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            liftLeftMotor = hardwareMap.get(DcMotorEx.class, "liftLeftMotor");
            liftLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            liftRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public Action TwoLiftDown2() {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        liftLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        liftRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        liftLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        liftRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        // sleep(200);
                        liftLeftMotor.setPower(-1);
                        liftRightMotor.setPower(-1);
                        initialized = true;
                    }

                    double rightPos = liftRightMotor.getCurrentPosition();
                    double leftPos = liftLeftMotor.getCurrentPosition();
                    packet.put("liftLeftMotor", leftPos);
                    packet.put("liftRightMotor", rightPos);
                    telemetry.addData("Left Encoder", leftPos);
                    telemetry.addData("Right Encoder", rightPos);
                    // telemetry.addData("Right Encoder", rightPos);

                    telemetry.update();

                    if (leftPos >= -6200 && rightPos >= -6200) {
                        liftLeftMotor.setPower(0);
                        liftRightMotor.setPower(0);
                        return false; // Stop the action
                    }
                    return true;
                }

            };
        }
        public Action TwoLiftDown3() {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        liftLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        liftRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        liftLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        liftRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        // sleep(200);
                        liftLeftMotor.setPower(-1);
                        liftRightMotor.setPower(-1);
                        initialized = true;
                    }

                    double rightPos = liftRightMotor.getCurrentPosition();
                    double leftPos = liftLeftMotor.getCurrentPosition();
                    packet.put("liftLeftMotor", leftPos);
                    packet.put("liftRightMotor", rightPos);
                    telemetry.addData("Left Encoder", leftPos);
                    telemetry.addData("Right Encoder", rightPos);
                    // telemetry.addData("Right Encoder", rightPos);

                    telemetry.update();

                    if (leftPos >= -6200 && rightPos >= -6200) {
                        liftLeftMotor.setPower(0);
                        liftRightMotor.setPower(0);
                        return false; // Stop the action
                    }
                    return true;
                }

            };
        }


        public Action TwoLiftDown() {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        liftLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        liftRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        liftLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        liftRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        // sleep(200);
                        liftLeftMotor.setPower(-1);
                        liftRightMotor.setPower(-1);
                        initialized = true;
                    }

                    double rightPos = liftRightMotor.getCurrentPosition();
                    double leftPos = liftLeftMotor.getCurrentPosition();
                    packet.put("liftLeftMotor", leftPos);
                    packet.put("liftRightMotor", rightPos);
                    telemetry.addData("Left Encoder", leftPos);
                    telemetry.addData("Right Encoder", rightPos);
                    // telemetry.addData("Right Encoder", rightPos);

                    telemetry.update();

                    if (leftPos >= -6000 && rightPos >= -6000) {
                        liftLeftMotor.setPower(0);
                        liftRightMotor.setPower(0);
                        return false; // Stop the action
                    }
                    return true;
                }

            };
        }
        public Action TwoLiftDown1() {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        liftLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        liftRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        liftLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        liftRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        // sleep(200);
                        liftLeftMotor.setPower(-1);
                        liftRightMotor.setPower(-1);
                        initialized = true;
                    }

                    double rightPos = liftRightMotor.getCurrentPosition();
                    double leftPos = liftLeftMotor.getCurrentPosition();
                    packet.put("liftLeftMotor", leftPos);
                    packet.put("liftRightMotor", rightPos);
                    telemetry.addData("Left Encoder", leftPos);
                    telemetry.addData("Right Encoder", rightPos);
                    // telemetry.addData("Right Encoder", rightPos);

                    telemetry.update();

                    if (leftPos >= -5700 && rightPos >= -5700) {
                        liftLeftMotor.setPower(0);
                        liftRightMotor.setPower(0);
                        return false; // Stop the action
                    }
                    return true;
                }

            };
        }
        public Action twoLiftUp() {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        liftLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        liftRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        liftLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        liftRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        // sleep(200);
                        liftLeftMotor.setPower(1);
                        liftRightMotor.setPower(1);
                        initialized = true;
                    }

                    double rightPos = liftRightMotor.getCurrentPosition();
                    double leftPos = liftLeftMotor.getCurrentPosition();
                    packet.put("liftLeftMotor", leftPos);
                    packet.put("liftRightMotor", rightPos);
                    telemetry.addData("Left Encoder", leftPos);
                    telemetry.addData("Right Encoder", rightPos);
                    // telemetry.addData("Right Encoder", rightPos);

                    telemetry.update();

                    if (leftPos <= -6200 && rightPos <= -6200) {
                        liftLeftMotor.setPower(0);
                        liftRightMotor.setPower(0);
                        return false; // Stop the action
                    }
                    return true;
                }

            };
        }
        public Action twoLiftUp1() {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        liftLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        liftRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        liftLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        liftRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        // sleep(200);
                        liftLeftMotor.setPower(1);
                        liftRightMotor.setPower(1);
                        initialized = true;
                    }

                    double rightPos = liftRightMotor.getCurrentPosition();
                    double leftPos = liftLeftMotor.getCurrentPosition();
                    packet.put("liftLeftMotor", leftPos);
                    packet.put("liftRightMotor", rightPos);
                    telemetry.addData("Left Encoder", leftPos);
                    telemetry.addData("Right Encoder", rightPos);
                    // telemetry.addData("Right Encoder", rightPos);

                    telemetry.update();

                    if (leftPos <= -6200 && rightPos <= -6200) {
                        liftLeftMotor.setPower(0);
                        liftRightMotor.setPower(0);
                        return false; // Stop the action
                    }
                    return true;
                }

            };
        }
        public Action twoLiftUp2() {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        liftLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        liftRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        liftLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        liftRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        // sleep(200);
                        liftLeftMotor.setPower(1);
                        liftRightMotor.setPower(1);
                        initialized = true;
                    }

                    double rightPos = liftRightMotor.getCurrentPosition();
                    double leftPos = liftLeftMotor.getCurrentPosition();
                    packet.put("liftLeftMotor", leftPos);
                    packet.put("liftRightMotor", rightPos);
                    telemetry.addData("Left Encoder", leftPos);
                    telemetry.addData("Right Encoder", rightPos);
                    // telemetry.addData("Right Encoder", rightPos);

                    telemetry.update();

                    if (leftPos <= -6200 && rightPos <= -6200) {
                        liftLeftMotor.setPower(0);
                        liftRightMotor.setPower(0);
                        return false; // Stop the action
                    }
                    return true;
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
                        // sleep(200);
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
