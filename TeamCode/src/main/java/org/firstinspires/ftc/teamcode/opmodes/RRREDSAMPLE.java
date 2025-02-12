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

@Autonomous(name = "RRSampleREDAuto")
public class RRREDSAMPLE extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-8, -65, Math.toRadians(180));
        Comp5218MecanumDrive drive = new Comp5218MecanumDrive(hardwareMap, initialPose);

        Lift lift = new Lift(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        WRClaw wrClaw = new WRClaw(hardwareMap);
        HangLift hangLift = new HangLift(hardwareMap);


        // Action waitAndCloseClaw = new SequentialAction( claw.closeClaw());

        // Correct the chaining of the toSample trajectory, and use .build() to convert TrajectoryActionBuilder to Action
        TrajectoryActionBuilder toBasket = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-28,-58) )
                .turn(-0.9 );


        //open claw after toBasket
        TrajectoryActionBuilder CloserBasket = toBasket.endTrajectory().fresh()
                .strafeTo(new Vector2d(-32,-60));
//        TrajectoryActionBuilder CloserBasket2 = toBasket2.endTrajectory().fresh()
//                .strafeTo(new Vector2d(30,60));
//        TrajectoryActionBuilder CloserBasket3 = toBasket.endTrajectory().fresh()
//                .strafeTo(new Vector2d(30,60));
        TrajectoryActionBuilder awayBasket = CloserBasket.endTrajectory().fresh()
                .lineToY(-55);

        TrajectoryActionBuilder toSample = CloserBasket.fresh()
                .strafeTo(new Vector2d(-29,-50))
                .turn(-2.55);
        //  .lineToY(48);
        //lower lift stimultanlous with toSample
        //close claw at the end of to Sample

        TrajectoryActionBuilder closerSample = toSample.fresh()
                .lineToY(-45);


        TrajectoryActionBuilder toBasket2 = closerSample.endTrajectory().fresh()
//                .lineToY(50)
                .turn(2.6 )
                .strafeTo(new Vector2d(-28,-60) );
        //  .turn(0.8 );

        TrajectoryActionBuilder CloserBasket2 = toBasket2.endTrajectory().fresh()
                .strafeTo(new Vector2d(-32,-62));

        TrajectoryActionBuilder awayBasket2 = CloserBasket2.endTrajectory().fresh()
                .lineToY(-50);

        //open claw at the end of toBasket 2
        // lift the lift up stimutanously with to Basket2

        TrajectoryActionBuilder toSample2 = awayBasket2.endTrajectory().fresh()
                .strafeTo(new Vector2d(-33,-40))
                .turn(-2.4)
                .lineToY(-38.25);
        //close claw after toSample2
        //lower lift stimutanouslytoSample2

        TrajectoryActionBuilder toBasket3 = toSample2.endTrajectory().fresh()
                .lineToY(-46)
                .turn(2.45)
                .strafeTo(new Vector2d(-55,-53));
        //open claw
        //lift lift stimutanously
        TrajectoryActionBuilder CloserBasket3 = toBasket3.endTrajectory().fresh()
                .strafeTo(new Vector2d(-30,-58.75));

        TrajectoryActionBuilder awayBasket3 = CloserBasket3.endTrajectory().fresh()
                .lineToY(-50);

//        TrajectoryActionBuilder toSample3 = awayBasket3.endTrajectory().fresh()
//                .strafeTo(new Vector2d(5,24));


        TrajectoryActionBuilder toPark = awayBasket2.endTrajectory().fresh()
                .strafeTo(new Vector2d(15,20));
                //   .turn(2.5)
               // .lineToX(-2);












        // Actions
        Action hangLiftUp = hangLift.hangLiftUp();
        Action liftAndOpenClaw = new SequentialAction(lift.liftUp(), claw.openClaw());
        Action moveBackAndLowerLift = new SequentialAction(lift.lowerLift());
        Action liftDownAndOpenClaw = new SequentialAction( claw.openClaw(),lift.lowerLift2());

        Action toHP = toSample.build(); // Ensure this uses the correct build method
        Action waitAndCloseClaw = new SequentialAction(new WaitAction(500), claw.closeClaw());
        Action waitAndOpenClaw = new SequentialAction(new WaitAction(1000), claw.openClaw());
        Action closeClawAndWait = new SequentialAction(claw.closeClaw(),new WaitAction(1500));

        //Actions.runBlocking(claw.closeClaw);
        Action waitAndOpenClaw1 = new SequentialAction(new WaitAction(1000), claw.openClaw());


        // Running Actions
        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        hangLiftUp,                        // Lift up the hang first
                        new ParallelAction(
                                wrClaw.upWRClaw(),// Perform parallel actions: move to basket and lift up
                                toBasket.build(),
                                lift.liftUp()
                        ),
                        CloserBasket.build(),
                        waitAndOpenClaw,
                        awayBasket.build(),


                        toSample.build(),
                        lift.lowerLift(),
                        wrClaw.downWRClaw(),
                        closerSample.build(),
                        waitAndCloseClaw,
                        new WaitAction(200),
                        new ParallelAction(
                                //  new WaitAction(2000),// Parallel actions: move to basket2 and lift up
                                toBasket2.build(),
                                wrClaw.upWRClaw()
                        ),
                        lift.liftUp(),
                        CloserBasket2.build(),
                        waitAndOpenClaw1,// Close the basket after moving to basket
                        new WaitAction(1000),
                        awayBasket.build(),
                        new ParallelAction(
                                toPark.build(),
// Parallel actions: move to sample2 and lower lift
//                                toSample2.build(),
//                                wrClaw.downWRClaw(),
                                lift.lowerLift()

                        ),
//                        waitAndCloseClaw,
//                        new WaitAction(1500), // Wait after lift
//                        new ParallelAction(                // Parallel actions: move to basket3 and lift up
//                                toBasket3.build(),
//                                lift.liftUp()
//                        ),
//                        CloserBasket3.build(),              // Close the basket after moving to basket
//                        new WaitAction(1500),
//                        claw.openClaw(),
//                        awayBasket2.build(),// Open the claw after basket3 actions
//                        new ParallelAction(                // Parallel actions: move to sample3 and lower lift
//                                awayBasket3.build(),// Open the claw after basket3 actions
//                                lift.lowerLift()
//                        ),
//                        awayBasket3.build(),// Open the claw after basket3 actions

                        // claw.closeClaw(),
                        toPark.build()

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
                    if (liftPos <= -10000) {
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
                    if (liftPos >= 0) {
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
