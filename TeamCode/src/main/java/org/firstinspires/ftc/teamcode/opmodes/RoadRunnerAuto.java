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

@Autonomous(name = "RRauto")
public class RoadRunnerAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-8, 65, 300);
        Comp5218MecanumDrive drive = new Comp5218MecanumDrive(hardwareMap, initialPose);

        Lift lift = new Lift(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        WRClaw wrClaw = new WRClaw(hardwareMap);
        HangLift hangLift = new HangLift(hardwareMap);

        // Trajectories
        TrajectoryActionBuilder toSub = drive.actionBuilder(initialPose).lineToY(35);
        TrajectoryActionBuilder fromSub = toSub.endTrajectory().lineToY(60);

        // Correct the chaining of the toSample trajectory, and use .build() to convert TrajectoryActionBuilder to Action
        TrajectoryActionBuilder toSample = fromSub.endTrajectory().strafeTo(new Vector2d(-42, 8))
                .lineToY(60)
                .lineToY(8)
                .lineToX(-52)
                .lineToY(60)
                .lineToY(35)
                .turn(3.2);

        // Actions
        Action hangLiftUp = hangLift.hangLiftUp();
        Action liftAndOpenClaw = new SequentialAction(lift.liftUp(), claw.openClaw());
        Action moveBackAndLowerLift = new SequentialAction(lift.lowerLift());
        Action toHP = toSample.build(); // Ensure this uses the correct build method
        Action waitAndCloseClaw = new SequentialAction(new WaitAction(1000), claw.closeClaw());

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
                        liftAndOpenClaw,
                        moveBackAndLowerLift,
                        new SequentialAction(
                                toSample.build(),  // Executes the trajectory toSample as an Action
                                lift.lowerLift(),  // Lowers the lift after completing toSample
                                toHP, // Executes the toHP trajectory as an Action
                                waitAndCloseClaw // Closes the claw after waiting
                        )
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

    // Lift mechanism class
    public class Lift {
        private DcMotorEx lift;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public Action liftUp() {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        lift.setPower(-1);
                        initialized = true;
                        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }

                    double liftPos = lift.getCurrentPosition();
                    packet.put("liftPos", liftPos);
                    if (liftPos <= -7000) {
                        lift.setPower(0);
                        return false; // Stop the action
                    }
                    return true;
                }
            };
        }

        public Action lowerLift() {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        lift.setPower(-1);
                        initialized = true;
                        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }

                    double liftPos = lift.getCurrentPosition();
                    packet.put("liftPos", liftPos);
                    if (liftPos <= 5000) {
                        lift.setPower(0);
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
                    miniClawServo.setPosition(0.66);
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
        }

        public Action upWRClaw() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    wheelieRotationServo.setPosition(0.6);
                    return false;
                }
            };
        }

        public Action downWRClaw() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    wheelieRotationServo.setPosition(1);
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
