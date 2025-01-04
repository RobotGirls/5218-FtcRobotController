package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "RRauto")
public class RoadRunnerAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Lift lift = new Lift(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        TrajectoryActionBuilder toSub = drive.actionBuilder(initialPose)
                .lineToY(30);

        Action liftAndOpenClaw = new SequentialAction(
                lift.liftUp(),
                claw.openClaw()
        );

        // Trajectory to move backward after lift and claw
        Action moveBackAndLowerLift = new SequentialAction(
                drive.actionBuilder(new Pose2d(0, 30, 0))
                        .lineToY(25)
                        .build(),
                lift.lowerLift()
        );

        Action toSample = drive.actionBuilder(new Pose2d(0, 0, 0))
                .strafeTo(new Vector2d(30, 25))
                .lineToY(35)
                .strafeTo(new Vector2d(40, 35))
                .lineToY(0)
                .build();


        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Robot position: ", drive.updatePoseEstimate());
            telemetry.update();
        }
        waitForStart();
        if (isStopRequested()) return;

        // Execute the sequence
        Actions.runBlocking(
                new SequentialAction(
                        toSub.build(),
                        liftAndOpenClaw,
                        moveBackAndLowerLift,
                        toSample

                )
        );
    }

    public class Lift {
        private DcMotorEx lift;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "lift");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public Action liftUp() {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        lift.setPower(1);
                        initialized = true;
                    }

                    double pos = lift.getCurrentPosition();
                    packet.put("liftPos", pos);
                    if (pos < 3300.0) {
                        return true;
                    } else {
                        lift.setPower(0);
                        return false;
                    }
                }
            };
        }
        public Action lowerLift() {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        lift.setPower(-1); // Lowering the lift
                        initialized = true;
                    }

                    double pos = lift.getCurrentPosition();
                    packet.put("liftPos", pos);
                    if (pos > 0) {
                        return true;
                    } else {
                        lift.setPower(0);
                        return false;
                    }
                }
            };
        }
    }

    public class Claw {
        private Servo gizaClawLeftServo;
        private Servo gizaClawRightServo;

        public Claw(HardwareMap hardwareMap) {
            gizaClawLeftServo = hardwareMap.get(Servo.class, "gizaClawLeftServo");
            gizaClawRightServo = hardwareMap.get(Servo.class, "gizaClawRightServo");
        }

        public Action openClaw() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    gizaClawLeftServo.setPosition(0.2);
                    gizaClawRightServo.setPosition(0.2);
                    return false;
                }
            };
        }

        public Action closeClaw() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    gizaClawLeftServo.setPosition(0.5);
                    gizaClawRightServo.setPosition(0.5);
                    return false;
                }
            };
        }
    }
}
