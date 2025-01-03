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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Disabled
@Autonomous(name = "BlueObservationAuto")
public class BlueAutoLM1ObsZone extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        TrajectoryActionBuilder toSubmersible = drive.actionBuilder(initialPose)
                //.turn(Math.toRadians(45))
                .strafeTo(new Vector2d(-20, 29))
                .waitSeconds(2);
        Action toSubmersibleTraj = toSubmersible.build();
        
        Action toObservation = toSubmersible.fresh()
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
                        toObservation

                )
        );
    }

    public class Lift {
       private DcMotorEx lift;

       public Lift(HardwareMap hardwareMap) {
           lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
           lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
           lift.setDirection(DcMotorSimple.Direction.FORWARD);
       }

       public class LiftUp implements Action {
           // checks if the lift motor has been powered on
           private boolean initialized = false;

           // actions are formatted via telemetry packets as below
           @Override
           public boolean run(@NonNull TelemetryPacket packet) {
               // powers on motor, if it is not on
               if (!initialized) {
                   lift.setPower(0.8);
                   initialized = true;
                   sleep(300);

               }
               // checks lift's current position
               double pos = lift.getCurrentPosition();
               packet.put("liftPos", pos);
               if (pos < 5000.0) {
                   // true causes the action to rerun
                   return true;

               } else {
                   // false stops action rerun
                   lift.setPower(0);
                   return false;
               }
               // overall, the action powers the lift until it surpasses
               // 3000 encoder ticks, then powers it off
           }
       }
       public Action liftUp() {
           return new LiftUp();
       }


       public class LiftDown implements Action {
           private boolean initialized = false;


           @Override
           public boolean run(@NonNull TelemetryPacket packet) {
               if (!initialized) {
                   lift.setPower(-0.8);
                   initialized = true;
               }

               double pos = lift.getCurrentPosition();
               packet.put("liftPos", pos);
               if (pos > 100.0) {
                   return true;
               } else {
                   lift.setPower(0);
                   return false;
               }
           }
       }
       public Action liftDown() {
           return new LiftDown();
       }

   }

    public class Claw {
        private Servo gizaClawLeftServo;
        private Servo gizaClawRightServo;

        public Claw(HardwareMap hardwareMap) {
            gizaClawLeftServo = hardwareMap.get(Servo.class, "gizaClawLeftServo");
            gizaClawRightServo = hardwareMap.get(Servo.class, "gizaClawRightServo");
        }


        public class CloseClaw implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gizaClawLeftServo.setPosition(0.5);
                gizaClawRightServo.setPosition(0.5);

                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gizaClawLeftServo.setPosition(.2);
                gizaClawRightServo.setPosition(0.2);
                return false;
            }
        }

        public Action openClaw() {
            return new CloseClaw();

        }
    }
}
