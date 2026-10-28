package org.firstinspires.ftc.teamcode.biobuzzautos;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
@Autonomous(name = "BLUE Mecanum Auto")
public class blueMechShortAuto extends LinearOpMode{

    @Override
    public void runOpMode(){
        Launcher launcher = new Launcher(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Pose2d initialPose = new Pose2d(-10,60,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder toShoot = drive.actionBuilder(initialPose)
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-30,10),Math.toRadians(0))
                .waitSeconds(3);

        TrajectoryActionBuilder toPickUp = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(55,17),Math.toRadians(5))
                .strafeToLinearHeading(new Vector2d(60,17),Math.toRadians(45))
                .waitSeconds(3);


        TrajectoryActionBuilder toShoot2 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(35,10),Math.toRadians(0))
                .waitSeconds(3);

        Action outOfZone = toShoot2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(40,56),Math.toRadians(0))
                .build();


        Action firstTraj = toShoot.build();
        Action secondTraj = toPickUp.build();
        Action thirdTraj = toShoot2.build();


        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        firstTraj,
                        launcher.launcherForward(),
                        new ParallelAction(
                                secondTraj,
                                intake.intakeIn()
                        ),
                        thirdTraj,
                        launcher.launcherForward(),

                        outOfZone
                )
        );
    }
    }
    class Launcher {
        private final DcMotorEx launcher;
        private final ElapsedTime timer;

        public Launcher(HardwareMap hardwareMap) {
            launcher = hardwareMap.get(DcMotorEx.class, "launcher");
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

        public Action launcherForward() {return new LauncherForward();
        }

        public class LauncherBackwards implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    launcher.setPower(0.8);
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

        public Action launcherBackwards(){return new LauncherBackwards();}
        }


     class Intake {
        private final DcMotorEx intakeMotor;
        private final ElapsedTime timer1;

        public Intake(HardwareMap hardwareMap) {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
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
                telemetry.addData("Intake Timer", timerValue);
                telemetry.update();
                if (timerValue < 2000) {
                    return true;
                } else {
                    intakeMotor.setPower(0);
                    return false;
                }
            }
        }

        public Action intakeIn() {return new IntakeIn();}



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
        }

        public Action intakeOut(){return new IntakeOut();} {

        }
    }

//yet to be tested