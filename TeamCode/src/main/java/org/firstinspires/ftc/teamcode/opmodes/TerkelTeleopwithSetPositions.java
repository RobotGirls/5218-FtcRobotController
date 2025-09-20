package org.firstinspires.ftc.teamcode.opmodes;

/*
 * Copyright (c) September 2017 FTC Teams 25/5218
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted (subject to the limitations in the disclaimer below) provided that
 *  the following conditions are met:
 *
 *  Redistributions of source code must retain the above copyright notice, this list
 *  of conditions and the following disclaimer.
 *
 *  Redistributions in binary form must reproduce the above copyright notice, this
 *  list of conditions and the following disclaimer in the documentation and/or
 *  other materials provided with the distribution.
 *
 *  Neither the name of FTC Teams 25/5218 nor the names of their contributors may be used to
 *  endorse or promote products derived from this software without specific prior
 *  written permission.
 *
 *  NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 *  LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ContinuousServoTask;

import java.util.HashSet;
import java.util.Set;

import team25core.DeadmanMotorTask;
import team25core.GamepadTask;
import team25core.MechanumGearedDrivetrain;
import team25core.OneWheelDriveTask;
import team25core.RobotEvent;
import team25core.RunToEncoderValueTask;
import team25core.StandardFourMotorRobot;
import team25core.TeleopDriveTask;
import team25core.TwoStickMechanumControlScheme;
import team25core.TwoWheelDirectDrivetrain;
import team25core.TwoWheelDriveTask;


@TeleOp(name = "CompRobotTeleopWithSP")
//@Disabled
public class TerkelTeleopwithSetPositions extends StandardFourMotorRobot {

    private TeleopDriveTask drivetask;

    private enum Direction {
        CLOCKWISE,
        COUNTERCLOCKWISE,
    }
    //added field centric
    private Telemetry.Item buttonTlm;

    //wheelieServo Positions
//    private static final double WHEELIE_GRAB = 0.1;
//    private static final double WHEELIE_RELEASE = 0.99;
    private static final double WHEELIE_GRAB =0.1;
    private static final double WHEELIE_RELEASE =0.9;
    private static final double WHEELIE_STOP = 0;
    private static final double WHEELIE_STOP_A = 0;




    //wheelieRotationServo Positions
    private static final double WHEELIE_UP = 0.1;
    private static final double WHEELIE_DOWN = 0.9;

    //gizaClawLeftServo Positions
    private static final double GIZA_CLAW_LEFT_OPEN = 0.5;
    private static final double GIZA_CLAW_LEFT_CLOSE = 0.;

    //gizaClawRightServo Positions
    private static final double GIZA_CLAW_RIGHT_OPEN = 0.6;
    private static final double GIZA_CLAW_RIGHT_CLOSE = 0.9;

    //miniClawServo Positions
    private static final double MINI_CLAW_OPEN = 0.66;
    private static final double MINI_CLAW_CLOSE = 0.3;



    private boolean wasButtonAPressed = false;
    private boolean wasButtonYPressed = false;


    private BNO055IMU imu;

    private Servo horizontalLiftServo;

    private Servo wheelieServo;
    private Servo wheelieRotationServo;

    private Servo miniClawServo;
    private Servo gizaClawLeftServo;
    private Servo gizaClawRightServo;
    private TwoWheelDriveTask liftMotorTask;



    private ContinuousServoTask horizontalLiftTask;
    private DcMotor hangLeftMotor;
    private DcMotor hangRightMotor;

    private DcMotor liftMotorLeft;
    private DcMotor liftMotorRight;
    DeadmanMotorTask liftLinearUp;
    private static final double LIFT_POWER_UP = 0.5;
    private static final int MAX_LINEAR_HEIGHT = 50;
    private static final int SIDEWALL_POSITION = 1977;
    private static final int HIGH_BAR_POSITION = 6994;
    private double liftCurPosLeft;
    private double liftCurPosRight;
    private Set<DcMotor> otherLift = new HashSet<>();


    private DcMotorSimple.Direction liftDirection;
    private Telemetry.Item whereamiTlm;
    private Telemetry.Item liftPosLeftTlm;
    private Telemetry.Item liftPosRightTlm;



    private boolean currentlySlow = false;

    //MecanumFieldCentricDriveScheme scheme;


    private MechanumGearedDrivetrain drivetrain;

    @Override
    public void handleEvent(RobotEvent e) {
        if (e instanceof RunToEncoderValueTask.RunToEncoderValueEvent) {
            whereamiTlm.setValue("handleEvent");
            if (((RunToEncoderValueTask.RunToEncoderValueEvent)e).kind == RunToEncoderValueTask.EventKind.DONE) {
                whereamiTlm.setValue("RunToEnconderTaskIsDone");

            }

        }
    }

    @Override
    public void init() {
        super.init();

        miniClawServo = hardwareMap.servo.get("miniClawServo");
        wheelieRotationServo = hardwareMap.servo.get("wheelieRotationServo");
        wheelieServo = hardwareMap.servo.get("wheelieServo");
        gizaClawLeftServo= hardwareMap.servo.get("gizaClawLeftServo");
        gizaClawRightServo= hardwareMap.servo.get("gizaClawRightServo");
        horizontalLiftServo=hardwareMap.servo.get("horizontalLiftClawServo");

        hangLeftMotor = hardwareMap.get(DcMotor.class,"hangLeftMotor");
        hangLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        hangRightMotor = hardwareMap.get(DcMotor.class,"hangRightMotor");
        hangRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        liftMotorLeft = hardwareMap.get(DcMotor.class,"liftMotorLeft");
        liftMotorRight = hardwareMap.get(DcMotor.class,"liftMotorRight");
        liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        otherLift.add(liftMotorRight);


        liftPosLeftTlm = telemetry.addData("liftPosLeft", 0);
        liftPosRightTlm = telemetry.addData("liftPosRight", 0);
        whereamiTlm = telemetry.addData("codeLocation", "init");

//        liftLinearUp = new DeadmanMotorTask(this, liftMotor, LIFT_POWER_UP, GamepadTask.GamepadNumber.GAMEPAD_2, DeadmanMotorTask.DeadmanButton.LEFT_STICK_UP);
//        liftLinearUp.setMaxMotorPosition(MAX_LINEAR_HEIGHT); using encoders to record ticks
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        wheelieRotationServo.setPosition(WHEELIE_DOWN);
        wheelieServo.setPosition(WHEELIE_STOP);
        miniClawServo.setPosition(MINI_CLAW_CLOSE);
        gizaClawLeftServo.setPosition(GIZA_CLAW_LEFT_CLOSE);
        gizaClawRightServo.setPosition(GIZA_CLAW_RIGHT_CLOSE);
       // horizontalLiftServo.setPosition(HORIZONTAL_RETRACTED);
// !!!!
// FIXME need to replace the current TwoWheelDriveTask with the one Maddie has
        liftMotorTask = new TwoWheelDriveTask(this, liftMotorRight, liftMotorLeft, true);

        liftMotorTask.slowDown(false);

        horizontalLiftTask = new ContinuousServoTask(this, horizontalLiftServo, true);  // 'true' for right joystick
        horizontalLiftTask.slowDown(false);

        //telemetry
        buttonTlm = telemetry.addData("buttonState", "unknown");

        TwoStickMechanumControlScheme scheme = new TwoStickMechanumControlScheme(gamepad1);
        drivetrain = new MechanumGearedDrivetrain(motorMap);
        drivetrain.setNoncanonicalMotorDirection();
        //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        // Note we are swapping the rights and lefts in the arguments below
        // since the gamesticks were switched for some reason and we need to do
        // more investigation
        //liftMotortask = new TeleopDriveTask(this, scheme, frontLeft, frontRight, backLeft, backRight);
        drivetask = new TeleopDriveTask(this, scheme, frontLeft, frontRight, backLeft, backRight);
    }

    public void initIMU()
    {
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

    }

    public void setLiftToHighBarPosition() {

        whereamiTlm.setValue("setLiftToHighBarPosition");
        liftCurPosLeft = liftMotorLeft.getCurrentPosition();
        liftCurPosRight = liftMotorRight.getCurrentPosition();
        liftPosLeftTlm.setValue(liftCurPosLeft);
        liftPosRightTlm.setValue(liftCurPosRight);

        if (liftCurPosLeft < HIGH_BAR_POSITION) {
            liftDirection = DcMotorSimple.Direction.REVERSE;

        } else {
            liftDirection = DcMotorSimple.Direction.FORWARD;
        }
        this.addTask(new RunToEncoderValueTask(this, liftMotorLeft, HIGH_BAR_POSITION, LIFT_POWER_UP));
        this.addTask(new RunToEncoderValueTask(this, liftMotorRight, HIGH_BAR_POSITION, LIFT_POWER_UP));

    }


    @Override
    public void start() {

        whereamiTlm.setValue("start");
        //Gamepad 1
        this.addTask(drivetask);
       // this.addTask(hangMotorTask);
        this.addTask(new GamepadTask(this, GamepadTask.GamepadNumber.GAMEPAD_1) {
            public void handleEvent(RobotEvent e) {
                GamepadEvent gamepadEvent = (GamepadEvent) e;

                switch (gamepadEvent.kind) {
                    case BUTTON_X_DOWN:
                        // If slow, then normal speed. If fast, then slow speed of motors.
                        //pertains to slowmode
                        if (currentlySlow) {
                            drivetask.slowDown(1.0);
                            currentlySlow = false;
                        } else {
                            drivetask.slowDown(0.3);
                            currentlySlow = true;
                        }
                        break;

                    default:
                        buttonTlm.setValue("Not Moving");
                        break;
                }
                switch (gamepadEvent.kind) {
                    case BUTTON_A_DOWN:
                        hangRightMotor.setPower(-0.6);
                        hangLeftMotor.setPower(-0.6);
                        break;


                    case BUTTON_Y_DOWN:
                        hangLeftMotor.setPower(0.6);
                        hangRightMotor.setPower(0.6);

                        break;
                    case BUTTON_A_UP:
                        hangRightMotor.setPower(0);
                        hangLeftMotor.setPower(0);
                        break;
                    case BUTTON_Y_UP:
                        hangRightMotor.setPower(0);
                        hangLeftMotor.setPower(0);
                        break;

                    default:
                        break;
                }
            }
        });

        //Gamepad 2
        this.addTask(liftMotorTask);
        this.addTask(horizontalLiftTask);

        this.addTask(new GamepadTask(this, GamepadTask.GamepadNumber.GAMEPAD_2) {
            public void handleEvent(RobotEvent e) {
                GamepadEvent gamepadEvent = (GamepadEvent) e;
                switch (gamepadEvent.kind) {
                    case DPAD_UP_DOWN:
                        // If slow, then normal speed. If fast, then slow speed of motors.
                        //pertains to slowmode
                        if (currentlySlow) {
                            liftMotorTask.slowDown(1.0);
                            currentlySlow = false;
                        } else {
                            liftMotorTask.slowDown(0.3);
                            currentlySlow = true;
                        }
                        break;
                        //Holaaa my name is Isabella Barrientos and my codename is Jason Durullo and i love eating food and cleaning my computer

                    default:
                        buttonTlm.setValue("Not Moving");
                        break;
                }


                switch (gamepadEvent.kind) {
                    case RIGHT_TRIGGER_DOWN:
                        // set claw's position to 0
                        miniClawServo.setPosition(MINI_CLAW_OPEN);
                        break;
                    case LEFT_TRIGGER_DOWN:
                        // set claw's position to 1
                        miniClawServo.setPosition(MINI_CLAW_CLOSE);
                        break;

                    case RIGHT_BUMPER_DOWN:
                        wheelieRotationServo.setPosition(WHEELIE_UP);
                        break;
                    case LEFT_BUMPER_DOWN:
                        wheelieRotationServo.setPosition(WHEELIE_DOWN);
                        break;
                    default:
                        buttonTlm.setValue("Not Moving");
                        break;
                    case BUTTON_X_DOWN:
                        // set position to sidewall

                        break;
                    case BUTTON_B_DOWN:
                        // set position to high bar
                        // FIXME !!!!! We need to uncomment this when we are ready
                        // meanwhile, we want to print out the encoder positions
                        // first for each motor, so we do not DAMAGE the lift
                        //setLiftToHighBarPosition();
                        break;


                }
            }
        });
    }
}