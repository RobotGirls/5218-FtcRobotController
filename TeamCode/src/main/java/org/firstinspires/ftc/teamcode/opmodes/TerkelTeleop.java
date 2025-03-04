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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ContinuousServoTask;

import team25core.ContinuousTwoServoTask;
import team25core.DeadmanMotorTask;
import team25core.GamepadTask;
import team25core.MechanumGearedDrivetrain;
import team25core.OneWheelDriveTask;
import team25core.RobotEvent;
import team25core.StandardFourMotorRobot;
import team25core.TwoStickMechanumControlScheme;
import team25core.TeleopDriveTask;
import team25core.TwoWheelDirectDrivetrain;
import team25core.vision.apriltags.TwoWheelDriveTask;


@TeleOp(name = "CompRobotTeleop")
//@Disabled
public class TerkelTeleop extends StandardFourMotorRobot {

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
   // private static final double WHEELIE_GRAB =0.1;
    private static final double WHEELIE_RELEASE =0.9;
    private static final double WHEELIE_STOP = 0;
    private static final double WHEELIE_STOP_A = 0;

    private static final double WHEELIE_GRAB = 0;



    //wheelieRotationServo Positions
    private static final double WHEELIE_UP = 0.05;
    private static final double WHEELIE_DOWN = .95;

    //gizaClawLeftServo Positions
    private static final double GIZA_CLAW_LEFT_OPEN = 0.5;
    private static final double GIZA_CLAW_LEFT_CLOSE = 0.;

    //gizaClawRightServo Positions
    private static final double GIZA_CLAW_RIGHT_OPEN = 0.6;
    private static final double GIZA_CLAW_RIGHT_CLOSE = 0.9;

    //miniClawServo Positions
    private static final double MINI_CLAW_OPEN = 0.3;
    private static final double MINI_CLAW_CLOSE = 0.75;

    //groundMiniClawServo Positions
    private static final double GM_CLAW_OPEN = 0.3;
    private static final double GM_CLAW_CLOSE = 0.75;


    //armServo Positions
    private static final double ARM_FRONT = .18;
    private static final double ARM_BACK = .83;
    //armServo Positions
    private static final double ARM_PASS = 0;

    private boolean wasButtonAPressed = false;
    private boolean wasButtonYPressed = false;


    private BNO055IMU imu;
//    private  Servo horizontalLeftServo;
//    private Servo horizontalRightServo;
private CRServo horizontalLeftServo;
private CRServo horizontalRightServo;

    private Servo horizontalLiftServo;
    private Servo armServo;
    private Servo groundMiniClawServo;


    private Servo wheelieServo;
    private Servo wheelieRotationServo;

    private Servo miniClawServo;
    private Servo gizaClawLeftServo;
    private Servo gizaClawRightServo;
    private OneWheelDriveTask liftLeftMotorTask;

    private OneWheelDriveTask liftRightMotorTask;

    //private ContinuousTwoServoTask horizontalLiftTask;
   // private ContinuousServoTask horizontalRightLiftTask;

    private ContinuousServoTask horizontalRightLiftTask;
    private ContinuousServoTask horizontalLeftLiftTask;
    private TwoWheelDriveTask twoLiftTask;
    private DcMotor hangLeftMotor;
    private DcMotor hangRightMotor;
    private DcMotor liftRightMotor;
    private DcMotor liftLeftMotor;

    private DcMotor liftMotor;
    DeadmanMotorTask liftLinearUp;
    private static final double LIFT_POWER_UP = 0.5;
    private static final int MAX_LINEAR_HEIGHT = 50;



    private boolean currentlySlow = false;

    //MecanumFieldCentricDriveScheme scheme;


    private MechanumGearedDrivetrain drivetrain;

    @Override
    public void handleEvent(RobotEvent e) {
    }

    @Override
    public void init() {
        super.init();
        groundMiniClawServo = hardwareMap.servo.get("groundMiniClawServo");
        armServo = hardwareMap.servo.get("armServo");
        miniClawServo = hardwareMap.servo.get("miniClawServo");
        wheelieRotationServo = hardwareMap.servo.get("wheelieRotationServo");
       // wheelieServo = hardwareMap.servo.get("wheelieServo");
      //  horizontalLiftServo=hardwareMap.servo.get("horizontalLiftClawServo");
        horizontalLeftServo=hardwareMap.crservo.get("hzLeftServo");
        horizontalRightServo=hardwareMap.crservo.get("horizontalRightServo");

        hangLeftMotor = hardwareMap.get(DcMotor.class,"hangLeftMotor");
        hangLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armServo.setPosition(ARM_BACK);
        groundMiniClawServo.setPosition(GM_CLAW_CLOSE);
        miniClawServo.setPosition(MINI_CLAW_CLOSE);
        wheelieRotationServo.setPosition(WHEELIE_UP);


        hangRightMotor = hardwareMap.get(DcMotor.class,"hangRightMotor");
        hangRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        liftRightMotor = hardwareMap.get(DcMotor.class,"liftRightMotor");
        liftRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftLeftMotor = hardwareMap.get(DcMotor.class,"liftLeftMotor");
        liftLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        liftLinearUp = new DeadmanMotorTask(this, liftMotor, LIFT_POWER_UP, GamepadTask.GamepadNumber.GAMEPAD_2, DeadmanMotorTask.DeadmanButton.LEFT_STICK_UP);
//        liftLinearUp.setMaxMotorPosition(MAX_LINEAR_HEIGHT); using encoders to record ticks
       // backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      //  backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      //  frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



//        wheelieRotationServo.setPosition(WHEELIE_DOWN);
//        wheelieServo.setPosition(WHEELIE_STOP);
//        miniClawServo.setPosition(MINI_CLAW_CLOSE);
//        gizaClawLeftServo.setPosition(GIZA_CLAW_LEFT_CLOSE);
//        gizaClawRightServo.setPosition(GIZA_CLAW_RIGHT_CLOSE);

         //left positive (backwards)
          horizontalLeftServo.setPower(0);
          // right negative (backwards)
         horizontalRightServo.setPower(0);

//        liftRightMotorTask = new OneWheelDriveTask(this, liftRightMotor, true);
//        liftRightMotorTask.slowDown(false);
//
//        liftLeftMotorTask = new OneWheelDriveTask(this, liftLeftMotor, true);
//        liftLeftMotorTask.slowDown(false);

      //  this.addTask(new ContinuousTwoServoTask(this, horizontalLeftServo, horizontalRightServo, true));

        horizontalRightLiftTask = new ContinuousServoTask(this, horizontalRightServo, true,false);
        horizontalLeftLiftTask = new ContinuousServoTask(this, horizontalLeftServo, true, true);
//        //horizontalLiftTask = new ContinuousServoTask(this, horizontalRightServo, true);  // 'true' for right joystick
        horizontalLeftLiftTask.slowDown(false);

        horizontalRightLiftTask.slowDown(false);

       twoLiftTask=new TwoWheelDriveTask(this, liftLeftMotor,liftRightMotor,true);
        //telemetry
        buttonTlm = telemetry.addData("buttonState", "unknown");

        TwoStickMechanumControlScheme scheme = new TwoStickMechanumControlScheme(gamepad1);
        drivetrain = new MechanumGearedDrivetrain(motorMap);

        drivetrain.setNoncanonicalMotorDirection();

        // liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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

    @Override
    public void start() {

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
                    case LEFT_TRIGGER_DOWN:
                        wheelieRotationServo.setPosition(WHEELIE_DOWN);
                        break;
                    case RIGHT_TRIGGER_DOWN:
                        // set claw's position to 1
                        wheelieRotationServo.setPosition(WHEELIE_UP);
                        break;

                    case RIGHT_BUMPER_DOWN:
                        groundMiniClawServo.setPosition(GM_CLAW_OPEN);

                        break;


                    case LEFT_BUMPER_DOWN:
                        groundMiniClawServo.setPosition(GM_CLAW_CLOSE);
                        break;
                    default:
                        buttonTlm.setValue("Not Moving");
                        break;
                }

            }
        });

        //Gamepad 2
       // this.addTask(liftRightMotorTask);
        // this.addTask(liftLeftMotorTask);

        // this.addTask(horizontalRightLiftTask);
        this.addTask(horizontalRightLiftTask);
        this.addTask(twoLiftTask);
        this.addTask(horizontalLeftLiftTask);
        double liftPosR = liftRightMotor.getCurrentPosition();

        double liftPosL = liftLeftMotor.getCurrentPosition();

        telemetry.addData("lift Encoder", liftPosL);
        telemetry.addData("lift Encoder", liftPosR);

        telemetry.update();



        this.addTask(new GamepadTask(this, GamepadTask.GamepadNumber.GAMEPAD_2) {
            public void handleEvent(RobotEvent e) {
                GamepadEvent gamepadEvent = (GamepadEvent) e;
             //   switch (gamepadEvent.kind) {
//                    case DPAD_UP_DOWN:
//                        // If slow, then normal speed. If fast, then slow speed of motors.
//                        //pertains to slowmode
//                        if (currentlySlow) {
//                            liftMotorTask.slowDown(1.0);
//                            currentlySlow = false;
//                        } else {
//                            liftMotorTask.slowDown(0.3);
//                            currentlySlow = true;
//                        }
//                        break;
                    //Holaaa my name is Isabella Barrientos and my codename is Jason Durullo and i love eating food and cleaning my computer

//                    default:
//                        buttonTlm.setValue("Not Moving");
//                        break;
             //   }
                switch (gamepadEvent.kind) {

//                    case DPAD_RIGHT_DOWN:
//                        armServo.setPosition(ARM_PASS);
//                        wheelieRotationServo.setPosition(WHEELIE_GRAB);
//                        miniClawServo.setPosition(MINI_CLAW_CLOSE);
//                       // groundMiniClawServo.setPosition(GM_CLAW_OPEN);



                        //break;
                    case DPAD_UP_DOWN:
                        hangRightMotor.setPower(-1);
                        hangLeftMotor.setPower(-1);
                        break;


                    case DPAD_DOWN_DOWN:
                        hangLeftMotor.setPower(1);
                        hangRightMotor.setPower(1);

                        break;
                    case DPAD_UP_UP:
                        hangRightMotor.setPower(0);
                        hangLeftMotor.setPower(0);

                        break;
                    case DPAD_DOWN_UP:
                        hangRightMotor.setPower(0);
                        hangLeftMotor.setPower(0);
                        break;

                    default:
                        break;
                }

                switch (gamepadEvent.kind) {
//                    case RIGHT_TRIGGER_DOWN:
//                        // set claw's position to 0
//                        groundMiniClawServo.setPosition(GM_CLAW_OPEN);
//                        break;
//                    case LEFT_TRIGGER_DOWN:
//                        // set claw's position to 1
//                        groundMiniClawServo.setPosition(GM_CLAW_CLOSE);
//                        break;
//
//                    case RIGHT_BUMPER_DOWN:
//                        wheelieRotationServo.setPosition(WHEELIE_UP);
//                        break;
//                    case LEFT_BUMPER_DOWN:
//                        wheelieRotationServo.setPosition(WHEELIE_DOWN);
//                        break;

                    case BUTTON_A_DOWN:
                        armServo.setPosition(ARM_FRONT);
                        break;
                    case BUTTON_Y_DOWN:
                        armServo.setPosition(ARM_BACK);
                        break;
                    case BUTTON_X_DOWN:
                        miniClawServo.setPosition(MINI_CLAW_OPEN);
                        break;
                    case BUTTON_B_DOWN:
                        miniClawServo.setPosition(MINI_CLAW_CLOSE);
                        break;

                    default:
                        buttonTlm.setValue("Not Moving");
                        break;
//                    case BUTTON_X_DOWN:
//                        // set claw's position to 0
//                        gizaClawLeftServo.setPosition(GIZA_CLAW_LEFT_OPEN);
//                        gizaClawRightServo.setPosition(GIZA_CLAW_RIGHT_OPEN);
//
//                        break;
//                    case BUTTON_B_DOWN:
//                        // set claw's position to 1
//                        gizaClawLeftServo.setPosition(GIZA_CLAW_LEFT_CLOSE);
//                        gizaClawRightServo.setPosition(GIZA_CLAW_RIGHT_CLOSE);
//                        break;


                }
            }
        });
    }
}