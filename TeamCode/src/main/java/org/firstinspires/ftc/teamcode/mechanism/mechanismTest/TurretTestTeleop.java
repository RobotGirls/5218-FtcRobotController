package org.firstinspires.ftc.teamcode.mechanism.mechanismTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.mechanism.mechanismTest.LimelightSensor;
import org.firstinspires.ftc.teamcode.mechanism.mechanismTest.RobotHardware;
import org.firstinspires.ftc.teamcode.mechanism.Turret;

@TeleOp(name = "TurretTestBiobuzzSCUBA")
public class TurretTestTeleop extends LinearOpMode {

//    private Turret turret;

    private static final boolean IS_RED = true;

    double turretPower;
    boolean autoAlign = false;
    boolean alignOn = false;
    RobotHardware robot;
    Turret turret;
    LimelightSensor limelightSensor;

    @Override
    public void runOpMode(){
        robot = new RobotHardware(hardwareMap);
        limelightSensor = new LimelightSensor(hardwareMap, telemetry, IS_RED);
        // instantiated the turret
       turret = new Turret(hardwareMap);

        telemetry.addData("Status", "Initialized..press play");
        telemetry.update();
        //Mechanisms
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//        DcMotorEx forwardFlywheel = hardwareMap.get(DcMotorEx.class, "ForwardFlywheel");
//        forwardFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        forwardFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        DcMotorEx backwardFlywheel = hardwareMap.get(DcMotorEx.class, "BackwardFlywheel");
//        backwardFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backwardFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            // Chassis drive handler for gamepad 1
            robot.gamepadHandler(gamepad1);
              //   Hold 'A' on gamepad2 or gamepad1 to auto-align turret using Limelight

//            forwardFlywheel.setPower(gamepad2.left_stick_y);
//            backwardFlywheel.setPower(gamepad2.left_stick_y);

            if (gamepad2.a) {
                if (autoAlign){
                    autoAlign = false;
                } else {
                    autoAlign = true;
                }
            }

            if (autoAlign) {
                if (limelightSensor.seeValidTag()) {
                    limelightSensor.PIDAiming(turret);
                    telemetry.addData("Turret Mode", "Limelight Auto-Aligning (Button A Held)");

                    if (limelightSensor.isValidShooting()){
                        limelightSensor.PIDAiming(turret);
                        telemetry.addData("Turret Mode", "Limelight Auto-Aligning (Button A Held)");

                    }

                }

            } else {
                //  Reset PID variables when auto-align button is released
                limelightSensor.resetPID();

                // Manual turret rotation driven by Gamepad 2 Left Stick X
                turretPower = gamepad2.left_stick_x;
                intakeMotor.setPower(gamepad2.left_stick_y);

                // Apply deadzone to prevent drift
                if (Math.abs(turretPower) < 0.05) {
                    turret.stop();
                } else {
                    turret.setPower(turretPower);
                }


                telemetry.addData("Turret Mode", "Manual Control (Gamepad 2 Left Stick X)");
                telemetry.addData("Current Power Input", turretPower);
                telemetry.update();
            } // end else

            // telemetry.addData("Turret Position (0-1)", turret.getPosition());
            // telemetry.addData("At Limit", turret.atLimit());

        } // end of while loop
    } // end of run opmode
} // end of TurretTest class
