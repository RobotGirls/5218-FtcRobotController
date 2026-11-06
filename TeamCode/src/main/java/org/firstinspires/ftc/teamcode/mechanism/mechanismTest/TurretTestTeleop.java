package org.firstinspires.ftc.teamcode.mechanism.mechanismTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanism.mechanismTest.LimelightSensor;
import org.firstinspires.ftc.teamcode.mechanism.mechanismTest.RobotHardware;
import org.firstinspires.ftc.teamcode.mechanism.Turret;

@TeleOp(name = "TurretTestBiobuzz2")
public class TurretTestTeleop extends LinearOpMode {

//    private Turret turret;

    private static final boolean IS_RED = true;

    double turretPower;
   // RobotHardware robot;
    LimelightSensor limelightSensor;

    @Override
    public void runOpMode(){

    //    robot = new RobotHardware(hardwareMap);
        limelightSensor = new LimelightSensor(hardwareMap, telemetry, IS_RED);
        // instantiated the turret
 //       turret = new Turret(hardwareMap);

        telemetry.addData("Status", "Initialized..press play");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Chassis drive handler for gamepad 1
          //  robot.gamepadHandler(gamepad1);
            if (limelightSensor.checkTag()) {
                limelightSensor.getDistance();
                telemetry.update();
                //limelightSensor.getXValue();
            }
            // Hold 'A' on gamepad2 or gamepad1 to auto-align turret using Limelight
            /*if (gamepad2.a || gamepad1.a) {
                limelightSensor.getDistance();
                telemetry.addData("Turret Mode", "Limelight Auto-Aligning (Button A Held)");
            } else {
                // Reset PID variables when auto-align button is released
                limelightSensor.resetPID();

                // Manual turret rotation driven by Gamepad 2 Left Stick X
                turretPower = gamepad2.left_stick_x;

                // Apply deadzone to prevent drift
                if (Math.abs(turretPower) < 0.05) {
                    turretPower = 0;
                }

                // Apply power to turret motor
              //  turret.setPower(turretPower);
                telemetry.addData("Turret Mode", "Manual Control (Gamepad 2 Left Stick X)");
                telemetry.addData("Current Power Input", turretPower);*/
            //}

           // telemetry.addData("Turret Position (0-1)", turret.getPosition());
        //    telemetry.addData("At Limit", turret.atLimit());

        } // end of while loop
    } // end of run opmode
} // end of TurretTest class
