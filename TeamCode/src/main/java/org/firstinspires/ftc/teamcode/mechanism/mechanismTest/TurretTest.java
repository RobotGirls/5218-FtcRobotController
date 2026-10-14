package org.firstinspires.ftc.teamcode.mechanism.mechanismTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanism.Turret;

@TeleOp(name = "TurretTestBiobuzz")
public class TurretTest extends LinearOpMode {
    private Turret turret;

    @Override
    public void runOpMode(){
        // instantiated the turret
        turret = new Turret(hardwareMap);

        telemetry.addData("Status", "Intialized..press play");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // use the left stick x-axis on gamepad 2 to drive manual rotation
            double turretPower = gamepad2.left_stick_x;

            //apply a minor deadzone to prevent accidental movement if the joystick
            if (Math.abs(turretPower) < 0.05)  {
                turretPower = 0;
            }

            // set the power to the turret motor
            turret.setPower(turretPower);

            telemetry.addData("Turret Control", "Use Gamepad 2 Left Stick X");
            telemetry.addData("Current Power Input", turretPower);
            telemetry.addData("Encoder Position Ticks", turret.getCurrentPosition());
        } // end of while loop
    } // end of run opmode
} // end of TurretTest class
