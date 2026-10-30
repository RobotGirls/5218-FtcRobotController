package org.firstinspires.ftc.teamcode.mechanism.mechanismTest;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanism.Turret;

@TeleOp(name = "TurretTestBiobuzz")
public class TurretTest extends LinearOpMode {
    private Turret turret;
    private Limelight3A limelight;

    double derivative;
    double integralSum;

    double lastError;

    double kp = 0.014;

    double ki;

    double kd;

    double turretPower;

    @Override
    public void runOpMode(){
        // instantiated the turret
        turret = new Turret(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "turret");


        telemetry.addData("Status", "Intialized..press play");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // use the left stick x-axis on gamepad 2 to drive manual rotation

            turretPower = PIDaiming();


            turret.setPower(turretPower);
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
    public double PIDaiming(){
        LLResult result = limelight.getLatestResult();
        double error = result.getTx();
        ElapsedTime timer =new ElapsedTime();
        if (Math.abs(error) > 3){
            telemetry.addData("Error", error);
            telemetry.addData("Tx", result.getTx());
            telemetry.addData("Motor Power", turretPower);
            error = -1 * result.getTx();
            derivative = (error - lastError) / timer.seconds();
            integralSum = integralSum +(error * timer.seconds());
            lastError = error;
            return (kp * error) + (ki * integralSum) + (kd * derivative);

        } else {
            return 0;
        }

    }
} // end of TurretTest class
