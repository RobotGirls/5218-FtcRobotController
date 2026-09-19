package org.firstinspires.ftc.teamcode.software;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.teleopIntakes.DoubleWheelIntake;

@TeleOp(name = "Intake TeleOp", group = "Linear OpMode")
public class RishaIntakeTeleop extends LinearOpMode {

    private DoubleWheelIntake intake;

    public RishaIntakeTeleop(HardwareMap hardwareMap) {
    }

    @Override
    public void runOpMode() {
        //instantiating Intake (calling the Intake() constructor in the Intake class
        // creating an instance or object of Intake
        intake = new DoubleWheelIntake(hardwareMap);


        waitForStart();

        while (opModeIsActive()) {
            // Press Left Bumper to spit out, Right Bumper to pull in
            if (gamepad1.right_bumper) {
                intake.setPower(1.0);
            } else if (gamepad1.left_bumper) {
                intake.setPower(-1.0);
            } else {
                intake.stop();
            }

            telemetry.addData("Intake Status", "Running");
            telemetry.update();
        }
    }




}

