package org.firstinspires.ftc.teamcode.software;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleopIntakes.CounterRotatingIntake;

@TeleOp(name = "CounterRotatingIntakeTeleop", group = "Linear OpMode")
public class CounterRotatingIntakeTeleop extends LinearOpMode{
    private CounterRotatingIntake intake;
    @Override
    public void runOpMode() {
        //instantiating Intake (calling the Intake() constructor in the Intake class
        // creating an instance or object of Intake
        intake = new CounterRotatingIntake(hardwareMap);

        teleopAcknowledge(); // Wait for start
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

    private void teleopAcknowledge() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}

