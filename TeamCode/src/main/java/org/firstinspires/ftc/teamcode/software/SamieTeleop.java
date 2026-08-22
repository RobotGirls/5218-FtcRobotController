package org.firstinspires.ftc.teamcode.software;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "Samie's Teleop", group ="software class")
public class SamieTeleop extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    GamepadHandler gamepadWrapper = null;
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        gamepadWrapper = new GamepadHandler(gamepad1);
        //this method will wait here until someone pushes the start button on the start button

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Press Play to lock settings");
            telemetry.update();
        }
    }
    private void handleMecanumDrive(){
        double y = gamepadWrapper.getDriveY();
        double x = gamepadWrapper.getDriveX();
        double rx = gamepadWrapper.getTurnX();

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x + rx) / denominator;
        double backRightPower = (y + x + rx) / denominator;
    }
}
