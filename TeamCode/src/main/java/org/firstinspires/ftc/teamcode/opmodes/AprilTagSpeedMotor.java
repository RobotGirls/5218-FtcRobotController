package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "HuskyLens with Motor", group = "Sensor")
//@Disabled
public class AprilTagSpeedMotor extends LinearOpMode {

    private final int READ_PERIOD = 1;
    private HuskyLens huskyLens;
    private DcMotor driveMotor;

    @Override
    public void runOpMode() {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        driveMotor = hardwareMap.get(DcMotor.class, "TestingMotor"); // configure this in Robot Config
        driveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        // Look for AprilTags
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.update();
        waitForStart();

        // --- Control constants (tune these) ---
        int targetWidth = 100;   // Pixel width at which robot should stop
        double kP = 0.01;        // Proportional constant

        while(opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);

            if (blocks.length > 0) {
                // Use first detected tag
                int tagWidthPx = blocks[0].width;

                if (tagWidthPx > 0) {
                    // Error = how far away from stopping point
                    int error = targetWidth - tagWidthPx;

                    // Motor power proportional to error
                    double motorPower = kP * error;

                    // Clamp between 0 and 1
                    motorPower = Math.max(0.0, Math.min(1.0, motorPower));

                    driveMotor.setPower(motorPower);

                    // Telemetry for debugging
                    telemetry.addData("Tag Width (px)", tagWidthPx);
                    telemetry.addData("Target Width", targetWidth);
                    telemetry.addData("Error", error);
                    telemetry.addData("Motor Power", motorPower);
                }
            } else {
                driveMotor.setPower(0); // stop if no tag detected
            }

            telemetry.update();
        }
    }
}
