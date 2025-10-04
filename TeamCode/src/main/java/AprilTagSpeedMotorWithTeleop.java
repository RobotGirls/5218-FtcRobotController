

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "TeleOp + AprilTag Speed (HUD)", group = "TeleOp")
public class AprilTagSpeedMotorWithTeleop extends LinearOpMode {

    private final double BLOCK_NOTHING = 0.25;
    private final double BLOCK_BOTH = 0.05;

    // HuskyLens
    private HuskyLens huskyLens;
    private final int READ_PERIOD = 1; // seconds

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Motor setup ---
        DcMotorEx leftFront, leftBack, rightBack, rightFront;
        DcMotorEx IntakeMotor, FlywheelMotor, TransportMotor;

        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftBack = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightBack = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");

        IntakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        FlywheelMotor = hardwareMap.get(DcMotorEx.class, "FlyWheelMotor");
        TransportMotor = hardwareMap.get(DcMotorEx.class, "TransportMotor");

        // --- Initialize HuskyLens ---
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addLine("❌ HuskyLens not detected or communication error!");
        } else {
            telemetry.addLine("✅ HuskyLens connected. Using AprilTag recognition mode.");
        }
        telemetry.update();

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        // --- Motor directions and modes ---
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx motor : new DcMotorEx[]{leftFront, leftBack, rightFront, rightBack}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        FlywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        TransportMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TransportMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;

        double distanceFactor = 1.0;
        double lastTagWidth = 0;
        double lastDistanceEstimate = 0;

        while (opModeIsActive()) {

            // === 1. HuskyLens distance-based speed factor ===
            if (rateLimit.hasExpired()) {
                rateLimit.reset();

                HuskyLens.Block[] blocks = huskyLens.blocks();

                if (blocks.length > 0) {
                    // Use first detected AprilTag
                    int tagWidthPx = blocks[0].width;
                    if (tagWidthPx > 0) {
                        // Estimate distance (inverse of pixel width)
                        double distanceEstimate = 1.0 / tagWidthPx;

                        // Scale factor: Farther → faster, Closer → slower
                        double k = 50.0; // tuning constant
                        double scaled = k * distanceEstimate;

                        // Limit factor between 0.3 (slow) and 1.0 (fast)
                        distanceFactor = Math.min(Math.max(scaled, 0.3), 1.0);

                        lastTagWidth = tagWidthPx;
                        lastDistanceEstimate = distanceEstimate;
                    }
                } else {
                    // If no tag detected, use normal driving speed
                    distanceFactor = 1.0;
                    lastTagWidth = 0;
                    lastDistanceEstimate = 0;
                }
            }

            // === 2. Driver control ===
            double y = -gamepad1.left_stick_y;  // Forward/Back
            double x = gamepad1.left_stick_x * 1.1;  // Strafe
            double rx = gamepad1.right_stick_x; // Turn

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator * distanceFactor;
            double backLeftPower = (y - x + rx) / denominator * distanceFactor;
            double frontRightPower = (y - x - rx) / denominator * distanceFactor;
            double backRightPower = (y + x - rx) / denominator * distanceFactor;

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

            // === 3. Intake, Flywheel, Transport controls ===
            double FlywheelPower = -gamepad2.right_stick_y;
            FlywheelMotor.setPower(FlywheelPower);

            double intakePower = -gamepad2.left_stick_y;
            IntakeMotor.setPower(intakePower);

            double TransportPower = 0.0;
            if (gamepad2.y) TransportPower = 1.0;
            else if (gamepad2.a) TransportPower = -1.0;
            TransportMotor.setPower(TransportPower);

            // === 4. TELEMETRY HUD ===
            telemetry.addLine("=== 📸 AprilTag + Drive HUD ===");
            telemetry.addData("Tag Detected", lastTagWidth > 0 ? "✅ Yes" : "❌ No");
            telemetry.addData("Tag Width (px)", lastTagWidth);
            telemetry.addData("Distance Est.", String.format("%.3f", lastDistanceEstimate));
            telemetry.addData("Speed Factor", String.format("%.2f", distanceFactor));
            telemetry.addLine("");
            telemetry.addData("Drive Power (L/R)",
                    String.format("FL: %.2f | FR: %.2f | BL: %.2f | BR: %.2f",
                            frontLeftPower, frontRightPower, backLeftPower, backRightPower));
            telemetry.addData("Flywheel", String.format("%.2f", FlywheelPower));
            telemetry.addData("Intake", String.format("%.2f", intakePower));
            telemetry.addData("Transport", TransportPower == 0 ? "Off" : (TransportPower > 0 ? "Forward" : "Reverse"));
            telemetry.update();
        }
    }
}
