package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "AprilTagTeleop", group = "Sensor")
public class AprilTagSpeedMotor extends LinearOpMode {

    private final int READ_PERIOD = 1;

    private HuskyLens huskyLens;
    private DcMotor flywheel;
    private Servo feederServo;

    // ---- SERVO POSITIONS ----
    private static final double SERVO_REST = 0.2;
    private static final double SERVO_FIRE = 0.6;

    // ---- CONTROL CONSTANTS ----
    private static final double kP = 0.01;
    private static final double SPEED_TOLERANCE = 0.05; // how close is "ready"
    private static final double MIN_POWER = 0.3;
    private static final double MAX_POWER = 1.0;

    @Override
    public void runOpMode() {

        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        flywheel = hardwareMap.get(DcMotor.class, "FlyWheelMotor");
        feederServo = hardwareMap.get(Servo.class, "flapServo");

        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setPower(0);

        feederServo.setPosition(SERVO_REST);

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData("ERROR", "HuskyLens not responding");
        } else {
            telemetry.addData("Status", "HuskyLens Ready");
        }

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        telemetry.update();

        waitForStart();

        double targetPower = 0;
        double currentPower = 0;

        while (opModeIsActive()) {

            if (!rateLimit.hasExpired()) continue;
            rateLimit.reset();

            if (gamepad1.a) { // HOLD TO SHOOT

                HuskyLens.Block[] blocks = huskyLens.blocks();

                if (blocks.length > 0) {

                    int tagWidthPx = blocks[0].width;

                    // Distance → power mapping
                    int error = 100 - tagWidthPx;
                    targetPower = kP * error;

                    targetPower = Math.max(MIN_POWER,
                            Math.min(MAX_POWER, targetPower));

                    // Smooth ramping
                    currentPower += (targetPower - currentPower) * 0.2;
                    flywheel.setPower(currentPower);

                    // Check if flywheel is "up to speed"
                    boolean atSpeed =
                            Math.abs(currentPower - targetPower) < SPEED_TOLERANCE;

                    if (atSpeed) {
                        feederServo.setPosition(SERVO_FIRE);
                    } else {
                        feederServo.setPosition(SERVO_REST);
                    }

                    telemetry.addData("Tag Width", tagWidthPx);
                    telemetry.addData("Target Power", targetPower);
                    telemetry.addData("Current Power", currentPower);
                    telemetry.addData("Shooter Ready", atSpeed);

                } else {
                    flywheel.setPower(0);
                    feederServo.setPosition(SERVO_REST);
                }

            } else {
                // Button released → reset everything
                flywheel.setPower(0);
                currentPower = 0;
                feederServo.setPosition(SERVO_REST);
            }

            telemetry.update();
        }
    }
}
