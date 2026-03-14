package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "HuskylensTeleop", group = "Sensor")
public class Huskylens extends LinearOpMode {

    private HuskyLens huskyLens;
    private DcMotor flywheel;
    private Servo feederServo;
    private Servo hoodServo;

    private static final double SERVO_REST = 0.05;
    private static final double SERVO_FIRE = 0.3;

    private static final double HOOD_DOWN = 0.3;
    private static final double HOOD_UP = 0.05;

    private static final double kP = 0.01;
    private static final double MIN_POWER = 0.3;
    private static final double MAX_POWER = 1.0;
    private static final double SPEED_TOLERANCE = 0.05;

    private static final int SHOTS_TO_FIRE = 3;
    private static final long FIRE_TIME_MS = 200;
    private static final long RESET_TIME_MS = 250;

    private enum ShooterState {
        IDLE,
        FIRING,
        RESETTING
    }

    private ShooterState shooterState = ShooterState.IDLE;
    private int shotsFired = 0;
    private long stateTimer = 0;

    @Override
    public void runOpMode() {

        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        flywheel = hardwareMap.get(DcMotor.class, "FlyWheelMotor");
        feederServo = hardwareMap.get(Servo.class, "flapServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setPower(0);

        feederServo.setPosition(SERVO_REST);
        hoodServo.setPosition(HOOD_UP);

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        waitForStart();

        double targetPower = 0;
        double currentPower = 0;

        while (opModeIsActive()) {

            if (gamepad2.left_bumper) hoodServo.setPosition(HOOD_DOWN);
            if (gamepad2.right_bumper) hoodServo.setPosition(HOOD_UP);

            if (gamepad1.a) {

                HuskyLens.Block[] blocks = huskyLens.blocks();

                if (blocks.length > 0) {

                    int tagWidth = blocks[0].width;
                    int error = 100 - tagWidth;

                    targetPower = kP * error;
                    targetPower = Math.max(MIN_POWER, Math.min(MAX_POWER, targetPower));

                    currentPower += (targetPower - currentPower) * 0.2;
                    flywheel.setPower(currentPower);

                    boolean atSpeed =
                            Math.abs(currentPower - targetPower) < SPEED_TOLERANCE;

                    runShooterStateMachine(atSpeed);

                    telemetry.addData("Tag Width", tagWidth);
                    telemetry.addData("Target Power", targetPower);
                    telemetry.addData("Current Power", currentPower);
                    telemetry.addData("At Speed", atSpeed);
                    telemetry.addData("Shots Fired", shotsFired);
                    telemetry.addData("Shooter State", shooterState);

                } else {
                    resetShooter();
                }

            } else {
                resetShooter();
                currentPower = 0;
                flywheel.setPower(0);
            }

            telemetry.update();
        }
    }

    private void runShooterStateMachine(boolean atSpeed) {

        long now = System.currentTimeMillis();

        switch (shooterState) {

            case IDLE:
                if (atSpeed && shotsFired < SHOTS_TO_FIRE) {
                    feederServo.setPosition(SERVO_FIRE);
                    shooterState = ShooterState.FIRING;
                    stateTimer = now;
                }
                break;

            case FIRING:
                if (now - stateTimer >= FIRE_TIME_MS) {
                    feederServo.setPosition(SERVO_REST);
                    shotsFired++;
                    shooterState = ShooterState.RESETTING;
                    stateTimer = now;
                }
                break;

            case RESETTING:
                if (now - stateTimer >= RESET_TIME_MS) {
                    if (shotsFired < SHOTS_TO_FIRE) {
                        feederServo.setPosition(SERVO_FIRE);
                        shooterState = ShooterState.FIRING;
                        stateTimer = now;
                    } else {
                        shooterState = ShooterState.IDLE;
                    }
                }
                break;
        }
    }

    private void resetShooter() {
        feederServo.setPosition(SERVO_REST);
        shotsFired = 0;
        shooterState = ShooterState.IDLE;
    }
}
