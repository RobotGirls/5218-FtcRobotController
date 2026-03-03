package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "gobulidaTeleop")
public class NewLinearOpmode extends LinearOpMode {

    private static final double HOOD_DOWN = 1.0;
    private static final double HOOD_UP = 0.0;

    private static final double FLAP_IN = 0.35;
    private static final double FLAP_OUT = .05;

    private static final double FLYWHEEL_HIGH_POWER = 0.6;
    private static final double FLYWHEEL_LOW_POWER  = 0.46;

    private static final double FLAP_PULSE_TIME = 0.25;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx leftFront  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx leftBack   = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx rightBack  = hardwareMap.get(DcMotorEx.class, "backRight");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx flywheelMotor = hardwareMap.get(DcMotorEx.class, "FlyWheelMotor");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Servo flapServo = hardwareMap.get(Servo.class, "flapServo");
        Servo hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        boolean flywheelHigh = false;
        boolean flywheelLow = false;
        boolean lastAState = false;
        boolean lastBState = false;

        boolean lastRBState = false;
        boolean flapActive = false;

        ElapsedTime flapTimer = new ElapsedTime();

        flapServo.setPosition(FLAP_IN);
        hoodServo.setPosition(HOOD_DOWN);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y  = -gamepad1.right_stick_y;
            double x  =  gamepad1.right_stick_x;
            double rx = -gamepad1.left_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            leftFront.setPower((y + x + rx) / denominator);
            leftBack.setPower((y - x + rx) / denominator);
            rightFront.setPower((y - x - rx) / denominator);
            rightBack.setPower((y + x - rx) / denominator);

            intakeMotor.setPower(-gamepad2.left_stick_y);

            boolean currentA = gamepad2.a;
            if (currentA && !lastAState) {
                flywheelHigh = !flywheelHigh;
                if (flywheelHigh) flywheelLow = false;
            }
            lastAState = currentA;

            boolean currentB = gamepad2.b;
            if (currentB && !lastBState) {
                flywheelLow = !flywheelLow;
                if (flywheelLow) flywheelHigh = false;
            }
            lastBState = currentB;

            if (flywheelHigh) {
                flywheelMotor.setPower(FLYWHEEL_HIGH_POWER);
            } else if (flywheelLow) {
                flywheelMotor.setPower(FLYWHEEL_LOW_POWER);
            } else {
                flywheelMotor.setPower(0);
            }

            boolean currentRB = gamepad2.right_bumper;
            if (currentRB && !lastRBState && !flapActive) {
                flapActive = true;
                flapTimer.reset();
                flapServo.setPosition(FLAP_OUT);
            }
            lastRBState = currentRB;

            if (flapActive && flapTimer.seconds() >= FLAP_PULSE_TIME) {
                flapServo.setPosition(FLAP_IN);
                flapActive = false;
            }

            if (gamepad2.x) hoodServo.setPosition(HOOD_UP);
            if (gamepad2.y) hoodServo.setPosition(HOOD_DOWN);

            telemetry.addData("Flywheel",
                    flywheelHigh ? "HIGH" :
                            flywheelLow  ? "LOW"  : "OFF");
            telemetry.update();
        }
    }
}
