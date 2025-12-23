package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "gobulidaTeleop ")
public class NewLinearOpmode extends LinearOpMode {

    private final double FLAP_IN = 0.3;
    private final double FLAP_OUT = 0.05;
    public void runOpMode() throws InterruptedException {

        DcMotorEx leftFront, leftBack, rightBack, rightFront;

        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftBack = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightBack = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx IntakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        IntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx FlywheelMotor = hardwareMap.get(DcMotorEx.class, "FlyWheelMotor");
        FlywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean flywheelOn = false;
        boolean lastAState = false;
       final double FLYWHEEL_POWER = .95;
        final double FLYWHEEL_POWER_BACK = -.2;


        Servo flapServo = hardwareMap.get(Servo.class, "flapServo");
        flapServo.setPosition(FLAP_IN);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_x;
            double x = gamepad1.right_stick_x * 1.1;
            double rx = gamepad1.right_stick_y;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            leftFront.setPower((y - x + rx) / denominator);
            leftBack.setPower((y + x + rx) / denominator);
            rightFront.setPower((y - x - rx) / denominator);
            rightBack.setPower((y + x - rx) / denominator);

            IntakeMotor.setPower(-gamepad2.left_stick_y);
            FlywheelMotor.setPower(gamepad2.right_stick_y);


//            boolean currentAState = gamepad2.a;
//            if (currentAState && !lastAState) {
//                flywheelOn = !flywheelOn;
//            }


        //   FlywheelMotor.setPower(flywheelOn ? FLYWHEEL_POWER : 0);
         //  lastAState = currentAState;

            if (gamepad2.left_bumper) {flapServo.setPosition(FLAP_IN);}
            if (gamepad2.right_bumper) flapServo.setPosition(FLAP_OUT);

            telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
            telemetry.update();
        }
    }
}
