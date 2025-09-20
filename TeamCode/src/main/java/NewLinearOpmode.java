import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "NewLinOpmode ")

public class NewLinearOpmode extends LinearOpMode {


    private final double BLOCK_NOTHING = 0.25;
    private final double BLOCK_BOTH = 0.05;


    @Override
    public void runOpMode() throws InterruptedException {


        boolean intakeOn = false;
        boolean outtakeOn = false;

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

        // note: you must set this after stop and reset encoder; otherwise, the robot won't move
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);


        // Intake
        // not complete yet, derived from TeleopDrive
        DcMotorEx IntakeMotor;
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);


    //flywheel motor
        // need to add this on the bottom too
        DcMotorEx FlywheelMotor;
        FlywheelMotor = hardwareMap.get(DcMotorEx.class, "FlyWheelMotor");
        FlywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Transport Motor
        DcMotorEx TransportMotor;

       TransportMotor = hardwareMap.get(DcMotorEx.class, "TransportMotor");
        TransportMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TransportMotor.setDirection(DcMotorSimple.Direction.REVERSE);



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // double FlywheelPower = gamepad2.right_stick_y; // example
            // FlywheelMotor.setPower(FlywheelPower);


            // Flywheel Motor (right stick)
            double FlywheelPower = -gamepad2.right_stick_y;
            FlywheelMotor.setPower(FlywheelPower);


            //transport
            double TransportPower = 0.0;

            if (gamepad2.y) {
                TransportPower = 1.0; // Forward
            } else if (gamepad2.a) {
                TransportPower = -1.0; // Reverse
            }

            TransportMotor.setPower(TransportPower);


            //double IntakeMotorPower = gamepad2.left_stick_y;
           // IntakeMotor.setPower(IntakeMotorPower);

            // Intake Motor (left stick)
            double intakePower = -gamepad2.left_stick_y;
            IntakeMotor.setPower(intakePower);





            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

            if (TransportPower>0){

                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }



            telemetry.update();
        }
    }
}


