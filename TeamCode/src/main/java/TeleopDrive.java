
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Teleop Drive ")

    public class TeleopDrive extends LinearOpMode {

        private final double BLOCK_NOTHING = 0.25;
        private final double BLOCK_BOTH = 0.05;


        @Override
        public void runOpMode() throws InterruptedException {


            boolean intakeOn = false;
            boolean outtakeOn = false;

            DcMotorEx leftFront, leftBack, rightBack, rightFront;


            leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
            rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
            rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

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

            // vertical lift motor
            DcMotor liftMotor;
            liftMotor = hardwareMap.get(DcMotor.class, "Lift Motor");
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Servo marshClawServo = hardwareMap.get(Servo.class, "Marsh Claw Servo");

            waitForStart();

            if (isStopRequested()) return;

            while (opModeIsActive() && !isStopRequested()) {
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = gamepad1.right_stick_x;

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
                rightFront.setPower(backRightPower);
                marshClawServo.setPosition(0.75);
                if (gamepad2.left_stick_button) {
                    liftMotor.setPower(1);
                }
                else {
                    liftMotor.setPower(0);
                }
                if (gamepad2.a) {
                    marshClawServo.setPosition(.75);
                }
                else if (gamepad1.b) {
                    marshClawServo.setPosition(.2);
                }
                telemetry.addData("Release Servo Target", marshClawServo.getPosition());
            }
        }
    }

