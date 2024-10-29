
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "LM0Teleop ")

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
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);


            // vertical lift motor
            DcMotorEx liftMotor;
            liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);



//
            DcMotorEx climbLeftMotor;
            DcMotorEx climbRightMotor;

            climbLeftMotor = hardwareMap.get(DcMotorEx.class, "climbLeftMotor");
            climbLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            climbRightMotor = hardwareMap.get(DcMotorEx.class, "climbRightMotor");
            climbRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            climbLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            climbRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            climbLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            Servo marshClawServo = hardwareMap.get(Servo.class, "marshClawServo");
            Servo marshClawRotationServo = hardwareMap.get(Servo.class, "marshClawRotationServo");
            marshClawServo.setPosition(0.99);
            marshClawRotationServo.setPosition(0.4);
            Servo horizantalLiftServo = hardwareMap.get(Servo.class, "horizantalLiftServo");
            horizantalLiftServo.setPosition(0.4);

            waitForStart();

            if (isStopRequested()) return;

            while (opModeIsActive() && !isStopRequested()) {
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = gamepad1.right_stick_x;



                double climbLeftPower = gamepad2.right_stick_y;
                climbLeftMotor.setPower(climbLeftPower);
                climbRightMotor.setPower(climbLeftPower);


               double liftMotorPower = gamepad2.left_stick_y;
               liftMotor.setPower(liftMotorPower);


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

                if (climbLeftPower>0){

                    leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }




                if (gamepad2.x) {
                    horizantalLiftServo.setPosition(0.4);
                    telemetry.addData("horizantal lift shrinks ", horizantalLiftServo.getPosition());
                }
                else if (gamepad2.b) {

                    horizantalLiftServo.setPosition(.7);
                    telemetry.addData(" horizantal lift extends ", horizantalLiftServo.getPosition());
                }

                if (gamepad2.y) {
                    marshClawServo.setPosition(.99);
                    telemetry.addData("Pick Up Servo ", marshClawServo.getPosition());
                }
                else if (gamepad2.a) {
                    marshClawServo.setPosition(0.15);
                    telemetry.addData("Release Servo ", marshClawServo.getPosition());
                }
                if (gamepad2.right_bumper) {
                   marshClawRotationServo.setPosition(.4);
                    telemetry.addData("Rotate Servo Up", marshClawRotationServo.getPosition());
                }
                else if (gamepad2.left_bumper) {
                    marshClawRotationServo.setPosition(.7);
                    telemetry.addData("Rotate Servo Down", marshClawRotationServo.getPosition());

                }

                telemetry.update();
            }
        }
    }


