package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.opmodes.Limelight3ASensor;

@TeleOp(name = "LimelightDistance")

public class LimeLightDistance extends LinearOpMode {

    private double rotationPower;
    private final double BLOCK_NOTHING = 0.25;
    private final double BLOCK_BOTH = 0.05;

    private boolean useAutoAlign = false;


    double y;
    double x;
    double rx;
    //DcMotorEx IntakeMotor;
    ElapsedTime timer;

    DcMotorEx leftFront, leftBack, rightBack, rightFront;

 //   DcMotorEx FlywheelMotor;

    private double adjustedFlywheelPower;
    private boolean firstTime;

    //DcMotorEx TransportMotor;
    private Limelight3ASensor limelightSensor = new Limelight3ASensor();

    @Override
    public void runOpMode() throws InterruptedException {

        boolean intakeOn = false;
        boolean outtakeOn = false;

        initHardware();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            x = gamepad1.left_stick_x ; // Counteract imperfect strafing
            rx = gamepad1.right_stick_x;

            checkGamepad2();

            if (useAutoAlign) {

                limelightSensor.limelightProcessing(telemetry);
                alignToTag();
                //not done yet

            }

            double frontLeftPower = (y + x + rx) ;
            double backLeftPower = (y - x + rx) ;
            double frontRightPower = (y - x - rx) ;
            double backRightPower = (y + x - rx) ;

            telemetry.update();

        }

        limelightSensor.stopLimelightProcessing();
    }

    public void checkGamepad2() {
        if (gamepad2.a) {
            if (useAutoAlign) {
                useAutoAlign = false;
            } else {
                useAutoAlign = true;
            }
        }

    }
    public void initHardware() {

//        FlywheelMotor = hardwareMap.get(DcMotorEx.class, "FlywheelMotor");
//        FlywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        FlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        timer = new ElapsedTime();
        limelightSensor.initLimelight(hardwareMap, telemetry);
        telemetry.update();
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

        // FIXME cindy need to figue out if this is run without or run with encoder
        // note: you must set this after stop and reset encoder; otherwise, the robot won't move
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);



    }


    public void alignToTag(){

        rx = limelightSensor.getStrafePower(telemetry);


    }
}



//            boolean autoAlign = gamepad1.left_bumper;
//            double turnPower = 0;
//
//             if (autoAlign) {
//                double tx = limelightSensor.getTx();
//                turnPower = tx * AUTO_ALIGN_KP;
//            } else {
//                turnPower = rx;
//            }



// double FlywheelPower = gamepad2.right_stick_y; // example
// FlywheelMotor.setPower(FlywheelPower);





//transport
//  double TransportPower = 0.0;

// if (gamepad2.y) {
//   TransportPower = 1.0; // Forward
//  } else if (gamepad2.a) {
//    TransportPower = -1.0; // Reverse
//   }

// TransportMotor.setPower(TransportPower);


//double IntakeMotorPower = gamepad2.left_stick_y;
// IntakeMotor.setPower(IntakeMotorPower);

// Intake Motor (left stick)
//double intakePower = -gamepad2.left_stick_y;
//IntakeMotor.setPower(intakePower);


// Denominator is the largest motor power (absolute value) or 1
// This ensures all the powers maintain the same ratio,
// but only if at least one is out of the range [-1, 1]
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            double frontLeftPower = (y + x + turnPower) / denominator;
//            double backLeftPower = (y - x + turnPower) / denominator;
//            double frontRightPower = (y - x - turnPower) / denominator;
//            double backRightPower = (y + x - turnPower) / denominator;

//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

//

//           // leftFront.setPower(frontLeftPower);
//            leftBack.setPower(backLeftPower);
//            rightFront.setPower(frontRightPower);
//            rightBack.setPower(backRightPower);

//         //   if (TransportPower > 0) {
//
//                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            }

// leftFront.setPower(frontLeftPower);
//            leftBack.setPower(backLeftPower);
//            rightFront.setPower(frontRightPower);
//            rightBack.setPower(backRightPower);

// (removed brake code)

// Flywheel Motor (right stick)
//            double flywheelPower = -gamepad2.right_stick_y;
//          //  telemetry.addData("right_stick_y", flywheelPower);
//            limelightSensor.limelightProcessing(telemetry,timer);
//            adjustedFlywheelPower = limelightSensor.adjustFlywheelSpeed(telemetry);
//            if (flywheelPower > 0){
////                if (firstTime) {
////                    timer.reset();
////                    firstTime = false;
////                }
//                // flywheelPower = 1; //FIXME this is temporary for testing
//
//                FlywheelMotor.setPower(adjustedFlywheelPower);
//            } else if (flywheelPower < 0){
////                if (firstTime) {
////                    timer.reset();
////                    firstTime = false;
////                }
//
//                FlywheelMotor.setPower( - adjustedFlywheelPower);
//            } else {
//                FlywheelMotor.setPower(0);
//            }
//
//            FlywheelMotor.setPower(flywheelPower);
//
//
//            telemetry.update();
//
//        }
//
//        limelightSensor.stopLimelightProcessing();
//    }
//    public void initHardware() {
//
//
//        // Intake
//        // not complete yet, derived from TeleopDrive
//
//       // IntakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
//      //  IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//      //  IntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//
//        //flywheel motor
//        // need to add this on the bottom too
//
//        FlywheelMotor = hardwareMap.get(DcMotorEx.class, "FlywheelMotor");
//        FlywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        FlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        //Transport Motor
//
//
//       // TransportMotor = hardwareMap.get(DcMotorEx.class, "TransportMotor");
//       // TransportMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//       // TransportMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        timer = new ElapsedTime();
//        limelightSensor.initLimelight(hardwareMap, telemetry);
//        telemetry.update();






















//@TeleOp(name = "LimelightDistance")
//public class LimelightDistance extends LinearOpMode {
//
//    private final double AUTO_ALIGN_KP = 0.02;
//    private final double BLOCK_NOTHING = 0.25;
//    private final double BLOCK_BOTH = 0.05;
//
//    ElapsedTime timer;
//
//    DcMotorEx leftFront, leftBack, rightBack, rightFront;
//    DcMotorEx FlywheelMotor;
//
//    private double adjustedFlywheelPower;
//    private boolean firstTime;
//
//    private Limelight3ASensor limelightSensor = new Limelight3ASensor();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        initHardware();
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        while (opModeIsActive()) {
//
//            double flywheelInput = -gamepad2.right_stick_y;
//
//            limelightSensor.limelightProcessing(telemetry, timer);
//            adjustedFlywheelPower = limelightSensor.adjustFlywheelSpeed(telemetry);
//
//            if (flywheelInput > 0) {
//                FlywheelMotor.setPower(adjustedFlywheelPower);
//            } else if (flywheelInput < 0) {
//                FlywheelMotor.setPower(-adjustedFlywheelPower);
//            } else {
//                FlywheelMotor.setPower(0);
//            }
//
//            telemetry.update();
//        }
//
//        limelightSensor.stopLimelightProcessing();
//    }
//
//    public void initHardware() {
//
//        FlywheelMotor = hardwareMap.get(DcMotorEx.class, "FlywheelMotor");
//        FlywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        FlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        timer = new ElapsedTime();
//        limelightSensor.initLimelight(hardwareMap, telemetry);
//        telemetry.update();
//    }
//}
