package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "LimelightTelop", group = "Competition")
public class LimeLightTestTelop extends LinearOpMode {

    // --- Drive Motors ---
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // --- Mechanism Motors (from second file) ---
    private DcMotorEx flywheelMotor;
    private DcMotorEx intakeMotor;

    // --- Sensors ---
    private Limelight3A limelight;
    private IMU imu;

    // --- Control Constants ---
    private static final double FLYWHEEL_KP = 0.02;
    private static final double FLYWHEEL_MIN_POWER = 0.2;
    private static final double FLYWHEEL_MAX_POWER = 1.0;
    private static final double TARGET_AREA = 5.0;

    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();

        if (limelight != null) {
            limelight.pipelineSwitch(0);
            limelight.start();
        }

        telemetry.addData("Status", "Init complete");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (imu != null) {
                updateIMUOrientation();
            }

            // Vision → flywheel control
            if (limelight != null) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    processVision(result);
                } else {
                    flywheelMotor.setPower(0);
                    telemetry.addData("Vision", "No target detected");
                }
            }

            // Drive using driving code from FIRST file
            driveControl();

            // Intake + flywheel manual control added from SECOND file
            manualMechanisms();

            telemetry.update();
        }
    }

    /** Initialize all hardware devices */
    private void initializeHardware() {

        // Sensors
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
        } catch (Exception e) { limelight = null; }

        try {
            imu = hardwareMap.get(IMU.class, "imu");
        } catch (Exception e) { imu = null; }

        // Mechanism motors (from second file)
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "FlywheelMotor");
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Drive motors
        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // IMU
        if (imu != null) {
            IMU.Parameters imuParams = new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    )
            );
            imu.initialize(imuParams);
        }
    }

    private void updateIMUOrientation() {
        if (imu == null || limelight == null) return;
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
    }

    /** Auto-flywheel control from FIRST file */
    private void processVision(LLResult result) {
        double ta = result.getTa();
        double error = TARGET_AREA - ta;

        double flywheelPower =
                Range.clip(FLYWHEEL_KP * error, FLYWHEEL_MIN_POWER, FLYWHEEL_MAX_POWER);

        if (ta > 0.05) {
            flywheelMotor.setPower(flywheelPower);
        } else {
            flywheelMotor.setPower(0);
        }

        telemetry.addLine("=== Limelight Vision ===");
        telemetry.addData("tx", result.getTx());
        telemetry.addData("ty", result.getTy());
        telemetry.addData("ta", ta);
        telemetry.addData("Flywheel Power", flywheelPower);
    }

    /** DRIVE CONTROL (UNTOUCHED — from your FIRST file) */
    private void driveControl() {
        double y  = -gamepad1.right_stick_y;
        double x  = gamepad1.right_stick_x ;
        double rx = -gamepad1.left_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double fl = (y + x - rx) / denominator;
        double bl = (y - x - rx) / denominator;
        double fr = (-y + x - rx) / denominator;
        double br = (-y - x - rx) / denominator;

        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }

    /** Intake + Manual Flywheel (from SECOND file) */
    private void manualMechanisms() {

        // Flywheel manual control (right stick)
       double flywheelPower = (gamepad2.a) ? 0.70 : (gamepad2.x) ? -0.70 : 0.0;
        flywheelMotor.setPower(flywheelPower);




        // Intake (left stick)
        double intakePower = -gamepad2.left_stick_y;
        intakeMotor.setPower(intakePower);

        telemetry.addData("Flywheel (manual)", flywheelPower);
        telemetry.addData("Intake", intakePower);
    }
}





//package org.firstinspires.ftc.teamcode.opmodes;
//
//
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.util.Range;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//
//import java.util.Calendar;
//
///**
// * ---------------------------------------------------------------------------
// *  LimelightTeleOp.java
// *  Team: [Your Team Name / Number]
// *  Description:
// *  TeleOp that integrates Limelight3A vision with a mecanum drive system,
// *  flywheel motor control (based on target distance), and intake motor.
// * ---------------------------------------------------------------------------
// */
//@TeleOp(name = "LimelightTelop2", group = "Sensor")
//public class LimeLightTestTelop extends LinearOpMode {
//
//    // --- Drive Motors ---
//    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
//
//    // --- Mechanism Motors ---
//    private DcMotorEx flywheelMotor;
//    private DcMotorEx intakeMotor;
//
//    // --- Sensors ---
//    private Limelight3A limelight;
//    private IMU imu;
//
//    // --- Control Constants ---
//    private static final double FLYWHEEL_KP = 0.02;
//    private static final double FLYWHEEL_MIN_POWER = 0.2;
//    private static final double FLYWHEEL_MAX_POWER = 1.0;
//    private static final double TARGET_AREA = 5.0; // Target area when tag is at ideal distance
//    private Calendar PortForwarder;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        initializeHardware();
//
//        if (limelight != null) {
//            limelight.pipelineSwitch(0);
//            limelight.start();
//        } else {
//            telemetry.addData("Limelight", "NOT found in hardwareMap");
//        }
//
//        telemetry.addData("Status", "✅ Initialization complete. Waiting for start...");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            if (imu != null) {
//                updateIMUOrientation();
//            }
//
//            // --- Process Limelight Data ---
//            if (limelight != null) {
//                LLResult result = limelight.getLatestResult();
//                if (result != null && result.isValid()) {
//                    processVision(result);
//                } else {
//                    flywheelMotor.setPower(0);
//                    telemetry.addData("Vision", "No target detected");
//                }
//            } else {
//                flywheelMotor.setPower(0);
//                telemetry.addData("Vision", "Limelight not configured");
//            }
//
//            // --- Drive & Intake Control ---
//            driveControl();
//            intakeControl();
//
//            telemetry.update();
//        }
//    }
//
//
//    /**
//     * Maps and configures all motors and sensors.
//     */
//    private void initializeHardware() {
//        // Sensors
//        try {
//            limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        } catch (Exception e) {
//            limelight = null;
//        }
//
//        try {
//            imu = hardwareMap.get(IMU.class, "imu");
//        } catch (Exception e) {
//            imu = null;
//        }
//
//        // Mechanism motors
//        flywheelMotor = hardwareMap.get(DcMotorEx.class, "FlywheelMotor");
//        intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
//
//        // Drive motors
//        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
//        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
//        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
//        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
//
//        // Motor directions
//        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        // Motor modes
//        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//
//        // IMU setup (only if it exists)
//        if (imu != null) {
//            IMU.Parameters imuParams = new IMU.Parameters(
//                    new RevHubOrientationOnRobot(
//                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
//            imu.initialize(imuParams);
//        }
//    }
//
//    /**
//     * Updates Limelight with the robot's yaw orientation.
//     */
//    private void updateIMUOrientation() {
//        if (imu == null || limelight == null) return;
//
//        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
//    }
//
//    /**
//     * Adjusts flywheel speed based on the target’s distance (area).
//     */
//    private void processVision(LLResult result) {
//        double tx = result.getTx();
//        double ty = result.getTy();
//        double ta = result.getTa();
//        Pose3D pose = result.getBotpose_MT2();
//
//        double error = TARGET_AREA - ta;
//        double flywheelPower = Range.clip(FLYWHEEL_KP * error, FLYWHEEL_MIN_POWER, FLYWHEEL_MAX_POWER);
//
//        if (ta > 0.05) {
//            flywheelMotor.setPower(flywheelPower);
//        } else {
//            flywheelMotor.setPower(0);
//        }
//
//        telemetry.addLine("=== Limelight Data ===");
//        telemetry.addData("tx", "%.2f", tx);
//        telemetry.addData("ty", "%.2f", ty);
//        telemetry.addData("ta", "%.2f", ta);
//        telemetry.addData("Flywheel Power", "%.2f", flywheelPower);
//        telemetry.addData("Pose (MT2)", pose.toString());
//    }
//
//    /**
//     * Mecanum drive control using Gamepad 1.
//     */
//    private void driveControl() {

//        double x = -gamepad1.right_stick_x;  //
//        double y = gamepad1.right_stick_y ; //
//        double rx = gamepad1.left_stick_x; //
//
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

//        double flPower = (y + x - rx) / denominator;
//        double blPower = (y - x - rx) / denominator;
//        double frPower = (-y + x - rx) / denominator;
//        double brPower = (-y - x - rx) / denominator;
//
//
//        frontLeft.setPower(flPower);
//        backLeft.setPower(blPower);
//        frontRight.setPower(frPower);
//        backRight.setPower(brPower);
//
//        telemetry.addLine("=== Drive Motors ===");
//        telemetry.addData("FL", "%.2f", flPower);
//        telemetry.addData("FR", "%.2f", frPower);
//        telemetry.addData("BL", "%.2f", blPower);
//        telemetry.addData("BR", "%.2f", brPower);
//    }
//
////    private void driveControl() {
////        double x = -gamepad1.right_stick_x;  // forward/backward
////        double y = gamepad1.left_stick_y ; // rotation
////        double rx = gamepad1.right_stick_y; // strafe
////
////
////
////        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
////        double flPower = (y + x + rx) / denominator;
////        double blPower = (y - x + rx) / denominator;
////        double frPower = (y - x - rx) / denominator;
////        double brPower = (y + x - rx) / denominator;
////
////        frontLeft.setPower(flPower);
////        backLeft.setPower(blPower);
////        frontRight.setPower(frPower);
////        backRight.setPower(brPower);
////
////        telemetry.addLine("=== Drive Motors ===");
////        telemetry.addData("FL", "%.2f", flPower);
////        telemetry.addData("FR", "%.2f", frPower);
////        telemetry.addData("BL", "%.2f", blPower);
////        telemetry.addData("BR", "%.2f", brPower);
////    }
//
//    /**
//     * Intake control using Gamepad 2 triggers.
//     */
//    private void intakeControl() {
//        double intakePower = 0.0;
//        if (gamepad2.right_trigger > 0.1) {
//            intakePower = gamepad2.right_trigger; // intake in
//        } else if (gamepad2.left_trigger > 0.1) {
//            intakePower = -gamepad2.left_trigger; // reverse
//        }
//        intakeMotor.setPower(intakePower);
//
//        telemetry.addLine("=== Intake ===");
//        telemetry.addData("Power", "%.2f", intakePower);
//    }
//}
