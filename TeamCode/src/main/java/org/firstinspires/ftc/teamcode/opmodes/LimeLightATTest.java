package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "Sensor: Limelight3A", group = "Sensor")
public class LimeLightATTest extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotorEx flywheel;
    private DcMotorEx feeder;
    private DcMotorEx intake;


    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor leftBack = null;


    // ---------------- Constants ----------------
    private final double AIM_TOLERANCE_TX = 3.0;         // degrees
    private final double READY_RPM_TOLERANCE = 80;       // RPM allowed error
    private final double FEED_POWER = 0.9;               // feeder wheel power
    private final double AUTO_ALIGN_KP = 0.02;           // drivetrain rotation proportional

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------------- DRIVETRAIN ----------------
        leftFront  = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        rightBack  = hardwareMap.get(DcMotor.class, "backRight");
        leftBack   = hardwareMap.get(DcMotor.class, "backLeft");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---------------- Limelight ----------------
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        // ---------------- Flywheel + Feeder ----------------
        flywheel = hardwareMap.get(DcMotorEx.class, "flyWheelMotor");
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        feeder = hardwareMap.get(DcMotorEx.class, "feederMotor");
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        limelight.start();

        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // ----------------- DRIVER INPUT -----------------
            double y  = -gamepad1.left_stick_y;   // forward/back
            double x  = gamepad1.left_stick_x * 1.1; // strafe
            double rx = gamepad1.right_stick_x;   // rotation

            // ----------------- LIMELIGHT TARGET -----------------
            LLResult result = limelight.getLatestResult();
            boolean autoAlign = gamepad1.left_bumper;  // driver presses LB to enable auto-align
            double turnPower = 0;

            if (autoAlign && result != null && result.isValid()) {
                double tx = result.getTx();
                turnPower = tx * AUTO_ALIGN_KP;      // simple proportional control
            } else {
                turnPower = rx;  // manual rotation
            }

            // ----------------- DRIVE CALC -----------------
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turnPower), 1);
            double frontLeftPower  = (y + x + turnPower) / denominator;
            double backLeftPower   = (y - x + turnPower) / denominator;
            double frontRightPower = (y - x - turnPower) / denominator;
            double backRightPower  = (y + x - turnPower) / denominator;

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

            // ----------------- FLYWHEEL & FEEDER -----------------
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                double tx = result.getTx();
                double distance = botpose.getPosition().z;
                double targetRPM = distanceToRPM(distance);
                boolean aimed = Math.abs(tx) < AIM_TOLERANCE_TX;

                // Flywheel spins only if aimed
                if (aimed) {
                    flywheel.setVelocity(rpmToTPS(targetRPM));
                } else {
                    flywheel.setVelocity(0);
                }

                double currentRPM = tpsToRPM(flywheel.getVelocity());
                boolean rpmReady = Math.abs(currentRPM - targetRPM) < READY_RPM_TOLERANCE;
                boolean fireButton = gamepad1.right_trigger > 0.5;

                // Feeder only runs when all conditions met
                if (aimed && rpmReady && fireButton) {
                    feeder.setPower(FEED_POWER);
                } else {
                    feeder.setPower(0);
                }

                telemetry.addData("TX", tx);
                telemetry.addData("Distance", distance);
                telemetry.addData("Target RPM", targetRPM);
                telemetry.addData("Current RPM", currentRPM);
                telemetry.addData("Aimed?", aimed);
                telemetry.addData("RPM Ready?", rpmReady);
                telemetry.addData("Feeding?", aimed && rpmReady && fireButton);

            } else {
                flywheel.setVelocity(0);
                feeder.setPower(0);
                telemetry.addData("Limelight", "No tag detected");
            }

            // ----------------- INTAKE -----------------
            double intakePower = -gamepad2.left_stick_y; // forward/backward on GP2
            intake.setPower(intakePower);


            telemetry.update();
        }

        limelight.stop();
    }

    // ---------------- DISTANCE → RPM ----------------
    private double distanceToRPM(double d) {
        if (d < 0.4) return 2300;
        if (d < 0.6) return 2500;
        if (d < 0.8) return 2700;
        if (d < 1.1) return 2900;
        return 3100;
    }

    // ---------------- UNIT CONVERSIONS ----------------
    private double rpmToTPS(double rpm) {
        return (rpm * 28) / 60.0;
    }

    private double tpsToRPM(double tps) {
        return (tps * 60.0) / 28.0;
    }
}
