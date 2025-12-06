package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.test.LimeLight3ASensor;

@TeleOp(name = "Sensor: Limelight3A", group = "Sensor")
public class LimeLightATTest extends LinearOpMode {

    private Limelight3A limelight;
    private LimeLight3ASensor limeLightSensor = new LimeLight3ASensor();

    private DcMotorEx flywheel;
    private DcMotorEx intake;

    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor leftBack = null;

    private final double AIM_TOLERANCE_TX = 3.0;
    private final double READY_RPM_TOLERANCE = 80;
    private final double AUTO_ALIGN_KP = 0.02;

    @Override
    public void runOpMode() throws InterruptedException {

        // Drivetrain
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

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        // Flywheel
        flywheel = hardwareMap.get(DcMotorEx.class, "FlywheelMotor");
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Intake
        intake = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight.start();

        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // Driver input
            double y  = -gamepad1.left_stick_y;
            double x  = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            // Limelight auto-align
            LLResult result = limelight.getLatestResult();
            boolean autoAlign = gamepad1.left_bumper;
            double turnPower;

            if (autoAlign && result != null && result.isValid()) {
                double tx = result.getTx();
                turnPower = tx * AUTO_ALIGN_KP;
            } else {
                turnPower = rx;
            }

            // Drive power calculation
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turnPower), 1);
            leftFront.setPower((y + x + turnPower) / denominator);
            leftBack.setPower((y - x + turnPower) / denominator);
            rightFront.setPower((y - x - turnPower) / denominator);
            rightBack.setPower((y + x - turnPower) / denominator);

            // Flywheel logic (no feeder)
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                double tx = result.getTx();
                double distance = botpose.getPosition().z;

                double targetRPM = distanceToRPM(distance);
                boolean aimed = Math.abs(tx) < AIM_TOLERANCE_TX;

                if (aimed) {
                    flywheel.setVelocity(rpmToTPS(targetRPM));
                } else {
                    flywheel.setVelocity(0);
                }

                double currentRPM = tpsToRPM(flywheel.getVelocity());

                double tagID = limeLightSensor.getFuducials(telemetry, result);
                telemetry.addData("aprilTagID", tagID);
                telemetry.addData("TX", tx);
                telemetry.addData("Distance", distance);
                telemetry.addData("Target RPM", targetRPM);
                telemetry.addData("Current RPM", currentRPM);
                telemetry.addData("Aimed?", aimed);

            } else {
                flywheel.setVelocity(0);
                telemetry.addData("Limelight", "No tag detected");
            }

            // Intake
            double intakePower = -gamepad2.left_stick_y;
            intake.setPower(intakePower);

            telemetry.update();
        }

        limelight.stop();
    }

    private double distanceToRPM(double d) {
        if (d < 0.4) return 2300;
        if (d < 0.6) return 2500;
        if (d < 0.8) return 2700;
        if (d < 1.1) return 2900;
        return 3100;
    }

    private double rpmToTPS(double rpm) {
        return (rpm * 28) / 60.0;
    }

    private double tpsToRPM(double tps) {
        return (tps * 60.0) / 28.0;
    }
}

