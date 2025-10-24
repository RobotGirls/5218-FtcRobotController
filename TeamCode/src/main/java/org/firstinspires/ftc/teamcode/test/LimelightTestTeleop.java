package org.firstinspires.ftc.teamcode.test;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "JasmineAprilTagTestTeleop3")

public class LimelightTestTeleop extends LinearOpMode {

    private Limelight3Asensor limelightSensor = new Limelight3Asensor();

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        while (opModeIsActive()) {
            limelightSensor.limelightProcessing(telemetry);

            // Share the CPU.
            sleep(20);
        }
        limelightSensor.stopLimeLightProccessing();
    }

    public void initHardware() {
        limelightSensor.initLimeLight(hardwareMap, telemetry);

    }


}
