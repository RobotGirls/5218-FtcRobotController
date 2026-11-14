package org.firstinspires.ftc.teamcode.mechanism.mechanismTest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Camera-only OpMode for watching the Limelight in FieldViewer, FTC Dashboard or the Driver
 * Station. Only the "limelight" device is initialized, so no motor or servo can move. The stream
 * starts in INIT and runs until STOP. Based on team 25's LimelightFieldViewerStream.
 */
@TeleOp(name = "Limelight FieldViewer Stream", group = "Testing")
public class LimelightFieldViewerStream extends LinearOpMode {

    /** Picks the pipeline: red uses pipeline 1, blue pipeline 0. */
    private static final boolean IS_RED = true;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        LimelightSensor limelight = new LimelightSensor(hardwareMap, telemetry, IS_RED);

        try {
            telemetry.addLine("Camera-only mode: no motors or servos are initialized.");
            telemetry.addLine("Video: FieldViewer Live, 192.168.43.1:8080/dash, or 192.168.43.1:11117");
            telemetry.update();

            while (opModeInInit()) {
                showTag(limelight);
            }
            while (opModeIsActive()) {
                showTag(limelight);
            }
        } finally {
            limelight.stop();
        }
    }

    /** Read-only tag readout so screenshots of the video come with matching numbers. */
    private void showTag(LimelightSensor limelight) {
        telemetry.addData("Alliance", IS_RED ? "RED (pipeline 1)" : "BLUE (pipeline 0)");
        if (limelight.seeValidTag()) {
            limelight.isValidShooting();
        } else {
            telemetry.addData("Tag", "none seen");
        }
        telemetry.update();
        sleep(50);
    }
}
