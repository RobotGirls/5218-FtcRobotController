package org.firstinspires.ftc.teamcode.purepursuitautos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.purepursuit.FieldViewerLink;
import org.firstinspires.ftc.teamcode.purepursuit.ftc.PurePursuitDrive;

/**
 * Drive around and watch the Road Runner localizer's pose in the FieldViewer (Live mode) and
 * FTC Dashboard. Start the robot at the field center facing +x (towards the right side of
 * biobuzz-field.png), or press X to reset the pose there.
 */
@TeleOp(name = "PP Localization Test", group = "Pure Pursuit")
public class FieldViewerLocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        PurePursuitDrive drive = PurePursuitDrive.create(hardwareMap, new Pose2d(0, 0, 0));
        drive.setRouteName("Localization Test");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x) {
                drive.setPose(new Pose2d(0, 0, 0));
            }
            drive.setPowers(new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                    -gamepad1.right_stick_x));
            drive.updatePoseEstimate();
            drive.reportIdle("driving");

            Pose2d pose = drive.getPose();
            telemetry.addData("x", "%.1f", pose.position.x);
            telemetry.addData("y", "%.1f", pose.position.y);
            telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("FieldViewer", FieldViewerLink.get().isViewerConnected()
                    ? "connected" : "not connected");
            telemetry.addLine("X: reset pose to (0, 0, 0)");
            telemetry.update();
        }
        drive.stop();
    }
}
