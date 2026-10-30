package org.firstinspires.ftc.teamcode.purepursuitautos;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.purepursuit.FieldViewerLink;
import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.ftc.PurePursuitDrive;
import org.firstinspires.ftc.teamcode.purepursuit.paths.Routes;
import org.firstinspires.ftc.teamcode.purepursuit.paths.TestRoutes;

/**
 * Push the robot around by hand and watch the localizer's pose in the FieldViewer (Live mode),
 * FTC Dashboard and telemetry. The drive motors coast so the wheels roll freely. Pick a route's
 * start pose with the dpad and put the robot there so the picture lines up with the field.
 * Tracks during init too, so pressing Start is optional.
 */
@TeleOp(name = "PP Push Localization Test", group = "Pure Pursuit")
public class PushLocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        int selected = Routes.ALL.indexOf(TestRoutes.STRAIGHT_24);
        Route route = Routes.ALL.get(selected);
        PurePursuitDrive drive = PurePursuitDrive.create(hardwareMap, route.startPose);
        drive.setRouteName("Push Localization Test");
        drive.setCoasting(true);
        boolean upWasPressed = false, downWasPressed = false;

        try {
            while (opModeInInit() || opModeIsActive()) {
                boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
                if ((up && !upWasPressed) || (down && !downWasPressed)) {
                    selected = (selected + (up ? -1 : 1) + Routes.ALL.size()) % Routes.ALL.size();
                    route = Routes.ALL.get(selected);
                    drive.setPose(route.startPose);
                }
                upWasPressed = up;
                downWasPressed = down;
                if (gamepad1.x) {
                    drive.setPose(route.startPose);
                }

                drive.updatePoseEstimate();
                drive.reportIdle("pushing by hand");

                Pose2d start = route.startPose;
                Pose2d pose = drive.getPose();
                telemetry.addLine("Push the robot by hand. DPAD UP/DOWN: pick start, X: back to start");
                telemetry.addData("Start", "%s (%.0f, %.0f, %.0f deg)", route.name,
                        start.position.x, start.position.y, Math.toDegrees(start.heading.toDouble()));
                telemetry.addData("x (in)", "%.1f", pose.position.x);
                telemetry.addData("y (in)", "%.1f", pose.position.y);
                telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(pose.heading.toDouble()));
                telemetry.addData("moved from start (in)", "%.1f", pose.position.minus(start.position).norm());
                telemetry.addData("turned from start (deg)", "%.1f", Math.toDegrees(pose.heading.minus(start.heading)));
                telemetry.addData("FieldViewer", FieldViewerLink.get().isViewerConnected()
                        ? "connected" : "not connected");
                telemetry.update();
            }
        } finally {
            drive.setCoasting(false);
        }
    }
}
