package org.firstinspires.ftc.teamcode.purepursuitautos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.purepursuit.CurvePoint;
import org.firstinspires.ftc.teamcode.purepursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.ftc.PurePursuitDrive;

import java.util.Collections;
import java.util.List;
import java.util.Map;

/**
 * Runs one {@link Route} with pure pursuit. The routes live in the PurePursuit module
 * (purepursuit/paths) so the FieldViewer can simulate the exact same thing.
 */
public abstract class PurePursuitAutoBase extends LinearOpMode {
    /** How long to keep tracking after the route ends, so the coast after stopping is measured. */
    private static final double SETTLE_SECONDS = 1.5;

    protected abstract Route route();

    /** Mechanism Actions to run at the route's marker steps, by marker name. */
    protected Map<String, Action> markers() {
        return Collections.emptyMap();
    }

    /** Set up mechanism hardware (called during init). */
    protected void initMechanisms() {}

    /** Turn mechanisms off; always called when the OpMode ends, even if stopped early. */
    protected void stopMechanisms() {}

    @Override
    public void runOpMode() {
        Route route = route();
        PurePursuitDrive drive = PurePursuitDrive.create(hardwareMap, route.startPose);
        drive.setRouteName(route.name);
        initMechanisms();
        Action plan = drive.run(route, markers());

        while (opModeInInit()) {
            drive.updatePoseEstimate();
            drive.reportIdle("waiting for start");
            telemetry.addData("Route", route.name);
            telemetry.addLine(route.description);
            if (route.requiresHolonomic && !drive.holonomic) {
                telemetry.addLine("WARNING: this route strafes and needs a mecanum drive");
            }
            telemetry.addData("Start pose", route.startPose);
            telemetry.addData("Localizer pose", drive.getPose());
            telemetry.update();
        }
        if (isStopRequested()) {
            return;
        }

        try {
            Actions.runBlocking(plan);
            drive.stop();
            reportSettle(drive, route);
        } finally {
            drive.stop();
            stopMechanisms();
        }
    }

    /**
     * Keeps tracking for a moment after the route ends, then shows on the Driver Station how far
     * the robot coasted after its motors stopped and how far it missed the end point. These are
     * the numbers for tuning forwardSlipSeconds / decelDistance.
     */
    private void reportSettle(PurePursuitDrive drive, Route route) {
        PoseVelocity2d velocity = drive.updatePoseEstimate();
        Pose2d atStop = drive.getPose();
        double speed = velocity.linearVel.norm();

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < SETTLE_SECONDS) {
            drive.updatePoseEstimate();
            drive.reportIdle("settling");
        }
        Pose2d settled = drive.getPose();
        double coast = settled.position.minus(atStop.position).norm();
        double headingDrift = Math.toDegrees(settled.heading.minus(atStop.heading));

        List<PurePursuitPath> paths = route.paths();
        List<CurvePoint> points = paths.isEmpty()
                ? Collections.<CurvePoint>emptyList() : paths.get(paths.size() - 1).points;

        while (opModeIsActive()) {
            telemetry.addData("Route", route.name);
            telemetry.addData("Speed at stop (in/s)", "%.1f", speed);
            telemetry.addData("Coast after stop (in)", "%.2f", coast);
            telemetry.addData("Heading drift after stop (deg)", "%.1f", headingDrift);
            if (speed > 1) {
                telemetry.addData("Measured slip (s)", "%.3f", coast / speed);
            }
            telemetry.addData("Pose at stop", formatPose(atStop));
            telemetry.addData("Final pose", formatPose(settled));
            if (!points.isEmpty()) {
                CurvePoint end = points.get(points.size() - 1);
                double dx = settled.position.x - end.x;
                double dy = settled.position.y - end.y;
                telemetry.addData("Miss from end point (in)", "%.2f", Math.hypot(dx, dy));
                if (points.size() >= 2) {
                    // along the last segment: + went past the end, - stopped short
                    CurvePoint prev = points.get(points.size() - 2);
                    double length = Math.hypot(end.x - prev.x, end.y - prev.y);
                    if (length > 1e-6) {
                        double along = (dx * (end.x - prev.x) + dy * (end.y - prev.y)) / length;
                        telemetry.addData("Past end (+) / short (-) (in)", "%.2f", along);
                    }
                }
            }
            telemetry.addLine("Measure the real distance with a tape too");
            telemetry.update();
            idle();
        }
    }

    private static String formatPose(Pose2d pose) {
        return String.format("(%.1f, %.1f) %.1f deg",
                pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
    }
}
