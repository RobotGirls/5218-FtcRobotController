package org.firstinspires.ftc.teamcode.purepursuitautos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.ftc.PurePursuitDrive;

import java.util.Collections;
import java.util.Map;

/**
 * Runs one {@link Route} with pure pursuit. The routes live in the PurePursuit module
 * (purepursuit/paths) so the FieldViewer can simulate the exact same thing.
 */
public abstract class PurePursuitAutoBase extends LinearOpMode {
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
        } finally {
            drive.stop();
            stopMechanisms();
        }
    }
}
