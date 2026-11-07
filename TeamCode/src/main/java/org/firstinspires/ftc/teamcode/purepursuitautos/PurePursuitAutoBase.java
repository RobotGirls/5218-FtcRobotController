package org.firstinspires.ftc.teamcode.purepursuitautos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.purepursuit.MechanismBindings;
import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.ftc.PurePursuitConfig;
import org.firstinspires.ftc.teamcode.purepursuit.ftc.PurePursuitDrive;

import java.util.Collections;
import java.util.List;
import java.util.Map;

/**
 * Runs one {@link Route} with pure pursuit. The routes live in the PurePursuit module
 * (purepursuit/paths) so the FieldViewer can simulate the exact same thing.
 */
public abstract class PurePursuitAutoBase extends LinearOpMode {
    protected abstract Route route();

    /**
     * Which mechanism runs at each of the route's markers, e.g.
     * {@code new MechanismBindings<Action>().register("shoot", launcher::launcherForward)}.
     * Called after {@link #initMechanisms()}. Return null to use {@link #markers()} instead.
     */
    protected MechanismBindings<Action> mechanisms() {
        return null;
    }

    /** Older form of {@link #mechanisms()}: one Action object per marker name. */
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
        MechanismBindings<Action> bindings = mechanisms();
        Action plan = bindings != null ? drive.run(route, bindings) : drive.run(route, markers());
        List<String> unbound = bindings != null ? bindings.unbound(route) : Collections.<String>emptyList();
        for (String name : unbound) {
            RobotLog.ww("PurePursuitAuto", "%s: no mechanism registered for marker '%s'", route.name, name);
        }

        while (opModeInInit()) {
            drive.updatePoseEstimate();
            drive.reportIdle("waiting for start");
            telemetry.addData("Route", route.name);
            telemetry.addLine(route.description);
            if (route.requiresHolonomic && !drive.holonomic) {
                telemetry.addLine("WARNING: this route strafes and needs a mecanum drive");
            }
            for (String name : unbound) {
                telemetry.addLine("WARNING: nothing registered for marker '" + name + "', it will just wait");
            }
            telemetry.addData("Tuning", PurePursuitConfig.tuningSource());
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
