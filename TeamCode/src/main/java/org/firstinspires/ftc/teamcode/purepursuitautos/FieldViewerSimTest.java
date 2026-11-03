package org.firstinspires.ftc.teamcode.purepursuitautos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.purepursuit.FieldViewerLink;
import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.RouteSimulation;
import org.firstinspires.ftc.teamcode.purepursuit.ViewerFrame;
import org.firstinspires.ftc.teamcode.purepursuit.paths.TestRoutes;

/**
 * Checks the FieldViewer connection without moving the robot: the Control Hub simulates a route
 * and streams the simulated robot to the FieldViewer (Live mode), over and over. No drive
 * hardware is used, so it's safe on a table. The Driver Station shows which side of the link
 * is failing.
 */
@TeleOp(name = "PP FieldViewer Sim Test", group = "Pure Pursuit")
public class FieldViewerSimTest extends LinearOpMode {
    private static final Route ROUTE = TestRoutes.STRAIGHT_24;
    /** Pause at the end of the route before it starts over. */
    private static final double RESTART_SECONDS = 2;

    private FieldViewerLink link;
    private long framesSent;

    @Override
    public void runOpMode() {
        link = FieldViewerLink.get();
        RouteSimulation sim = new RouteSimulation(ROUTE, false);

        while (opModeInInit()) {
            publish(sim, "SIM waiting for start");
            report(sim);
            sleep(20);
        }

        long last = System.nanoTime();
        double finishedFor = 0;
        while (opModeIsActive()) {
            long now = System.nanoTime();
            double dt = (now - last) / 1e9;
            last = now;

            if (sim.isFinished()) {
                finishedFor += dt;
                if (finishedFor > RESTART_SECONDS) {
                    sim = new RouteSimulation(ROUTE, false);
                    finishedFor = 0;
                }
            } else {
                sim.advance(dt);
            }
            publish(sim, "SIM " + sim.getStatus());
            report(sim);
            sleep(20);
        }
    }

    private void publish(RouteSimulation sim, String status) {
        ViewerFrame frame = sim.frame();
        frame.time = System.nanoTime() / 1e9;
        frame.status = status;
        if (link.isViewerConnected()) {
            framesSent++;
        }
        link.publish(frame);
    }

    private void report(RouteSimulation sim) {
        String viewer = link.getViewerAddress();
        if (viewer == null) {
            telemetry.addData("FieldViewer", "NO HELLO RECEIVED YET");
            telemetry.addLine("The laptop's messages aren't reaching the robot: check the laptop "
                    + "is on this robot's wifi, FieldViewer is in Live mode, and its host is 192.168.43.1");
        } else if (link.isViewerConnected()) {
            telemetry.addData("FieldViewer", "connected, sending");
            telemetry.addLine("If FieldViewer still shows nothing, the laptop's firewall is "
                    + "blocking the replies");
        } else {
            telemetry.addData("FieldViewer", "lost (no hello for 3 s)");
        }
        telemetry.addData("Laptop address", viewer == null ? "-" : viewer);
        telemetry.addData("Frames sent", framesSent);
        String error = link.getError();
        telemetry.addData("Network error", error == null ? "none" : error);
        telemetry.addData("Route", ROUTE.name);
        telemetry.addData("Sim", "%.1f s  %s", sim.getTime(), sim.getStatus());
        telemetry.addLine("Simulation only: the robot will not move");
        telemetry.update();
    }
}
