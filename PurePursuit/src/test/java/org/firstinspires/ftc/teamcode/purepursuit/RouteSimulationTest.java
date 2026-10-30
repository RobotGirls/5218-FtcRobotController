package org.firstinspires.ftc.teamcode.purepursuit;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.purepursuit.paths.BioBuzzRoutes;
import org.firstinspires.ftc.teamcode.purepursuit.paths.Routes;
import org.firstinspires.ftc.teamcode.purepursuit.paths.TestRoutes;
import org.junit.Test;

/** Drives every route in the simulator, the same way the FieldViewer does. */
public class RouteSimulationTest {
    private static final double MAX_SECONDS = 60;

    @Test
    public void everyRouteFinishesOnMecanum() {
        for (Route route : Routes.ALL) {
            checkRoute(route, true);
        }
    }

    @Test
    public void everyTankRouteFinishesOnTank() {
        for (Route route : Routes.ALL) {
            if (!route.requiresHolonomic) {
                checkRoute(route, false);
            }
        }
    }

    @Test
    public void testRoutesStayClearOfTheCenterStructure() {
        for (Route route : Routes.ALL) {
            if (!route.name.startsWith("TEST")) {
                continue;
            }
            for (boolean holonomic : new boolean[] {true, false}) {
                if (route.requiresHolonomic && !holonomic) {
                    continue;
                }
                RouteSimulation sim = new RouteSimulation(route, holonomic);
                sim.runToEnd(MAX_SECONDS);
                assertEquals(route.name + " hit the center structure", 0, sim.getCollisionSeconds(), 0);
            }
        }
    }

    @Test
    public void reversedLegsKeepTheRobotFacingTheSameWay() {
        // the Road Runner version backed up without turning around; pure pursuit should too
        RouteSimulation sim = new RouteSimulation(BioBuzzRoutes.BLUE_CORNER_PARK, false);
        double maxHeadingChange = 0;
        while (!sim.isFinished()) {
            sim.loop();
            maxHeadingChange = Math.max(maxHeadingChange,
                    Math.abs(PurePursuitMath.angleWrap(sim.getPose().heading.toDouble())));
        }
        assertTrue("turned " + Math.toDegrees(maxHeadingChange) + " deg",
                maxHeadingChange < Math.toRadians(5));
    }

    @Test
    public void endHeadingIsReached() {
        for (boolean holonomic : new boolean[] {true, false}) {
            RouteSimulation sim = new RouteSimulation(TestRoutes.FIELD_LOOP, holonomic);
            sim.runToEnd(MAX_SECONDS);
            assertEquals(0, PurePursuitMath.angleWrap(sim.getPose().heading.toDouble()), Math.toRadians(6));

            sim = new RouteSimulation(TestRoutes.TURN_IN_PLACE, holonomic);
            sim.runToEnd(MAX_SECONDS);
            assertEquals(0, PurePursuitMath.angleWrap(sim.getPose().heading.toDouble()), Math.toRadians(6));
            assertEquals("should turn in place", 0,
                    PurePursuitMath.distance(sim.getPose().position, new Vector2d(-48, 48)), 2);
        }
    }

    @Test
    public void strafeSquareKeepsItsHeading() {
        RouteSimulation sim = new RouteSimulation(TestRoutes.STRAFE_SQUARE, true);
        double worst = 0;
        while (!sim.isFinished()) {
            sim.loop();
            worst = Math.max(worst, Math.abs(PurePursuitMath.angleWrap(sim.getPose().heading.toDouble())));
        }
        assertTrue("heading drifted " + Math.toDegrees(worst) + " deg", worst < Math.toRadians(5));
    }

    @Test
    public void markerStepWaitsItsTime() {
        RouteSimulation withShooter = new RouteSimulation(BioBuzzRoutes.BLUE_GARDEN_SHOOTER, false);
        withShooter.runToEnd(MAX_SECONDS);
        RouteSimulation without = new RouteSimulation(BioBuzzRoutes.BLUE_GARDEN_PARK, false);
        without.runToEnd(MAX_SECONDS);
        assertEquals(3.5, withShooter.getRouteDoneTime() - without.getRouteDoneTime(), 0.1);
    }

    private static void checkRoute(Route route, boolean holonomic) {
        String name = route.name + (holonomic ? " (mecanum)" : " (tank)");
        RouteSimulation sim = new RouteSimulation(route, holonomic);
        boolean leftField = false;
        double driven = 0;
        Pose2d last = sim.getPose();
        while (!sim.isFinished() && sim.getTime() < MAX_SECONDS) {
            sim.loop();
            Pose2d pose = sim.getPose();
            leftField |= Math.abs(pose.position.x) > BioBuzzField.HALF_FIELD
                    || Math.abs(pose.position.y) > BioBuzzField.HALF_FIELD;
            driven += PurePursuitMath.distance(pose.position, last.position);
            last = pose;
        }

        assertTrue(name + " didn't finish: " + sim.getStatus(), sim.isFinished());
        // finishing isn't enough, it has to actually drive the route (pure pursuit cuts corners
        // so it's allowed to be shorter than the waypoints, e.g. ~5in per 90 degree corner with
        // an 8in lookahead)
        double planned = plannedLength(route);
        assertTrue(name + " only drove " + driven + " of " + planned + " in", driven >= 0.75 * planned);
        assertTrue(name + " timed out on " + sim.getTimedOutSteps(), sim.getTimedOutSteps().isEmpty());
        assertFalse(name + " left the field", leftField);
        assertTrue(name + " strayed " + sim.getMaxCrossTrackError() + " in from its path",
                sim.getMaxCrossTrackError() < 12);

        PurePursuitPath lastPath = null;
        for (PurePursuitPath path : route.paths()) {
            lastPath = path;
        }
        if (lastPath != null) {
            CurvePoint end = lastPath.last();
            double miss = PurePursuitMath.distance(sim.getPose().position, new Vector2d(end.x, end.y));
            // the follower stops within tolerance; allow for a little coasting afterwards
            assertTrue(name + " stopped " + miss + " in from the end", miss < 3.5);
        }
    }

    private static double plannedLength(Route route) {
        double total = 0;
        Vector2d at = route.startPose.position;
        for (PurePursuitPath path : route.paths()) {
            for (CurvePoint p : path.points) {
                Vector2d next = new Vector2d(p.x, p.y);
                total += PurePursuitMath.distance(at, next);
                at = next;
            }
        }
        return total;
    }
}
