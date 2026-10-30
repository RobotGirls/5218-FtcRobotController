package org.firstinspires.ftc.teamcode.purepursuit;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * Runs a {@link Route} against {@link DriveSimulator} using the exact same
 * {@link PurePursuitFollower} code the robot runs. Used by the FieldViewer and the unit tests.
 */
public final class RouteSimulation {
    /** How often the follower runs, like one loop of an OpMode. */
    public static final double LOOP_SECONDS = 0.02;
    /** Paths without their own timeout give up after this long so a bad path can't hang forever. */
    public static final double DEFAULT_STEP_TIMEOUT = 30;

    private static final int PHYSICS_SUBSTEPS = 4;
    private static final PoseVelocity2d STOPPED = new PoseVelocity2d(new Vector2d(0, 0), 0);

    public final Route route;
    public final boolean holonomic;
    private final PurePursuitFollower.Params followerParams;
    private final DriveSimulator drive;

    private int stepIndex = 0;
    private double time = 0, stepTime = 0, leftover = 0;
    private PurePursuitFollower follower;
    private PurePursuitFollower.Output output;
    private PoseVelocity2d powers = STOPPED;
    private final List<Pose2d> trail = new ArrayList<>();
    private final List<String> timedOutSteps = new ArrayList<>();
    private double collisionSeconds = 0;
    private double maxCrossTrackError = 0;
    private double routeDoneTime = -1;
    private boolean stopped = false;

    public RouteSimulation(Route route, boolean holonomic, PurePursuitFollower.Params followerParams,
                           DriveSimulator.Params driveParams) {
        this.route = route;
        this.holonomic = holonomic;
        this.followerParams = followerParams;
        this.drive = new DriveSimulator(route.startPose, holonomic, driveParams);
        trail.add(route.startPose);
    }

    public RouteSimulation(Route route, boolean holonomic) {
        this(route, holonomic, new PurePursuitFollower.Params(), new DriveSimulator.Params());
    }

    /** True once every step has run and the robot has coasted to a stop. */
    public boolean isFinished() {
        return isRouteDone() && stopped;
    }

    /** True once every step has run (the robot may still be coasting). */
    public boolean isRouteDone() {
        return stepIndex >= route.steps.size();
    }

    /** When the last step finished, like when the OpMode's actions would end, or -1. */
    public double getRouteDoneTime() {
        return routeDoneTime;
    }

    public double getTime() {
        return time;
    }

    public Pose2d getPose() {
        return drive.getPose();
    }

    public PoseVelocity2d getVelocity() {
        return drive.getVelocity();
    }

    public List<Pose2d> getTrail() {
        return Collections.unmodifiableList(trail);
    }

    /** Names of follow steps that hit their timeout instead of finishing. */
    public List<String> getTimedOutSteps() {
        return Collections.unmodifiableList(timedOutSteps);
    }

    /** Seconds the robot footprint spent overlapping the central structure. */
    public double getCollisionSeconds() {
        return collisionSeconds;
    }

    /** Largest distance (in) seen between the robot and the closest point on its path. */
    public double getMaxCrossTrackError() {
        return maxCrossTrackError;
    }

    public int getStepIndex() {
        return stepIndex;
    }

    /** Runs simulated time forward, in whole robot loops. */
    public void advance(double seconds) {
        leftover += seconds;
        while (leftover >= LOOP_SECONDS && !isFinished()) {
            loop();
            leftover -= LOOP_SECONDS;
        }
        if (isFinished()) {
            leftover = 0;
        }
    }

    /** Runs until the route is done or maxSeconds have gone by. */
    public void runToEnd(double maxSeconds) {
        while (!isFinished() && time < maxSeconds) {
            loop();
        }
    }

    /** One robot loop: run the current step, then move the simulated robot. */
    public void loop() {
        if (isFinished()) {
            return;
        }
        powers = STOPPED;

        if (isRouteDone()) {
            // motors are off, let the robot coast to a stop so we see where it really ends up
            PoseVelocity2d v = drive.getVelocity();
            stopped = v.linearVel.norm() < 0.2 && Math.abs(v.angVel) < 0.01;
        } else if (route.steps.get(stepIndex) instanceof Route.Marker) {
            Route.Step step = route.steps.get(stepIndex);
            output = null;
            if (stepTime >= ((Route.Marker) step).seconds) {
                nextStep();
            }
        } else {
            Route.Step step = route.steps.get(stepIndex);
            if (follower == null) {
                follower = step instanceof Route.Follow
                        ? new PurePursuitFollower(((Route.Follow) step).path, followerParams, holonomic)
                        : PurePursuitFollower.turnTo(((Route.Turn) step).heading, followerParams, holonomic);
            }
            output = follower.update(drive.getPose(), drive.getVelocity());
            if (output.phase == PurePursuitFollower.Phase.FOLLOWING) {
                maxCrossTrackError = Math.max(maxCrossTrackError,
                        PurePursuitMath.distance(drive.getPose().position, output.clippedPoint));
            }

            double timeout = step instanceof Route.Follow && ((Route.Follow) step).path.timeoutSeconds > 0
                    ? ((Route.Follow) step).path.timeoutSeconds : DEFAULT_STEP_TIMEOUT;
            if (output.isDone()) {
                nextStep();
            } else if (stepTime >= timeout) {
                timedOutSteps.add(stepName(step));
                nextStep();
            } else {
                powers = output.powers;
            }
        }

        double dt = LOOP_SECONDS / PHYSICS_SUBSTEPS;
        for (int i = 0; i < PHYSICS_SUBSTEPS; i++) {
            drive.step(powers, dt);
        }
        time += LOOP_SECONDS;
        stepTime += LOOP_SECONDS;
        trail.add(drive.getPose());
        if (BioBuzzField.hitsCenterStructure(drive.getPose(),
                BioBuzzField.ROBOT_LENGTH, BioBuzzField.ROBOT_WIDTH)) {
            collisionSeconds += LOOP_SECONDS;
        }
    }

    private void nextStep() {
        stepIndex++;
        if (isRouteDone()) {
            routeDoneTime = time;
        }
        stepTime = 0;
        follower = null;
        output = null;
    }

    private static String stepName(Route.Step step) {
        if (step instanceof Route.Follow) {
            return ((Route.Follow) step).path.name;
        } else if (step instanceof Route.Turn) {
            return String.format(Locale.US, "turn to %.0f deg", Math.toDegrees(((Route.Turn) step).heading));
        }
        return ((Route.Marker) step).name;
    }

    /** Human readable description of what's happening right now. */
    public String getStatus() {
        if (isRouteDone()) {
            return String.format(Locale.US, "Done in %.2f s%s", routeDoneTime,
                    timedOutSteps.isEmpty() ? "" : " (timed out: " + timedOutSteps + ")");
        }
        Route.Step step = route.steps.get(stepIndex);
        String prefix = String.format(Locale.US, "Step %d/%d: ", stepIndex + 1, route.steps.size());
        if (step instanceof Route.Marker) {
            Route.Marker marker = (Route.Marker) step;
            return prefix + String.format(Locale.US, "%s %.1f / %.1f s", marker.name, stepTime, marker.seconds);
        }
        String phase = output == null ? "" : " (" + output.phase.name().toLowerCase(Locale.US) + ")";
        return prefix + stepName(step) + phase;
    }

    /** The same kind of frame the robot sends, so the viewer draws sim and live the same way. */
    public ViewerFrame frame() {
        ViewerFrame frame = new ViewerFrame();
        frame.time = time;
        frame.routeName = route.name;
        frame.status = getStatus();
        frame.pose = drive.getPose();
        frame.powers = powers;
        return frame.withFollower(follower, output);
    }
}
