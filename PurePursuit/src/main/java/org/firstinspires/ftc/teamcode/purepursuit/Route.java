package org.firstinspires.ftc.teamcode.purepursuit;

import com.acmerobotics.roadrunner.Pose2d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * A whole autonomous drive plan: where the robot starts and the steps it runs in order.
 *
 * The same Route object is run on the robot (as Road Runner Actions) and simulated in the
 * FieldViewer, so what you see on the laptop is exactly what the robot will try to do.
 * Marker steps are where mechanisms run: the OpMode maps the marker's name to an Action, and
 * the simulator just waits for the marker's estimated duration.
 */
public final class Route {
    public enum Alliance { RED, BLUE, NONE }

    public abstract static class Step {
        private Step() {}
    }

    /** Drive a pure pursuit path. */
    public static final class Follow extends Step {
        public final PurePursuitPath path;

        Follow(PurePursuitPath path) {
            this.path = path;
        }
    }

    /** Stop and run a mechanism (or just wait) for about this long. */
    public static final class Marker extends Step {
        public final String name;
        public final double seconds;

        Marker(String name, double seconds) {
            this.name = name;
            this.seconds = seconds;
        }
    }

    /** Turn in place to a field heading. */
    public static final class Turn extends Step {
        public final double heading;

        Turn(double heading) {
            this.heading = heading;
        }
    }

    public final String name;
    public final String description;
    public final Alliance alliance;
    public final Pose2d startPose;
    public final List<Step> steps;
    /** True if a path uses fixedHeading (strafing), which a tank drive can't do. */
    public final boolean requiresHolonomic;

    private Route(String name, String description, Alliance alliance, Pose2d startPose,
                  List<Step> steps) {
        this.name = name;
        this.description = description;
        this.alliance = alliance;
        this.startPose = startPose;
        this.steps = Collections.unmodifiableList(new ArrayList<>(steps));

        boolean holonomic = false;
        for (Step step : steps) {
            if (step instanceof Follow && ((Follow) step).path.fixedHeading != null) {
                holonomic = true;
            }
        }
        this.requiresHolonomic = holonomic;
    }

    public List<PurePursuitPath> paths() {
        List<PurePursuitPath> paths = new ArrayList<>();
        for (Step step : steps) {
            if (step instanceof Follow) {
                paths.add(((Follow) step).path);
            }
        }
        return paths;
    }

    @Override
    public String toString() {
        return name;
    }

    public static Builder builder(String name, Alliance alliance, Pose2d startPose) {
        return new Builder(name, alliance, startPose);
    }

    public static final class Builder {
        private final String name;
        private final Alliance alliance;
        private final Pose2d startPose;
        private String description = "";
        private final List<Step> steps = new ArrayList<>();

        private Builder(String name, Alliance alliance, Pose2d startPose) {
            this.name = name;
            this.alliance = alliance;
            this.startPose = startPose;
        }

        public Builder description(String description) {
            this.description = description;
            return this;
        }

        public Builder follow(PurePursuitPath path) {
            steps.add(new Follow(path));
            return this;
        }

        /** A named stop where the OpMode can run a mechanism Action. */
        public Builder marker(String name, double estimatedSeconds) {
            steps.add(new Marker(name, estimatedSeconds));
            return this;
        }

        public Builder waitSeconds(double seconds) {
            return marker("wait", seconds);
        }

        public Builder turnTo(double heading) {
            steps.add(new Turn(heading));
            return this;
        }

        public Route build() {
            return new Route(name, description, alliance, startPose, steps);
        }
    }
}
