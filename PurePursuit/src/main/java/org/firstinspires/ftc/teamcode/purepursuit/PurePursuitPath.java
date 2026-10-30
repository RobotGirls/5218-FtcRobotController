package org.firstinspires.ftc.teamcode.purepursuit;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * A list of waypoints for {@link PurePursuitFollower} plus how the robot should face while
 * driving them.
 *
 * The robot's position when the path starts is automatically used as the first point (just like
 * Gluten Free added the robot's location as the first CurvePoint), so only list where to go:
 *
 * <pre>
 * PurePursuitPath toPark = PurePursuitPath.builder()
 *         .moveSpeed(0.5)
 *         .to(34, 34)
 *         .to(34, 60)
 *         .build();
 * </pre>
 */
public final class PurePursuitPath {
    public final String name;
    public final List<CurvePoint> points;
    /**
     * Which side of the robot faces along the path, relative to the robot's front.
     * 0 drives forwards, PI drives backwards, PI/2 drives with the left side leading.
     * (Gluten Free used 90 degrees for "forwards"; this uses Road Runner's convention instead.)
     */
    public final double followAngle;
    /**
     * Mecanum only: if not null, hold this field heading (rad) the whole time instead of facing
     * along the path. Lets the robot strafe.
     */
    public final Double fixedHeading;
    /** If not null, turn to this field heading (rad) after reaching the last point. */
    public final Double endHeading;
    /** Give up on the path after this many seconds. 0 means no limit. */
    public final double timeoutSeconds;

    private PurePursuitPath(String name, List<CurvePoint> points, double followAngle,
                            Double fixedHeading, Double endHeading, double timeoutSeconds) {
        this.name = name;
        this.points = Collections.unmodifiableList(new ArrayList<>(points));
        this.followAngle = followAngle;
        this.fixedHeading = fixedHeading;
        this.endHeading = endHeading;
        this.timeoutSeconds = timeoutSeconds;
    }

    public CurvePoint last() {
        return points.get(points.size() - 1);
    }

    public static Builder builder() {
        return new Builder();
    }

    public static final class Builder {
        // defaults converted from the values Gluten Free used in their autos (50 cm lookahead,
        // 60 degree slow down at 0.6), slightly shortened for a 144 inch field
        private double moveSpeed = 0.6;
        private double turnSpeed = 0.6;
        private double followDistance = 16;
        private double pointLength = 16;
        private double slowDownTurnRadians = Math.toRadians(60);
        private double slowDownTurnAmount = 0.6;

        private String name = "path";
        private final List<CurvePoint> points = new ArrayList<>();
        private double followAngle = 0;
        private Double fixedHeading = null;
        private Double endHeading = null;
        private double timeoutSeconds = 0;

        private Builder() {}

        public Builder name(String name) {
            this.name = name;
            return this;
        }

        /** Max translational power for the points added after this call. */
        public Builder moveSpeed(double moveSpeed) {
            this.moveSpeed = moveSpeed;
            return this;
        }

        /** Max turning power for the points added after this call. */
        public Builder turnSpeed(double turnSpeed) {
            this.turnSpeed = turnSpeed;
            return this;
        }

        /** Lookahead distance (in) for the points added after this call; sets pointLength too. */
        public Builder followDistance(double followDistance) {
            this.followDistance = followDistance;
            this.pointLength = followDistance;
            return this;
        }

        /** Lookahead distance (in) used for facing, for the points added after this call. */
        public Builder pointLength(double pointLength) {
            this.pointLength = pointLength;
            return this;
        }

        public Builder slowDownTurn(double radians, double amount) {
            this.slowDownTurnRadians = radians;
            this.slowDownTurnAmount = amount;
            return this;
        }

        /** Add a waypoint using the current speed / lookahead settings. */
        public Builder to(double x, double y) {
            if (!points.isEmpty()) {
                CurvePoint prev = points.get(points.size() - 1);
                if (Math.hypot(prev.x - x, prev.y - y) < 1e-6) {
                    return this; // zero length segments break the geometry, skip duplicates
                }
            }
            points.add(new CurvePoint(x, y, moveSpeed, turnSpeed, followDistance, pointLength,
                    slowDownTurnRadians, slowDownTurnAmount));
            return this;
        }

        /** Add a fully custom waypoint. */
        public Builder to(CurvePoint point) {
            points.add(point);
            return this;
        }

        /** Drive the path backwards (robot's back leads). */
        public Builder reversed() {
            return followAngle(Math.PI);
        }

        /** See {@link PurePursuitPath#followAngle}. */
        public Builder followAngle(double radians) {
            this.followAngle = radians;
            return this;
        }

        /** Mecanum only: keep this field heading (rad) while driving. */
        public Builder fixedHeading(double radians) {
            this.fixedHeading = radians;
            return this;
        }

        /** Turn to this field heading (rad) once the last point is reached. */
        public Builder endHeading(double radians) {
            this.endHeading = radians;
            return this;
        }

        public Builder timeout(double seconds) {
            this.timeoutSeconds = seconds;
            return this;
        }

        public PurePursuitPath build() {
            if (points.isEmpty()) {
                throw new IllegalStateException("path '" + name + "' needs at least one point");
            }
            return new PurePursuitPath(name, points, followAngle, fixedHeading, endHeading,
                    timeoutSeconds);
        }
    }
}
