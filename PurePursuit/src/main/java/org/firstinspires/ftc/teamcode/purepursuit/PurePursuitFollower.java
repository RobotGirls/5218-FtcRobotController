package org.firstinspires.ftc.teamcode.purepursuit;

import static org.firstinspires.ftc.teamcode.purepursuit.PurePursuitMath.angleWrap;
import static org.firstinspires.ftc.teamcode.purepursuit.PurePursuitMath.clip;
import static org.firstinspires.ftc.teamcode.purepursuit.PurePursuitMath.distance;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Pure pursuit path follower ported from Gluten Free's MovementEssentials
 * (followCurve / gunToPosition / pointAngle), https://github.com/nwatx/Gluten-Free-Code-Reuploaded
 *
 * Road Runner does the localization: feed {@link #update} the pose and velocity from a Road
 * Runner Localizer every loop and send the returned powers to the drive's setDrivePowers().
 *
 * Differences from the original:
 * <ul>
 *   <li>inches and Road Runner's frame (robot x forward, y left, heading 0 = facing +x)</li>
 *   <li>slip is predicted from the localizer's velocity instead of the SpeedOmeter class</li>
 *   <li>the follower remembers how far along the path it is, so it never jumps backwards or
 *       skips ahead where a path doubles back on itself</li>
 *   <li>works for tank drives too: lateral power is dropped and the robot steers instead</li>
 *   <li>only decelerates on the final approach; mid-path it drives at moveSpeed towards the
 *       follow point so short lookaheads still steer firmly back onto the path</li>
 *   <li>min power goes on translation after the slow downs, so the robot can't stall just
 *       short of a point; it is never applied to a component that is exactly zero</li>
 * </ul>
 */
public final class PurePursuitFollower {
    /** Tuning shared by every path. Public non-final fields so FTC Dashboard can edit them. */
    public static class Params {
        /** Min power for the largest translation component so friction can't stall the robot. */
        public double minMovePower = 0.11;
        /** Min power for turning (fades out over the last turnSmoothRadians). */
        public double minTurnPower = 0.11;

        /** How far (in) the robot coasts per in/s of forward speed if power is cut. */
        public double forwardSlipSeconds = 0.14;
        /** How far (in) the robot coasts per in/s of strafe speed if power is cut. */
        public double strafeSlipSeconds = 0.153;
        /** How far (rad) the robot keeps turning per rad/s if power is cut. */
        public double turnSlipSeconds = 0.09;

        /** Distance (in) over which the robot slows from full speed to the final point. */
        public double decelDistance = 12;
        /** While holding position for an end heading, translation fades out over this many inches. */
        public double holdSmoothDistance = 1;
        /** Heading error (rad) at which full turn power is used. */
        public double turnDecelRadians = Math.toRadians(45);
        /** Turning fades out over the last this-many radians. */
        public double turnSmoothRadians = Math.toRadians(3);

        /** Start driving straight to the last point this many inches before follow distance. */
        public double finalApproachExtra = 6;
        /** The last segment is extended by pointLength * this so facing stays smooth at the end. */
        public double endExtensionScale = 1.5;

        /** The path is done once the robot is this close (in) to the last point. */
        public double positionTolerance = 1;
        /** ...or once it has driven past the last point and is within this many inches. */
        public double passedTolerance = 3;
        /** End heading tolerance (rad). */
        public double headingTolerance = Math.toRadians(3);
    }

    public enum Phase { FOLLOWING, TURNING, DONE }

    /** Everything computed in one update, for driving the robot and for drawing. */
    public static final class Output {
        /** Robot relative drive powers: x forward, y left, angVel counter-clockwise. */
        public final PoseVelocity2d powers;
        /** Point the robot drives towards. */
        public final Vector2d followPoint;
        /** Point the robot faces. */
        public final Vector2d pointToPoint;
        /** Closest point on the path to the robot. */
        public final Vector2d clippedPoint;
        public final double followRadius, pointRadius;
        public final int segment;
        /** Distance along the path (in) and total path length (in). */
        public final double progress, length;
        public final double distanceToEnd, headingError;
        public final Phase phase;

        Output(PoseVelocity2d powers, Vector2d followPoint, Vector2d pointToPoint,
               Vector2d clippedPoint, double followRadius, double pointRadius, int segment,
               double progress, double length, double distanceToEnd, double headingError,
               Phase phase) {
            this.powers = powers;
            this.followPoint = followPoint;
            this.pointToPoint = pointToPoint;
            this.clippedPoint = clippedPoint;
            this.followRadius = followRadius;
            this.pointRadius = pointRadius;
            this.segment = segment;
            this.progress = progress;
            this.length = length;
            this.distanceToEnd = distanceToEnd;
            this.headingError = headingError;
            this.phase = phase;
        }

        public boolean isDone() {
            return phase == Phase.DONE;
        }
    }

    private static final PoseVelocity2d STOPPED = new PoseVelocity2d(new Vector2d(0, 0), 0);

    private final Params params;
    private final boolean holonomic;
    private PurePursuitPath path;
    private final double turnOnlyHeading;

    private List<CurvePoint> points;
    private Polyline line, extended;
    private double progress = 0;
    private Phase phase = Phase.FOLLOWING;
    private Output lastOutput;

    /**
     * @param holonomic true for mecanum (can strafe), false for tank
     */
    public PurePursuitFollower(PurePursuitPath path, Params params, boolean holonomic) {
        this.path = path;
        this.params = params;
        this.holonomic = holonomic;
        this.turnOnlyHeading = Double.NaN;
    }

    private PurePursuitFollower(double heading, Params params, boolean holonomic) {
        this.path = null;
        this.params = params;
        this.holonomic = holonomic;
        this.turnOnlyHeading = heading;
    }

    /** A follower that just turns in place to a field heading (Gluten Free's pointAngle). */
    public static PurePursuitFollower turnTo(double heading, Params params, boolean holonomic) {
        return new PurePursuitFollower(heading, params, holonomic);
    }

    public Phase getPhase() {
        return phase;
    }

    public boolean isDone() {
        return phase == Phase.DONE;
    }

    /** The path actually being followed (starts at the robot), or empty before the first update. */
    public List<Vector2d> getPathPoints() {
        return line == null ? Collections.<Vector2d>emptyList() : line.vertices;
    }

    public PurePursuitPath getPath() {
        return path;
    }

    public Output getLastOutput() {
        return lastOutput;
    }

    /**
     * Runs one loop of the follower.
     *
     * @param pose     current pose from the Road Runner localizer
     * @param robotVel current robot relative velocity from the localizer (in/s, rad/s)
     */
    public Output update(Pose2d pose, PoseVelocity2d robotVel) {
        Vector2d position = pose.position;
        double heading = pose.heading.toDouble();
        if (line == null) {
            anchor(position);
        }

        Vector2d end = line.vertices.get(line.vertices.size() - 1);
        double distanceToEnd = distance(position, end);
        int segment = line.segmentAt(progress);
        CurvePoint segmentPoint = points.get(segment + 1);

        if (phase == Phase.FOLLOWING) {
            double followRadius = segmentPoint.followDistance;
            progress = Math.max(progress, line.closestS(position, progress, progress + followRadius));
            segment = line.segmentAt(progress);
            segmentPoint = points.get(segment + 1);
            followRadius = segmentPoint.followDistance;
            double pointRadius = segmentPoint.pointLength;
            Vector2d clipped = line.pointAt(progress);

            // the point to drive towards: furthest place along the path the lookahead circle hits
            double followS = line.furthestIntersectionS(position, followRadius,
                    progress, progress + 2 * followRadius);
            if (Double.isNaN(followS)) {
                // too far off the path for the circle to touch it, head for a point ahead instead
                followS = Math.min(progress + followRadius, line.length());
            }
            Vector2d follow = line.pointAt(followS);

            // the point to face, on a copy of the path with the last segment extended
            double pointS = extended.furthestIntersectionS(position, pointRadius,
                    progress, progress + 2 * pointRadius);
            if (Double.isNaN(pointS)) {
                pointS = Math.min(progress + pointRadius, extended.length());
            }
            Vector2d pointTo = extended.pointAt(pointS);

            // close to the end: drive straight at the last point. Measured along the path (not
            // straight line distance) so paths that finish where they started don't end early.
            boolean finalApproach = line.length() - progress <= followRadius + params.finalApproachExtra;
            if (finalApproach) {
                follow = end;
            }

            if (reachedEnd(position, end, distanceToEnd, finalApproach)) {
                phase = path.endHeading != null ? Phase.TURNING : Phase.DONE;
            } else {
                double desiredHeading;
                if (path.fixedHeading != null && holonomic) {
                    desiredHeading = path.fixedHeading;
                } else if (distance(position, pointTo) > 1e-6) {
                    desiredHeading = Math.atan2(pointTo.y - position.y, pointTo.x - position.x)
                            + path.followAngle;
                } else {
                    desiredHeading = heading;
                }

                // Gluten Free always decelerated towards the follow point; that was harmless with
                // their 50cm lookahead but with shorter ones it squashes the steering correction,
                // so only slow down when the follow point is the end of the path
                double[] move = gunToPosition(position, heading, robotVel, follow,
                        segmentPoint.moveSpeed, finalApproach);
                double turnError = angleWrap(desiredHeading - heading);
                double turn = pointAngle(turnError, robotVel, segmentPoint.turnSpeed);

                double forward = move[0];
                double strafe = holonomic ? move[1] : 0;

                // slow down translation while the heading is far off
                double slowDown = 1 - clip(Math.abs(turnError) / segmentPoint.slowDownTurnRadians,
                        0, segmentPoint.slowDownTurnAmount);
                forward *= slowDown;
                strafe *= slowDown;

                // keep creeping towards the target even when the decel ramp asks for less power
                // than it takes to overcome friction (otherwise the robot stalls just short)
                if (Math.abs(forward) >= Math.abs(strafe)) {
                    forward = minPower(forward, params.minMovePower);
                } else {
                    strafe = minPower(strafe, params.minMovePower);
                }

                // turning fades out over the last few degrees so it doesn't wobble
                turn = minPower(turn, params.minTurnPower)
                        * clip(Math.abs(turnError) / params.turnSmoothRadians, 0, 1);

                lastOutput = new Output(new PoseVelocity2d(new Vector2d(forward, strafe), turn),
                        follow, pointTo, clipped, followRadius, pointRadius, segment, progress,
                        line.length(), distanceToEnd, turnError, phase);
                return lastOutput;
            }
        }

        if (phase == Phase.TURNING) {
            double turnError = angleWrap(path.endHeading - heading);
            if (Math.abs(turnError) < params.headingTolerance) {
                phase = Phase.DONE;
            } else {
                // hold position on the last point while turning
                double[] move = gunToPosition(position, heading, robotVel, end,
                        segmentPoint.moveSpeed, true);
                double forward = move[0] * clip(move[2] / params.holdSmoothDistance, 0, 1);
                double strafe = holonomic ? move[1] * clip(move[3] / params.holdSmoothDistance, 0, 1) : 0;
                double turn = minPower(pointAngle(turnError, robotVel, segmentPoint.turnSpeed),
                        params.minTurnPower);
                turn *= clip(Math.abs(turnError) / params.turnSmoothRadians, 0, 1);

                lastOutput = new Output(new PoseVelocity2d(new Vector2d(forward, strafe), turn),
                        end, end, end, 0, 0, segment, line.length(), line.length(),
                        distanceToEnd, turnError, phase);
                return lastOutput;
            }
        }

        double headingError = path.endHeading != null ? angleWrap(path.endHeading - heading) : 0;
        lastOutput = new Output(STOPPED, end, end, end, 0, 0, segment, line.length(),
                line.length(), distanceToEnd, headingError, Phase.DONE);
        return lastOutput;
    }

    private boolean reachedEnd(Vector2d position, Vector2d end, double distanceToEnd,
                               boolean finalApproach) {
        if (!finalApproach) {
            return false;
        }
        if (distanceToEnd < params.positionTolerance) {
            return true;
        }
        if (distanceToEnd > params.passedTolerance) {
            return false;
        }
        // drove past the last point (e.g. a tank drive that can't fix a small sideways error)
        Vector2d before = line.vertices.get(line.vertices.size() - 2);
        double length = distance(before, end);
        if (length < 1e-6) {
            return false;
        }
        double along = ((end.x - position.x) * (end.x - before.x)
                + (end.y - position.y) * (end.y - before.y)) / length;
        return along < 0;
    }

    /**
     * Gluten Free's gunToPosition: translation powers towards a target, aiming short of it by
     * however far the robot would slip if it cut power right now.
     *
     * @param stop slow down over decelDistance to stop on the target
     * @return {forward power, left power, |forward distance|, |left distance|}
     */
    private double[] gunToPosition(Vector2d position, double heading, PoseVelocity2d robotVel,
                                   Vector2d target, double moveSpeed, boolean stop) {
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double slipForward = robotVel.linearVel.x * params.forwardSlipSeconds;
        double slipLeft = robotVel.linearVel.y * params.strafeSlipSeconds;
        double slipX = slipForward * cos - slipLeft * sin;
        double slipY = slipForward * sin + slipLeft * cos;

        double dx = target.x - slipX - position.x;
        double dy = target.y - slipY - position.y;
        double relForward = dx * cos + dy * sin;
        double relLeft = -dx * sin + dy * cos;
        double absForward = Math.abs(relForward);
        double absLeft = Math.abs(relLeft);

        // keep the direction but make the components add up to 1...
        double total = absForward + absLeft;
        double forward = total < 1e-9 ? 0 : relForward / total;
        double left = total < 1e-9 ? 0 : relLeft / total;

        // ...then (if stopping) slow down over decelDistance, and cap at the requested speed
        if (stop) {
            forward *= absForward / params.decelDistance;
            left *= absLeft / params.decelDistance;
        } else {
            forward *= moveSpeed;
            left *= moveSpeed;
        }
        forward = clip(forward, -moveSpeed, moveSpeed);
        left = clip(left, -moveSpeed, moveSpeed);

        return new double[] {forward, left, absForward, absLeft};
    }

    /** Gluten Free's pointAngle: turn power for a heading error, accounting for turn slip. */
    private double pointAngle(double turnError, PoseVelocity2d robotVel, double turnSpeed) {
        double slipAdjusted = angleWrap(turnError - robotVel.angVel * params.turnSlipSeconds);
        return clip(slipAdjusted / params.turnDecelRadians * turnSpeed, -turnSpeed, turnSpeed);
    }

    private static double minPower(double value, double min) {
        if (Math.abs(value) < 1e-9) {
            return 0;
        }
        return Math.copySign(Math.max(Math.abs(value), min), value);
    }

    /** Builds the real path to follow, starting from wherever the robot is right now. */
    private void anchor(Vector2d start) {
        if (path == null) {
            path = PurePursuitPath.builder()
                    .name("turn")
                    .to(start.x, start.y)
                    .endHeading(turnOnlyHeading)
                    .build();
        }

        points = new ArrayList<>();
        points.add(path.points.get(0).withPosition(start.x, start.y));
        for (CurvePoint p : path.points) {
            CurvePoint prev = points.get(points.size() - 1);
            if (Math.hypot(p.x - prev.x, p.y - prev.y) > 0.25) {
                points.add(p);
            }
        }
        if (points.size() == 1) {
            // already sitting on the only point, keep a zero length segment so the math still works
            points.add(path.last());
        }

        List<Vector2d> vertices = new ArrayList<>();
        for (CurvePoint p : points) {
            vertices.add(new Vector2d(p.x, p.y));
        }
        line = new Polyline(vertices);

        // extend the last segment so the robot keeps facing along it as it arrives
        List<Vector2d> extendedVertices = new ArrayList<>(vertices);
        Vector2d end = vertices.get(vertices.size() - 1);
        Vector2d before = vertices.get(vertices.size() - 2);
        double lastLength = distance(before, end);
        if (lastLength > 1e-6) {
            double extension = points.get(points.size() - 1).pointLength * params.endExtensionScale;
            extendedVertices.set(extendedVertices.size() - 1, new Vector2d(
                    end.x + (end.x - before.x) / lastLength * extension,
                    end.y + (end.y - before.y) / lastLength * extension));
        }
        extended = new Polyline(extendedVertices);
    }
}
