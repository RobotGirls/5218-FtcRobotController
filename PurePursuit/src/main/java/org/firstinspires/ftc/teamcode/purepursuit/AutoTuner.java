package org.firstinspires.ftc.teamcode.purepursuit;

import static org.firstinspires.ftc.teamcode.purepursuit.PurePursuitMath.angleWrap;
import static org.firstinspires.ftc.teamcode.purepursuit.PurePursuitMath.clip;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Locale;

/**
 * Measures the robot and tunes {@link PurePursuitFollower.Params} to match it, in two parts:
 *
 * <ol>
 *   <li>Measure: short open-loop bursts forward, sideways and spinning find static friction,
 *       top speed, response lag and how far the robot coasts once power is cut. Each burst is
 *       sized to the free space ahead so the robot stays on the field and off the center
 *       structure.</li>
 *   <li>Refine: drive the tuning course with pure pursuit, compare where the robot really
 *       stops with the coded end points and headings, and correct the slip and min power
 *       values until they match (up to {@link #MAX_ROUNDS} rounds).</li>
 * </ol>
 *
 * Call {@link #update} once per loop with the localizer's pose and velocity and send the powers
 * it returns to the drive. The same code runs on the robot (PP Auto Tuner) and in the unit tests
 * against {@link DriveSimulator}.
 */
public final class AutoTuner {
    /** Refinement rounds over the course before giving up. */
    public static final int MAX_ROUNDS = 4;
    /** A course leg matches once it stops this close (in / rad), or the follower tolerance if larger. */
    public static final double LINEAR_MATCH = 0.75, TURN_MATCH = Math.toRadians(2);

    private static final double BURST_POWER = 0.7, TURN_BURST_POWER = 0.5;
    private static final double LONGEST_BURST = 40, LONGEST_LEG = 48;
    /** Space kept free past the end of a burst for coasting, plus a safety margin. */
    private static final double COAST_ROOM = 16, WALL_MARGIN = 3;

    private enum Axis { FORWARD, STRAFE, TURN }

    /** Measured robot numbers, also what FieldViewer's simulator needs to match the robot. */
    public final DriveSimulator.Params measuredDrive = new DriveSimulator.Params();
    private final PurePursuitFollower.Params follower;
    private final boolean holonomic;
    private final Pose2d start;
    private final double robotLength, robotWidth;

    private final Deque<Task> tasks = new ArrayDeque<>();
    private final List<String> log = new ArrayList<>();
    private final List<Double> frictions = new ArrayList<>(), turnFrictions = new ArrayList<>();
    private final List<double[]> forwardBursts = new ArrayList<>(), strafeBursts = new ArrayList<>(),
            turnBursts = new ArrayList<>();
    private Task task;
    private String status = "starting";
    private boolean aborted = false;
    private int round = 0;
    private double legLength;
    private final List<double[]> roundErrors = new ArrayList<>();

    /**
     * @param start    where the robot starts; it comes back here between tests
     * @param follower the tuning to update in place (PurePursuitConfig.FOLLOWER on the robot)
     */
    public AutoTuner(Pose2d start, PurePursuitFollower.Params follower, boolean holonomic,
                     double robotLength, double robotWidth) {
        this.start = start;
        this.follower = follower;
        this.holonomic = holonomic;
        this.robotLength = robotLength;
        this.robotWidth = robotWidth;

        measure(new FrictionRamp(Axis.FORWARD, 1));
        measure(new FrictionRamp(Axis.FORWARD, -1));
        measure(new Burst(Axis.FORWARD, 1));
        measure(new Burst(Axis.FORWARD, -1));
        tasks.add(new GoTo(start.position, start.heading.toDouble()));
        if (holonomic) {
            // strafe along the field the same way the forward bursts went, facing 90 degrees off
            tasks.add(new Turn(Math.PI / 2));
            measure(new Burst(Axis.STRAFE, 0));
            measure(new Burst(Axis.STRAFE, 0));
            tasks.add(new GoTo(start.position, start.heading.toDouble() + Math.PI / 2));
            tasks.add(new Turn(-Math.PI / 2));
        }
        measure(new FrictionRamp(Axis.TURN, 1));
        measure(new FrictionRamp(Axis.TURN, -1));
        measure(new Burst(Axis.TURN, 1));
        measure(new Burst(Axis.TURN, -1));
        tasks.add(new Apply());
        tasks.add(new GoTo(start.position, start.heading.toDouble()));
        tasks.add(new Round());
    }

    public AutoTuner(Pose2d start, PurePursuitFollower.Params follower, boolean holonomic) {
        this(start, follower, holonomic, BioBuzzField.ROBOT_LENGTH, BioBuzzField.ROBOT_WIDTH);
    }

    private void measure(Task measurement) {
        tasks.add(new WaitStill());
        tasks.add(measurement);
    }

    /** How close (in) a course leg has to stop; the follower itself only aims this close. */
    private double linearMatch() {
        return Math.max(LINEAR_MATCH, follower.positionTolerance);
    }

    private double turnMatch() {
        return Math.max(TURN_MATCH, follower.headingTolerance + Math.toRadians(1));
    }

    public boolean isDone() {
        return task == null && tasks.isEmpty() || aborted;
    }

    public boolean isAborted() {
        return aborted;
    }

    public String getStatus() {
        return status;
    }

    /** What was measured and changed, one line per entry, for telemetry and the log. */
    public List<String> getLog() {
        return log;
    }

    /** Worst linear (in) and turn (rad) miss of each refinement round. */
    public List<double[]> getRoundErrors() {
        return roundErrors;
    }

    /** True if the last refinement round matched every coded end point within tolerance. */
    public boolean isMatched() {
        if (roundErrors.isEmpty()) {
            return false;
        }
        double[] last = roundErrors.get(roundErrors.size() - 1);
        return last[0] < linearMatch() && last[1] < turnMatch();
    }

    /** The tuned values as Java, to paste into PurePursuitConfig / DriveSimulator.Params. */
    public List<String> javaLines() {
        List<String> lines = new ArrayList<>();
        lines.add(String.format(Locale.US, "FOLLOWER.forwardSlipSeconds = %.3f;", follower.forwardSlipSeconds));
        lines.add(String.format(Locale.US, "FOLLOWER.strafeSlipSeconds = %.3f;", follower.strafeSlipSeconds));
        lines.add(String.format(Locale.US, "FOLLOWER.turnSlipSeconds = %.3f;", follower.turnSlipSeconds));
        lines.add(String.format(Locale.US, "FOLLOWER.minMovePower = %.3f;", follower.minMovePower));
        lines.add(String.format(Locale.US, "FOLLOWER.minTurnPower = %.3f;", follower.minTurnPower));
        lines.add(String.format(Locale.US, "// FieldViewer drive: top speed %.0f in/s, strafe %.0f in/s, turn %.0f deg/s,"
                        + " response lag %.2f s, static friction %.2f",
                measuredDrive.maxForwardVel, measuredDrive.maxStrafeVel,
                Math.toDegrees(measuredDrive.maxAngVel), measuredDrive.timeConstant,
                measuredDrive.staticFrictionPower));
        return lines;
    }

    /** Stop and give up, e.g. when the OpMode is stopped early. */
    public void abort(String why) {
        if (!aborted) {
            aborted = true;
            status = "stopped: " + why;
            log.add(status);
        }
    }

    /**
     * One loop.
     *
     * @param pose     localizer pose
     * @param robotVel localizer robot relative velocity (in/s, rad/s)
     * @param dt       seconds since the last call
     * @return robot relative drive powers
     */
    public PoseVelocity2d update(Pose2d pose, PoseVelocity2d robotVel, double dt) {
        if (aborted) {
            return stopped();
        }
        if (BioBuzzField.outsideField(pose, robotLength - 2, robotWidth - 2)
                || BioBuzzField.hitsCenterStructure(pose, robotLength - 2, robotWidth - 2)) {
            abort(String.format(Locale.US, "robot at (%.0f, %.0f) is off the safe area", pose.position.x, pose.position.y));
            return stopped();
        }
        while (true) {
            if (task == null) {
                task = tasks.poll();
                if (task == null) {
                    status = isMatched() ? "done: robot matches the coded paths" : "done";
                    return stopped();
                }
            }
            task.time += dt;
            PoseVelocity2d powers = task.update(pose, robotVel, dt);
            if (aborted) {
                return stopped();
            }
            if (powers != null) {
                return powers;
            }
            task = null;
        }
    }

    // ---- tasks -------------------------------------------------------------------------------

    private abstract static class Task {
        double time;

        /** Powers for this loop, or null once finished. */
        abstract PoseVelocity2d update(Pose2d pose, PoseVelocity2d vel, double dt);
    }

    /** Waits for the robot to stop moving so a measurement doesn't start while it's still rolling. */
    private final class WaitStill extends Task {
        private double stillFor;

        @Override
        PoseVelocity2d update(Pose2d pose, PoseVelocity2d vel, double dt) {
            status = "waiting for the robot to stop";
            boolean still = Math.abs(vel.angVel) < Math.toRadians(1) && Math.hypot(vel.linearVel.x, vel.linearVel.y) < 0.3;
            stillFor = still ? stillFor + dt : 0;
            return stillFor >= 0.3 || time > 2 ? null : stopped();
        }
    }

    /** Ramps power up slowly until the robot starts to move: that power is static friction. */
    private final class FrictionRamp extends Task {
        private final Axis axis;
        private final double sign;
        private Pose2d from;
        private double moved = -1, settle;

        FrictionRamp(Axis axis, double sign) {
            this.axis = axis;
            this.sign = sign;
        }

        @Override
        PoseVelocity2d update(Pose2d pose, PoseVelocity2d vel, double dt) {
            if (from == null) {
                from = pose;
            }
            if (moved >= 0) {
                settle += dt;
                return settle < 0.6 ? stopped() : null;
            }
            double power = Math.min(0.06 * time, 0.4);
            status = String.format(Locale.US, "static friction (%s): power %.2f", axis.name().toLowerCase(Locale.US), power);
            boolean started = axis == Axis.TURN ? Math.abs(vel.angVel) > Math.toRadians(2)
                    : Math.hypot(vel.linearVel.x, vel.linearVel.y) > 0.75;
            double travelled = axis == Axis.TURN
                    ? Math.abs(angleWrap(pose.heading.toDouble() - from.heading.toDouble()))
                    : PurePursuitMath.distance(pose.position, from.position);
            if (started || power >= 0.4 || travelled > (axis == Axis.TURN ? Math.toRadians(30) : 4)) {
                moved = power;
                (axis == Axis.TURN ? turnFrictions : frictions).add(power);
                log.add(String.format(Locale.US, "%s starts moving at power %.2f",
                        axis == Axis.TURN ? "turning" : "driving", power));
                return stopped();
            }
            return axisPowers(axis, sign * power);
        }
    }

    /**
     * Full power along one axis for as far as the field allows, then cut power and coast.
     * Fits v(t) = top * (1 - e^(-t / lag)) to the speed-up, and coast / speed gives the slip.
     */
    private final class Burst extends Task {
        private final Axis axis;
        private double sign;
        private Pose2d from;
        private Vector2d direction;
        private double limit = -1, cutAt = -1, cutSpeed, cutTravel, stillFor;
        private double unwrapped, lastHeading;
        private final List<double[]> samples = new ArrayList<>();

        /** @param sign +1 / -1, or 0 to pick whichever way has more room */
        Burst(Axis axis, double sign) {
            this.axis = axis;
            this.sign = sign;
        }

        @Override
        PoseVelocity2d update(Pose2d pose, PoseVelocity2d vel, double dt) {
            if (from == null) {
                from = pose;
                lastHeading = pose.heading.toDouble();
                if (axis == Axis.TURN) {
                    limit = spinIsClear(pose) ? Math.toRadians(300) : 0;
                } else {
                    if (sign == 0) {
                        sign = room(pose, axisDirection(pose, axis, 1)) >= room(pose, axisDirection(pose, axis, -1)) ? 1 : -1;
                    }
                    direction = axisDirection(pose, axis, sign);
                    limit = Math.min(LONGEST_BURST, room(pose, direction) - COAST_ROOM);
                }
                if (limit < (axis == Axis.TURN ? Math.toRadians(90) : 12)) {
                    log.add("not enough room for the " + axis.name().toLowerCase(Locale.US) + " burst, skipped");
                    return null;
                }
            }
            unwrapped += angleWrap(pose.heading.toDouble() - lastHeading);
            lastHeading = pose.heading.toDouble();
            double travel = axis == Axis.TURN ? Math.abs(unwrapped)
                    : (pose.position.x - from.position.x) * direction.x + (pose.position.y - from.position.y) * direction.y;
            double speed = Math.abs(axisSpeed(axis, vel));
            double power = axis == Axis.TURN ? TURN_BURST_POWER : BURST_POWER;

            if (cutAt < 0) {
                status = String.format(Locale.US, "%s burst: %.0f %s", axis.name().toLowerCase(Locale.US),
                        axis == Axis.TURN ? Math.toDegrees(travel) : travel, axis == Axis.TURN ? "deg" : "in");
                samples.add(new double[] {time, speed});
                if (travel >= limit || time > 3) {
                    cutAt = time;
                    cutSpeed = speed;
                    cutTravel = travel;
                    return stopped();
                }
                return axisPowers(axis, sign * power);
            }

            // coasting
            stillFor = speed < (axis == Axis.TURN ? Math.toRadians(2) : 0.5) ? stillFor + dt : 0;
            if (stillFor < 0.25 && time - cutAt < 2.5) {
                return stopped();
            }
            double[] fit = fitSpeedUp(samples);
            double friction = axis == Axis.TURN ? average(turnFrictions, 0.05) : average(frictions, 0.05);
            double effective = (power - friction) / (1 - friction);
            double slip = cutSpeed < 1e-3 ? 0 : (travel - cutTravel) / cutSpeed;
            (axis == Axis.FORWARD ? forwardBursts : axis == Axis.STRAFE ? strafeBursts : turnBursts)
                    .add(new double[] {fit[0] / effective, fit[1], slip});
            log.add(axis == Axis.TURN
                    ? String.format(Locale.US, "turn burst: top %.0f deg/s, lag %.2f s, slip %.3f s",
                            Math.toDegrees(fit[0] / effective), fit[1], slip)
                    : String.format(Locale.US, "%s burst: top %.1f in/s, lag %.2f s, coasted %.1f in (slip %.3f s)",
                            axis.name().toLowerCase(Locale.US), fit[0] / effective, fit[1], travel - cutTravel, slip));
            return null;
        }
    }

    /** Turn in place by an amount with the pure pursuit follower. */
    private final class Turn extends Task {
        private final PurePursuitFollower turn;

        Turn(double radians) {
            turn = PurePursuitFollower.turnBy(radians, follower, holonomic);
        }

        @Override
        PoseVelocity2d update(Pose2d pose, PoseVelocity2d vel, double dt) {
            status = "turning";
            PurePursuitFollower.Output out = turn.update(pose, vel);
            return out.isDone() || time > 6 ? null : out.powers;
        }
    }

    /** Drive back to a pose with the pure pursuit follower. */
    private final class GoTo extends Task {
        private final Vector2d target;
        private final double heading;
        private PurePursuitFollower drive;

        GoTo(Vector2d target, double heading) {
            this.target = target;
            this.heading = heading;
        }

        @Override
        PoseVelocity2d update(Pose2d pose, PoseVelocity2d vel, double dt) {
            status = "returning to the start";
            if (drive == null) {
                PurePursuitPath.Builder path = PurePursuitPath.builder().name("return").moveSpeed(0.5).to(target.x, target.y);
                if (holonomic) {
                    path.fixedHeading(heading);
                }
                drive = new PurePursuitFollower(path.endHeading(heading).build(), follower, holonomic);
            }
            PurePursuitFollower.Output out = drive.update(pose, vel);
            return out.isDone() || time > 10 ? null : out.powers;
        }
    }

    /** Copies the measurements into the follower tuning and the simulator numbers. */
    private final class Apply extends Task {
        @Override
        PoseVelocity2d update(Pose2d pose, PoseVelocity2d vel, double dt) {
            double friction = average(frictions, measuredDrive.staticFrictionPower);
            measuredDrive.staticFrictionPower = friction;
            follower.minMovePower = clip(friction * 1.15 + 0.02, 0.03, 0.35);
            follower.minTurnPower = clip(average(turnFrictions, friction) * 1.15 + 0.02, 0.03, 0.35);
            if (!forwardBursts.isEmpty()) {
                measuredDrive.maxForwardVel = column(forwardBursts, 0);
                measuredDrive.timeConstant = column(forwardBursts, 1);
                follower.forwardSlipSeconds = clip(column(forwardBursts, 2), 0, 0.5);
            }
            if (!strafeBursts.isEmpty()) {
                measuredDrive.maxStrafeVel = column(strafeBursts, 0);
                follower.strafeSlipSeconds = clip(column(strafeBursts, 2), 0, 0.5);
            } else if (!holonomic) {
                measuredDrive.maxStrafeVel = measuredDrive.maxForwardVel;
            }
            if (!turnBursts.isEmpty()) {
                measuredDrive.maxAngVel = column(turnBursts, 0);
                follower.turnSlipSeconds = clip(column(turnBursts, 2), 0, 0.5);
            }
            log.add("measured: " + javaLines().get(5).substring(3));
            return null;
        }
    }

    /**
     * One pass over the tuning course: drive each coded leg with pure pursuit, see where the
     * robot actually stops, then correct the slip and min power values. Queues another round
     * until the robot matches the coded paths or MAX_ROUNDS is reached.
     */
    private final class Round extends Task {
        private final List<Leg> legs = new ArrayList<>();
        private int index = -1;

        @Override
        PoseVelocity2d update(Pose2d pose, PoseVelocity2d vel, double dt) {
            if (index < 0) {
                round++;
                if (legLength == 0) {
                    legLength = Math.min(LONGEST_LEG, room(pose, axisDirection(pose, Axis.FORWARD, 1)) - 8);
                }
                buildLegs(pose);
                index = 0;
            }
            while (index < legs.size()) {
                PoseVelocity2d powers = legs.get(index).update(pose, vel, dt);
                if (powers != null) {
                    status = String.format(Locale.US, "matching round %d/%d: %s", round, MAX_ROUNDS, legs.get(index).name);
                    return powers;
                }
                index++;
            }
            correct();
            return null;
        }

        private void buildLegs(Pose2d pose) {
            double h = start.heading.toDouble();
            Vector2d ahead = new Vector2d(start.position.x + legLength * Math.cos(h), start.position.y + legLength * Math.sin(h));
            legs.add(new Leg("forward", Axis.FORWARD, ahead, h));
            legs.add(new Leg("backward", Axis.FORWARD, start.position, h));
            if (holonomic) {
                legs.add(new Leg("turn +90", h + Math.PI / 2));
                legs.add(new Leg("strafe", Axis.STRAFE, ahead, h + Math.PI / 2));
                legs.add(new Leg("strafe back", Axis.STRAFE, start.position, h + Math.PI / 2));
                legs.add(new Leg("turn -90", h));
            }
            legs.add(new Leg("turn +180", h + Math.PI));
            legs.add(new Leg("turn -180", h));
        }

        private void correct() {
            double[] worst = new double[3], change = new double[3];
            int[] count = new int[3];
            boolean stalled = false, turnStalled = false;
            for (Leg leg : legs) {
                int i = leg.axis.ordinal();
                worst[i] = Math.max(worst[i], Math.abs(leg.miss));
                if (leg.stalled) {
                    if (leg.axis == Axis.TURN) {
                        turnStalled = true;
                    } else {
                        stalled = true;
                    }
                } else {
                    double slowest = leg.axis == Axis.TURN ? Math.toRadians(20) : 4;
                    change[i] += leg.miss / Math.max(leg.finishSpeed, slowest);
                    count[i]++;
                }
            }
            double worstLinear = Math.max(worst[Axis.FORWARD.ordinal()], worst[Axis.STRAFE.ordinal()]);
            double worstTurn = worst[Axis.TURN.ordinal()];
            roundErrors.add(new double[] {worstLinear, worstTurn});
            log.add(String.format(Locale.US, "round %d: worst miss %.2f in, %.1f deg%s", round, worstLinear,
                    Math.toDegrees(worstTurn), stalled || turnStalled ? ", stalled short" : ""));

            // overshooting means the robot coasts further than the follower expects: aim shorter.
            // Axes already inside the follower's own tolerance are left alone.
            double linearMatch = linearMatch(), turnMatch = turnMatch();
            if (count[0] > 0 && worst[0] >= linearMatch) {
                follower.forwardSlipSeconds = nudge(follower.forwardSlipSeconds, change[0] / count[0]);
            }
            if (count[1] > 0 && worst[1] >= linearMatch) {
                follower.strafeSlipSeconds = nudge(follower.strafeSlipSeconds, change[1] / count[1]);
            }
            if (count[2] > 0 && worstTurn >= turnMatch) {
                follower.turnSlipSeconds = nudge(follower.turnSlipSeconds, change[2] / count[2]);
            }
            if (stalled) {
                follower.minMovePower = clip(follower.minMovePower + 0.02, 0, 0.35);
            }
            if (turnStalled) {
                follower.minTurnPower = clip(follower.minTurnPower + 0.02, 0, 0.35);
            }

            boolean matched = worstLinear < linearMatch && worstTurn < turnMatch && !stalled && !turnStalled;
            if (!matched && round < MAX_ROUNDS) {
                tasks.addFirst(new Round());
            } else {
                log.add(matched ? "the robot now stops where the paths say"
                        : "still off after " + MAX_ROUNDS + " rounds; check the localizer and wheels");
            }
        }
    }

    /** One coded course leg: drive it, wait for the robot to stop, and measure the miss. */
    private final class Leg {
        final String name;
        final Axis axis;
        final Vector2d target;
        final double heading;
        private PurePursuitFollower drive;
        private Pose2d from;
        private boolean finished;
        private double time, stillFor, lastHeading, unwrapped;
        /** Signed miss past the target (+ = overshot), in in or rad. */
        double miss, finishSpeed;
        boolean stalled;

        Leg(String name, Axis axis, Vector2d target, double heading) {
            this.name = name;
            this.axis = axis;
            this.target = target;
            this.heading = heading;
        }

        Leg(String name, double heading) {
            this(name, Axis.TURN, null, heading);
        }

        PoseVelocity2d update(Pose2d pose, PoseVelocity2d vel, double dt) {
            time += dt;
            if (drive == null) {
                from = pose;
                lastHeading = pose.heading.toDouble();
                if (axis == Axis.TURN) {
                    drive = PurePursuitFollower.turnBy(angleWrapPositive(heading - pose.heading.toDouble()), follower, holonomic);
                } else {
                    PurePursuitPath.Builder path = PurePursuitPath.builder().name(name).to(target.x, target.y);
                    if (holonomic) {
                        path.fixedHeading(heading);
                    } else if (Math.cos(Math.atan2(target.y - pose.position.y, target.x - pose.position.x) - heading) < 0) {
                        path.reversed();
                    }
                    drive = new PurePursuitFollower(path.build(), follower, holonomic);
                }
            }
            unwrapped += angleWrap(pose.heading.toDouble() - lastHeading);
            lastHeading = pose.heading.toDouble();

            if (!finished) {
                PurePursuitFollower.Output out = drive.update(pose, vel);
                boolean timedOut = time > 8;
                if (out.isDone() || timedOut) {
                    finished = true;
                    stalled = timedOut;
                    finishSpeed = Math.abs(axisSpeed(axis, vel));
                    stillFor = 0;
                    return stopped();
                }
                return out.powers;
            }
            // let it coast to a stop, then measure where it ended up
            stillFor = Math.abs(vel.angVel) < Math.toRadians(2) && Math.hypot(vel.linearVel.x, vel.linearVel.y) < 0.5
                    ? stillFor + dt : 0;
            if (stillFor < 0.25 && time < 12) {
                return stopped();
            }
            if (axis == Axis.TURN) {
                double wanted = angleWrapPositive(heading - from.heading.toDouble());
                miss = Math.signum(wanted) * (unwrapped - wanted);
            } else {
                double dx = target.x - from.position.x, dy = target.y - from.position.y;
                double length = Math.hypot(dx, dy);
                miss = length < 1e-6 ? 0
                        : ((pose.position.x - target.x) * dx + (pose.position.y - target.y) * dy) / length;
            }
            return null;
        }
    }

    // ---- helpers -----------------------------------------------------------------------------

    private static PoseVelocity2d stopped() {
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }

    private static PoseVelocity2d axisPowers(Axis axis, double power) {
        switch (axis) {
            case FORWARD: return new PoseVelocity2d(new Vector2d(power, 0), 0);
            case STRAFE: return new PoseVelocity2d(new Vector2d(0, power), 0);
            default: return new PoseVelocity2d(new Vector2d(0, 0), power);
        }
    }

    private static double axisSpeed(Axis axis, PoseVelocity2d vel) {
        switch (axis) {
            case FORWARD: return vel.linearVel.x;
            case STRAFE: return vel.linearVel.y;
            default: return vel.angVel;
        }
    }

    /** Field direction the robot moves for a positive (or negative) power on a linear axis. */
    private static Vector2d axisDirection(Pose2d pose, Axis axis, double sign) {
        double angle = pose.heading.toDouble() + (axis == Axis.STRAFE ? Math.PI / 2 : 0);
        return new Vector2d(sign * Math.cos(angle), sign * Math.sin(angle));
    }

    /** How far (in) the robot can slide along a direction before touching a wall or the structure. */
    private double room(Pose2d pose, Vector2d direction) {
        double length = robotLength + 2 * WALL_MARGIN, width = robotWidth + 2 * WALL_MARGIN;
        for (double d = 0; d <= 110; d += 1) {
            Pose2d moved = new Pose2d(pose.position.x + direction.x * d, pose.position.y + direction.y * d,
                    pose.heading.toDouble());
            if (BioBuzzField.outsideField(moved, length, width) || BioBuzzField.hitsCenterStructure(moved, length, width)) {
                return Math.max(0, d - 1);
            }
        }
        return 110;
    }

    private boolean spinIsClear(Pose2d pose) {
        for (int i = 0; i < 16; i++) {
            Pose2d turned = new Pose2d(pose.position, pose.heading.toDouble() + i * Math.PI / 8);
            if (BioBuzzField.outsideField(turned, robotLength + WALL_MARGIN, robotWidth + WALL_MARGIN)
                    || BioBuzzField.hitsCenterStructure(turned, robotLength + WALL_MARGIN, robotWidth + WALL_MARGIN)) {
                return false;
            }
        }
        return true;
    }

    /** Best fit of v(t) = top * (1 - e^(-t / lag)); returns {top, lag}. */
    static double[] fitSpeedUp(List<double[]> samples) {
        double bestTop = 0, bestLag = 0.15, bestError = Double.MAX_VALUE;
        for (double lag = 0.02; lag <= 0.8; lag += 0.005) {
            double num = 0, den = 0;
            for (double[] s : samples) {
                double shape = 1 - Math.exp(-s[0] / lag);
                num += shape * s[1];
                den += shape * shape;
            }
            if (den < 1e-9) {
                continue;
            }
            double top = num / den, error = 0;
            for (double[] s : samples) {
                double e = top * (1 - Math.exp(-s[0] / lag)) - s[1];
                error += e * e;
            }
            if (error < bestError) {
                bestError = error;
                bestTop = top;
                bestLag = lag;
            }
        }
        return new double[] {bestTop, bestLag};
    }

    /** A damped, capped step towards the slip a course leg suggested. */
    private static double nudge(double slip, double suggestedChange) {
        return clip(slip + clip(0.7 * suggestedChange, -0.05, 0.05), 0, 0.5);
    }

    private static double average(List<Double> values, double fallback) {
        if (values.isEmpty()) {
            return fallback;
        }
        double sum = 0;
        for (double v : values) {
            sum += v;
        }
        return sum / values.size();
    }

    private static double column(List<double[]> rows, int i) {
        double sum = 0;
        for (double[] row : rows) {
            sum += row[i];
        }
        return sum / rows.size();
    }

    /** Wraps to (-PI, PI] but keeps exactly PI positive, so "turn 180" goes counter-clockwise. */
    private static double angleWrapPositive(double radians) {
        double wrapped = angleWrap(radians);
        return wrapped <= -Math.PI + 1e-9 ? Math.PI : wrapped;
    }
}
