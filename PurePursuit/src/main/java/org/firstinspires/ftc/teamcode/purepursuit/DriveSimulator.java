package org.firstinspires.ftc.teamcode.purepursuit;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * Simple robot physics for testing paths off the robot: power sets a target speed, the real
 * speed lags behind it (so the robot coasts and slips like a real one), and a little power is
 * lost to static friction. Good enough to see how pure pursuit behaves, not a replacement for
 * tuning on the real field.
 */
public final class DriveSimulator {
    public static class Params {
        /** Top forward speed (in/s) at full power. */
        public double maxForwardVel = 50;
        /** Top strafe speed (in/s) at full power, mecanum only. */
        public double maxStrafeVel = 40;
        /** Top turn speed (rad/s) at full power. */
        public double maxAngVel = Math.toRadians(240);
        /** Seconds for the speed to get ~63% of the way to its target. */
        public double timeConstant = 0.15;
        /** Power below which the robot doesn't move at all. */
        public double staticFrictionPower = 0.06;
    }

    private final Params params;
    private final boolean holonomic;
    private Pose2d pose;
    private double forwardVel, strafeVel, angVel;

    public DriveSimulator(Pose2d startPose, boolean holonomic, Params params) {
        this.pose = startPose;
        this.holonomic = holonomic;
        this.params = params;
    }

    public Pose2d getPose() {
        return pose;
    }

    /** Robot relative velocity, the same thing a Road Runner localizer returns. */
    public PoseVelocity2d getVelocity() {
        return new PoseVelocity2d(new Vector2d(forwardVel, strafeVel), angVel);
    }

    /** Advance the simulation by dt seconds with the given robot relative powers. */
    public void step(PoseVelocity2d powers, double dt) {
        PoseVelocity2d normalized = DrivePowers.normalize(powers, holonomic);
        double targetForward = afterFriction(normalized.linearVel.x) * params.maxForwardVel;
        double targetStrafe = afterFriction(normalized.linearVel.y) * params.maxStrafeVel;
        double targetTurn = afterFriction(normalized.angVel) * params.maxAngVel;

        double blend = 1 - Math.exp(-dt / params.timeConstant);
        forwardVel += (targetForward - forwardVel) * blend;
        strafeVel += (targetStrafe - strafeVel) * blend;
        angVel += (targetTurn - angVel) * blend;

        pose = pose.plus(new Twist2d(new Vector2d(forwardVel * dt, strafeVel * dt), angVel * dt));
    }

    private double afterFriction(double power) {
        double magnitude = Math.abs(power);
        if (magnitude <= params.staticFrictionPower) {
            return 0;
        }
        return Math.copySign((magnitude - params.staticFrictionPower)
                / (1 - params.staticFrictionPower), power);
    }
}
