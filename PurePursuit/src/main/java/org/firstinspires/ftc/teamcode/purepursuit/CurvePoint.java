package org.firstinspires.ftc.teamcode.purepursuit;

/**
 * One waypoint of a pure pursuit path. Ported from Gluten Free's CurvePoint
 * (https://github.com/nwatx/Gluten-Free-Code-Reuploaded), converted to inches and radians.
 *
 * The speed and slow down values belong to the segment that ENDS at this point, so they are
 * what the robot uses while it is driving towards this waypoint.
 */
public final class CurvePoint {
    /** Field position in inches (Road Runner field coordinates, origin at field center). */
    public final double x, y;
    /** Max translational power on the segment ending here, 0 to 1. */
    public final double moveSpeed;
    /** Max turning power on the segment ending here, 0 to 1. */
    public final double turnSpeed;
    /** Lookahead radius (in) used to find the point the robot drives towards. */
    public final double followDistance;
    /** Lookahead radius (in) used to find the point the robot faces. */
    public final double pointLength;
    /** Heading error (rad) at which translation is slowed down the most. */
    public final double slowDownTurnRadians;
    /** Max fraction (0 to 1) of translation removed while the heading error is large. */
    public final double slowDownTurnAmount;

    public CurvePoint(double x, double y, double moveSpeed, double turnSpeed,
                      double followDistance, double pointLength,
                      double slowDownTurnRadians, double slowDownTurnAmount) {
        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        this.pointLength = pointLength;
        this.slowDownTurnRadians = slowDownTurnRadians;
        this.slowDownTurnAmount = slowDownTurnAmount;
    }

    /** Same tuning values at a different location. */
    public CurvePoint withPosition(double x, double y) {
        return new CurvePoint(x, y, moveSpeed, turnSpeed, followDistance, pointLength,
                slowDownTurnRadians, slowDownTurnAmount);
    }

    @Override
    public String toString() {
        return String.format("CurvePoint(%.1f, %.1f)", x, y);
    }
}
