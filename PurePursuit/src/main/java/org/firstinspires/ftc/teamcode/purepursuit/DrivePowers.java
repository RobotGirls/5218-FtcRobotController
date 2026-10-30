package org.firstinspires.ftc.teamcode.purepursuit;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

/** Drive power helpers used by both the robot and the simulator so they behave the same. */
public final class DrivePowers {
    private DrivePowers() {}

    /**
     * Scales robot relative powers down (keeping their ratio) so no wheel is asked for more
     * than full power. Tank drives can't strafe, so the lateral part is dropped.
     */
    public static PoseVelocity2d normalize(PoseVelocity2d powers, boolean holonomic) {
        double forward = powers.linearVel.x;
        double strafe = holonomic ? powers.linearVel.y : 0;
        double turn = powers.angVel;

        double biggestWheel = Math.abs(forward) + Math.abs(strafe) + Math.abs(turn);
        double scale = biggestWheel > 1 ? 1 / biggestWheel : 1;
        return new PoseVelocity2d(new Vector2d(forward * scale, strafe * scale), turn * scale);
    }
}
