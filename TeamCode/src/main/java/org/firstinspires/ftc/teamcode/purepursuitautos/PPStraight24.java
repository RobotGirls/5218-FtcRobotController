package org.firstinspires.ftc.teamcode.purepursuitautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.paths.TestRoutes;

/** Drives 24 inches straight forward and stops. */
@Autonomous(name = "PP Straight 24in", group = "Pure Pursuit")
public class PPStraight24 extends PurePursuitAutoBase {
    @Override
    protected Route route() {
        return TestRoutes.STRAIGHT_24;
    }
}
