package org.firstinspires.ftc.teamcode.purepursuitautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.paths.BioBuzzRoutes;

/** Pure pursuit version of biobuzzautos/redCornerAuto. */
@Autonomous(name = "PP RED Corner Park", group = "Pure Pursuit")
public class PPRedCornerPark extends PurePursuitAutoBase {
    @Override
    protected Route route() {
        return BioBuzzRoutes.RED_CORNER_PARK;
    }
}
