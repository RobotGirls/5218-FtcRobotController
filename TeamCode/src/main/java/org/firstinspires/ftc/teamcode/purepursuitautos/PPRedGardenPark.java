package org.firstinspires.ftc.teamcode.purepursuitautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.paths.BioBuzzRoutes;

/** Pure pursuit version of biobuzzautos/redGardenAuto. */
@Autonomous(name = "PP RED Garden Park", group = "Pure Pursuit")
public class PPRedGardenPark extends PurePursuitAutoBase {
    @Override
    protected Route route() {
        return BioBuzzRoutes.RED_GARDEN_PARK;
    }
}
