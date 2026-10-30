package org.firstinspires.ftc.teamcode.purepursuitautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.paths.BioBuzzRoutes;

/** Pure pursuit version of biobuzzautos/blueGardenAuto. */
@Autonomous(name = "PP BLUE Garden Park", group = "Pure Pursuit")
public class PPBlueGardenPark extends PurePursuitAutoBase {
    @Override
    protected Route route() {
        return BioBuzzRoutes.BLUE_GARDEN_PARK;
    }
}
