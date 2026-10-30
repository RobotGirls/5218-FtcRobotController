package org.firstinspires.ftc.teamcode.purepursuitautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.paths.BioBuzzRoutes;

/** Pure pursuit version of biobuzzautos/blueCornerAuto. */
@Autonomous(name = "PP BLUE Corner Park", group = "Pure Pursuit")
public class PPBlueCornerPark extends PurePursuitAutoBase {
    @Override
    protected Route route() {
        return BioBuzzRoutes.BLUE_CORNER_PARK;
    }
}
