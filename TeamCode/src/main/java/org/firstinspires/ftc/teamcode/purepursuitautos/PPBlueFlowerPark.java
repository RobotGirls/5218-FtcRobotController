package org.firstinspires.ftc.teamcode.purepursuitautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.paths.BioBuzzRoutes;

/** Pure pursuit version of biobuzzautos/blueFlowerAuto. */
@Autonomous(name = "PP BLUE Flower Park", group = "Pure Pursuit")
public class PPBlueFlowerPark extends PurePursuitAutoBase {
    @Override
    protected Route route() {
        return BioBuzzRoutes.BLUE_FLOWER_PARK;
    }
}
