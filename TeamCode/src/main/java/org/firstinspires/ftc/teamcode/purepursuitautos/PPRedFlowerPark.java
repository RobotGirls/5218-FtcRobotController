package org.firstinspires.ftc.teamcode.purepursuitautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.paths.BioBuzzRoutes;

/** Pure pursuit version of biobuzzautos/redFlowerAuto. */
@Autonomous(name = "PP RED Flower Park", group = "Pure Pursuit")
public class PPRedFlowerPark extends PurePursuitAutoBase {
    @Override
    protected Route route() {
        return BioBuzzRoutes.RED_FLOWER_PARK;
    }
}
