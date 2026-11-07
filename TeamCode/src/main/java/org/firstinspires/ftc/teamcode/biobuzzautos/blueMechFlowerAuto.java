package org.firstinspires.ftc.teamcode.biobuzzautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.paths.BioBuzzRoutes;

/** Pure pursuit; the path is BioBuzzRoutes.BLUE_MECH_FLOWER, so FieldViewer can simulate it. */
@Autonomous(name = "BLUE mechanum Flower Auto")
public class blueMechFlowerAuto extends BioBuzzMechAuto {
    @Override
    protected Route route() {
        return BioBuzzRoutes.BLUE_MECH_FLOWER;
    }
}


//yet to be tested
