package org.firstinspires.ftc.teamcode.biobuzzautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.paths.BioBuzzRoutes;

/** Pure pursuit; the path is BioBuzzRoutes.RED_MECH_FLOWER, so FieldViewer can simulate it. */
@Autonomous(name = "RED mechanum Flower Auto")
public class redMechFlowerAuto extends BioBuzzMechAuto {
    @Override
    protected Route route() {
        return BioBuzzRoutes.RED_MECH_FLOWER;
    }
}


//yet to be tested
