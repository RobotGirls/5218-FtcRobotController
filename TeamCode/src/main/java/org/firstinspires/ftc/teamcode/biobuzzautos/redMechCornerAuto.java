package org.firstinspires.ftc.teamcode.biobuzzautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.paths.BioBuzzRoutes;

/** Pure pursuit; the path is BioBuzzRoutes.RED_MECH_CORNER, so FieldViewer can simulate it. */
@Autonomous(name = "RED mechanum Corner Auto")
public class redMechCornerAuto extends BioBuzzMechAuto {
    @Override
    protected Route route() {
        return BioBuzzRoutes.RED_MECH_CORNER;
    }
}


//yet to be tested
