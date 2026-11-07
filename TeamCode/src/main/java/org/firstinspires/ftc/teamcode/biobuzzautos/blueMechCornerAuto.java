package org.firstinspires.ftc.teamcode.biobuzzautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.paths.BioBuzzRoutes;

/** Pure pursuit; the path is BioBuzzRoutes.BLUE_MECH_CORNER, so FieldViewer can simulate it. */
@Autonomous(name = "BLUE Mechanum Corner Auto")
public class blueMechCornerAuto extends BioBuzzMechAuto {
    @Override
    protected Route route() {
        return BioBuzzRoutes.BLUE_MECH_CORNER;
    }
}


//yet to be tested
