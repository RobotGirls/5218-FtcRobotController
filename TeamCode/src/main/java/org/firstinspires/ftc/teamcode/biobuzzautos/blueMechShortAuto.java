package org.firstinspires.ftc.teamcode.biobuzzautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.paths.BioBuzzRoutes;

/** Pure pursuit; the path is BioBuzzRoutes.BLUE_MECH_SHORT, so FieldViewer can simulate it. */
@Autonomous(name = "BLUE Mecanum Auto")
public class blueMechShortAuto extends BioBuzzMechAuto {
    @Override
    protected Route route() {
        return BioBuzzRoutes.BLUE_MECH_SHORT;
    }
}


//yet to be tested
