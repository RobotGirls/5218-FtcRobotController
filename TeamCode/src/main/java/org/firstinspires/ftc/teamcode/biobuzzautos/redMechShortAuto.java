package org.firstinspires.ftc.teamcode.biobuzzautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.paths.BioBuzzRoutes;

/** Pure pursuit; the path is BioBuzzRoutes.RED_MECH_SHORT, so FieldViewer can simulate it. */
@Autonomous(name = "RED Mecanum Auto")
public class redMechShortAuto extends BioBuzzMechAuto {
    @Override
    protected Route route() {
        return BioBuzzRoutes.RED_MECH_SHORT;
    }

    // the launcher is commented out in this auto for now
    @Override
    protected boolean hasLauncher() {
        return false;
    }
}


//yet to be tested
