package org.firstinspires.ftc.teamcode.purepursuitautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.paths.BioBuzzRoutes;

/** Pure pursuit version of biobuzzautos/blueGardenAutocopy. */
@Autonomous(name = "PP BLUE Garden Park COPY", group = "Pure Pursuit")
public class PPBlueGardenParkCopy extends PurePursuitAutoBase {
    @Override
    protected Route route() {
        return BioBuzzRoutes.BLUE_GARDEN_PARK_COPY;
    }
}
