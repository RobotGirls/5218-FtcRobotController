package org.firstinspires.ftc.teamcode.biobuzzautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.paths.BioBuzzRoutes;

/**
 * Red version of blueStarterBotShootAuto: the same shooter and tank drive, on
 * BioBuzzRoutes.RED_STARTERBOT_SHOOT (the blue route turned 180 degrees about the field center).
 */
@Autonomous(name = "RED StarterBot Shoot Auto", group = "StarterBot")
public class redStarterBotShootAuto extends blueStarterBotShootAuto {
    @Override
    protected Route route() {
        return BioBuzzRoutes.RED_STARTERBOT_SHOOT;
    }
}

//yet to be tested
