package org.firstinspires.ftc.teamcode.biobuzzautos;

import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.purepursuit.MechanismBindings;
import org.firstinspires.ftc.teamcode.purepursuit.paths.BioBuzzRoutes;
import org.firstinspires.ftc.teamcode.purepursuitautos.PurePursuitAutoBase;

/**
 * Shared setup for the mecanum autos: runs a BioBuzzRoutes route with pure pursuit and binds its
 * markers to the Launcher and Intake.
 */
public abstract class BioBuzzMechAuto extends PurePursuitAutoBase {
    protected Launcher launcher;
    protected Intake intake;

    /** False for autos whose robot has no launcher yet, so init doesn't look for its motor. */
    protected boolean hasLauncher() {
        return true;
    }

    @Override
    protected void initMechanisms() {
        if (hasLauncher()) {
            launcher = new Launcher(hardwareMap);
        }
        intake = new Intake(hardwareMap);
    }

    @Override
    protected MechanismBindings<Action> mechanisms() {
        MechanismBindings<Action> bindings = new MechanismBindings<Action>()
                .register(BioBuzzRoutes.INTAKE, intake::intakeIn);
        if (hasLauncher()) {
            bindings.register(BioBuzzRoutes.SHOOT, launcher::launcherForward);
        }
        return bindings;
    }

    @Override
    protected void stopMechanisms() {
        if (launcher != null) {
            launcher.stop();
        }
        intake.stop();
    }
}
