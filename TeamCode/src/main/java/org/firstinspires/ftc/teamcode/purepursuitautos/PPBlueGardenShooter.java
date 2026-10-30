package org.firstinspires.ftc.teamcode.purepursuitautos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.paths.BioBuzzRoutes;

import java.util.Collections;
import java.util.Map;

/**
 * Pure pursuit version of biobuzzautos/integrateMechAuto: park in the blue zone, then spin up
 * the launcher and feed artifacts, as one Road Runner Action at the route's "shoot" marker.
 */
@Config
@Autonomous(name = "PP BLUE Garden Park + Shooter", group = "Pure Pursuit")
public class PPBlueGardenShooter extends PurePursuitAutoBase {
    // same numbers as integrateMechAuto / the TeleOp
    public static double LAUNCHER_VELOCITY = 1250;
    public static double LAUNCHER_READY_VELOCITY = 1200;
    /** Feed anyway if the launcher hasn't reached speed by now, instead of waiting forever. */
    public static double MAX_SPIN_UP_SECONDS = 3;
    public static double FEED_SECONDS = 2;

    private DcMotorEx launcher;
    private DcMotor intake;
    private CRServo windmill;

    @Override
    protected Route route() {
        return BioBuzzRoutes.BLUE_GARDEN_SHOOTER;
    }

    @Override
    protected void initMechanisms() {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake = hardwareMap.get(DcMotor.class, "intake");
        windmill = hardwareMap.get(CRServo.class, "windmill");

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setVelocityPIDFCoefficients(40, 0, 0, 12.5);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    protected Map<String, Action> markers() {
        return Collections.<String, Action>singletonMap(BioBuzzRoutes.SHOOT, new ShootAction());
    }

    @Override
    protected void stopMechanisms() {
        launcher.setVelocity(0);
        intake.setPower(0);
        windmill.setPower(0);
    }

    /** Spin up the flywheel, then run the intake and windmill to feed it. */
    private class ShootAction implements Action {
        private long spinUpStart = -1;
        private long feedStart = -1;

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            long now = System.nanoTime();
            if (spinUpStart < 0) {
                spinUpStart = now;
                launcher.setVelocity(LAUNCHER_VELOCITY);
            }
            p.put("launcher velocity", launcher.getVelocity());

            if (feedStart < 0) {
                boolean ready = launcher.getVelocity() >= LAUNCHER_READY_VELOCITY;
                if (!ready && (now - spinUpStart) / 1e9 < MAX_SPIN_UP_SECONDS) {
                    return true;
                }
                feedStart = now;
                intake.setPower(1.0);
                windmill.setPower(1.0);
            }

            if ((now - feedStart) / 1e9 < FEED_SECONDS) {
                return true;
            }
            stopMechanisms();
            return false;
        }
    }
}
