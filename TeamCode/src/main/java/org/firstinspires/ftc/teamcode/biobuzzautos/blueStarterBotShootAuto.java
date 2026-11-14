package org.firstinspires.ftc.teamcode.biobuzzautos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.ftc.PurePursuitDrive;
import org.firstinspires.ftc.teamcode.purepursuit.paths.BioBuzzRoutes;
import org.firstinspires.ftc.teamcode.purepursuitautos.PurePursuitAutoBase;

import java.util.HashMap;
import java.util.Map;

/**
 * Simplest scoring auto for the goBILDA StarterBot (tank drive), run with pure pursuit. The path
 * is BioBuzzRoutes.BLUE_STARTERBOT_SHOOT, so FieldViewer can simulate it: up the right wall, turn
 * to the blue hive, shoot the preloads, then park in the blue zone.
 *
 * Launcher settings and hardware names match FTCStarterBioBuz so the auto shoots like teleop.
 */
@Autonomous(name = "BLUE StarterBot Shoot Auto", group = "StarterBot")
public class blueStarterBotShootAuto extends PurePursuitAutoBase {

    private static final int LAUNCHER_TARGET_VELOCITY = 1250;
    private static final int LAUNCHER_MIN_VELOCITY = 1200;
    /** Long enough for the windmill to feed all three preloads; keep BLUE_STARTERBOT_SHOOT's marker time in step. */
    private static final double FEED_SECONDS = 2;
    /** Give up waiting for spin-up so a bad encoder can't stall the whole auto. */
    private static final double SPIN_UP_TIMEOUT_SECONDS = 3;

    private DcMotorEx launcher;
    private DcMotor intake;
    private CRServo windmill;

    @Override
    protected Route route() {
        return BioBuzzRoutes.BLUE_STARTERBOT_SHOOT;
    }

    /** Always the starter bot's tank drive, whatever PurePursuitConfig.DRIVE_TYPE says. */
    @Override
    protected PurePursuitDrive createDrive(Pose2d startPose) {
        return PurePursuitDrive.tank(new TankDrive(hardwareMap, startPose));
    }

    @Override
    protected void initMechanisms() {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake = hardwareMap.get(DcMotor.class, "intake");
        windmill = hardwareMap.get(CRServo.class, "windmill");
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5));
        windmill.setPower(0);
    }

    @Override
    protected Map<String, Action> markers() {
        Map<String, Action> markers = new HashMap<>();
        // spin up while driving so the launcher is at speed when the robot reaches the hive
        markers.put(BioBuzzRoutes.SPIN_UP, packet -> {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            return false;
        });
        markers.put(BioBuzzRoutes.SHOOT, new Shoot());
        return markers;
    }

    @Override
    protected void stopMechanisms() {
        launcher.setVelocity(0);
        intake.setPower(0);
        windmill.setPower(0);
    }

    /** Spins the launcher up, then runs the windmill to feed every preload into it. */
    private class Shoot implements Action {
        private final ElapsedTime timer = new ElapsedTime();
        private boolean started = false;
        private boolean feeding = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!started) {
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                timer.reset();
                started = true;
            }

            double velocity = launcher.getVelocity();
            packet.put("launcher velocity", velocity);

            if (!feeding) {
                if (velocity > LAUNCHER_MIN_VELOCITY || timer.seconds() > SPIN_UP_TIMEOUT_SECONDS) {
                    feeding = true;
                    timer.reset();
                    windmill.setPower(1);
                    intake.setPower(1);   // nudges stuck artifacts toward the windmill
                }
                return true;
            }

            if (timer.seconds() < FEED_SECONDS) {
                return true;
            }

            stopMechanisms();
            return false;
        }
    }
}

//yet to be tested
