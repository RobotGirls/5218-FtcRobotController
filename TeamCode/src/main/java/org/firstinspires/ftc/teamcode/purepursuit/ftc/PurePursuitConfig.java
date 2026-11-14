package org.firstinspires.ftc.teamcode.purepursuit.ftc;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.purepursuit.PurePursuitFollower;
import org.firstinspires.ftc.teamcode.purepursuit.TuningFile;

import java.io.File;
import java.io.IOException;

/** Pure pursuit settings, live editable from FTC Dashboard (192.168.43.1:8080/dash). */
@Config
public final class PurePursuitConfig {
    private PurePursuitConfig() {}

    /** Where PP Auto Tuner saves its results on the Control Hub. */
    private static final File TUNING_FILE = new File(AppUtil.FIRST_FOLDER, "purepursuit_tuning.properties");
    private static long loadedModified = -1;
    private static TuningFile savedTuning;

    public enum DriveType {
        /** BioBuzz starter bot: TankDrive with the Pinpoint localizer. */
        TANK,
        /** Road Runner quickstart MecanumDrive with the Pinpoint localizer. */
        MECANUM,
        /** Comp5218MecanumDrive with drive encoder + IMU localization. */
        COMP_MECANUM
    }

    /** Which drive class (and therefore which Road Runner localizer) the pure pursuit OpModes use. */
    public static DriveType DRIVE_TYPE = DriveType.TANK;

    /** Follower tuning shared by every path. */
    public static PurePursuitFollower.Params FOLLOWER = new PurePursuitFollower.Params();

    /** Send frames to the FieldViewer laptop app when it's connected. */
    public static boolean STREAM_TO_FIELD_VIEWER = true;

    /** Use the values PP Auto Tuner saved on the Control Hub instead of the defaults. */
    public static boolean USE_SAVED_TUNING = true;

    /**
     * Copies PP Auto Tuner's saved values into FOLLOWER. Called when each pure pursuit OpMode
     * starts; only reads the file again when it changed, so Dashboard edits made since stay.
     */
    public static synchronized void loadSavedTuning() {
        if (!USE_SAVED_TUNING) {
            return;
        }
        long modified = TUNING_FILE.isFile() ? TUNING_FILE.lastModified() : 0;
        if (modified == loadedModified) {
            return;
        }
        loadedModified = modified;
        savedTuning = TuningFile.load(TUNING_FILE);
        if (savedTuning != null) {
            savedTuning.applyTo(FOLLOWER);
            RobotLog.ii("PurePursuitConfig", "loaded auto tuner values saved %s", savedTuning.savedAt);
        }
    }

    /** Saves the tuner's results so every pure pursuit OpMode loads them from now on. */
    public static synchronized void saveTuning(TuningFile tuning) throws IOException {
        tuning.save(TUNING_FILE);
        tuning.applyTo(FOLLOWER);
        savedTuning = tuning;
        loadedModified = TUNING_FILE.lastModified();
    }

    /** Deletes the saved values and puts the defaults back. */
    public static synchronized void clearSavedTuning() {
        if (TUNING_FILE.isFile() && !TUNING_FILE.delete()) {
            RobotLog.ww("PurePursuitConfig", "couldn't delete %s", TUNING_FILE);
        }
        PurePursuitFollower.Params defaults = new PurePursuitFollower.Params();
        FOLLOWER.forwardSlipSeconds = defaults.forwardSlipSeconds;
        FOLLOWER.strafeSlipSeconds = defaults.strafeSlipSeconds;
        FOLLOWER.turnSlipSeconds = defaults.turnSlipSeconds;
        FOLLOWER.minMovePower = defaults.minMovePower;
        FOLLOWER.minTurnPower = defaults.minTurnPower;
        savedTuning = null;
        loadedModified = 0;
    }

    /** The values loaded from PP Auto Tuner's file, or null if the defaults are in use. */
    public static synchronized TuningFile savedTuning() {
        return USE_SAVED_TUNING ? savedTuning : null;
    }

    /** One line for telemetry saying where the follower values came from. */
    public static String tuningSource() {
        TuningFile saved = savedTuning();
        return saved == null ? "defaults (run PP Auto Tuner to tune)" : "PP Auto Tuner, saved " + saved.savedAt;
    }
}
