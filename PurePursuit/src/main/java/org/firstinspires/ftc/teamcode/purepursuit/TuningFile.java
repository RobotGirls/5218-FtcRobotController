package org.firstinspires.ftc.teamcode.purepursuit;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;
import java.util.Properties;

/**
 * The values PP Auto Tuner found, saved on the Control Hub so every pure pursuit OpMode loads
 * them on its own, and sent to the FieldViewer so its simulator matches the robot.
 *
 * Holds the follower values the tuner changes and the measured drive numbers. Saved as a
 * properties file; sent to the FieldViewer as one line of key=value pairs.
 */
public final class TuningFile {
    /** Follower values the auto tuner sets. */
    public static final String[] FOLLOWER_KEYS = {
            "forwardSlipSeconds", "strafeSlipSeconds", "turnSlipSeconds", "minMovePower", "minTurnPower"
    };
    /** Measured drive numbers, the same names as DriveSimulator.Params. */
    public static final String[] DRIVE_KEYS = {
            "maxForwardVel", "maxStrafeVel", "maxAngVel", "timeConstant", "staticFrictionPower"
    };
    private static final String SAVED_AT = "savedAt";

    public final Map<String, Double> follower = new LinkedHashMap<>();
    public final Map<String, Double> drive = new LinkedHashMap<>();
    /** When the tuner saved these, as shown to people (e.g. "Nov 7 4:12 PM"). */
    public final String savedAt;

    private TuningFile(String savedAt) {
        this.savedAt = savedAt == null ? "" : savedAt;
    }

    public static TuningFile of(PurePursuitFollower.Params follower, DriveSimulator.Params drive, String savedAt) {
        TuningFile tuning = new TuningFile(savedAt);
        tuning.follower.put("forwardSlipSeconds", follower.forwardSlipSeconds);
        tuning.follower.put("strafeSlipSeconds", follower.strafeSlipSeconds);
        tuning.follower.put("turnSlipSeconds", follower.turnSlipSeconds);
        tuning.follower.put("minMovePower", follower.minMovePower);
        tuning.follower.put("minTurnPower", follower.minTurnPower);
        tuning.drive.put("maxForwardVel", drive.maxForwardVel);
        tuning.drive.put("maxStrafeVel", drive.maxStrafeVel);
        tuning.drive.put("maxAngVel", drive.maxAngVel);
        tuning.drive.put("timeConstant", drive.timeConstant);
        tuning.drive.put("staticFrictionPower", drive.staticFrictionPower);
        return tuning;
    }

    /** Copies the saved follower values into params; values that weren't saved are left alone. */
    public void applyTo(PurePursuitFollower.Params params) {
        params.forwardSlipSeconds = get(follower, "forwardSlipSeconds", params.forwardSlipSeconds);
        params.strafeSlipSeconds = get(follower, "strafeSlipSeconds", params.strafeSlipSeconds);
        params.turnSlipSeconds = get(follower, "turnSlipSeconds", params.turnSlipSeconds);
        params.minMovePower = get(follower, "minMovePower", params.minMovePower);
        params.minTurnPower = get(follower, "minTurnPower", params.minTurnPower);
    }

    /** Copies the measured drive numbers into a simulator's params. */
    public void applyTo(DriveSimulator.Params params) {
        params.maxForwardVel = get(drive, "maxForwardVel", params.maxForwardVel);
        params.maxStrafeVel = get(drive, "maxStrafeVel", params.maxStrafeVel);
        params.maxAngVel = get(drive, "maxAngVel", params.maxAngVel);
        params.timeConstant = get(drive, "timeConstant", params.timeConstant);
        params.staticFrictionPower = get(drive, "staticFrictionPower", params.staticFrictionPower);
    }

    public void save(File file) throws IOException {
        Properties properties = new Properties();
        properties.setProperty(SAVED_AT, savedAt);
        for (Map.Entry<String, Double> e : follower.entrySet()) {
            properties.setProperty("follower." + e.getKey(), num(e.getValue()));
        }
        for (Map.Entry<String, Double> e : drive.entrySet()) {
            properties.setProperty("drive." + e.getKey(), num(e.getValue()));
        }
        File folder = file.getParentFile();
        if (folder != null && !folder.isDirectory() && !folder.mkdirs()) {
            throw new IOException("can't create " + folder);
        }
        try (OutputStream out = new FileOutputStream(file)) {
            properties.store(out, "Saved by PP Auto Tuner. Delete this file to go back to the defaults.");
        }
    }

    /** The saved tuning, or null if there's no file or it can't be read. */
    public static TuningFile load(File file) {
        if (!file.isFile()) {
            return null;
        }
        Properties properties = new Properties();
        try (InputStream in = new FileInputStream(file)) {
            properties.load(in);
        } catch (IOException e) {
            return null;
        }
        TuningFile tuning = new TuningFile(properties.getProperty(SAVED_AT));
        for (String key : FOLLOWER_KEYS) {
            put(tuning.follower, key, properties.getProperty("follower." + key));
        }
        for (String key : DRIVE_KEYS) {
            put(tuning.drive, key, properties.getProperty("drive." + key));
        }
        return tuning.follower.isEmpty() && tuning.drive.isEmpty() ? null : tuning;
    }

    /** One line for the FieldViewer link: savedAt=...;follower.key=value;drive.key=value */
    public String encode() {
        StringBuilder sb = new StringBuilder(SAVED_AT).append('=').append(savedAt.replace(';', ' ').replace(',', ' '));
        for (Map.Entry<String, Double> e : follower.entrySet()) {
            sb.append(";follower.").append(e.getKey()).append('=').append(num(e.getValue()));
        }
        for (Map.Entry<String, Double> e : drive.entrySet()) {
            sb.append(";drive.").append(e.getKey()).append('=').append(num(e.getValue()));
        }
        return sb.toString();
    }

    /** Reads {@link #encode()}'s line, skipping anything it doesn't recognize. */
    public static TuningFile decode(String text) {
        String savedAt = "";
        Map<String, String> values = new LinkedHashMap<>();
        for (String pair : text.split(";")) {
            int equals = pair.indexOf('=');
            if (equals < 0) {
                continue;
            }
            String key = pair.substring(0, equals).trim();
            String value = pair.substring(equals + 1).trim();
            if (key.equals(SAVED_AT)) {
                savedAt = value;
            } else {
                values.put(key, value);
            }
        }
        TuningFile tuning = new TuningFile(savedAt);
        for (String key : FOLLOWER_KEYS) {
            put(tuning.follower, key, values.get("follower." + key));
        }
        for (String key : DRIVE_KEYS) {
            put(tuning.drive, key, values.get("drive." + key));
        }
        return tuning;
    }

    private static void put(Map<String, Double> map, String key, String text) {
        if (text == null) {
            return;
        }
        try {
            double value = Double.parseDouble(text);
            if (!Double.isNaN(value) && !Double.isInfinite(value)) {
                map.put(key, value);
            }
        } catch (NumberFormatException ignored) {
            // a hand-edited file with a typo just keeps that default
        }
    }

    private static double get(Map<String, Double> map, String key, double fallback) {
        Double value = map.get(key);
        return value == null ? fallback : value;
    }

    private static String num(double value) {
        return String.format(Locale.US, "%.4f", value);
    }
}
