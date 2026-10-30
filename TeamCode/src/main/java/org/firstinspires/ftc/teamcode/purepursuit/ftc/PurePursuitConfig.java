package org.firstinspires.ftc.teamcode.purepursuit.ftc;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.purepursuit.PurePursuitFollower;

/** Pure pursuit settings, live editable from FTC Dashboard (192.168.43.1:8080/dash). */
@Config
public final class PurePursuitConfig {
    private PurePursuitConfig() {}

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
}
