package org.firstinspires.ftc.teamcode.software;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class GamepadHandler {
    private Gamepad localgamepad;

    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevY = false;

    private boolean isRedAlliance = true;
    private boolean isCloseSide = true;
    public GamepadHandler(Gamepad gamepad) {
        this.localgamepad = gamepad;
    }

    public double getDriveY() {
        return 0;
    }
    public double getDriveX() {
        return 0;
    }
    public double getTurnX() {
        return 0;
    }
}
