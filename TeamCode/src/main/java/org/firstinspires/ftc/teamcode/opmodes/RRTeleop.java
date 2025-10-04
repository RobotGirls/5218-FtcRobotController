package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.List;

public class RRTeleop extends OpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void init() {
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        // updated based on gamepads

        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
//            if (gamepad1.a) {
//                runningActions.add(new SequentialAction(
//                        new SleepAction(0.5),
//                        new InstantAction(() -> servo.setPosition(0.5))
//                ));
//            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
    }
}
