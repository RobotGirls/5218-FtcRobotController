package org.firstinspires.ftc.teamcode.purepursuitautos;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.purepursuit.Route;
import org.firstinspires.ftc.teamcode.purepursuit.paths.Routes;
import org.firstinspires.ftc.teamcode.purepursuit.ftc.PurePursuitDrive;

/**
 * Pick any route from purepursuit/paths with the dpad during init and run it. Mechanism markers
 * just wait. Open the FieldViewer in Live mode to watch the follower while it drives.
 */
@Autonomous(name = "PP Path Tester", group = "Pure Pursuit")
public class PurePursuitPathTester extends LinearOpMode {
    @Override
    public void runOpMode() {
        int selected = 0;
        Route route = Routes.ALL.get(selected);
        PurePursuitDrive drive = PurePursuitDrive.create(hardwareMap, route.startPose);
        boolean upWasPressed = false, downWasPressed = false;

        while (opModeInInit()) {
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
            if ((up && !upWasPressed) || (down && !downWasPressed)) {
                selected = (selected + (up ? -1 : 1) + Routes.ALL.size()) % Routes.ALL.size();
                route = Routes.ALL.get(selected);
                // put the robot at the new start pose so the path lines up with the field
                drive.setPose(route.startPose);
            }
            upWasPressed = up;
            downWasPressed = down;

            drive.setRouteName(route.name);
            drive.updatePoseEstimate();
            drive.reportIdle("pick a route with the dpad");

            telemetry.addLine("DPAD UP/DOWN to pick a route, then place the robot at its start");
            telemetry.addData("Route", "(%d/%d) %s", selected + 1, Routes.ALL.size(), route.name);
            telemetry.addLine(route.description);
            if (route.requiresHolonomic && !drive.holonomic) {
                telemetry.addLine("WARNING: this route strafes and needs a mecanum drive");
            }
            telemetry.addData("Start pose", route.startPose);
            telemetry.update();
        }
        if (isStopRequested()) {
            return;
        }

        try {
            Actions.runBlocking(drive.run(route));
        } finally {
            drive.stop();
        }
    }
}
