package org.firstinspires.ftc.teamcode.purepursuitautos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.purepursuit.AutoTuner;
import org.firstinspires.ftc.teamcode.purepursuit.DriveSimulator;
import org.firstinspires.ftc.teamcode.purepursuit.TuningFile;
import org.firstinspires.ftc.teamcode.purepursuit.paths.TestRoutes;
import org.firstinspires.ftc.teamcode.purepursuit.ftc.PurePursuitConfig;
import org.firstinspires.ftc.teamcode.purepursuit.ftc.PurePursuitDrive;

import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Measures the robot and tunes the pure pursuit follower to match it (see {@link AutoTuner}).
 * Put the robot at the TEST Tuning Course start, (-48, -48) facing +x in the bottom-left tile
 * lane, with the lane clear. Watch it in the FieldViewer's Live mode.
 *
 * The results are saved on the Control Hub, so every pure pursuit OpMode loads them on its own
 * from then on, and the FieldViewer picks them up in Live mode. Press B during init to delete
 * them and go back to the defaults.
 */
@Autonomous(name = "PP Auto Tuner", group = "Pure Pursuit")
public class PPAutoTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d start = TestRoutes.TUNING_COURSE.startPose;
        PurePursuitDrive drive = PurePursuitDrive.create(hardwareMap, start);
        drive.setRouteName("Auto Tuner");
        AutoTuner tuner = new AutoTuner(start, PurePursuitConfig.FOLLOWER, drive.holonomic);
        boolean bWasPressed = false;

        while (opModeInInit()) {
            if (gamepad1.b && !bWasPressed) {
                PurePursuitConfig.clearSavedTuning();
            }
            bWasPressed = gamepad1.b;

            drive.updatePoseEstimate();
            drive.reportIdle("place the robot at the tuning course start");
            telemetry.addLine("Put the robot at (-48, -48) facing +x (bottom-left lane), lane clear.");
            telemetry.addLine("It drives about 4 ft forward, strafes, and spins. Press Stop to abort.");
            telemetry.addData("Drive", PurePursuitConfig.DRIVE_TYPE);
            telemetry.addData("Tuning now", PurePursuitConfig.tuningSource());
            telemetry.addLine("Press B (gamepad 1) to delete the saved tuning and use the defaults.");
            telemetry.addData("Localizer pose", drive.getPose());
            telemetry.update();
        }
        if (isStopRequested()) {
            return;
        }

        // the tuner changes FOLLOWER as it goes; put it back if the run doesn't finish
        TuningFile before = TuningFile.of(PurePursuitConfig.FOLLOWER, new DriveSimulator.Params(), "");
        ElapsedTime loopTimer = new ElapsedTime();
        try {
            while (opModeIsActive() && !tuner.isDone()) {
                PoseVelocity2d velocity = drive.updatePoseEstimate();
                double dt = Math.min(loopTimer.seconds(), 0.1);
                loopTimer.reset();
                drive.setPowers(tuner.update(drive.getPose(), velocity, dt));
                drive.reportIdle(tuner.getStatus());

                telemetry.addData("Status", tuner.getStatus());
                for (String line : tuner.getLog()) {
                    telemetry.addLine(line);
                }
                telemetry.update();
            }
        } finally {
            drive.stop();
        }
        if (!tuner.isDone()) {
            tuner.abort("stopped early");
        }

        String saved;
        if (tuner.isAborted()) {
            before.applyTo(PurePursuitConfig.FOLLOWER);
            saved = "Not saved (the tuner didn't finish). Tuning is back to: " + PurePursuitConfig.tuningSource();
        } else {
            String when = new SimpleDateFormat("MMM d h:mm a", Locale.US).format(new Date());
            try {
                PurePursuitConfig.saveTuning(TuningFile.of(PurePursuitConfig.FOLLOWER, tuner.measuredDrive, when));
                saved = "Saved on the Control Hub. Every pure pursuit OpMode uses these now.";
            } catch (IOException e) {
                saved = "Couldn't save (" + e.getMessage() + "). In use until the robot restarts.";
            }
        }
        RobotLog.ii("PPAutoTuner", saved);
        for (String line : tuner.javaLines()) {
            RobotLog.ii("PPAutoTuner", line);
        }

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            drive.reportIdle(tuner.getStatus());
            telemetry.addData("Status", tuner.getStatus());
            telemetry.addLine(saved);
            if (!tuner.isAborted()) {
                telemetry.addLine(tuner.isMatched() ? "The robot stops where the paths say."
                        : "Not fully matched yet; these are the best values found.");
                telemetry.addLine("To keep them in git too, copy these into PurePursuitConfig:");
                for (String line : tuner.javaLines()) {
                    telemetry.addLine(line);
                }
            }
            telemetry.addLine("");
            for (String line : tuner.getLog()) {
                telemetry.addLine(line);
            }
            telemetry.update();
        }
    }
}
