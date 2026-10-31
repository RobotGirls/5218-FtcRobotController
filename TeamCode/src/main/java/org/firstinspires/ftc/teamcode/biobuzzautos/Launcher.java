package org.firstinspires.ftc.teamcode.biobuzzautos;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {
    private final DcMotorEx launcher;
    private final ElapsedTime timer;

    public Launcher(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);

        timer = new ElapsedTime();
    }

    public class LauncherForward implements Action {
        // move the motor in the direction that launches the ball
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                launcher.setPower(-0.6);
                initialized = true;
                timer.reset();
            }
            double timerValueShooter = timer.milliseconds();
            telemetry.addData("Shooter timer", timerValueShooter);
            telemetry.update();
            if (timerValueShooter < 2000) {
                return true;
            } else {
                launcher.setPower(0);
                return false;
            }
        }
    }

    public Action launcherForward() {
        return new LauncherForward();
    }

    public class LauncherBackwards implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                launcher.setPower(0.8);
                initialized = true;
                timer.reset();
            }
            double timerValueShooter = timer.milliseconds();
            telemetry.addData("Shooter timer", timerValueShooter);
            telemetry.update();
            if (timerValueShooter < 2000) {
                return true;
            } else {
                launcher.setPower(0);
                return false;
            }
        }
    }

    public Action launcherBackwards() {
        return new LauncherBackwards();
    }
}
