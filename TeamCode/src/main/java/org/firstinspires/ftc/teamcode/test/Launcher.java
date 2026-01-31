package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
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
    private DcMotorEx launcher;
    private ElapsedTime timer;

    public Launcher() {
        launcher = hardwareMap.get(DcMotorEx.class, "FlyWheelMotor");
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
            if (timerValueShooter < 4000) {
                return true;
            } else {
                launcher.setPower(0);
                return false;
            }
        }
    }

   

    public class LauncherBackwards implements Action {
        private boolean initialized = false;
        private ElapsedTime timer1;

        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                timer1.reset();
                if (timer1.milliseconds() < 2000) {
                    launcher.setPower(0.8);

                } else {
                    launcher.setPower(0);
                    timer1.reset();
                    initialized = true;
                }
            }
            return true;
        }
    }
    

public class Intake {
    private DcMotorEx intakeMotor;

    private ElapsedTime timer1;


    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        timer1 = new ElapsedTime();

    }
    public class IntakeIn implements Action {
        // move the motor in the direction that moves the ball into the robot;
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                intakeMotor.setPower(-0.8);
                initialized = true;
                timer1.reset();
            }
            double timerValue = timer1.milliseconds();
            telemetry.addData("Intake Timer",timerValue);
            telemetry.update();
            if (timerValue < 4000) {
                return true;
            } else {
                intakeMotor.setPower(0);
                return false;
            }

        }
    }
   

    public class IntakeOut implements Action {
        private boolean initialized = false;


        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                intakeMotor.setPower(0.8);
                initialized = true;
                timer1.reset();
            }
            double timerValue = timer1.milliseconds();
            telemetry.addData("Intake Timer", timerValue);
            telemetry.update();
            if (timerValue < 4000) {
                return true;
            } else {
                intakeMotor.setPower(0);
                return false;
            }

        }


    }
}


    

}
