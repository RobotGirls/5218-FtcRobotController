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

public class Intake {
    private final DcMotorEx intakeMotor;
    private final ElapsedTime timer1;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
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
            telemetry.addData("Intake Timer", timerValue);
            telemetry.update();
            if (timerValue < 2000) {
                return true;
            } else {
                intakeMotor.setPower(0);
                return false;
            }
        }
    }

    public Action intakeIn() {
        return new IntakeIn();
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
            if (timerValue < 2000) {
                return true;
            } else {
                intakeMotor.setPower(0);
                return false;
            }
        }
    }

    public Action intakeOut() {
        return new IntakeOut();
    }

    {

    }
}
