package org.firstinspires.ftc.teamcode.teleopIntakes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DoubleWheelIntake {




        private DcMotor intakeMotor;

        // constructor
        public DoubleWheelIntake(HardwareMap hardwareMap) {
            intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        }

        public void setPower(double power) {
            intakeMotor.setPower(power);
        }

        public void stop() {
            intakeMotor.setPower(0.0);
        }
    }

