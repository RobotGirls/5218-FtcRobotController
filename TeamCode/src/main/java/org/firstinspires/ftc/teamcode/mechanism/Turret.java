package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    private DcMotor turretMotor;

    // constructor
    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotor.class, "turret");

        // FIXME check with team if they want the turret to be break
        // set the motor behavoir when theres no power; break prevents the turret from turing
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // reset and use encoders for telmetry tracking
        resetEncoder();

    } // end of turret constructer

    public void resetEncoder() {
        // resets encoder value to 0 and stops motor
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // lets the motor run after stop and reset
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    } // end of reset encoder

    public void setPower(double turretPower){
        turretMotor.setPower(turretPower);
    }

    public int getCurrentPosition(){
        return turretMotor.getCurrentPosition();
    }
} // end of Turret class
