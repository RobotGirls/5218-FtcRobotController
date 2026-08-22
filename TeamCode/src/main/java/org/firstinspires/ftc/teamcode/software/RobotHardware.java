package org.firstinspires.ftc.teamcode.software;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware {
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;

    private HardwareMap hwMap = null;


    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;
    frontLeft = hwMap.get(DcMotor.class, "frontLeft");
    frontRight = hwMap.get(DcMotor.class, "frontRight");
    backLeft = hwMap.get(DcMotor.class, "backLeft");
    backRight = hwMap.get(DcMotor.class, "backRight");
    //SEt directions for drivetrain
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        setDrivePower(0,0,0,0);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
   public void setDrivePower(double fl, double fr, double bl, double br){
        frontRight.setPower(fr);
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        backRight.setPower(br);
   }
}

