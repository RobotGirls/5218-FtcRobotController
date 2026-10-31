package org.firstinspires.ftc.teamcode.mechanism.mechanismTest;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware {
    // Declare drive motors
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;

    // Local copy of hardware map
    private HardwareMap hwMap = null;


    /*--------------------------------------------------------------
     * Initialize standard hardware interfaces
     * @param ahwMap The Robot's Hardware Map
     */
    public RobotHardware(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Define and Initialize Motors using the exact names from the
        // Driver Station config
        frontLeft = hwMap.get(DcMotor.class, "leftFront");
        frontRight = hwMap.get(DcMotor.class, "rightFront");
        backLeft = hwMap.get(DcMotor.class, "leftBack");
        backRight = hwMap.get(DcMotor.class, "rightBack");

        // Set motor directions (Adjust based on your physical robot's gearing)
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Setting all motors to zero power during initialization is a safety
        // standard in robotics. When you initialize an OpMode in FTC, the robot
        // transitions from an unconfigured state to an active state. Setting
        // the power to 0 right away ensures 1) Preventing Unexpected Jumps: If
        // the robot was previously running another program that crashed, or if
        // a motor controller retained a small residual signal, setting the power
        // to 0 explicitly forces the motors to stay completely still when you
        // hit the "Init" button. 2) Establishing a Known Baseline: It guarantees
        // that the software's internal command matches the physical state of
        // the robot before the drivers are allowed to touch the gamepads. It
        // acts as a digital "handbrake" until the waitForStart() command is
        // passed and the actual TeleOp loop begins.
        setDrivePower(0, 0, 0, 0);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to run without encoders for TeleOp
        //setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Reset and establish explicit run mode for predictable movements
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Convenience method to set all drive motor powers quickly
     */
    public void setDrivePower(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
    /**
     * Convenience method to change all drive motor modes simultaneously
     * @param mode Target DcMotor RunMode (e.g., RUN_USING_ENCODER, RUN_TO_POSITION)
     */
    public void setDriveMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }
    /**
     * Controls all drive motors for a set amount of time.
     * @param fl Front Left power (-1.0 to 1.0)
     * @param fr Front Right power (-1.0 to 1.0)
     * @param bl Back Left power (-1.0 to 1.0)
     * @param br Back Right power (-1.0 to 1.0)
     * @param durationMs Time in milliseconds to run the motors
     * @param opMode Pass 'this' from your autonomous file to keep it safe
     */
    public void driveTime(double fl, double fr, double bl, double br, long durationMs, com.qualcomm.robotcore.eventloop.opmode.LinearOpMode opMode) {
        setDrivePower(fl, fr, bl, br);

        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < durationMs && !opMode.isStopRequested()) {
            opMode.idle();
        }

        setDrivePower(0, 0, 0, 0); // Always stop at the end
    }


}