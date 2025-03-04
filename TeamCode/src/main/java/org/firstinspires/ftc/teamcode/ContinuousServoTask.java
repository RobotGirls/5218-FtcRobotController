package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import team25core.Robot;
import team25core.RobotTask;

public class ContinuousServoTask extends RobotTask

{
    protected Robot robot;
    protected CRServo servo;

    public double right;
    public double left;

    public double ceiling;

    public boolean slow = false;
    public boolean clockwise = false;
    public boolean useRightJoystick = false;
    public boolean ceilingOn = false;

    public double slowMultiplier = 0.5;

    public ContinuousServoTask(Robot robot, CRServo servo, boolean useRightJoystick) {
        super(robot);
        this.servo = servo;
        this.robot = robot;
        this.useRightJoystick = useRightJoystick;
    }
    public ContinuousServoTask(Robot robot, CRServo servo, boolean useRightJoystick, boolean myClockwise) {
        super(robot);
        this.servo = servo;
        this.robot = robot;
        this.useRightJoystick = useRightJoystick;
        this.clockwise = myClockwise;
    }
    private void getJoystick()
    {
        Gamepad gamepad = robot.gamepad2;

        left = -gamepad.left_stick_y * slowMultiplier;
        right = gamepad.right_stick_y * slowMultiplier;
    }

    public void slowDown(boolean slow)
    {
        if (slow) {
            slowMultiplier = 0.5;
        } else {
            slowMultiplier = 1;
        }
    }

    public void slowDown(double mult)
    {
        slowMultiplier = mult;
    }

    @Override
    public void start()
    {
        // Nothing.
    }

    @Override
    public void stop()
    {
    }

    public void useCeiling(double ceiling)
    {
        ceilingOn = true;
        this.ceiling = ceiling;
    }

    @Override
    public boolean timeslice()
    {
        getJoystick();
        double joystickValue = useRightJoystick? right: left;

        double breakPosition = 0;
        double servoPosition;

        if (Math.abs(joystickValue) < breakPosition) {
            servoPosition = 0;
            servo.setPower(servoPosition);
        } else {
            servoPosition = (joystickValue);
        }

        if (ceilingOn) {
            if (servoPosition > ceiling) {
                servoPosition = ceiling;
            } else if (servoPosition < (1 - ceiling)) {
                servoPosition = 1 - ceiling;
            }
        }
        //telemetry.addData("ContinuousServoTask Servo Position", servoPosition);

        //servo.setPosition(servoPosition);
        if (clockwise) {
            servoPosition=(-servoPosition);
        }

        servo.setPower(servoPosition);


        return false;

    }
}
