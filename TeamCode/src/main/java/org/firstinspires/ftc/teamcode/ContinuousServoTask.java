package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import team25core.Robot;
import team25core.RobotTask;

public class ContinuousServoTask extends RobotTask

{
    protected Robot robot;
    protected Servo servo;
    public double right;
    public double left;

    public double ceiling;

    public boolean slow = false;
    public boolean useRightJoystick = false;
    public boolean ceilingOn = false;

    public double slowMultiplier = 0.5;

    public ContinuousServoTask(Robot robot, Servo servo, boolean useRightJoystick) {
        super(robot);
        this.servo = servo;
        this.robot = robot;
        this.useRightJoystick = useRightJoystick;
    }
    private void getJoystick()
    {
        Gamepad gamepad = robot.gamepad2;

        left = -gamepad.left_stick_y * slowMultiplier;
        right = -gamepad.right_stick_y * slowMultiplier;
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

        double breakPosition = 0.05;
        double servoPosition;

        if (Math.abs(joystickValue) < breakPosition) {
            servoPosition = 0.5;
        } else {
            servoPosition = 0.5 + (joystickValue * 0.5);
        }

        if (ceilingOn) {
            if (servoPosition > ceiling) {
                servoPosition = ceiling;
            } else if (servoPosition < (1 - ceiling)) {
                servoPosition = 1 - ceiling;
            }
        }

        servo.setPosition(servoPosition);

        return false;

    }
}
