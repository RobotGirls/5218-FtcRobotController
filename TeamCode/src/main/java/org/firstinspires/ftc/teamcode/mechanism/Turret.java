package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Turret {
    public CRServo turretServo;

    // servo travel limits (0.0 to 1.0); shrink these if the turret hits the robot
    private static final double MAX_POWER = 0.7;
    private static final double MIN_POS = 0.0;
    private static final double MAX_POS = 1.0;
    private static final double CENTER = 0.5;

    // how fast the turret moves at full power, in servo units per second (tune this)
    private static final double MAX_SPEED = 0.5;

    private double targetPos = CENTER;
    private final ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;

    // constructor
    public Turret(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(CRServo.class, "turret");

        // flip this if the turret turns the wrong way
        // turretServo.setDirection(Servo.Direction.REVERSE);

        // start at the center position
        //setPosition(CENTER);
    } // end of turret constructor

    // sends the servo to a position between MIN_POS and MAX_POS
//    public void setPosition(double position) {
//        targetPos = Range.clip(position, MIN_POS, MAX_POS);
//        turretServo.setPosition(targetPos);
//    }
    public void setPower(double power) {
        turretServo.setPower(power* MAX_POWER);
    }

    public void stop(){
        turretServo.setPower(0);
    }

    // power is -1 to 1; call once per loop
    // turns the power into a speed by moving the target position a little each loop
//    public void setPower(double power) {
//        double now = timer.seconds();
//        double dt = Math.min(now - lastTime, 0.1); // cap dt so a slow loop can't cause a jump
//        lastTime = now;
//        setPosition(targetPos + power * MAX_SPEED * dt);
//    }

    // last commanded position, not a measured one (servos have no encoder)
    public double getPosition() {
        return targetPos;
    }

    // true when the turret has hit either end of its range
    public boolean atLimit() {
        return targetPos <= MIN_POS || targetPos >= MAX_POS;
    }
} // end of Turret class