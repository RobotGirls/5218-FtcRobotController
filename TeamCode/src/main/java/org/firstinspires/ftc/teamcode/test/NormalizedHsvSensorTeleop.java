package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

// I2C port ??  colorSensor
// Sero Port ??: servoOne
// https://www.revrobotics.com/rev-31-1557/
@TeleOp(name = "Teleop HsvColorSensor")

// @Disabled
public class NormalizedHsvSensorTeleop extends LinearOpMode {
    private NormalizedColorSensor colorSensor;
    private float hue;
    private float sat;
    private float val;
    private double targetValue = 1000;

    private Servo servoOne;
    private double servoOneInitPosition = 0.5;
    private double servoOnePositionOne = 0.0;
    private double servoOnePositionTwo = 1.0;

    public enum DetectedColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }
    private DetectedColor currentDetectedColor = DetectedColor.UNKNOWN;
    private DetectedColor latchedDetectedColor = DetectedColor.UNKNOWN;

    @Override
    public void runOpMode() {
        initColorSensor();
        initHardware();
            while (!isStarted()) {
                getDetectedColor();
                colorTelemetry();

            }

            // wait for PLAY button to be pushed
            waitForStart();
            while (opModeIsActive()) {
                getDetectedColor();
                colorTelemetry();
                teleopControls();
            }

    } // end runOpMode

    public void initHardware() {
        //initServoOne();
        initColorSensor();
    }

    public void initServoOne() {
        servoOne = hardwareMap.get(Servo.class, "servoOne");
        servoOne.setDirection(Servo.Direction.FORWARD);
        servoOne.setPosition(servoOneInitPosition);
    }

    public void initColorSensor() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensorV3");
        colorSensor.setGain(8);
    }

    public void getDetectedColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        
        hue = JavaUtil.colorToHue(colors.toColor());
        sat = JavaUtil.colorToSaturation(colors.toColor());
        val = JavaUtil.colorToValue(colors.toColor());

        // Green Ball:  h=155, sat=0.8, val=0.4
        // Purple Ball: h=242, sat=0.5, val=0.4
        // Green Ball:  R=.08, G=0.4, B=.3
        // Purple Ball: R=.25, G=.25, B=.5

        final int HUE_GREEN = 155;
        final int HUE_PURPLE = 242;
        final int HUE_TOL = 10;

        if (hue > HUE_GREEN - HUE_TOL && hue < HUE_GREEN + HUE_TOL) {
            currentDetectedColor = DetectedColor.GREEN;
            latchedDetectedColor = currentDetectedColor;
        }
        else if (hue > HUE_PURPLE - HUE_TOL && hue < HUE_PURPLE + HUE_TOL) {
            currentDetectedColor = DetectedColor.PURPLE;
            latchedDetectedColor = currentDetectedColor;
        }
        else {
            currentDetectedColor = DetectedColor.UNKNOWN;
            // do not change latchedDetectedColor here
        }
    }

    public void colorTelemetry() {
        telemetry.addData("hue", hue);
        telemetry.addData("sat", sat);
        telemetry.addData("val", val);
        telemetry.addData("currentDetectedColor", currentDetectedColor);
        telemetry.addData("latchedDetectedColor", latchedDetectedColor);
        telemetry.update();
    }

    public void teleopControls() {
//        if (alphaValue > targetValue) {
//            servoOne.setPosition(servoOnePositionTwo);
//
//        } else {
//            servoOne.setPosition(servoOnePositionOne);
//        }
    }
}
