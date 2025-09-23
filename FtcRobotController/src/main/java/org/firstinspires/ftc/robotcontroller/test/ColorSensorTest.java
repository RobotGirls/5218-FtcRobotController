package org.firstinspires.ftc.robotcontroller.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
@TeleOp(name = "TeleOp Color Sensor")
public class ColorSensorTest extends LinearOpMode {
    private ColorSensor colorSensor;
    private double redValue;
    private double blueValue;
    private double greenValue;
    private double alphaValue; // Alpha value is light intensity
    @Override
    public void runOpMode() throws InterruptedException {

        initColorSensor();
        while (!isStarted()) {
            getColor();
            colorTelemetry();
        }


    }
    public void getColor() {
        redValue = colorSensor.red();
        greenValue = colorSensor.green();
        blueValue = colorSensor.blue();
        alphaValue = colorSensor.alpha();
    }
    public void colorTelemetry() {
       telemetry.addData("redValue","%0.2f", redValue);
       telemetry.addData("blueValue","%0.2f", blueValue);
       telemetry.addData("greenValue","%0.2f", greenValue);
       telemetry.addData("alphaValue","%0.2f", alphaValue);
    }
    public void initColorSensor() {
        colorSensor = hardwareMap.get(ColorSensor.class, "ArtifactColorSensor");
    }

}


















