package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

public class Limelight3ASensor {
    private Limelight3A limelight;
    private int alignThreshold = 3;
    private double lastError = 0;
    private double derivative;
    private double integralSum = 0;
    private double lastErrorTlm = 0;
    private double deltaTimeTlm = 0;
    private double powerTlm = 0;

    private double errorTlm = 0;

    private double Kp = 0.025; // Tx range is 0 to 26 --> at max offset 26, when Kp is 0.02, speed is half power
    //private double Kp = 0.014; // Tx range is 0 to 26 --> at max offset 26, when Kp is 0.02, speed is half power

    private double Ki = 0;
    private double Kd = 0;
    private double currTime = 0;
    private double prevTime = 0;
    private Position position;
    private Pose3D localBotPose;
    private Telemetry myTelemetry;
    Pose3D botpose;


    LLResult localResult;
    public void initLimelight(HardwareMap hardwareMap, Telemetry telemetry) {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        // Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
        limelight.start();
        myTelemetry = telemetry;
        telemetry.addData(">", "Robot Ready2.  Press Play.");
        // telemetry.update();
    }

    public void getGeneralInformation(Telemetry telemetry, LLResult result) {
        // Access general information
        botpose = result.getBotpose();
        double captureLatency = result.getCaptureLatency();
        double targetingLatency = result.getTargetingLatency();
        double parseLatency = result.getParseLatency();
        myTelemetry.addData("/n/n/nflywheelSpeedError", errorTlm);
        myTelemetry.addData("adjustedPower", powerTlm);
        myTelemetry.addData("error", errorTlm);
        myTelemetry.addData("deltaTime", deltaTimeTlm);
        myTelemetry.addData("lastError", lastError);
        myTelemetry.addData("LL Latency", captureLatency + targetingLatency);
        myTelemetry.addData("Parse Latency", parseLatency);
        myTelemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

        myTelemetry.addData("tx", result.getTx());
        myTelemetry.addData("txnc", result.getTxNC());
        myTelemetry.addData("ty", result.getTy());
        myTelemetry.addData("tync", result.getTyNC());

        myTelemetry.addData("Botpose", botpose.toString());

    }
    public void getBarcodeResults(Telemetry telemetry, LLResult result) {
        // Access barcode results
        List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
        for (LLResultTypes.BarcodeResult br : barcodeResults) {
            myTelemetry.addData("Barcode", "Data: %s", br.getData());
        }
    }

    public void getMiscResults(Telemetry telemetry, LLResult result) {
        // Access classifier results
        List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
        for (LLResultTypes.ClassifierResult cr : classifierResults) {
            myTelemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
        }

        // Access detector results
        List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
        for (LLResultTypes.DetectorResult dr : detectorResults) {
            myTelemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
        }

    }

    public double getFiducials(Telemetry telemetry, LLResult result) {
        // Access fiducial results
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            double tagId = fr.getFiducialId();
//          myTelemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            return tagId;
        }
        return 0;
    }
    public double getTx() {
        //
        LLResult result = limelight.getLatestResult();
        if (  result != null && result.isValid()) {
            return result.getTx();
        }
        return 0;
    }
    public void getColor(Telemetry telemetry, LLResult result) {
        // Access color results
        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
        for (LLResultTypes.ColorResult cr : colorResults) {
            myTelemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
        }
    }

    public double adjustFlywheelSpeed(Telemetry telemetry) {
        double deltaTime;
        localBotPose = localResult.getBotpose();
        position = localBotPose.getPosition();
        double error = position.y;

        if (Math.abs(error) > alignThreshold) {
            error = -1 * position.y;
            deltaTime = currTime-prevTime;
            derivative = (error - lastError) / deltaTime;
            integralSum = integralSum + (error * deltaTime);
            prevTime = currTime;
            double power = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
            lastError = error;
            lastErrorTlm = lastError;
            deltaTimeTlm = deltaTime;
            powerTlm = power;
            errorTlm = error;

            //telemetry.update();
            return power;
        } else {
            //telemetry.update();
            return 0;
        }


    }

    public void limelightProcessing(Telemetry telemetry, ElapsedTime timer) {
        LLStatus status = limelight.getStatus();
        myTelemetry.addData("Name", "%s",
                status.getName());
        //myTelemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
               // status.getTemp(), status.getCpu(),(int)status.getFps());
        //myTelemetry.addData("Pipeline", "Index: %d, Type: %s",
                //status.getPipelineIndex(), status.getPipelineType());
        myTelemetry.update(); // this one printed telem

        LLResult result = limelight.getLatestResult();
        currTime = timer.seconds();
        localResult = result;
        if (result.isValid()) {
            getGeneralInformation(telemetry, result);
            //getBarcodeResults(telemetry, result);
            //getMiscResults(telemetry, result);
            //getColor(telemetry, result);
            //getFiducials(telemetry, result);

        } else {
            myTelemetry.addData("Limelight", "No data available");
        }

    }

    public void stopLimelightProcessing() {
        limelight.stop();
    }

    public boolean hasValidTarget() {


        return false;
    }

    public double getTagDistanceCm() {
        return 0;
    }
}

