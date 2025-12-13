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

    private double Kp = 0.014; // Tx range is 0 to 26 --> at max offset 26, when Kp is 0.02, speed is half power
    private double Ki = 0;
    private double Kd = 0;
    private double currTime = 0;
    private double prevTime = 0;
    private Position position;
    private Pose3D localBotPose;

    Pose3D botpose;


    LLResult localResult;
    public void initLimelight(HardwareMap hardwareMap, Telemetry telemetry) {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        // Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
    }

    public void getGeneralInformation(Telemetry telemetry, LLResult result) {
        // Access general information
        botpose = result.getBotpose();
        double captureLatency = result.getCaptureLatency();
        double targetingLatency = result.getTargetingLatency();
        double parseLatency = result.getParseLatency();
        telemetry.addData("LL Latency", captureLatency + targetingLatency);
        telemetry.addData("Parse Latency", parseLatency);
        telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

        telemetry.addData("tx", result.getTx());
        telemetry.addData("txnc", result.getTxNC());
        telemetry.addData("ty", result.getTy());
        telemetry.addData("tync", result.getTyNC());

        telemetry.addData("Botpose", botpose.toString());
    }
    public void getBarcodeResults(Telemetry telemetry, LLResult result) {
        // Access barcode results
        List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
        for (LLResultTypes.BarcodeResult br : barcodeResults) {
            telemetry.addData("Barcode", "Data: %s", br.getData());
        }
    }

    public void getMiscResults(Telemetry telemetry, LLResult result) {
        // Access classifier results
        List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
        for (LLResultTypes.ClassifierResult cr : classifierResults) {
            telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
        }

        // Access detector results
        List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
        for (LLResultTypes.DetectorResult dr : detectorResults) {
            telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
        }

    }

    public double getFiducials(Telemetry telemetry, LLResult result) {
        // Access fiducial results
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            double tagId = fr.getFiducialId();
//          telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
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
            telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
        }
    }

    public double adjustFlywheelSpeed(Telemetry telemetry) {
        double deltaTime;
        localBotPose = localResult.getBotpose();
        position = localBotPose.getPosition();
        double error = position.y;

        if (Math.abs(error) > alignThreshold) {
            error = -1 * localResult.getTy();
            deltaTime = currTime-prevTime;
            derivative = (error - lastError) / deltaTime;
            integralSum = integralSum + (error * deltaTime);
            prevTime = currTime;
            double power = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
            lastError = error;
            telemetry.addData("flywheelSpeedError", error);
            return power;
        } else {
            return 0;
        }


    }

    public void limelightProcessing(Telemetry telemetry, ElapsedTime timer) {
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        currTime = timer.seconds();
        localResult = result;
        if (result.isValid()) {
            getGeneralInformation(telemetry, result);
            getBarcodeResults(telemetry, result);
            getMiscResults(telemetry, result);
            getColor(telemetry, result);
            getFiducials(telemetry, result);

        } else {
            telemetry.addData("Limelight", "No data available");
        }
        telemetry.update();
    }

    public void stopLimelightProcessing() {
        limelight.stop();
    }
}

