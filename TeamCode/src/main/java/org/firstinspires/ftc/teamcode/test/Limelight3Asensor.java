package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class Limelight3Asensor {
    private Limelight3A limelight;


    public void initLimeLight(HardwareMap hardwareMap, Telemetry telemetry){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        //Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

    }
    public void getGeneralInformation(Telemetry telemetry, LLResult result){
        // Access general information
        Pose3D botpose = result.getBotpose();
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

    public void  getBarcodeResults(Telemetry telemetry, LLResult result ) {
        // Access barcode results
        List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
        for (LLResultTypes.BarcodeResult br : barcodeResults) {
            telemetry.addData("Barcode", "Data: %s", br.getData());
        }
    }
    public void getMiscResults (Telemetry telemetry, LLResult result) {
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

        // Access fiducial results
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
        }
    }
    public void getColor(Telemetry telemetry, LLResult result) {
        // Access color results
        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
        for (LLResultTypes.ColorResult cr : colorResults) {
            telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
        }
    }

    public void limelightProcessing(Telemetry telemetry) {
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            getGeneralInformation(telemetry, result);
            getBarcodeResults(telemetry, result);
            getMiscResults(telemetry, result);



            // Access color results
            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
            for (LLResultTypes.ColorResult cr : colorResults) {
                telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
            }
        } else {
            telemetry.addData("Limelight", "No data available");
        }

        telemetry.update();
    }

    public void stopLimeLightProccessing() {
        limelight.stop();
    }


}
