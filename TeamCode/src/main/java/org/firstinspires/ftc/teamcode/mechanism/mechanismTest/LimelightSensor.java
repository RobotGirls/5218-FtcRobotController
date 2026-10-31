package org.firstinspires.ftc.teamcode.mechanism.mechanismTest;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LimelightSensor {
    private Limelight3A limelight;

    LLResultTypes.FiducialResult myTag;
    LLResult myResult;

    Telemetry myTelemetry;
    boolean isRed;
    //for odometry tuning and debugging, if you are tracking exact
    // ticks or tuning data, 20ms can be useful, MAKE SURE TO RAISE IT BACK UP AFTER TO 50-100
    private static final int TimeInterval = 20;

    double derivative = 0;
    double integralSum = 0;
    double lastError = 0;
    double kp = 0.014;
    double ki = 0;
    double kd = 0;



    public LimelightSensor(HardwareMap hardwareMap, Telemetry telemetry,
                           boolean isRedAlliance) {
        myTelemetry = telemetry;
        isRed = isRedAlliance;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(20);
        if (isRed) {
            limelight.pipelineSwitch(1);
        } else {
            limelight.pipelineSwitch(0);
        }
        //Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
        limelight.start();
    }
}
