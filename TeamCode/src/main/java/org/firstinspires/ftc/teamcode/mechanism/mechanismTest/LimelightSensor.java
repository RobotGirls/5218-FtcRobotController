package org.firstinspires.ftc.teamcode.mechanism.mechanismTest;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class LimelightSensor {
    private Limelight3A limelight;

    LLResultTypes.FiducialResult myTag;
    LLResult myResult;
    final double Min_X = 1.5;
    final double Max_X = 2.5;
    final double Min_D = -2.5;
    final double Max_D = -1.5;
    boolean validShooting;
    Telemetry myTelemetry;
    boolean isRed;
    public static final int[] BLUE_TAGS = {38, 39, 40, 41, 42, 43, 44, 45};
    public static final int[] RED_TAGS = {30, 31, 32, 33, 34, 35, 36, 37};
    int[] targetIds;

    //for odometry tuning and debugging, if you are tracking exact
    // ticks or tuning data, 20ms can be useful, MAKE SURE TO RAISE IT BACK UP AFTER TO 50-100
    private static final int TimeInterval = 20;

    double derivative = 0;
    double integralSum = 0;
    double lastError = 0;
    double lastTime = 0;
    private ElapsedTime timer;

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
        timer = new ElapsedTime();
        //Starts polling for data. If you neglect to call start(), getLatestResult() will return null.
        limelight.start();
    }

    public boolean checkTag(){
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {

            List<LLResultTypes.FiducialResult> fiducials =
                    result.getFiducialResults();
            if (isRed){
               targetIds = RED_TAGS;
            }else {
                targetIds = BLUE_TAGS;
            }
            for (LLResultTypes.FiducialResult tag : fiducials) {
                int tagID = tag.getFiducialId();

                 myTag = tag;

                 for (int targetId : targetIds) {
                    if (tagID == targetId) {
                        myResult = result;
                        return true;
                    }
            }
        }
}
        return false;
    }

    public double getDistance() {
        // Get the AprilTag's 3D pose relative to target space
        // Index 2 of translation array corresponds to z-axis (distance)
        Pose3D tagPose = myTag.getRobotPoseTargetSpace();

        double x = tagPose.getPosition().x;
        double y = tagPose.getPosition().y;
        double z = tagPose.getPosition().z;

        double distanceToTag = Math.sqrt(
                (x*x) + (y*y) + (z*z));

        myTelemetry.addData("X", "%.3f m", x);
        myTelemetry.addData("Y", "%.3f m", y);
        myTelemetry.addData("Z", "%.3f m", z);

        return distanceToTag;

    }
    public double getXValue() {
        Pose3D tagPose = myTag.getRobotPoseTargetSpace();

        return tagPose.getPosition().x;
    }

    public boolean isValidShooting(double xValue1, double dToTag1) {

        myTelemetry.addData("Current X", xValue1);
        myTelemetry.addData("Distance to Tag", dToTag1);
        myTelemetry.update();

        if (xValue1 >= Min_X && xValue1 <= Max_X && dToTag1 >= Min_D && dToTag1 <= Max_D) {
            validShooting = true;
        }

        return validShooting;
    }

    public void PIDAiming(DcMotor turretMotor){
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double error = result.getTx();
            if (Math.abs(error) > 3) {
                double currentTime = timer.seconds();
                double deltaTime = currentTime - lastTime;
                if (deltaTime <= 0) deltaTime = 0.001;

                error = -1 * result.getTx();
                derivative = (error - lastError) / deltaTime;
                integralSum = integralSum + (error * deltaTime);

                double motorPower = (kp * error) + (ki * integralSum) + (kd * derivative);
                // Clamp power between -1.0 and 1.0
                if (motorPower > 1.0) motorPower = 1.0;
                if (motorPower < -1.0) motorPower = -1.0;

                turretMotor.setPower(motorPower);

                lastError = error;
                lastTime = currentTime;

                myTelemetry.addData("Limelight Target", "Tracking");
                myTelemetry.addData("Error (tx)", error);
                myTelemetry.addData("Motor Power", motorPower);
            } else {
                turretMotor.setPower(0);
                myTelemetry.addData("Limelight Target", "Aligned (error <= 3 deg)");
            }
        } else {
            turretMotor.setPower(0);
            resetPID();
            myTelemetry.addData("Limelight Target", "No target detected");
        }
    }




    public void resetPID() {
        integralSum = 0;
        lastError = 0;
        if (timer != null) {
            lastTime = timer.seconds();
        }
    }
}
