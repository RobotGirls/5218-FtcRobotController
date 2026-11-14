package org.firstinspires.ftc.teamcode.mechanism.mechanismTest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.mechanism.Turret;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;

import java.util.List;

public class LimelightSensor {
    private Limelight3A limelight;

    private static final String LIMELIGHT_HOST = "172.29.0.1";
    private static final int LIMELIGHT_STREAM_PORT = 5800;
    private static final int RELAY_PORT = 11117;
    private static final int DASHBOARD_FPS = 5;
    private LimelightStreamRelay streamRelay;
    private LimelightDriverStationStream cameraStream;

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
    private boolean isFirstRun = true;

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
        startLivestream(telemetry);
    }

    /**
     * The Limelight is only reachable from the Control Hub over USB, so relay its MJPEG feed onto
     * the robot Wi-Fi (http://192.168.43.1:11117/) and feed frames to FTC Dashboard and the
     * Driver Station camera preview. Relay and frame grabber come from team 25's code.
     */
    private void startLivestream(Telemetry telemetry) {
        streamRelay = new LimelightStreamRelay(LIMELIGHT_HOST, LIMELIGHT_STREAM_PORT, RELAY_PORT);
        streamRelay.start();
        telemetry.addData("Limelight video", streamRelay.getStatus());

        cameraStream = new LimelightDriverStationStream(LIMELIGHT_HOST, LIMELIGHT_STREAM_PORT);
        CameraStreamServer.getInstance().setSource(cameraStream);
        FtcDashboard.getInstance().startCameraStream(cameraStream, DASHBOARD_FPS);
        telemetry.addLine("Video: 192.168.43.1:8080/dash or 192.168.43.1:11117");
    }

    /** Frees the relay port so the next OpMode can open it. Call at the end of the OpMode. */
    public void stop() {
        FtcDashboard.getInstance().stopCameraStream();
        CameraStreamServer.getInstance().setSource(null);
        if (cameraStream != null) {
            cameraStream.stop();
            cameraStream = null;
        }
        if (streamRelay != null) {
            streamRelay.stop();
            streamRelay = null;
        }
        limelight.stop();
    }

    public boolean seeValidTag(){

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



                 for (int targetId : targetIds) {
                    if (tagID == targetId) {
                        myResult = result;
                        myTag = tag;
                        myTelemetry.addData("tagId", tagID);
                        return true;

                    }
                 }
            }
        }
        return false;
    }

    public double getDistance(LLResultTypes.FiducialResult tag1) {
        // Get the AprilTag's 3D pose relative to target space
        // Index 2 of translation array corresponds to z-axis (distance)
        Pose3D tagPose = tag1.getRobotPoseTargetSpace();
        myTelemetry.addData("Distance", "Method");

        double x = tagPose.getPosition().x;
        double y = tagPose.getPosition().y;
        double z = tagPose.getPosition().z;

        double distanceToTag = Math.sqrt(
                (x*x) + (y*y) + (z*z));

        myTelemetry.addData("X", "%.3f m", x);
        myTelemetry.addData("Y", "%.3f m", y);
        myTelemetry.addData("Z", "%.3f m", z);

      //  myTelemetry.update();

        return distanceToTag;

    }
    public double getXValue() {
        myTelemetry.addData("getX", "Method");
        Pose3D tagPose = myTag.getRobotPoseTargetSpace();

        return tagPose.getPosition().x;
    }

    public boolean isValidShooting() {

        double xValue1;
        double dToTag1;
        dToTag1 = getDistance(myTag);
        xValue1 = getXValue();
        myTelemetry.addData("isValidShooting", "Method");
        myTelemetry.addData("Current X", xValue1);
        myTelemetry.addData("Distance to Tag", dToTag1);


        if (xValue1 >= Min_X && xValue1 <= Max_X && dToTag1 >= Min_D && dToTag1 <= Max_D) {
            validShooting = true;
        }

        return validShooting;
    }

    public void PIDAiming(Turret turret){
        myTelemetry.addData("PID Aiming", "Method");
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

                double servoPower = (kp * error) + (ki * integralSum) + (kd * derivative);
                // Clamp power between -1.0 and 1.0
                if (servoPower > 1.0) servoPower = 1.0;
                if (servoPower < -1.0) servoPower = -1.0;

                turret.setPower(servoPower);

                lastError = error;
                lastTime = currentTime;

                myTelemetry.addData("Limelight Target", "Tracking");
                myTelemetry.addData("Error (tx)", error);
                myTelemetry.addData("Turret Power", servoPower);
                myTelemetry.update();
            } else {
                turret.setPower(0);
                myTelemetry.addData("Limelight Target", "Aligned (error <= 3 deg)");
            }
        } else {
            turret.setPower(0);
            resetPID();
            myTelemetry.addData("Limelight Target", "No target detected");
        }
    }




    public void resetPID() {
        myTelemetry.addData("Turret", "Method");
        integralSum = 0;
        lastError = 0;
        if (timer != null) {
            lastTime = timer.seconds();
        }
    }
}
